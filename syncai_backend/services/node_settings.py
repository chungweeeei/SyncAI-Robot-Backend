"""A settings page for one node: read, change and reset what its schema lists.

One service per node (``controller_server`` with ``controller_schema``,
``planner_server`` with ``planner_schema``). A change is a small saga over two
places that must agree, the running node and the overlay file it boots from:

1. Validate: each value against its field (type, range), then the rules that
   tie it to other parameters, judged on the values as they will be.
2. Set the live and idle fields on the running node -- undotted (the
   server's own) first, in a set of their own, because controller_server
   refuses those mid-goal (see ``gateways/parameters``).
3. Read them back. A set the node reports as successful can still leave
   another value in force (a callback that "rejects" by skipping), so the
   read, not the report, is what gets persisted.
4. Write the overlay. Restart fields go only here.

A failure after step 2 restores the values read before it, best effort, so a
change is not live without being persisted -- or persisted half live. With the
nav session down (MANUAL mode, or the node restarting) there is nothing
to set: the change is written to the overlay alone and applies at its start.

One change at a time (409 ``settings_busy``): two interleaved sagas would each
restore the other's values on a failure.
"""

import functools
import math
import threading
from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Iterable, Iterator, Optional

import structlog

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.repositories.params.overlay import ParamOverlayRepo

if TYPE_CHECKING:
    # Type only: the gateway imports rclpy, and this saga is tested without it.
    from syncai_backend.gateways.parameters.parameters import ParameterGateway
from syncai_backend.services.settings_schema import (
    APPLY_RESTART,
    Group,
    SettingField,
    SettingsSchema,
    Value,
    coerce,
)


@dataclass(frozen=True)
class FieldState:
    setting: SettingField
    # What the form shows: the value in force, or for a restart field the
    # value the next start will load.
    value: Optional[Value]
    # The running node's, None with the nav session down.
    current: Optional[Value]
    # The factory params file's, None when it is not mounted.
    default: Optional[Value]
    # The overlay's, None when the operator never changed it.
    persisted: Optional[Value]
    # A restart field whose next-boot value is not the one in force.
    pending_restart: bool


@dataclass(frozen=True)
class SettingsSnapshot:
    live_available: bool
    groups: tuple[Group, ...]
    fields: tuple[FieldState, ...]


def _same(a: Optional[Value], b: Optional[Value]) -> bool:
    if isinstance(a, float) or isinstance(b, float):
        if a is None or b is None or isinstance(a, bool) or isinstance(b, bool):
            return False
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)
    return a == b


def _batches(values: dict[str, Value]) -> tuple[dict[str, Value], dict[str, Value]]:
    """``values`` split for set_atomically: the undotted (the server's own,
    refused mid-goal) first, alone, so a refusal there leaves the plugins
    untouched; then the dotted."""
    return (
        {n: v for n, v in values.items() if "." not in n},
        {n: v for n, v in values.items() if "." in n},
    )


# Whether the node is up, asked at most once per request: every probe of a
# node that is down waits out ParameterGateway's SERVICE_WAIT_S.
LiveProbe = Callable[[], bool]


class NodeSettingsService:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        schema: SettingsSchema,
        gateway: "ParameterGateway",
        overlay_repo: ParamOverlayRepo,
        subject: str,
    ):
        """``subject`` is the node in the operator's words ("controller",
        "planner"), for the sentences a refusal is told in."""
        self._logger = logger.bind(settings=subject)
        self._schema = schema
        self._gw = gateway
        self._repo = overlay_repo
        self._subject = subject
        self._changing = threading.Lock()

    # --- Reading -----------------------------------------------------------

    def snapshot(self, live_up: Optional[LiveProbe] = None) -> SettingsSnapshot:
        """``live_up`` is the probe of the request this read ends, if any."""
        factory = self._factory()
        overlay = self._overlay()
        live = self._live(
            [name for f in self._schema.fields for name in f.targets],
            live_up or self._probe(),
        )

        fields = []
        for setting in self._schema.fields:
            name = setting.source
            current = live.get(name) if live is not None else None
            default = factory.get(name)
            persisted = overlay.get(name)
            next_boot = persisted if persisted is not None else default
            restart = setting.apply == APPLY_RESTART
            pending = (
                restart
                and current is not None
                and next_boot is not None
                and not _same(current, next_boot)
            )
            if restart or current is None:
                value = next_boot
            else:
                value = current
            fields.append(
                FieldState(
                    setting=setting,
                    value=value,
                    current=current,
                    default=default,
                    persisted=persisted,
                    pending_restart=pending,
                )
            )
        return SettingsSnapshot(
            live_available=live is not None, groups=self._schema.groups, fields=tuple(fields)
        )

    # --- Changing ----------------------------------------------------------

    def apply(self, values: dict[str, object]) -> SettingsSnapshot:
        if not values:
            raise BadRequestError("No settings to change.", code="invalid_setting")
        live_up = self._probe()
        with self._one_change_at_a_time():
            new_values = self._validate(values, live_up)
            settings = [self._schema.by_id[field_id] for field_id in values]
            self._commit(settings, new_values, drop=(), live_up=live_up)
        return self.snapshot(live_up)

    def reset(self, field_ids: Optional[Iterable[str]] = None) -> SettingsSnapshot:
        """Back to the factory values: the keys leave the overlay, and the live
        and idle fields are set to the factory file's values."""
        ids = list(field_ids) if field_ids is not None else [f.id for f in self._schema.fields]
        unknown = [field_id for field_id in ids if field_id not in self._schema.by_id]
        if unknown:
            raise BadRequestError(
                f"Unknown setting: {', '.join(unknown)}", code="invalid_setting"
            )
        live_up = self._probe()
        with self._one_change_at_a_time():
            factory = self._factory()
            settings = [self._schema.by_id[field_id] for field_id in ids]
            names = [name for s in settings for name in s.targets]
            missing = [name for name in names if name not in factory]
            if missing:
                raise ConflictError(
                    "The robot's default settings cannot be read, so they cannot "
                    "be restored from here.",
                    code="defaults_unavailable",
                )
            self._commit(
                settings,
                {name: factory[name] for name in names},
                drop=tuple(names),
                live_up=live_up,
            )
        return self.snapshot(live_up)

    # --- The saga ----------------------------------------------------------

    def _commit(
        self,
        settings: list[SettingField],
        new_values: dict[str, Value],
        drop: tuple[str, ...],
        live_up: LiveProbe,
    ) -> None:
        """Set ``new_values`` live where that applies, then write the overlay:
        ``new_values`` merged in, or for a reset the ``drop`` keys removed."""
        live_names = [
            name
            for s in settings
            if s.apply != APPLY_RESTART
            for name in s.targets
        ]
        live_values = {name: new_values[name] for name in live_names}

        # Read before anything is set live: an unreadable overlay refuses
        # (params_unreadable), and must do so while there is nothing to undo.
        overlay = self._overlay()
        if drop:
            for name in drop:
                overlay.pop(name, None)
        else:
            overlay.update(new_values)

        restore: dict[str, Value] = {}
        if live_values and live_up():
            restore = self._set_live(live_values)
        elif live_values:
            self._logger.info(
                "[NodeSettings] Node not running; change written to "
                "the overlay only, in force from its next start",
                names=sorted(live_values),
            )

        try:
            self._repo.write(overlay)
        except (OSError, ValueError) as exc:
            self._logger.error(
                "[NodeSettings] Overlay write failed; restoring the live values",
                path=self._repo.overlay_path,
                error=str(exc),
            )
            self._restore(restore)
            raise ConflictError(
                f"The settings could not be saved on the robot ({exc}); nothing "
                "was changed.",
                code="params_not_writable",
            ) from exc

    def _set_live(self, values: dict[str, Value]) -> dict[str, Value]:
        """Set ``values`` on the node and confirm them; the values they
        replaced are returned for a later undo. Raises on any failure, after
        undoing whatever it had set."""
        names = list(values)
        ok, message, before = self._gw.get(names)
        if not ok:
            raise UpstreamError(message)
        unknown = [name for name in names if before.get(name) is None]
        if unknown:
            # The schema names a parameter this node does not declare: a
            # renamed parameter in the workspace, i.e. a schema out of date.
            self._logger.error(
                "[NodeSettings] Node lacks parameters the schema names",
                names=unknown,
            )
            raise UpstreamError(
                f"The robot's {self._subject} has no parameter {', '.join(unknown)}; "
                "this backend does not match the robot's software."
            )
        restore = {name: before[name] for name in names}

        done: dict[str, Value] = {}
        for batch in _batches(values):
            ok, message = self._gw.set_atomically(batch)
            if not ok:
                self._restore({n: restore[n] for n in done})
                raise self._set_error(message)
            done.update(batch)

        ok, message, after = self._gw.get(names)
        wrong = (
            [name for name in names if not _same(after.get(name), values[name])]
            if ok
            else names
        )
        if wrong:
            self._logger.error(
                "[NodeSettings] Node reports values other than those set",
                read_ok=ok,
                message=message,
                names=wrong,
                wanted={n: values[n] for n in wrong},
                read={n: after.get(n) for n in wrong} if ok else None,
            )
            self._restore(restore)
            raise UpstreamError(
                "The robot did not take the change (it reports other values); "
                "nothing was changed."
            )
        return restore

    def _restore(self, values: dict[str, Value]) -> None:
        """Best-effort undo of a live set; logged, never raised."""
        if not values:
            return
        for batch in _batches(values):
            ok, message = self._gw.set_atomically(batch)
            if not ok:
                self._logger.error(
                    "[NodeSettings] Could not restore the node's "
                    "previous values; it runs on the new ones until restarted",
                    values=batch,
                    message=message,
                )

    @staticmethod
    def _set_error(message: str) -> Exception:
        code = failure_code(message)
        if code == Failure.CONTROLLER_BUSY:
            return ConflictError(message, code=code.value)
        if code == Failure.PARAMETER_REFUSED:
            return BadRequestError(message, code=code.value)
        return UpstreamError(message)

    # --- Validation --------------------------------------------------------

    def _validate(
        self, values: dict[str, object], live_up: LiveProbe
    ) -> dict[str, Value]:
        new_values: dict[str, Value] = {}
        for field_id, raw in values.items():
            setting = self._schema.by_id.get(field_id)
            if setting is None:
                raise BadRequestError(f"Unknown setting: {field_id}", code="invalid_setting")
            try:
                value = coerce(setting, raw)
            except ValueError as exc:
                raise BadRequestError(str(exc), code="invalid_setting") from exc
            for name in setting.targets:
                new_values[name] = value

        overlay = self._overlay()
        factory = self._factory()
        live_cache: dict[str, Optional[Value]] = {}

        # New value, then the overlay (what the next start loads, and for a
        # live field what is in force), then the running node, then the
        # factory file. The node is only asked when a rule gets this far.
        def value_of(name: str) -> Optional[Value]:
            for source in (new_values, overlay):
                if name in source:
                    return source[name]
            if live_up():
                if name not in live_cache:
                    ok, _, read = self._gw.get([name])
                    live_cache[name] = read.get(name) if ok else None
                if live_cache[name] is not None:
                    return live_cache[name]
            return factory.get(name)

        for rule in self._schema.rules:
            if not set(rule.fields) & set(values):
                continue
            sentence = rule.check(value_of)
            if sentence:
                raise BadRequestError(sentence, code="invalid_setting")
        return new_values

    # --- Helpers -----------------------------------------------------------

    @contextmanager
    def _one_change_at_a_time(self) -> Iterator[None]:
        if not self._changing.acquire(blocking=False):
            raise ConflictError(
                f"The {self._subject} settings are already being changed; wait for "
                "that to finish.",
                code="settings_busy",
            )
        try:
            yield
        finally:
            self._changing.release()

    def _probe(self) -> LiveProbe:
        """A fresh probe for one request: the first call asks the node, the
        rest replay that answer."""
        return functools.cache(self._gw.available)

    def _live(
        self, names: list[str], live_up: LiveProbe
    ) -> Optional[dict[str, Optional[Value]]]:
        if not live_up():
            return None
        ok, message, values = self._gw.get(names)
        if not ok:
            self._logger.warning(
                "[NodeSettings] Could not read the node's parameters",
                message=message,
            )
            return None
        return values

    def _factory(self) -> dict[str, Value]:
        try:
            return self._repo.factory()
        except (OSError, ValueError) as exc:
            self._logger.error(
                "[NodeSettings] Factory params file unreadable",
                path=self._repo.factory_path,
                error=str(exc),
            )
            return {}

    def _overlay(self) -> dict[str, Value]:
        try:
            return self._repo.read()
        except (OSError, ValueError) as exc:
            # Refuse rather than treat it as empty: the next write would
            # replace the operator's whole overlay with this one change.
            raise ConflictError(
                f"The robot's saved settings file cannot be read ({exc}); fix or "
                f"delete {self._repo.overlay_path}.",
                code="params_unreadable",
            ) from exc


def init_node_settings_service(
    logger: structlog.stdlib.BoundLogger,
    schema: SettingsSchema,
    gateway: "ParameterGateway",
    overlay_repo: ParamOverlayRepo,
    subject: str,
) -> NodeSettingsService:
    return NodeSettingsService(
        logger=logger,
        schema=schema,
        gateway=gateway,
        overlay_repo=overlay_repo,
        subject=subject,
    )
