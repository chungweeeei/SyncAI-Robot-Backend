"""A live map switch: the localizer, map_server, Octo3D and the INI, in step.

POST /api/v1/maps/{name}/activate is the only caller. The switch is a small
saga -- four parts that must agree, each later failure compensating the
earlier ones -- and lives here rather than in the router so the steps and
their undos read as one list and can be driven without HTTP.

Two halves, because the router has to await Temporal between them (the
"is a job running" question belongs to the uvicorn loop):

- ``plan`` is every refusal that reads disk alone, in the order the operator
  should hear them. It mutates nothing.
- ``execute`` blocks for up to a minute on ROS services, so the router runs
  it in a worker thread. On the loop it froze every WebSocket, including the
  teleop watchdog.

Order and why:

1. The localizer (`relocalize`) first, because a refusal there mutates
   nothing.
2. map_server (`load_map`), when the map has a grid and map_server runs
   (a 3D session has none; a 2D map is refused without it).
3. Octo3D (`planner_server/Octo3D/load_map`) after the grid: a 3D map's tree
   takes seconds to build, and a refusal from map_server should not have paid
   for it. For a 2D map this *unloads* the previous tree, or the robot would
   keep planning on it ("3D wins").
4. `[map] name` in the INI last, preflighted for writability in ``plan``,
   since an unrecorded swap would leave REST naming the wrong map.

Compensations run in the order the steps were done, not reversed, and each
returns the sentence the operator reads; they are run as statements before
the message is built, so the order is the one done, not the one the
sentences would imply.
"""

import os
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import structlog

from syncai_backend.exceptions import ConflictError, UpstreamError
from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.helpers.system_config import system_ini_path
from syncai_backend.repositories.map.catalog import GridInfo, MapCatalogRepo, StoredMap
from syncai_backend.services.map_usage import MapUsagePolicy


@dataclass(frozen=True)
class SwitchPlan:
    """What ``plan`` judged; everything ``execute`` needs, read once."""

    name: str
    previous: Optional[str]
    is_3d: bool
    previous_is_3d: bool
    pcd_path: str
    # None for a 3D map without a grid: map_server is not touched.
    yaml_path: Optional[str]
    octomap_path: Optional[str]
    grid: Optional[GridInfo]
    ini_path: str


@dataclass(frozen=True)
class SwitchOutcome:
    name: str
    previous: Optional[str]
    map_kind: str
    localized: Optional[bool]
    keepout_reloaded: bool
    message: str


# A compensation: given why the switch failed, put one part back and return
# the operator's sentence about it ("" when there is nothing to say).
_Undo = Callable[[str], str]


class MapSwitchService:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        map_catalog_repo: MapCatalogRepo,
        map_gw: MapGateway,
        map_usage: MapUsagePolicy,
    ):
        self._logger = logger
        self._catalog = map_catalog_repo
        self._map_gw = map_gw
        self._map_usage = map_usage

    # --- Preflight ---------------------------------------------------------

    def plan(
        self,
        stored: StoredMap,
        grid_usable: bool,
        grid_error: Optional[str],
    ) -> Optional[SwitchPlan]:
        """Every refusal that needs no ROS call, or None for the no-op.

        ``grid_usable`` / ``grid_error`` are the router's reading of the
        conversion record (``_grid_status``), which it already owns for the
        catalogue listing.
        """
        name = stored.name
        previous = self._catalog.active_name()
        if name == previous:
            # The no-op: reloading would only throw away a good registration.
            return None

        self._map_usage.refuse_while_converting(
            name, "switch to it once the conversion has finished."
        )

        # 2D or 3D. The robot's own rule is "3D wins": its task_runner runs the
        # 3D trees whenever Octo3D has a map, so a map with both is a 3D map
        # here too (its grid is loaded as well, keeping map_server in step).
        octomap_path = self._catalog.octomap_path(name)
        is_3d = octomap_path is not None
        if not grid_usable and not is_3d:
            # Refuse up front, before the localizer is swapped (and an
            # unloadable yaml would crash map_server at the next launch).
            detail = f" ({grid_error})" if grid_error else ""
            raise ConflictError(
                f"'{name}' has no usable gridmap{detail} and no 3D map "
                "(octomap.bt). Build a grid from the map's Rebuild-grid control, "
                "or let the robot's map cleaning write its 3D map, before "
                "switching to it.",
                code="grid_missing",
            )

        pcd_path = self._catalog.pointcloud_path(name)
        if pcd_path is None:
            raise ConflictError(
                f"'{name}' has no map.pcd, so the localizer has nothing to "
                "match against. Only a map saved from a mapping run can be "
                "switched to.",
                code="pointcloud_missing",
            )

        yaml_path = self._catalog.gridmap_yaml_path(name) if grid_usable else None
        if yaml_path is None and not is_3d:
            raise ConflictError(
                f"'{name}' has no gridmap.yaml to hand map_server.",
                code="grid_missing",
            )
        previous_is_3d = (
            previous is not None and self._catalog.octomap_path(previous) is not None
        )

        ini_path = system_ini_path()
        if not os.access(ini_path, os.W_OK):
            # Preflighted so the swap's last step cannot fail after the move.
            raise ConflictError(
                f"The system INI ({ini_path}) is not writable, so a switch "
                "could not be recorded and would not survive a restart.",
                code="ini_not_writable",
            )

        return SwitchPlan(
            name=name,
            previous=previous,
            is_3d=is_3d,
            previous_is_3d=previous_is_3d,
            pcd_path=pcd_path,
            yaml_path=yaml_path,
            octomap_path=octomap_path,
            grid=stored.grid,
            ini_path=ini_path,
        )

    # --- The switch --------------------------------------------------------

    def execute(self, plan: SwitchPlan) -> SwitchOutcome:
        """Run the switch; blocking. Raises ConflictError before anything
        moves, UpstreamError (with what was put back) after."""
        name, previous = plan.name, plan.previous
        map_gw = self._map_gw

        if not map_gw.localizer_ready():
            raise ConflictError(
                "The localizer is not reachable, so there is no running map to "
                "switch. The robot is in mapping mode, or the nav stack is down.",
                code="stack_not_ready",
            )

        # A 3D nav session runs without map_server, so it is required for a
        # 2D map only; a 3D map's grid is loaded when map_server is there,
        # to keep it in step, and skipped when it is not.
        map_server = map_gw.map_server_ready()
        if not plan.is_3d and not map_server:
            raise ConflictError(
                f"'{name}' is a 2D map, but map_server is not reachable to load "
                "its grid. The nav stack is a 3D session, or is down.",
                code="stack_not_ready",
            )

        # Asked before anything moves. Absent on a stack built without
        # OctomapPlanner3D: fatal for a 3D map, nothing to unload for a 2D.
        planner_3d = map_gw.planner_3d_available()
        if plan.is_3d and not planner_3d:
            raise ConflictError(
                f"'{name}' is a 3D map, but the nav stack has no 3D planner "
                "(planner_server/Octo3D/load_map) to navigate it.",
                code="planner_3d_unavailable",
            )

        # What has been done, as the undo for each step, in the order done.
        done: List[_Undo] = []

        swapped, detail = map_gw.swap_localizer_map(plan.pcd_path, 0.0, 0.0, 0.0)
        if not swapped:
            # Nothing has changed yet, so nothing to compensate.
            self._logger.error("Map switch failed at the localizer", map=name, error=detail)
            raise UpstreamError(
                f"Could not point the localizer at '{name}': {detail}. The robot "
                f"is still on '{previous}'."
            )
        done.append(self._localizer_undo(plan))

        if plan.yaml_path is not None and map_server:
            reloaded, detail = map_gw.reload_map(plan.yaml_path)
            if not reloaded:
                self._logger.error("Map switch failed at map_server", map=name, error=detail)
                raise UpstreamError(
                    f"The localizer moved to '{name}' but map_server would not load "
                    f"its gridmap: {detail}." + _undo(done, "load_map failed")
                )
            done.append(self._grid_undo(plan))

        if planner_3d:
            # Registered before the call: Octo3D may have dropped the previous
            # tree on its way to failing.
            done.append(self._octomap_undo(plan))
            octomap_loaded, detail = map_gw.reload_octomap(
                plan.octomap_path if plan.is_3d else ""
            )
            if not octomap_loaded:
                self._logger.error(
                    "Map switch failed at the 3D planner", map=name, error=detail
                )
                what = (
                    "load its 3D map" if plan.is_3d else "unload the previous 3D map"
                )
                raise UpstreamError(
                    f"The localizer moved to '{name}' but the 3D planner would not "
                    f"{what}: {detail}." + _undo(done, "Octo3D load_map failed")
                )

        try:
            # The INI and the catalogue's cache of it, in that order.
            self._catalog.set_active(name)
        except (OSError, ValueError) as exc:
            self._logger.error(
                "Map switch failed at the INI write", map=name, error=str(exc)
            )
            raise UpstreamError(
                f"The robot moved to '{name}' but the switch could not be "
                f"recorded in {plan.ini_path}: {exc}." + _undo(done, "INI write failed")
            )

        # The switch is recorded; nothing can fail it now. The keepout mask
        # (still the previous map's) is carried over best-effort after this
        # commit point: a mask problem goes in the message, not a failure.
        # A 3D map's zones need no carrying: the 3D planner read the map's
        # keepout.json (if any) as part of the load_map that just succeeded
        # (2026-10-23; until then it read no zones at all).
        if plan.is_3d:
            count = len(self._catalog.read_keepout_zones(name))
            keepout_reloaded, keepout_note = True, (
                " It is a 3D map: the 3D planner navigates it, with its "
                f"{count} forbidden zone{'s' if count != 1 else ''}."
            )
        else:
            keepout_reloaded, keepout_note = self._carry_keepout(name, plan.grid)

        localized = map_gw.localization_converged()
        map_kind = "3d" if plan.is_3d else "2d"

        self._logger.info(
            "Switched the active map",
            map=name,
            previous=previous,
            map_kind=map_kind,
            previous_was_3d=plan.previous_is_3d,
            localized=localized,
            keepout_reloaded=keepout_reloaded,
        )

        message = f"The robot is now on '{name}'"
        message += f" (was '{previous}')." if previous else "."
        if localized:
            message += " The localizer converged on the new map."
        else:
            message += (
                " The pose was reset to the map origin; set an initial pose on "
                "the dashboard, because the localizer has not converged"
            )
            message += " yet." if localized is False else " and could not be asked."
        message += keepout_note
        return SwitchOutcome(
            name=name,
            previous=previous,
            map_kind=map_kind,
            localized=localized,
            keepout_reloaded=keepout_reloaded,
            message=message,
        )

    # --- Compensations -----------------------------------------------------

    def _localizer_undo(self, plan: SwitchPlan) -> _Undo:
        name, previous = plan.name, plan.previous
        previous_pcd = self._catalog.pointcloud_path(previous) if previous else None

        def _restore(reason: str) -> str:
            """Put the localizer back on the old cloud."""
            if previous_pcd is None:
                # The INI named no map (the likely case when its write fails),
                # or that map lost its map.pcd.
                where = (
                    f"'{previous}' has no map.pcd"
                    if previous
                    else "the INI named no previous map"
                )
                return (
                    f" The localizer is now on '{name}' and could not be put "
                    f"back — {where}."
                )
            restored, detail = self._map_gw.swap_localizer_map(previous_pcd, 0.0, 0.0, 0.0)
            if restored:
                return f" The localizer was put back on '{previous}'."
            self._logger.error(
                "Could not restore the localizer after a failed map switch",
                map=name,
                previous=previous,
                reason=reason,
                error=detail,
            )
            return (
                f" The localizer is still on '{name}' and could not be put back "
                f"({detail}) — set the map by hand and restart the stack."
            )

        return _restore

    def _grid_undo(self, plan: SwitchPlan) -> _Undo:
        name, previous = plan.name, plan.previous

        def _restore(reason: str) -> str:
            """Put map_server back on the old grid."""
            previous_yaml = (
                self._catalog.gridmap_yaml_path(previous) if previous else None
            )
            if previous_yaml is None:
                # Say so rather than leave map_server quietly serving a grid
                # that does not match the restored cloud.
                where = (
                    f"'{previous}' has no gridmap.yaml"
                    if previous
                    else "the INI named no previous map"
                )
                return f" map_server is still serving '{name}' — {where} to put back."
            back, back_detail = self._map_gw.reload_map(previous_yaml)
            return "" if back else f" map_server is still serving '{name}' ({back_detail})."

        return _restore

    def _octomap_undo(self, plan: SwitchPlan) -> _Undo:
        name, previous = plan.name, plan.previous

        def _restore(reason: str) -> str:
            """Put Octo3D back as the old map had it: its tree, or none."""
            previous_bt = self._catalog.octomap_path(previous) if previous else None
            back, back_detail = self._map_gw.reload_octomap(previous_bt or "")
            if back:
                return ""
            self._logger.error(
                "Could not restore the 3D planner after a failed map switch",
                map=name,
                previous=previous,
                reason=reason,
                error=back_detail,
            )
            return (
                f" The 3D planner could not be put back on '{previous}' "
                f"({back_detail}) — restart the nav stack before navigating."
            )

        return _restore

    # --- After the commit point --------------------------------------------

    def _carry_keepout(self, name: str, grid: Optional[GridInfo]) -> Tuple[bool, str]:
        """Point filter_mask_server at ``name``'s keepout mask; never raises.

        A map the nav session never booted into has no ``keepout.*``, so the
        pair is written first (from ``keepout.json`` if any, else blank) —
        the previous map's zones must be replaced by something.

        Returns ``(reloaded, note)``; the note is an operator sentence, empty
        when there is nothing to say.
        """
        zones = self._catalog.read_keepout_zones(name)
        yaml_path = self._catalog.keepout_yaml_path(name)
        if yaml_path is None:
            try:
                self._catalog.write_keepout(name, zones, grid)
            except OSError as exc:
                self._logger.error(
                    "Could not write the keepout mask for the new map",
                    map=name,
                    error=str(exc),
                )
                return False, f" The keepout mask could not be written: {exc}."
            yaml_path = self._catalog.keepout_yaml_path(name)
            if yaml_path is None:
                return False, " The keepout mask could not be written."

        reloaded, detail = self._map_gw.reload_keepout(yaml_path)
        if not reloaded:
            self._logger.error(
                "Switched maps but the keepout mask did not reload", map=name, error=detail
            )
            return False, f" The keepout mask could not be reloaded: {detail}."
        if zones:
            count = len(zones)
            return True, f" Its {count} forbidden zone{'s' if count != 1 else ''} are active."
        return True, ""


def _undo(done: List[_Undo], reason: str) -> str:
    """Run every compensation in the order its step was done; their sentences."""
    return "".join([undo(reason) for undo in done])


def init_map_switch_service(
    logger: structlog.stdlib.BoundLogger,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
    map_usage: MapUsagePolicy,
) -> MapSwitchService:
    return MapSwitchService(
        logger=logger,
        map_catalog_repo=map_catalog_repo,
        map_gw=map_gw,
        map_usage=map_usage,
    )
