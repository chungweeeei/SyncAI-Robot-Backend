"""A node's runtime parameter overlay on disk, beside its factory params file.

The workspace launches controller_server and planner_server each on two
files, in order: the package's factory params (e.g.
``src/syncai_controller/params/controller_server_params.yaml``, tracked, mostly
comments on why each value is what it is, never written here) and the robot's
overlay, ``config/params/<robot_id>/<node>.yaml`` -- this process's to write,
gitignored there. A later file wins key by key, so
the overlay holds **only the keys the operator changed**: a key nobody touched
keeps following the factory file through a ``git pull``, resetting one setting
is deleting its key, and deleting the file is the fall back to the factory
values (the launch's ``use_runtime_params:=false`` is the other).

The file is rewritten whole, from what this process holds, so it carries no
hand-written comments worth keeping; it is a ROS 2 params file
(``/**/<node>: ros__parameters:``) because rcl reads it, not this process.
Written tmp + fsync + rename: ``config/`` is a directory bind mount, so the
rename lands on the host, and a reader -- the node's launch at the next boot
-- sees the old file or the new one, never half of one. A file rcl cannot
parse would take the node down on every boot (the launch skips one
it cannot parse, but a parsable file with a wrong type in it it cannot catch).

``.bak`` is the previous overlay, kept for the operator, never read here.
"""

import os
import tempfile
from typing import Optional, Union

import structlog
import yaml

# A parameter value as a params file holds it. Not the schema's type: a
# repository imports nothing from services/.
Value = Union[float, int, bool, str, list]

# Absolute, like every ~/robot_ws path here: entrypoints do not reliably run
# from the workspace root. config/ is mounted read-write into the backend's
# container; each factory file needs its package's params/ mounted
# (read-only) beside it -- without it that page has no defaults to show.
DEFAULT_PARAMS_DIR = os.path.expanduser("~/robot_ws/config/params")
DEFAULT_CONTROLLER_FACTORY = os.path.expanduser(
    "~/robot_ws/src/syncai_controller/params/controller_server_params.yaml"
)
DEFAULT_PLANNER_FACTORY = os.path.expanduser(
    "~/robot_ws/src/syncai_global_planner/params/planner_server_params.yaml"
)


def node_key(node: str) -> str:
    """The key the workspace's params files use for ``node``: wildcard
    namespace, so one file serves any robot_id."""
    return f"/**/{node}"


CONTROLLER_NODE_KEY = node_key("controller_server")
PLANNER_NODE_KEY = node_key("planner_server")

PARAMS_KEY = "ros__parameters"
BACKUP_SUFFIX = ".bak"

_HEADER = (
    "# Runtime parameter overlay, written by syncai_backend's settings page.\n"
    "# Loaded by the node's launch after its factory params file; only the\n"
    "# keys changed on the page are here. Delete this file (and restart the nav\n"
    "# session) to fall back to the factory params. Do not hand-edit while the\n"
    "# backend runs: it rewrites the whole file from what it holds.\n"
)


def flatten(tree: dict, prefix: str = "") -> dict[str, Value]:
    """``{"FollowPath": {"vx_max": 0.8}}`` -> ``{"FollowPath.vx_max": 0.8}``.

    ROS parameter names are dotted; a params file nests them.
    """
    flat: dict[str, Value] = {}
    for key, value in tree.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(flatten(value, prefix=f"{name}."))
        else:
            flat[name] = value
    return flat


def nest(flat: dict[str, Value]) -> dict:
    """The inverse of ``flatten``, keys sorted so a rewrite diffs cleanly."""
    tree: dict = {}
    for name in sorted(flat):
        node = tree
        *parents, leaf = name.split(".")
        for part in parents:
            node = node.setdefault(part, {})
        node[leaf] = flat[name]
    return tree


def _node_params(path: str, node_key: str) -> dict[str, Value]:
    """Every parameter a params file sets for ``node_key``, flattened.

    {} when the file does not exist. Raises ValueError for a file that exists
    but is not a params file.
    """
    try:
        with open(path, encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except FileNotFoundError:
        return {}
    except yaml.YAMLError as exc:
        raise ValueError(f"{path} is not valid YAML: {exc}") from exc

    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} is not a ROS params file (top level is not a mapping)")
    section = data.get(node_key) or {}
    params = section.get(PARAMS_KEY) if isinstance(section, dict) else None
    if params is None:
        return {}
    if not isinstance(params, dict):
        raise ValueError(f"{path}: {node_key}.{PARAMS_KEY} is not a mapping")
    return flatten(params)


class ParamOverlayRepo:
    """One node's overlay file and the factory file under it."""

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        overlay_path: str,
        factory_path: str,
        node_key: str,
    ):
        self.logger = logger
        self.overlay_path = overlay_path
        self.factory_path = factory_path
        self.node_key = node_key
        self.logger.info(
            "[ParamOverlayRepo] Serving a runtime params overlay",
            overlay=overlay_path,
            factory=factory_path,
        )

    def factory(self) -> dict[str, Value]:
        """The factory file's values, flattened; {} when it is not mounted."""
        if not os.path.isfile(self.factory_path):
            self.logger.warning(
                "[ParamOverlayRepo] Factory params file not found; defaults unknown",
                path=self.factory_path,
            )
            return {}
        return _node_params(self.factory_path, self.node_key)

    def read(self) -> dict[str, Value]:
        """The overlay's values, flattened; {} when there is none."""
        return _node_params(self.overlay_path, self.node_key)

    def write(self, values: dict[str, Value]) -> None:
        """Replace the overlay with exactly ``values`` (flattened names).

        An empty ``values`` deletes the file: no overlay is the factory state,
        and the node's log then says "factory params only" rather than
        "0 key(s)". The previous file is kept as ``.bak`` either way.
        """
        directory = os.path.dirname(self.overlay_path)
        os.makedirs(directory, exist_ok=True)

        previous: Optional[str] = None
        try:
            with open(self.overlay_path, encoding="utf-8") as handle:
                previous = handle.read()
        except FileNotFoundError:
            pass
        if previous is not None:
            self._write_atomic(self.overlay_path + BACKUP_SUFFIX, previous)

        if not values:
            if previous is not None:
                os.unlink(self.overlay_path)
            self.logger.info(
                "[ParamOverlayRepo] Overlay removed; factory params only",
                path=self.overlay_path,
            )
            return

        document = {self.node_key: {PARAMS_KEY: nest(values)}}
        content = _HEADER + yaml.safe_dump(
            document, default_flow_style=False, sort_keys=False
        )
        self._write_atomic(self.overlay_path, content)

        # Read back what rcl will read: a value that does not survive the
        # round trip (an int where a double was meant) would be refused by
        # the node at its next start, not here.
        written = self.read()
        if written != values:
            raise ValueError(
                f"{self.overlay_path} reads back differently from what was written"
            )
        self.logger.info(
            "[ParamOverlayRepo] Overlay written",
            path=self.overlay_path,
            keys=sorted(values),
        )

    @staticmethod
    def _write_atomic(path: str, content: str) -> None:
        handle = tempfile.NamedTemporaryFile(
            mode="w",
            dir=os.path.dirname(path),
            prefix=".params-",
            suffix=".tmp",
            delete=False,
            encoding="utf-8",
        )
        try:
            with handle:
                handle.write(content)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(handle.name, path)
        except BaseException:
            try:
                os.unlink(handle.name)
            except OSError:
                pass
            raise


def init_overlay_repo(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    node: str,
    factory_path: str,
    params_dir: str = DEFAULT_PARAMS_DIR,
) -> ParamOverlayRepo:
    # The same path the node's launch derives: <config>/params/<robot_id>/<node>.yaml.
    return ParamOverlayRepo(
        logger=logger,
        overlay_path=os.path.join(params_dir, robot_id, f"{node}.yaml"),
        factory_path=factory_path,
        node_key=node_key(node),
    )
