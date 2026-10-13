"""Read and write the per-robot system INI.

``config/system.ini`` carries ``[system] robot_id`` (read by every launch file)
and ``[map]`` (which map the stack loads); docker-compose bind-mounts
``config/instances/robotNN.ini`` over it.

This is the only INI reader and `set_active_map` the only writer; keep it that
way, since reader and writer must agree about the interpolation in ``[map]``.
"""

import configparser
import os
import re
import threading
from typing import Optional

import structlog

# Absolute: entrypoints do not reliably run from the workspace root.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")

SYSTEM_INI_ENV = "SYNCAI_SYSTEM_INI"

# Serialises INI reads and writes in this process. `set_active_map` rewrites in
# place, and a read of the half-written file answers None, which the rename and
# delete routes take as "not active" -- letting rmtree through on the loaded
# map. RLock so the writer's verifying re-read does not deadlock. Process-local.
_INI_LOCK = threading.RLock()


def system_ini_path() -> str:
    return os.environ.get(SYSTEM_INI_ENV, DEFAULT_SYSTEM_INI)


def active_map_name(logger: structlog.stdlib.BoundLogger) -> Optional[str]:
    """Return the bare name of the map the stack was launched with, or None.

    From the INI, not ``RobotState.map``: the cached state is only kept once
    localization is valid, so a freshly booted robot would report no active map.
    Falls back to the parent directory of ``[map] map``
    (``map/dp2f/gridmap.yaml`` -> ``dp2f``) for older files without ``name``.
    Never raises, so a robot without a usable INI can still list its maps.
    """
    path = system_ini_path()
    config = configparser.ConfigParser()

    try:
        with _INI_LOCK:
            found = config.read(path)
        if not found:
            logger.warning("[SystemConfig] System INI not found", path=path)
            return None
    except configparser.Error as exc:
        logger.warning("[SystemConfig] System INI is malformed", path=path,
                       error=str(exc))
        return None

    name = config.get("map", "name", fallback="").strip()
    if name:
        return name

    yaml_path = config.get("map", "map", fallback="").strip()
    if yaml_path:
        return os.path.basename(os.path.dirname(yaml_path)) or None

    logger.warning("[SystemConfig] No [map] name in system INI", path=path)
    return None


# The keys `set_active_map` rewrites. `[initial_pose]` is zeroed so a pose from
# the old map's frame is not seeded into the new one; localizer.launch.py reads
# each key with fallback=0.0, so a missing key needs no write. z is unused by it.
_MAP_SECTION = "map"
_MAP_NAME_KEY = "name"
_INITIAL_POSE_SECTION = "initial_pose"
_INITIAL_POSE_RESET = {"x": "0.0", "y": "0.0", "z": "0.0", "yaw": "0.0"}

_SECTION_RE = re.compile(r"^\s*\[(?P<name>[^]]+)\]\s*$")

# indent / key / separator / value / trailing, so only the value changes.
_KEY_RE = re.compile(
    r"^(?P<indent>\s*)(?P<key>[^\s:=#;][^:=]*?)(?P<sep>\s*[:=]\s*)"
    r"(?P<value>.*?)(?P<trail>\s*)$"
)


def set_active_map(name: str, logger: structlog.stdlib.BoundLogger) -> str:
    """Point the INI at ``name`` and zero ``[initial_pose]``. Returns the path.

    Makes ``POST /api/v1/maps/{name}/activate`` survive a stack restart.

    **Rewritten in place, never temp file + rename.** The INI is a single-file
    bind mount; a rename gives it a new inode the container never sees. A torn
    write is caught by the verifying re-read.

    **Edited line by line, not via ConfigParser.write().** ``[map]`` uses
    interpolation (``pcd: map/%(name)s/map.pcd``); a write-back would bake in
    the old name. Line edits also keep comments, key order and delimiters.

    Raises OSError on I/O failure, ValueError if there is no ``[map] name``.
    Holds ``_INI_LOCK`` from the first read to the re-read.
    """
    with _INI_LOCK:
        return _set_active_map_locked(name, logger)


def _set_active_map_locked(name: str, logger: structlog.stdlib.BoundLogger) -> str:
    path = system_ini_path()

    with open(path, "r", encoding="utf-8") as handle:
        lines = handle.readlines()

    section = ""
    seen_name = False
    out: list[str] = []

    for line in lines:
        match = _SECTION_RE.match(line)
        if match:
            section = match.group("name").strip().lower()
            out.append(line)
            continue

        replacement = None
        if section == _MAP_SECTION:
            replacement = {_MAP_NAME_KEY: name}
        elif section == _INITIAL_POSE_SECTION:
            replacement = _INITIAL_POSE_RESET

        if replacement is None:
            out.append(line)
            continue

        key_match = _KEY_RE.match(line.rstrip("\n"))
        if key_match is None:
            # Blank line, comment, or a continuation of a multi-line value.
            out.append(line)
            continue

        key = key_match.group("key").strip().lower()
        if key not in replacement:
            out.append(line)
            continue

        if section == _MAP_SECTION:
            seen_name = True

        newline = "\n" if line.endswith("\n") else ""
        out.append(
            key_match.group("indent")
            + key_match.group("key")
            + key_match.group("sep")
            + replacement[key]
            + key_match.group("trail")
            + newline
        )

    if not seen_name:
        raise ValueError(f"{path} has no [{_MAP_SECTION}] {_MAP_NAME_KEY} to rewrite")

    with open(path, "w", encoding="utf-8") as handle:
        handle.writelines(out)

    # Verify before reporting success; interpolation off so the raw `%(name)s`
    # values are read as written.
    check = configparser.ConfigParser(interpolation=None)
    try:
        if not check.read(path):
            raise ValueError(f"{path} disappeared while being rewritten")
    except configparser.Error as exc:
        raise ValueError(f"{path} is malformed after the rewrite: {exc}") from exc

    written = check.get(_MAP_SECTION, _MAP_NAME_KEY, fallback="").strip()
    if written != name:
        raise ValueError(
            f"{path} still reads [{_MAP_SECTION}] {_MAP_NAME_KEY} = {written!r} "
            f"after being rewritten to {name!r}"
        )

    logger.info("[SystemConfig] Active map written to the system INI",
                path=path, map=name)
    return path
