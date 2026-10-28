import json
import math
import os
import re
import shutil
import tempfile
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from typing import Any, BinaryIO, Callable, List, Optional, Tuple

import structlog

from syncai_backend.exceptions import BadRequestError, ConflictError, NotFoundError
from syncai_backend.helpers.disk import walk_stats
from syncai_backend.helpers.floors import read_pose_heights
from syncai_backend.helpers.pgm import write_text_atomic
from syncai_backend.helpers.system_config import active_map_name, set_active_map


# Deliberately strict: the only thing between a URL path segment (or a rename
# body) and the filesystem.
_NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

# Sidecar of the robot's post-save map cleaning (pgo_node writes ``converting``,
# its ``clean_map`` turns it ``ok`` / ``failed``, swaps in a cleaned map.pcd and
# builds octomap.bt). Read for the 3D map's status, see
# ``_read_map_clean_record``.
MAP_CLEAN_SIDECAR = "map_clean.recipe.json"
# A sidecar saying ``converting`` this long after its ``started_at`` belongs to
# a clean that died hard (the longest measured is ~5 min; cleans queue per host
# behind a flock, and the tree is built after map.pcd is rewritten). Before
# that, the clean is taken to be running: it is writing into the directory.
MAP_CLEAN_STALE_AFTER_S = 30 * 60.0

POINTCLOUD_PCD = "map.pcd"

# A map saved before 2026-10-28 may still carry a 2D gridmap (``gridmap.*``,
# ``gridmap_prev.*``, ``gridmap.recipe.json``, ``traversable_debug/``) and a
# keepout mask (``keepout.pgm`` + ``keepout.yaml``). Nothing reads them any
# more: the stack navigates on 3D maps only. They are left on disk, counted in
# a map's size and carried by its export like any other file.

# Forbidden zones: polygons in map-frame metres, which the robot's 3D planner
# (Octo3D) reads beside ``octomap.bt`` itself (2026-10-23). Each zone may carry
# ``name`` and a ``z_min`` / ``z_max`` band naming the floor it forbids;
# without a band it covers every floor. Rewrites must keep the three keys -- a
# save that dropped them would silently widen a zone to every storey.
KEEPOUT_JSON = "keepout.json"
KEEPOUT_JSON_VERSION = 1

# The robot's 3D map: the occupancy tree ``clean_map`` writes beside map.pcd
# (``map_clean_octomap``), which the 3D planner plans on. Served to the
# console byte for byte; only its text header is read here, for ``res``.
# Whether a save got one is in MAP_CLEAN_SIDECAR's ``measurements``
# (``octomap_written`` / ``octomap_error``), see ``_read_map_clean_record``.
OCTOMAP_BT = "octomap.bt"
# The tree as the mapping wrote it, set aside by the robot's
# ``apply_octomap_edits`` the first time it patches ``octomap.bt`` from the
# edits below. Named for the layout; nothing in this process touches it.
OCTOMAP_RAW_BT = "octomap.raw.bt"
# Hand-patched ground from the console's editor: the tree columns to mark as
# floor and the height each gets, as absolute OcTree keys. Source of truth the
# way keepout.json is, and applied to the tree on the robot, never here. The
# document keeps the shape the standalone viewer exported, so one tool reads a
# console save and a hand export alike.
OCTOMAP_EDITS_JSON = "octomap_edits.json"
OCTOMAP_EDITS_FORMAT = "syncai-octomap-edits"
OCTOMAP_EDITS_VERSION = 1
# An OcTree key is 16 bits with the map origin at its midpoint: a voxel's
# centre is ``(key - OCTOMAP_KEY_ORIGIN + 0.5) * res``.
OCTOMAP_KEY_ORIGIN = 32768
OCTOMAP_KEY_MAX = 65535

# pgo's keyframe trajectory, ``<N>.pcd x y z qw qx qy qz`` per line, written
# by every save. Read here for the heights the robot walked at, which is what
# the floors are detected from (helpers/floors.py).
POSES_TXT = "poses.txt"
# The floors an operator named for a map that spans storeys: a name and the
# body height the robot stands at on it, in the map frame. Only the console
# reads it, to show one storey of the 3D map at a time; absent means the
# floors are the ones detected from poses.txt.
FLOORS_JSON = "floors.json"
FLOORS_JSON_FORMAT = "syncai-map-floors"
FLOORS_JSON_VERSION = 1

# Map import staging: ``map/.import-<token>/`` (unpacked + verified) and
# ``map/.import-old-<token>/`` (the replaced map, until the DB agrees). Inside
# ``maps_dir`` because it is a bind mount: a sibling is another filesystem and
# the one-``os.rename`` publish would become a copy. ``list_maps`` skips the
# prefix and ``resolve_dir`` refuses it explicitly (``_NAME_RE`` admits a dot).
IMPORT_STAGING_PREFIX = ".import-"


# What _read_json answers for a file that is not there or cannot be read. A
# sentinel rather than None, because a file holding JSON ``null`` is a document
# (malformed, and each reader's shape check says so).
_NO_DOCUMENT = object()


def _read_json(path: str, complain: Callable[[str], None]) -> Any:
    """Parse the JSON file at ``path``, or ``_NO_DOCUMENT``.

    Absent is quiet: most maps have no keepout, floors or edits file. Unreadable
    or unparsable (a torn write, a hand edit, a newer format) calls
    ``complain`` with the error and answers ``_NO_DOCUMENT`` too: every reader
    here degrades one map rather than failing a listing or locking the operator
    out of the save that would overwrite it. How loudly is the caller's call.
    """
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        return _NO_DOCUMENT
    except (OSError, ValueError) as exc:
        complain(str(exc))
        return _NO_DOCUMENT


class MapCleanStatus(str, Enum):
    """The ``status`` values the cleaning sidecar on disk can carry.

    Only what a clean writes; ``interrupted`` is derived by the REST layer
    (``converting`` for too long).

    Not ``OctomapStatus`` in ``interfaces/rest/routers/maps/schemas.py``, the
    wire contract: disk and wire vocabularies may drift apart, and only that
    module's ``_octomap_status`` converts between them.

    ``str`` mixin (``enum.StrEnum`` is 3.11+) so members ``json.dumps`` bare.
    """

    CONVERTING = "converting"
    OK = "ok"
    FAILED = "failed"

    @classmethod
    def parse(cls, value: object) -> Optional["MapCleanStatus"]:
        """The member ``value`` names, or None if it names nothing this build knows.

        Tolerant on purpose: an unknown value (torn read, newer backend after a
        rollback) must degrade one card, not fail the listing. None is treated
        like a sidecar with no status.
        """
        if not isinstance(value, str):
            return None
        try:
            return cls(value)
        except ValueError:
            return None


@dataclass(frozen=True)
class KeepoutZone:
    """One forbidden zone: a closed polygon in map-frame metres.

    ``points`` are the corners in drawing order, not closed (the console's
    ``ZonePolygon``). The id is the console's handle. ``name`` is for
    display. ``z_min`` / ``z_max`` bound the floor heights the zone applies to
    (the robot's 3D planner compares them with the ground's height); None is
    unbounded on that side, so a zone without them covers every floor.
    """

    id: str
    points: Tuple[Tuple[float, float], ...]
    name: Optional[str] = None
    z_min: Optional[float] = None
    z_max: Optional[float] = None


@dataclass(frozen=True)
class OctomapInfo:
    """``octomap.bt`` as it stands on disk: what versioning and serving it need.

    No parse of the tree: ``resolution`` comes from the text header alone and
    is None when that header is not one this build reads.
    """

    size_bytes: int
    mtime_ns: int
    resolution: Optional[float]

    @property
    def version(self) -> str:
        """An opaque tag for the file's current bytes.

        Size and mtime, not a content hash: the tree is only ever replaced
        wholesale (tmp + rename, by the cleaning or the robot's edit tool),
        never edited in place. The console keys its
        cache on it and the GET route serves it as the ETag.
        """
        return f"{self.size_bytes}-{self.mtime_ns}"


@dataclass(frozen=True)
class MapCleanRecord:
    """What the cleaning sidecar says, read for the tree's sake.

    ``status`` is about map.pcd; the ``octomap_*`` fields are the sidecar's
    ``params.octomap`` and ``measurements.octomap_written`` /
    ``octomap_error``, which is where a tree that was asked for and not
    written is reported. ``error`` is the clean's own, on ``failed``.
    """

    status: MapCleanStatus
    started_at: Optional[str]
    octomap_requested: bool
    octomap_written: bool
    octomap_error: Optional[str]
    error: Optional[str]

    def running(self, now: Optional[datetime] = None) -> bool:
        """Is the clean still at work on the map directory?

        ``converting`` until ``started_at`` is ``MAP_CLEAN_STALE_AFTER_S``
        ago; past that it is a clean killed hard, which never writes its
        outcome. No start time to age it by counts as running.
        """
        if self.status is not MapCleanStatus.CONVERTING:
            return False
        started = parse_iso_utc(self.started_at)
        if started is None:
            return True
        current = now if now is not None else datetime.now(timezone.utc)
        return (current - started).total_seconds() <= MAP_CLEAN_STALE_AFTER_S


def parse_iso_utc(value: object) -> Optional[datetime]:
    """``2026-10-12T05:47:05Z`` -> an aware datetime; None for anything else."""
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed if parsed.tzinfo is not None else parsed.replace(tzinfo=timezone.utc)


@dataclass(frozen=True)
class MapFloor:
    """One storey: what it is called and the body height walked on it, metres."""

    name: str
    level: float


@dataclass(frozen=True)
class OctomapEdits:
    """The hand-patched ground recorded in ``octomap_edits.json``."""

    res: float
    fill_key: Tuple[Tuple[int, int, int], ...]
    created_at: Optional[str]


@dataclass(frozen=True)
class StoredMap:
    name: str
    has_pointcloud: bool
    size_bytes: int
    modified_at: datetime
    # The tree on disk, or None without one, and what the cleaning sidecar
    # says about it (see the REST layer's _octomap_status).
    octomap: Optional[OctomapInfo] = None
    map_clean_record: Optional[MapCleanRecord] = None


def _octomap_edits_text(document: dict) -> str:
    """``document`` indented, but each ``[x, y, z]`` triple on one line.

    Plain ``indent=2`` spends five lines on every triple, and a map's worth of
    patches is tens of thousands of them; this keeps the file readable at one
    line per cell.
    """
    lines = ["{"]
    items = list(document.items())
    for index, (key, value) in enumerate(items):
        comma = "," if index < len(items) - 1 else ""
        if isinstance(value, list) and value:
            rows = ",\n".join(f"    {json.dumps(row)}" for row in value)
            lines.append(f"  {json.dumps(key)}: [\n{rows}\n  ]{comma}")
        else:
            lines.append(f"  {json.dumps(key)}: {json.dumps(value)}{comma}")
    lines.append("}")
    return "\n".join(lines) + "\n"


class MapCatalogRepo:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger
        self.maps_dir = os.path.expanduser("~/robot_ws/map")
        # Logged: the path is not configurable, so an unexpected HOME is
        # otherwise invisible.
        self.logger.info("[MapCatalogRepo] Serving maps", path=self.maps_dir)

        # The loaded map, read from the INI once here (at startup, off any
        # event loop) and then kept by set_active; see active_name.
        self._active_lock = threading.Lock()
        self._active: Optional[str] = None
        self.load_active()
        self.logger.info("[MapCatalogRepo] Active map", map=self._active)

    # --- Paths --------------------------------------------------------------

    def resolve_dir(self, name: str) -> str:
        """Return the absolute path of a map directory, or raise BadRequestError.

        Both checks are needed: the pattern rejects separators and ``..``, the
        realpath check catches a symlink pointing out of ``maps_dir``.
        """
        if (
            not _NAME_RE.match(name)
            or name in (".", "..")
            or name.startswith(IMPORT_STAGING_PREFIX)
        ):
            raise BadRequestError(f"Invalid map name: {name!r}")

        candidate = os.path.realpath(os.path.join(self.maps_dir, name))
        root = os.path.realpath(self.maps_dir)
        if os.path.dirname(candidate) != root:
            raise BadRequestError(f"Invalid map name: {name!r}")

        return candidate

    def _artifact_path(self, name: str, filename: str) -> Optional[str]:
        """Path of ``filename`` inside map ``name``, or None if it is not there.

        Always through ``resolve_dir``, the path-traversal check; no caller
        joins a name onto ``maps_dir`` itself.
        """
        path = os.path.join(self.resolve_dir(name), filename)
        return path if os.path.isfile(path) else None

    def pointcloud_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``map.pcd``, or None if absent.

        The file ``has_pointcloud`` reports on.
        """
        return self._artifact_path(name, POINTCLOUD_PCD)

    def octomap_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``octomap.bt``, or None if absent."""
        return self._artifact_path(name, OCTOMAP_BT)

    def keepout_json_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``keepout.json``, or None if absent.

        What the robot's 3D planner is pointed at (``load_keepout``); absolute
        and ``~``-free, as the planner needs.
        """
        return self._artifact_path(name, KEEPOUT_JSON)

    def read_keepout_zones(self, name: str) -> List[KeepoutZone]:
        """Return the forbidden zones recorded in ``keepout.json``, or ``[]``.

        ``[]`` also for a json this build cannot parse (logged): an error here
        would lock the operator out of the save that overwrites it. Malformed
        entries are skipped one at a time.
        """
        path = os.path.join(self.resolve_dir(name), KEEPOUT_JSON)
        document = _read_json(
            path,
            lambda error: self.logger.warning(
                "[MapCatalogRepo] Unreadable keepout.json; reporting no zones",
                map=name,
                error=error,
            ),
        )
        if document is _NO_DOCUMENT:
            return []

        if (
            not isinstance(document, dict)
            or document.get("version") != KEEPOUT_JSON_VERSION
            or not isinstance(document.get("zones"), list)
        ):
            self.logger.warning(
                "[MapCatalogRepo] keepout.json has an unexpected shape; reporting no zones",
                map=name,
            )
            return []

        zones: List[KeepoutZone] = []
        for entry in document["zones"]:
            zone = _parse_keepout_zone(entry)
            if zone is None:
                self.logger.warning(
                    "[MapCatalogRepo] Skipping a malformed zone in keepout.json", map=name
                )
                continue
            zones.append(zone)
        return zones

    def read_octomap_edits(self, name: str) -> Optional[OctomapEdits]:
        """Return the ground patches recorded in ``octomap_edits.json``, or None.

        None for no file and (logged) for one this build cannot read, for
        ``read_keepout_zones``'s reason: an error here would lock the operator
        out of the save that overwrites it. Malformed keys are skipped one at
        a time.
        """
        path = os.path.join(self.resolve_dir(name), OCTOMAP_EDITS_JSON)
        document = _read_json(
            path,
            lambda error: self.logger.warning(
                "[MapCatalogRepo] Unreadable octomap_edits.json; reporting no patches",
                map=name,
                error=error,
            ),
        )
        if document is _NO_DOCUMENT:
            return None

        if (
            not isinstance(document, dict)
            or document.get("format") != OCTOMAP_EDITS_FORMAT
            or document.get("version") != OCTOMAP_EDITS_VERSION
            or isinstance(document.get("res"), bool)
            or not isinstance(document.get("res"), (int, float))
            or not isinstance(document.get("fill_key"), list)
        ):
            self.logger.warning(
                "[MapCatalogRepo] octomap_edits.json has an unexpected shape; "
                "reporting no patches",
                map=name,
            )
            return None

        keys: List[Tuple[int, int, int]] = []
        for entry in document["fill_key"]:
            key = _parse_octomap_key(entry)
            if key is None:
                self.logger.warning(
                    "[MapCatalogRepo] Skipping a malformed key in octomap_edits.json",
                    map=name,
                )
                continue
            keys.append(key)
        created_at = document.get("created_at")
        return OctomapEdits(
            res=float(document["res"]),
            fill_key=tuple(keys),
            created_at=created_at if isinstance(created_at, str) else None,
        )

    def write_octomap_edits(
        self, name: str, fill_key: List[Tuple[int, int, int]], res: float
    ) -> OctomapEdits:
        """Replace the map's ground patches with ``fill_key``; ``[]`` clears.

        One file through a temp file + rename. The document is the standalone
        viewer's export, field for field (``fill_xyz`` included: derivable,
        but what a reader without the key arithmetic looks at), so the robot's
        ``apply_octomap_edits`` reads a console save and a hand export alike.
        Nothing written names the map or holds a path, so the directory stays
        renamable; the keys are the tree's own and hold for exactly this
        save's map frame.
        """
        directory = self.resolve_dir(name)
        created_at = (
            datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
        )
        keys = [(int(key[0]), int(key[1]), int(key[2])) for key in fill_key]
        document = {
            "format": OCTOMAP_EDITS_FORMAT,
            "version": OCTOMAP_EDITS_VERSION,
            "res": res,
            "source": OCTOMAP_BT,
            "created_at": created_at,
            "fill_key": [list(key) for key in keys],
            "fill_xyz": [
                [round((value - OCTOMAP_KEY_ORIGIN + 0.5) * res, 3) for value in key]
                for key in keys
            ],
        }
        write_text_atomic(
            os.path.join(directory, OCTOMAP_EDITS_JSON), _octomap_edits_text(document)
        )
        self.logger.info("[MapCatalogRepo] Wrote ground patches", map=name, cells=len(keys))
        return OctomapEdits(res=res, fill_key=tuple(keys), created_at=created_at)

    def read_pose_heights(self, name: str) -> Optional[List[float]]:
        """The z of every keyframe in the map's ``poses.txt``, or None without one."""
        path = os.path.join(self.resolve_dir(name), POSES_TXT)
        try:
            return read_pose_heights(path)
        except FileNotFoundError:
            return None
        except (OSError, UnicodeDecodeError) as exc:
            self.logger.warning(
                "[MapCatalogRepo] Unreadable poses.txt; detecting no floors",
                map=name,
                error=str(exc),
            )
            return None

    def read_floors(self, name: str) -> Optional[List[MapFloor]]:
        """The floors saved in ``floors.json``, lowest first, or None.

        None for no file and (logged) for one this build cannot read, for
        ``read_keepout_zones``'s reason. Malformed entries are skipped.
        """
        path = os.path.join(self.resolve_dir(name), FLOORS_JSON)
        document = _read_json(
            path,
            lambda error: self.logger.warning(
                "[MapCatalogRepo] Unreadable floors.json; using the detected floors",
                map=name,
                error=error,
            ),
        )
        if document is _NO_DOCUMENT:
            return None

        if (
            not isinstance(document, dict)
            or document.get("format") != FLOORS_JSON_FORMAT
            or document.get("version") != FLOORS_JSON_VERSION
            or not isinstance(document.get("floors"), list)
        ):
            self.logger.warning(
                "[MapCatalogRepo] floors.json has an unexpected shape; "
                "using the detected floors",
                map=name,
            )
            return None

        floors: List[MapFloor] = []
        for entry in document["floors"]:
            floor = _parse_floor(entry)
            if floor is None:
                self.logger.warning(
                    "[MapCatalogRepo] Skipping a malformed floor in floors.json", map=name
                )
                continue
            floors.append(floor)
        return sorted(floors, key=lambda floor: floor.level)

    def write_floors(self, name: str, floors: List[MapFloor]) -> List[MapFloor]:
        """Replace the map's floors, lowest first; ``[]`` saves "one storey".

        A temp file + rename, like every write here. The levels are body
        heights in this map's frame, the same quantity poses.txt holds, so a
        saved floor and a detected one compare directly.
        """
        ordered = sorted(floors, key=lambda floor: floor.level)
        document = {
            "format": FLOORS_JSON_FORMAT,
            "version": FLOORS_JSON_VERSION,
            "floors": [
                {"name": floor.name, "level": round(float(floor.level), 3)}
                for floor in ordered
            ],
        }
        write_text_atomic(
            os.path.join(self.resolve_dir(name), FLOORS_JSON),
            json.dumps(document, indent=2) + "\n",
        )
        self.logger.info("[MapCatalogRepo] Wrote floors", map=name, floors=len(ordered))
        return ordered

    # --- Listing ------------------------------------------------------------

    def list_maps(self) -> List[StoredMap]:
        """Return every map directory, by name.

        Directories only (loose files at the root of ``map/`` are not maps),
        minus import staging.
        """
        try:
            entries = sorted(os.scandir(self.maps_dir), key=lambda e: e.name)
        except FileNotFoundError:
            self.logger.warning(
                "[MapCatalogRepo] Maps directory does not exist", path=self.maps_dir
            )
            return []

        maps: List[StoredMap] = []
        for entry in entries:
            if not entry.is_dir() or entry.name.startswith(IMPORT_STAGING_PREFIX):
                continue
            stored = self._read(entry.name, entry.path)
            if stored is not None:
                maps.append(stored)
        return maps

    def get_map(self, name: str) -> Optional[StoredMap]:
        """Everything the console shows about map ``name``, or None.

        Not cheap: ``_read`` walks every file under the map (``walk_stats``;
        ``patches/`` alone holds hundreds) and opens its sidecars. A caller
        that only needs to know the map is there wants ``exists``.
        """
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            return None
        return self._read(name, path)

    def map_clean_running(self, name: str) -> bool:
        """Is the robot's map cleaning still writing into map ``name``?

        Off its sidecar alone (one small read), not ``get_map``: the guards
        that ask must not walk the whole map. ``clean_map`` rewrites map.pcd
        and builds octomap.bt in place for minutes after a save.
        """
        path = self.resolve_dir(name)
        record = self._read_map_clean_record(name, path)
        return record is not None and record.running()

    def exists(self, name: str) -> bool:
        """Whether map ``name`` is on this robot: one ``isdir``.

        ``get_map(name) is not None`` without reading the map. Same refusals:
        an invalid or staging name is ``resolve_dir``'s BadRequestError.
        """
        return os.path.isdir(self.resolve_dir(name))

    def active_name(self) -> Optional[str]:
        """The loaded map: the INI's ``[map] name`` as read at startup, then
        as ``set_active`` last wrote it. No disk read.

        Every dispatch, every map-lock check and every MOVE goal asks, several
        of them on the event loop; the INI has one writer and it is
        ``set_active``, in this process. An INI edited by hand is seen at the
        next restart (README *Gotchas*: configuration is read once).
        """
        return self._active

    def load_active(self) -> Optional[str]:
        """Read the active map from the INI into the cache; return it.

        The constructor's read. Tests call it again after pointing
        ``SYNCAI_SYSTEM_INI`` at their own INI; nothing in production does.
        """
        with self._active_lock:
            self._active = active_map_name(self.logger)
            return self._active

    def set_active(self, name: str) -> str:
        """Record ``name`` as the loaded map: the INI first, then the cache.
        Returns the INI's path.

        Raises what ``set_active_map`` raises (OSError, ValueError) with the
        cache untouched, so a failed write never leaves this process naming a
        map the next boot will not load. The lock keeps the cache in the
        order the writes happened.
        """
        with self._active_lock:
            path = set_active_map(name, self.logger)
            self._active = name
            return path

    # --- Writing ------------------------------------------------------------

    def create_map_dir(self, name: str) -> str:
        """Create an empty directory for ``pgo/save_maps`` to fill; return it.

        ``saveMapsCB`` requires the directory to exist already.

        An existing directory is a ``ConflictError``, never reused: save_maps
        would overwrite ``map.pcd`` and wipe ``patches/``. ``makedirs``
        (``exist_ok=False``) backs the check at the syscall against races.
        """
        directory = self.resolve_dir(name)
        if os.path.exists(directory):
            raise ConflictError(f"A map named '{name}' already exists.")

        os.makedirs(directory)
        self.logger.info("[MapCatalogRepo] Created map directory", map=name)
        return directory

    def discard_empty_map_dir(self, name: str) -> None:
        """Remove a map directory only if it is empty — create_map_dir's unwind.

        After a failed save_maps, so no empty ghost map is listed. ``rmdir``,
        not ``rmtree``: a non-empty directory is left alone and logged.
        """
        try:
            os.rmdir(self.resolve_dir(name))
        except OSError as exc:
            self.logger.warning(
                "[MapCatalogRepo] Left the map directory in place",
                map=name,
                error=str(exc),
            )

    def rename_map_dir(self, old: str, new: str) -> str:
        """Move ``map/<old>/`` to ``map/<new>/``; return the new absolute path.

        One ``os.rename`` suffices because **no file inside a map directory
        names the map or holds an absolute path** (``image:`` keys are relative
        basenames). A future sidecar that embeds the path must be rewritten here.

        The caller must update the ``map_vertices`` / ``task_templates`` rows
        keyed on the name (a foreign key by convention, no constraint).

        Refusing an existing target is a check, not a syscall guarantee (POSIX
        ``rename`` onto an empty directory succeeds); the window is accepted.
        Refusing the *active* map is the router's job, with its other 409s.
        """
        src = self.resolve_dir(old)
        dst = self.resolve_dir(new)
        if old == new:
            raise BadRequestError(f"Map '{old}' is already named {new!r}.")
        if not os.path.isdir(src):
            raise NotFoundError(f"No map named '{old}' on this robot.")
        if os.path.exists(dst):
            raise ConflictError(f"A map named '{new}' already exists.", code="name_taken")

        os.rename(src, dst)
        self.logger.info("[MapCatalogRepo] Renamed map directory", map=old, new_name=new)
        return dst

    def delete_map_dir(self, name: str) -> None:
        """Remove ``map/<name>/`` and everything under it. There is no undo.

        ``resolve_dir`` confines the path under the maps root before ``rmtree``.

        The caller deletes the ``map_vertices`` rows first (irreversible step
        last), and the router refuses while a template names the map or the
        map is active.
        """
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            raise NotFoundError(f"No map named '{name}' on this robot.")

        shutil.rmtree(path)
        self.logger.info("[MapCatalogRepo] Deleted map directory", map=name)

    # --- Importing ----------------------------------------------------------
    #
    # begin (stage) -> commit (publish, park the old map) -> undo (DB refused)
    # / abort (remove a hidden dir). ``_check_staging`` confines every path to
    # the ``IMPORT_STAGING_PREFIX`` family, so an ``rmtree`` never hits a map.

    def _check_staging(self, path: str) -> str:
        """Return ``path`` if it is a hidden import directory of ours, else raise."""
        real = os.path.realpath(path)
        if (
            os.path.dirname(real) != os.path.realpath(self.maps_dir)
            or not os.path.basename(real).startswith(IMPORT_STAGING_PREFIX)
        ):
            raise BadRequestError(f"Not an import staging directory: {path!r}")
        return real

    def begin_import(self, name: str) -> str:
        """Create and return a fresh staging directory for an import of ``name``.

        ``name`` is only validated, not checked for a clash: an import may
        replace a map, and the router decides. ``mkdtemp`` gives an unguessable
        token and 0o700.
        """
        self.resolve_dir(name)
        os.makedirs(self.maps_dir, exist_ok=True)
        staging = tempfile.mkdtemp(prefix=IMPORT_STAGING_PREFIX, dir=self.maps_dir)
        self.logger.info("[MapCatalogRepo] Staging a map import", map=name, path=staging)
        return staging

    def commit_import(self, name: str, staging: str) -> Tuple[str, Optional[str]]:
        """Publish ``staging`` as ``map/<name>/``; return ``(path, displaced)``.

        An existing map of that name is parked as ``.import-old-<token>``
        (``displaced``, else None) for the caller to remove once the DB follows,
        or to hand to ``undo_import``.

        Park-then-publish because POSIX ``rename`` cannot replace a non-empty
        directory. The name is briefly absent; the router only gets here for a
        map neither active nor converting.
        """
        target = self.resolve_dir(name)
        staging = self._check_staging(staging)

        displaced: Optional[str] = None
        if os.path.exists(target):
            # mkdtemp for the unique, hidden name; rmdir so rename can take it.
            displaced = tempfile.mkdtemp(prefix=IMPORT_STAGING_PREFIX + "old-", dir=self.maps_dir)
            os.rmdir(displaced)
            os.rename(target, displaced)

        os.rename(staging, target)
        self.logger.info(
            "[MapCatalogRepo] Published an imported map",
            map=name,
            replaced=displaced is not None,
        )
        return target, displaced

    def undo_import(self, name: str, displaced: Optional[str]) -> None:
        """Reverse ``commit_import`` after the database refused to follow it.

        The published directory is removed and the displaced one restored.
        Failures are logged, never raised: the caller is already raising.
        """
        target = self.resolve_dir(name)
        try:
            parked = tempfile.mkdtemp(prefix=IMPORT_STAGING_PREFIX + "undo-", dir=self.maps_dir)
            os.rmdir(parked)
            os.rename(target, parked)
            shutil.rmtree(parked, ignore_errors=True)
        except OSError as exc:
            self.logger.error(
                "[MapCatalogRepo] Could not remove the imported directory",
                map=name,
                error=str(exc),
            )
        if displaced is not None:
            try:
                os.rename(self._check_staging(displaced), target)
            except (OSError, BadRequestError) as exc:
                self.logger.error(
                    "[MapCatalogRepo] Could not put the previous map back",
                    map=name,
                    displaced=displaced,
                    error=str(exc),
                )

    def abort_import(self, path: str) -> None:
        """Remove a hidden import directory. Never raises — it runs on error paths.

        A failed staging directory, or the displaced map once the DB accepted
        its replacement. A path outside the ``.import-*`` family is logged and left.
        """
        try:
            shutil.rmtree(self._check_staging(path), ignore_errors=True)
        except BadRequestError as exc:
            self.logger.error(
                "[MapCatalogRepo] Refusing to remove a path that is not an import directory",
                path=path,
                error=str(exc),
            )

    def free_bytes(self) -> int:
        """Free space on the filesystem that holds the maps, in bytes."""
        return shutil.disk_usage(self.maps_dir).free

    def scratch_file(self) -> BinaryIO:
        """An anonymous temp file on the maps' filesystem, for spooling an
        archive (an import's upload, an export's build) instead of holding it
        in memory.

        On the maps' filesystem because that is where the space for a map is
        known to be (the container's own /tmp is its overlay layer). Anonymous
        (``TemporaryFile``: unlinked at once on Linux), so it never shows up in
        ``map/`` and nothing is left behind when the request is cut off or the
        process dies: the space is freed when the descriptor closes.
        """
        return tempfile.TemporaryFile(dir=self.maps_dir)

    def write_keepout(self, name: str, zones: List[KeepoutZone]) -> None:
        """Replace the map's forbidden zones in ``keepout.json``.

        Through a temp file + rename. Empty ``zones`` is how zones are
        cleared: the file is rewritten with none, never deleted. A zone's
        ``name`` / ``z_min`` / ``z_max`` are written only when set. Nothing
        written holds a path.

        Until 2026-10-28 this also rasterised a ``keepout.pgm`` +
        ``keepout.yaml`` mask for the 2D keepout filter; a map saved before
        keeps its last pair, which nothing reads.
        """
        directory = self.resolve_dir(name)
        document = {
            "version": KEEPOUT_JSON_VERSION,
            "zones": [_keepout_zone_document(zone) for zone in zones],
        }
        write_text_atomic(
            os.path.join(directory, KEEPOUT_JSON), json.dumps(document, indent=2) + "\n"
        )
        self.logger.info("[MapCatalogRepo] Wrote keepout zones", map=name, zones=len(zones))

    # --- Internals ----------------------------------------------------------

    def _read(self, name: str, path: str) -> Optional[StoredMap]:
        """Describe one directory; None only if it disappeared mid-scan."""
        try:
            size_bytes, newest_mtime = walk_stats(path)
        except FileNotFoundError:
            return None

        return StoredMap(
            name=name,
            has_pointcloud=os.path.isfile(os.path.join(path, POINTCLOUD_PCD)),
            size_bytes=size_bytes,
            modified_at=datetime.fromtimestamp(newest_mtime, tz=timezone.utc),
            octomap=self._read_octomap(name, path),
            map_clean_record=self._read_map_clean_record(name, path),
        )

    def _read_octomap(self, name: str, path: str) -> Optional[OctomapInfo]:
        """Describe ``octomap.bt`` if the directory has one, else None.

        A tree that vanishes between the listing and the stat (a save into
        the same directory removes the old one first) is None, not an error,
        like a patch gone mid-walk in ``walk_stats``.
        """
        tree = os.path.join(path, OCTOMAP_BT)
        try:
            stats = os.stat(tree)
        except FileNotFoundError:
            return None
        except OSError as exc:
            self.logger.debug(
                "[MapCatalogRepo] Unreadable octomap.bt", map=name, error=str(exc)
            )
            return None
        return OctomapInfo(
            size_bytes=stats.st_size,
            mtime_ns=stats.st_mtime_ns,
            resolution=_read_octomap_resolution(tree),
        )

    def _read_map_clean_record(self, name: str, path: str) -> Optional[MapCleanRecord]:
        """Read what the cleaning sidecar says, or None; never raises.

        A sidecar caught mid-write must degrade one card, not fail the
        listing: pgo_node writes it ``converting`` before the save answers,
        the clean rewrites it minutes later, and the listing is polled
        meanwhile. Debug, not warning, for the same reason.
        """
        sidecar = os.path.join(path, MAP_CLEAN_SIDECAR)
        document = _read_json(
            sidecar,
            lambda error: self.logger.debug(
                "[MapCatalogRepo] Unreadable map clean sidecar", map=name, error=error
            ),
        )
        if document is _NO_DOCUMENT:
            return None

        if not isinstance(document, dict):
            return None
        status = MapCleanStatus.parse(document.get("status"))
        if status is None:
            return None
        params = document.get("params")
        measurements = document.get("measurements")
        params = params if isinstance(params, dict) else {}
        measurements = measurements if isinstance(measurements, dict) else {}
        started_at = document.get("started_at")
        error = document.get("error")
        octomap_error = measurements.get("octomap_error")
        return MapCleanRecord(
            status=status,
            started_at=started_at if isinstance(started_at, str) else None,
            octomap_requested=params.get("octomap") is True,
            octomap_written=measurements.get("octomap_written") is True,
            octomap_error=(
                octomap_error if isinstance(octomap_error, str) and octomap_error else None
            ),
            error=error if isinstance(error, str) and error else None,
        )


def _parse_keepout_zone(entry: object) -> Optional[KeepoutZone]:
    """One ``keepout.json`` entry -> ``KeepoutZone``, or None if malformed."""
    if not isinstance(entry, dict):
        return None
    zone_id = entry.get("id")
    raw_points = entry.get("points")
    if not isinstance(zone_id, str) or not zone_id or not isinstance(raw_points, list):
        return None
    points = []
    for raw in raw_points:
        if not isinstance(raw, dict):
            return None
        x, y = raw.get("x"), raw.get("y")
        if isinstance(x, bool) or isinstance(y, bool):
            return None
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return None
        points.append((float(x), float(y)))
    if len(points) < 3:
        return None
    # The optional keys; null is absent (an unset optional field serialises so).
    name = entry.get("name")
    if name is not None and not isinstance(name, str):
        return None
    band = []
    for key in ("z_min", "z_max"):
        value = entry.get(key)
        if value is None:
            band.append(None)
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        if not math.isfinite(value):
            return None
        band.append(float(value))
    z_min, z_max = band
    if z_min is not None and z_max is not None and z_min > z_max:
        return None
    return KeepoutZone(id=zone_id, points=tuple(points), name=name, z_min=z_min, z_max=z_max)


def _keepout_zone_document(zone: KeepoutZone) -> dict:
    """One ``KeepoutZone`` -> its ``keepout.json`` entry; unset optionals omitted."""
    document: dict = {"id": zone.id, "points": [{"x": x, "y": y} for x, y in zone.points]}
    if zone.name is not None:
        document["name"] = zone.name
    if zone.z_min is not None:
        document["z_min"] = zone.z_min
    if zone.z_max is not None:
        document["z_max"] = zone.z_max
    return document


# The banner every ``.bt`` opens with; a few short text lines follow (``id``,
# ``size``, ``res``), then ``data`` and the tree.
_OCTOMAP_BANNER = b"# Octomap OcTree binary file"
_OCTOMAP_HEADER_MAX_BYTES = 4096


def _read_octomap_resolution(path: str) -> Optional[float]:
    """The ``res`` line of a ``.bt``'s text header, or None.

    Reads the first few KB only; the tree after ``data`` is never parsed
    here. None for a file without the banner, or with no ``res`` before
    ``data``.
    """
    try:
        with open(path, "rb") as handle:
            head = handle.read(_OCTOMAP_HEADER_MAX_BYTES)
    except OSError:
        return None
    if not head.startswith(_OCTOMAP_BANNER):
        return None
    for line in head.split(b"\n"):
        if line == b"data":
            return None
        if line.startswith(b"res "):
            try:
                return float(line[4:].decode("ascii").strip())
            except (UnicodeDecodeError, ValueError):
                return None
    return None


def _parse_octomap_key(entry: object) -> Optional[Tuple[int, int, int]]:
    """One ``fill_key`` triple as three in-range ints, or None."""
    if not isinstance(entry, list) or len(entry) != 3:
        return None
    for value in entry:
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        if value < 0 or value > OCTOMAP_KEY_MAX:
            return None
    return (entry[0], entry[1], entry[2])


def _parse_floor(entry: object) -> Optional[MapFloor]:
    """One ``floors`` entry as a named, finite level, or None."""
    if not isinstance(entry, dict):
        return None
    name = entry.get("name")
    level = entry.get("level")
    if not isinstance(name, str) or not name.strip():
        return None
    if isinstance(level, bool) or not isinstance(level, (int, float)):
        return None
    if not math.isfinite(level):
        return None
    return MapFloor(name=name.strip(), level=float(level))


def init_map_catalog_repo(logger: structlog.stdlib.BoundLogger) -> MapCatalogRepo:
    return MapCatalogRepo(logger=logger)
