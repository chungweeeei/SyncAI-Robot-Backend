import json
import math
import os
import re
import shutil
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from typing import List, Optional, Tuple

import structlog
import yaml

from syncai_backend.exceptions import BadRequestError, ConflictError, NotFoundError
from syncai_backend.helpers.floors import read_pose_heights
from syncai_backend.helpers.keepout import keepout_yaml_text, rasterize_zones
from syncai_backend.helpers.pgm import read_pgm_size, write_pgm, write_text_atomic
from syncai_backend.helpers.system_config import active_map_name


# Deliberately strict: the only thing between a URL path segment (or a rename
# body) and the filesystem.
_NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

# Sidecar of the last pcd -> gridmap conversion: recipe, measurements, outcome.
# services/gridmap_conversion writes it; named here with the rest of the map
# directory's layout.
GRIDMAP_RECIPE_SIDECAR = "gridmap.recipe.json"
GRIDMAP_RECIPE_SIDECAR_PREV = "gridmap_prev.recipe.json"

# Sidecar of the robot's post-save map cleaning (pgo_node writes ``converting``,
# its ``clean_map`` turns it ``ok`` / ``failed`` and swaps in a cleaned map.pcd).
# See services/gridmap_conversion.wait_for_map_clean.
MAP_CLEAN_SIDECAR = "map_clean.recipe.json"

# The current files, then the generation ``archive_gridmap`` sets aside.
# ``gridmap_raw.pgm`` is the pristine pre-edit snapshot.
GRIDMAP_PGM = "gridmap.pgm"
GRIDMAP_YAML = "gridmap.yaml"
GRIDMAP_RAW_PGM = "gridmap_raw.pgm"
POINTCLOUD_PCD = "map.pcd"

GRIDMAP_PREV_PGM = "gridmap_prev.pgm"
GRIDMAP_PREV_YAML = "gridmap_prev.yaml"
GRIDMAP_PREV_RAW_PGM = "gridmap_prev_raw.pgm"

# Forbidden zones. ``keepout.json`` (polygons in map-frame metres) is the source
# of truth; ``keepout.pgm`` + ``keepout.yaml`` are derived by ``write_keepout``
# in the gridmap's geometry and served by ``filter_mask_server``. The nav
# session writes a blank pair (no json) at boot for a map with none, so a pair
# without a json means "no zones drawn yet".
KEEPOUT_PGM = "keepout.pgm"
KEEPOUT_YAML = "keepout.yaml"
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


class GridRecordStatus(str, Enum):
    """The ``status`` values the sidecar on disk can carry.

    Only what a conversion writes; ``interrupted`` is derived by the REST layer
    (``converting`` with no thread behind it).

    Not ``GridStatus`` in ``interfaces/rest/routers/map.py``, the wire contract:
    disk and wire vocabularies may drift apart, and only that module's
    ``_grid_status`` converts between them.

    ``str`` mixin (``enum.StrEnum`` is 3.11+) so members ``json.dumps`` bare.
    """

    CONVERTING = "converting"
    OK = "ok"
    FAILED = "failed"

    @classmethod
    def parse(cls, value: object) -> Optional["GridRecordStatus"]:
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
class GridInfo:
    resolution: float
    origin: Tuple[float, float, float]
    width: int
    height: int


@dataclass(frozen=True)
class KeepoutZone:
    """One forbidden zone: a closed polygon in map-frame metres.

    ``points`` are the corners in drawing order, not closed (the console's
    ``ZonePolygon``). The id is the console's handle; the mask ignores it.
    """

    id: str
    points: Tuple[Tuple[float, float], ...]


@dataclass(frozen=True)
class GridRecord:
    """How the last conversion of this map ended, per its sidecar.

    Only the fields callers act on; the rest of the sidecar is for whoever
    reads files on the robot. ``error`` is None unless the status is failed.
    """

    status: GridRecordStatus
    error: Optional[str]


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
        never edited in place the way the gridmap is. The console keys its
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

    status: GridRecordStatus
    started_at: Optional[str]
    octomap_requested: bool
    octomap_written: bool
    octomap_error: Optional[str]
    error: Optional[str]


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
    grid: Optional[GridInfo]
    has_pointcloud: bool
    size_bytes: int
    modified_at: datetime
    # None without a usable sidecar; a grid with no record counts as converted
    # (see the REST layer's _grid_status).
    grid_record: Optional[GridRecord]
    # The tree on disk, or None without one, and what the cleaning sidecar
    # says about it (see the REST layer's _octomap_status).
    octomap: Optional[OctomapInfo] = None
    map_clean_record: Optional[MapCleanRecord] = None


class MapCatalogRepo:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger
        self.maps_dir = os.path.expanduser("~/robot_ws/map")
        # Logged: the path is not configurable, so an unexpected HOME is
        # otherwise invisible.
        self.logger.info("[MapCatalogRepo] Serving maps", path=self.maps_dir)

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

    def gridmap_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``gridmap.pgm``, or None if absent."""
        return self._artifact_path(name, GRIDMAP_PGM)

    def gridmap_yaml_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``gridmap.yaml``, or None if absent.

        Absolute and ``~``-free by construction, as the consumer needs:
        ``syncai_map_server`` resolves ``image:`` against the unexpanded path,
        so a ``~`` url fails with ``RESULT_INVALID_MAP_DATA``. Do not derive it
        from the INI's ``[map] map``, which is relative to the workspace root.
        """
        return self._artifact_path(name, GRIDMAP_YAML)

    def pointcloud_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``map.pcd``, or None if absent.

        The file ``has_pointcloud`` reports on.
        """
        return self._artifact_path(name, POINTCLOUD_PCD)

    def octomap_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``octomap.bt``, or None if absent."""
        return self._artifact_path(name, OCTOMAP_BT)

    def keepout_yaml_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``keepout.yaml``, or None if unloadable.

        None unless **both** the yaml and ``keepout.pgm`` exist:
        ``filter_mask_server/load_map`` answers ``RESULT_INVALID_MAP_DATA`` for
        a yaml whose image is missing. Absolute and ``~``-free, as above.
        """
        yaml_path = self._artifact_path(name, KEEPOUT_YAML)
        if yaml_path is None or self._artifact_path(name, KEEPOUT_PGM) is None:
            return None
        return yaml_path

    def read_keepout_zones(self, name: str) -> List[KeepoutZone]:
        """Return the forbidden zones recorded in ``keepout.json``, or ``[]``.

        ``[]`` also for a json this build cannot parse (logged): an error here
        would lock the operator out of the save that overwrites it. Malformed
        entries are skipped one at a time.
        """
        path = os.path.join(self.resolve_dir(name), KEEPOUT_JSON)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                document = json.load(handle)
        except FileNotFoundError:
            return []
        except (OSError, ValueError) as exc:
            self.logger.warning(
                "[MapCatalogRepo] Unreadable keepout.json; reporting no zones",
                map=name,
                error=str(exc),
            )
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
        try:
            with open(path, "r", encoding="utf-8") as handle:
                document = json.load(handle)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as exc:
            self.logger.warning(
                "[MapCatalogRepo] Unreadable octomap_edits.json; reporting no patches",
                map=name,
                error=str(exc),
            )
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
        # Compact on purpose: a map's worth of patches is tens of thousands of
        # triples, three lines each when indented.
        write_text_atomic(
            os.path.join(directory, OCTOMAP_EDITS_JSON), json.dumps(document) + "\n"
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
        try:
            with open(path, "r", encoding="utf-8") as handle:
                document = json.load(handle)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as exc:
            self.logger.warning(
                "[MapCatalogRepo] Unreadable floors.json; using the detected floors",
                map=name,
                error=str(exc),
            )
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
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            return None
        return self._read(name, path)

    def active_name(self) -> Optional[str]:
        return active_map_name(self.logger)

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

    def gridmap_edited(self, name: str) -> bool:
        """Report whether this map's gridmap carries hand edits.

        ``gridmap_raw.pgm`` existing **and differing** from ``gridmap.pgm``: a
        save without changes leaves an identical raw. A full byte compare,
        since size/mtime are unreliable here (see the router's ``_content_tag``).
        """
        directory = self.resolve_dir(name)
        raw_path = os.path.join(directory, GRIDMAP_RAW_PGM)
        grid_path = os.path.join(directory, GRIDMAP_PGM)
        if not os.path.isfile(raw_path) or not os.path.isfile(grid_path):
            return False
        try:
            with open(raw_path, "rb") as raw, open(grid_path, "rb") as grid:
                return raw.read() != grid.read()
        except OSError as exc:
            # Cannot tell: claim edits, so the caller refuses the overwrite.
            self.logger.warning(
                "[MapCatalogRepo] Could not compare gridmap with its raw copy",
                map=name,
                error=str(exc),
            )
            return True

    def archive_gridmap(self, name: str) -> None:
        """Set the current gridmap aside as ``gridmap_prev.*`` before a re-convert.

        One undo level, not a version store:

        - ``gridmap.pgm`` is *copied*, so the active map keeps a file to serve
          until the new conversion atomically replaces it.
        - ``gridmap.yaml`` is copied too: the extent and origin may change.
        - ``gridmap_raw.pgm`` is **moved**: ``write_gridmap`` only snapshots
          when absent, so a stale raw would never refresh.
        - ``gridmap.recipe.json`` is **moved**: the new conversion overwrites it
          with ``converting`` first, losing the served grid's record on failure.

        ``copy2`` keeps mtimes, so ``_walk_stats``'s modified_at does not jump.
        No gridmap yet is a no-op.
        """
        directory = self.resolve_dir(name)
        grid_path = os.path.join(directory, GRIDMAP_PGM)
        if not os.path.isfile(grid_path):
            return

        shutil.copy2(grid_path, os.path.join(directory, GRIDMAP_PREV_PGM))
        yaml_path = os.path.join(directory, GRIDMAP_YAML)
        if os.path.isfile(yaml_path):
            shutil.copy2(yaml_path, os.path.join(directory, GRIDMAP_PREV_YAML))
        raw_path = os.path.join(directory, GRIDMAP_RAW_PGM)
        if os.path.isfile(raw_path):
            os.replace(raw_path, os.path.join(directory, GRIDMAP_PREV_RAW_PGM))
        sidecar_path = os.path.join(directory, GRIDMAP_RECIPE_SIDECAR)
        if os.path.isfile(sidecar_path):
            os.replace(sidecar_path, os.path.join(directory, GRIDMAP_RECIPE_SIDECAR_PREV))
        self.logger.info(
            "[MapCatalogRepo] Archived the gridmap before re-conversion",
            map=name,
        )

    def write_gridmap(self, name: str, data: bytes) -> bytes:
        """Replace an existing gridmap's cells with ``data``; return the file.

        ``data`` is one byte per cell in .pgm row order (row 0 = max y), exactly
        ``width * height`` of the existing file: this repo never resizes a map.
        Returns the file as written, for the ETag.

        **``gridmap.yaml`` is never touched**: nothing in it depends on cell
        values, ``yaml.safe_dump`` would reformat it (``image:`` must stay
        relative), and a torn yaml stops the map loading.
        """
        directory = self.resolve_dir(name)
        path = os.path.join(directory, GRIDMAP_PGM)

        # Re-checked despite the router: a race must not create a pgm without
        # a yaml, which _read_grid would report as no grid.
        if not os.path.isfile(path):
            raise NotFoundError(f"Map '{name}' has no gridmap to overwrite.")

        # Only when absent, so the raw stays the pristine conversion output.
        # copy2 keeps the mtime, so modified_at does not jump.
        raw_path = os.path.join(directory, GRIDMAP_RAW_PGM)
        if not os.path.exists(raw_path):
            shutil.copy2(path, raw_path)
            self.logger.info("[MapCatalogRepo] Kept the pre-edit gridmap", map=name, path=raw_path)

        try:
            width, height = read_pgm_size(path)
        except ValueError as exc:
            # Only reachable on a race (the router read geometry the same way).
            self.logger.warning(
                "[MapCatalogRepo] Refusing to overwrite an unreadable gridmap",
                map=name,
                error=str(exc),
            )
            raise NotFoundError(f"Map '{name}' has no readable gridmap.")

        if len(data) != width * height:
            raise BadRequestError(
                f"Gridmap body is {len(data)} bytes; '{name}' is "
                f"{width}x{height} = {width * height} cells."
            )

        written = write_pgm(path=path, width=width, height=height, body=data)
        self.logger.info("[MapCatalogRepo] Wrote gridmap", map=name, width=width, height=height)
        return written

    def write_keepout(self, name: str, zones: List[KeepoutZone], grid: GridInfo) -> None:
        """Replace the map's forbidden zones: rasterise ``zones`` and record them.

        Three files, in this order, each through a temp file + rename:

        1. ``keepout.pgm`` -- ``zones`` painted into a mask of ``grid``'s
           geometry (``helpers/keepout.py``; background 205 = unknown, zones 0).
        2. ``keepout.yaml`` -- the map-server yaml naming it, relative.
        3. ``keepout.json`` -- the polygons themselves, the source of truth.

        pgm before yaml so the yaml never names a missing image; json *last*,
        so a crash leaves the robot enforcing more than the console shows.

        Empty ``zones`` writes the all-unknown mask, which is how zones are
        cleared; the files are never deleted (a running KeepoutFilter keeps its
        last mask anyway). ``grid`` is the gridmap's geometry, which the mask
        must match cell for cell. Nothing written holds a path.
        """
        directory = self.resolve_dir(name)
        mask = rasterize_zones(
            [zone.points for zone in zones],
            width=grid.width,
            height=grid.height,
            resolution=grid.resolution,
            origin_xy=(grid.origin[0], grid.origin[1]),
        )
        write_pgm(
            os.path.join(directory, KEEPOUT_PGM), grid.width, grid.height, mask.tobytes()
        )
        write_text_atomic(
            os.path.join(directory, KEEPOUT_YAML),
            keepout_yaml_text(
                resolution=grid.resolution,
                origin_xy=(grid.origin[0], grid.origin[1]),
                image=KEEPOUT_PGM,
            ),
        )
        document = {
            "version": KEEPOUT_JSON_VERSION,
            "zones": [
                {"id": zone.id, "points": [{"x": x, "y": y} for x, y in zone.points]}
                for zone in zones
            ],
        }
        write_text_atomic(
            os.path.join(directory, KEEPOUT_JSON), json.dumps(document, indent=2) + "\n"
        )
        self.logger.info(
            "[MapCatalogRepo] Wrote keepout mask",
            map=name,
            zones=len(zones),
            width=grid.width,
            height=grid.height,
        )

    # --- Internals ----------------------------------------------------------

    def _read(self, name: str, path: str) -> Optional[StoredMap]:
        """Describe one directory; None only if it disappeared mid-scan."""
        try:
            size_bytes, newest_mtime = _walk_stats(path)
        except FileNotFoundError:
            return None

        return StoredMap(
            name=name,
            grid=self._read_grid(name, path),
            has_pointcloud=os.path.isfile(os.path.join(path, POINTCLOUD_PCD)),
            size_bytes=size_bytes,
            modified_at=datetime.fromtimestamp(newest_mtime, tz=timezone.utc),
            grid_record=self._read_grid_record(name, path),
            octomap=self._read_octomap(name, path),
            map_clean_record=self._read_map_clean_record(name, path),
        )

    def _read_octomap(self, name: str, path: str) -> Optional[OctomapInfo]:
        """Describe ``octomap.bt`` if the directory has one, else None.

        A tree that vanishes between the listing and the stat (a save into
        the same directory removes the old one first) is None, not an error,
        like a patch gone mid-walk in ``_walk_stats``.
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

        Tolerant for ``_read_grid_record``'s reason: pgo_node writes it
        ``converting`` before the save answers, the clean rewrites it minutes
        later, and the listing is polled meanwhile.
        """
        sidecar = os.path.join(path, MAP_CLEAN_SIDECAR)
        try:
            with open(sidecar, "r", encoding="utf-8") as handle:
                document = json.load(handle)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as exc:
            self.logger.debug(
                "[MapCatalogRepo] Unreadable map clean sidecar", map=name, error=str(exc)
            )
            return None

        if not isinstance(document, dict):
            return None
        status = GridRecordStatus.parse(document.get("status"))
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

    def _read_grid_record(self, name: str, path: str) -> Optional[GridRecord]:
        """Read how the last conversion ended off the sidecar, or None.

        None for a missing, unreadable, malformed or status-less sidecar; the
        grid on disk is then the only evidence.

        Never raises: a sidecar caught mid-write must degrade one card, not fail
        the listing.
        """
        sidecar = os.path.join(path, GRIDMAP_RECIPE_SIDECAR)
        try:
            with open(sidecar, "r", encoding="utf-8") as handle:
                document = json.load(handle)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as exc:
            # Debug, not warning: a torn read is expected mid-conversion and
            # the listing is polled every two seconds.
            self.logger.debug(
                "[MapCatalogRepo] Unreadable gridmap recipe sidecar",
                map=name,
                error=str(exc),
            )
            return None

        if not isinstance(document, dict):
            return None
        status = GridRecordStatus.parse(document.get("status"))
        if status is None:
            return None
        error = document.get("error")
        return GridRecord(status=status, error=error if isinstance(error, str) else None)

    def _read_grid(self, name: str, path: str) -> Optional[GridInfo]:
        """Read geometry from gridmap.yaml + the .pgm header, or None.

        An unreadable gridmap is reported as no grid (logged), so one map
        mid-save cannot fail the listing.
        """
        yaml_path = os.path.join(path, GRIDMAP_YAML)
        pgm_path = os.path.join(path, GRIDMAP_PGM)
        if not os.path.isfile(yaml_path) or not os.path.isfile(pgm_path):
            return None

        try:
            with open(yaml_path, "r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle) or {}

            resolution = float(document["resolution"])
            origin = document["origin"]
            width, height = read_pgm_size(pgm_path)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            self.logger.warning(
                "[MapCatalogRepo] Unreadable gridmap; reporting the map without one",
                map=name,
                error=str(exc),
            )
            return None

        return GridInfo(
            resolution=resolution,
            origin=(float(origin[0]), float(origin[1]), float(origin[2])),
            width=width,
            height=height,
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
    return KeepoutZone(id=zone_id, points=tuple(points))


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


def _walk_stats(path: str) -> Tuple[int, float]:
    """Return total bytes and newest mtime under ``path``, dir itself included.

    A real walk: ``patches/`` holds hundreds of small files.
    """
    total = 0
    newest = os.stat(path).st_mtime

    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                stats = os.stat(os.path.join(root, filename))
            except FileNotFoundError:
                # Removed mid-walk (a save in flight).
                continue
            total += stats.st_size
            newest = max(newest, stats.st_mtime)

    return total, newest


def init_map_catalog_repo(logger: structlog.stdlib.BoundLogger) -> MapCatalogRepo:
    return MapCatalogRepo(logger=logger)
