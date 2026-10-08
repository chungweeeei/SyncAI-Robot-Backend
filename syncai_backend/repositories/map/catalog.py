import json
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
from syncai_backend.helpers.keepout import keepout_yaml_text, rasterize_zones
from syncai_backend.helpers.pgm import read_pgm_size, write_pgm, write_text_atomic
from syncai_backend.helpers.system_config import active_map_name


# Deliberately strict. This is the only thing standing between a URL path
# segment and the filesystem, and it also has to hold when the write endpoint
# lands, so it rejects everything that is not obviously a directory name.
_NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

# Sidecar recording the last pcd -> gridmap conversion of a map: which recipe
# ran, its measurements, and — since 2026-09 — how it ended. The REST layer
# writes it (routers/map.py owns the conversion), this repo reads it back, and
# the filename lives here rather than there because the on-disk layout of a map
# directory is this repo's vocabulary: it is the module that already knows what
# ``gridmap.pgm``, ``gridmap_raw.pgm`` and the ``gridmap_prev.*`` generation are
# called.
GRIDMAP_RECIPE_SIDECAR = "gridmap.recipe.json"
GRIDMAP_RECIPE_SIDECAR_PREV = "gridmap_prev.recipe.json"

# The rest of that vocabulary, named here for the same reason and used
# throughout this module instead of the literals it grew up with: the four
# current files, then the generation ``archive_gridmap`` sets aside before a
# re-convert. ``gridmap_raw.pgm`` is the pristine snapshot the z-band recipe
# writes once, so "the grid as converted" survives an operator's patches.
GRIDMAP_PGM = "gridmap.pgm"
GRIDMAP_YAML = "gridmap.yaml"
GRIDMAP_RAW_PGM = "gridmap_raw.pgm"
POINTCLOUD_PCD = "map.pcd"

GRIDMAP_PREV_PGM = "gridmap_prev.pgm"
GRIDMAP_PREV_YAML = "gridmap_prev.yaml"
GRIDMAP_PREV_RAW_PGM = "gridmap_prev_raw.pgm"

# The forbidden-zone (keepout) mask, since 2026-09. ``keepout.json`` is the
# source of truth -- the polygons the operator drew, in map-frame metres, which
# is what the console reads back to edit them. ``keepout.pgm`` + ``keepout.yaml``
# are *derived* from it by ``write_keepout``: the same yaml + image format as
# ``gridmap.*``, with the gridmap's geometry, and the pair ``filter_mask_server``
# actually serves to the planner's KeepoutFilter. The nav session's
# ``costmap_filter_info.launch.py`` writes a blank pgm + yaml (no json) of the
# same geometry when it boots a map that has none, and never touches an
# existing one -- so a map that has been booted into always has the pair, and
# a pair without a json is "no zones drawn here yet" (or a GIMP-authored mask,
# which the next write from the console overwrites).
KEEPOUT_PGM = "keepout.pgm"
KEEPOUT_YAML = "keepout.yaml"
KEEPOUT_JSON = "keepout.json"
KEEPOUT_JSON_VERSION = 1

# The 3D map, since 2026-10. Written by the ROBOT side, never by this process:
# a successful ``pgo/save_maps`` with patches makes syncai_mapping's pgo_node
# spawn ``build_octomap`` in the robot container, which ray-casts every patch
# from its keyframe pose into an OctoMap and exports the two layers the console
# draws. It runs for minutes, detached from the mapping session (it survives a
# mode switch), and reports only through ``octomap.recipe.json`` -- the same
# converting / ok / failed vocabulary as the gridmap's sidecar, but written by
# another process in another container, which is why this repo only reads it.
#
# - ``octomap.bt``: the OctoMap itself, for octovis on a workstation. Nothing
#   here reads it, and the export leaves it out (tens of MB nobody downstream
#   opens).
# - ``octomap_road.pcd``: per (x, y) column, the lowest voxel observed free
#   within a band of the local floor -- "where the lidar saw floor-level air".
# - ``octomap_occupied.pcd``: occupied voxels from just under the local floor
#   up to a ceiling cut, i.e. the walls.
#
# Both PCDs are binary ``pcl::PointXYZ`` of voxel centres, so the layer route
# serves them as they are, without the voxel merge ``map.pcd`` needs.
OCTOMAP_BT = "octomap.bt"
OCTOMAP_ROAD_PCD = "octomap_road.pcd"
OCTOMAP_OCCUPIED_PCD = "octomap_occupied.pcd"
OCTOMAP_RECIPE_SIDECAR = "octomap.recipe.json"

# Directories a map import works in, since 2026-10: an archive is unpacked and
# verified in ``map/.import-<token>/`` and the map it replaces (if any) is
# parked as ``map/.import-old-<token>/`` until the database has agreed. Both
# live *inside* ``maps_dir`` because ``map/`` is a bind mount (docker-compose)
# — a sibling path is another filesystem and the one-``os.rename`` publish
# would become a copy. The shared prefix is what keeps them invisible:
# ``list_maps`` skips it and ``resolve_dir`` refuses it, so no route can see a
# half-unpacked map or address one by name. ``_NAME_RE`` admits a leading dot,
# which is why the refusal has to be explicit.
IMPORT_STAGING_PREFIX = ".import-"


class GridRecordStatus(str, Enum):
    """The ``status`` values the sidecar on disk can carry.

    Deliberately only the three a conversion itself can write: "a thread is
    running" is process state, not disk state, so ``interrupted`` is not one of
    them — it is what the REST layer *derives* from a sidecar that says
    ``converting`` with no thread behind it.

    **Not to be confused with ``GridStatus`` in ``interfaces/rest/routers/map.py``**,
    which is the *wire* contract and has five members. Three of its values are
    spelled identically to these, and that overlap is the reason these two are
    separate types rather than one: this enum is the vocabulary of a file on
    disk, that one is what a client is promised, and the two sets are allowed to
    drift apart. Nothing should convert between them except that module's
    ``_grid_status``, which is where the reconciliation deliberately lives.

    ``str`` mixin (not ``enum.StrEnum``, which is 3.11+ and this runs on ROS
    Humble's 3.10) so members serialise through ``json.dumps`` as their bare
    value and compare equal to the plain strings older sidecars hold.
    """

    CONVERTING = "converting"
    OK = "ok"
    FAILED = "failed"

    @classmethod
    def parse(cls, value: object) -> Optional["GridRecordStatus"]:
        """The member ``value`` names, or None if it names nothing this build knows.

        Tolerant on purpose, and the tolerance is load-bearing twice over. The
        sidecar is untrusted input: it is read once per map on every catalogue
        listing, possibly while a conversion is rewriting it, so a value this
        build does not recognise must degrade one card rather than raise into
        the listing every screen depends on. And a status written by a *newer*
        backend is read by an older one after a rollback — the same
        forward-compatibility rule the stored task-template steps follow.

        None puts the caller on the same path as a sidecar with no status at
        all: what the map has on disk is the only evidence.
        """
        if not isinstance(value, str):
            return None
        try:
            return cls(value)
        except ValueError:
            return None


class OctomapLayer(str, Enum):
    """The two display layers of a map's 3D map, by the name the route takes.

    On-disk vocabulary (which file is which layer), so it lives beside the
    filenames; the REST layer uses it directly as its path-parameter type, which
    is what turns an unknown layer into FastAPI's 422.
    """

    ROAD = "road"
    OCCUPIED = "occupied"

    @property
    def filename(self) -> str:
        """The file in a map directory that holds this layer."""
        return OCTOMAP_ROAD_PCD if self is OctomapLayer.ROAD else OCTOMAP_OCCUPIED_PCD


@dataclass(frozen=True)
class GridInfo:
    resolution: float
    origin: Tuple[float, float, float]
    width: int
    height: int


@dataclass(frozen=True)
class KeepoutZone:
    """One forbidden zone: a closed polygon in map-frame metres.

    ``points`` are the corners in drawing order, not closed (the first is not
    repeated at the end) -- the console's ``ZonePolygon`` shape. The id is the
    console's handle for the zone between one save and the next; it means
    nothing to the mask.
    """

    id: str
    points: Tuple[Tuple[float, float], ...]


@dataclass(frozen=True)
class GridRecord:
    """How the last conversion of this map ended, per its sidecar.

    Only the two fields a caller outside this repo acts on. The sidecar also
    carries the recipe, its parameters and the area diagnostics, which are for
    whoever is reading files on the robot — reflecting all of it through the
    catalogue would be an API surface nobody asked for.

    ``error`` is the pipeline's own diagnosis of a failure, and is None for
    every other status.
    """

    status: GridRecordStatus
    error: Optional[str]


@dataclass(frozen=True)
class OctomapRecord:
    """How the robot-side 3D map build of this map stands, per its sidecar.

    ``GridRecordStatus`` is reused for ``status`` on purpose: the robot side
    writes the same three values the gridmap conversion does, and a second
    enum spelling the same file vocabulary would only be two things to keep in
    step.

    ``started_at`` and ``recorded_at`` exist for the one derivation this repo
    cannot do alone. The build runs in the robot container, so this process has
    no registry saying whether it is still alive; a ``converting`` record is
    aged instead (the REST layer's ``_octomap_status``). ``started_at`` is the
    build's own ISO-8601 stamp, None when absent or unparseable; ``recorded_at``
    is the sidecar's mtime, the fallback.

    ``resolution`` is the voxel edge from ``params.resolution`` -- the console
    sizes its points with it. None when the sidecar does not carry one.
    """

    status: GridRecordStatus
    error: Optional[str]
    started_at: Optional[datetime]
    recorded_at: datetime
    resolution: Optional[float]


@dataclass(frozen=True)
class StoredMap:
    name: str
    grid: Optional[GridInfo]
    has_pointcloud: bool
    size_bytes: int
    modified_at: datetime
    # None for a map with no sidecar at all: every map saved before 2026-09, and
    # every map whose conversion never started. A grid with no record is a
    # successful conversion as far as anyone can tell from disk — see the REST
    # layer's _grid_status, which is where the reconciliation with the live
    # conversion registry happens.
    grid_record: Optional[GridRecord]
    # Both octomap layer PCDs are regular files. Defaulted, like the record
    # below, so the many StoredMap literals that predate the 3D map stay valid.
    has_octomap: bool = False
    # None for a map whose 3D map was never built (every map saved before
    # 2026-10, every patch-less save) or whose sidecar says nothing usable.
    octomap_record: Optional[OctomapRecord] = None


class MapCatalogRepo:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger
        self.maps_dir = os.path.expanduser("~/robot_ws/map")
        # Logged because the path is neither a parameter nor an env var: an empty
        # catalogue on a robot whose HOME is not what the container expects would
        # otherwise leave no breadcrumb at all about where we looked.
        self.logger.info("[MapCatalogRepo] Serving maps", path=self.maps_dir)

    # --- Paths --------------------------------------------------------------

    def resolve_dir(self, name: str) -> str:
        """Return the absolute path of a map directory, or raise BadRequestError.

        Two independent checks, because either alone has a hole: the pattern
        rejects separators and ``..`` before they reach the filesystem, and the
        realpath comparison catches a symlink inside ``maps_dir`` that points
        out of it (the pattern cannot see through a link).
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

        The shared body of the accessors below, which differ only in the
        filename. They stay as named methods rather than collapsing into
        one public ``artifact_path(name, GRIDMAP_PGM)``: every caller is in the
        REST layer asking for one specific thing, and ``gridmap_yaml_path(name)``
        says what it wants where a constant passed as an argument would make the
        reader look it up.

        ``resolve_dir`` first, always -- it is the path-traversal check, and the
        reason no caller is allowed to join a name onto ``maps_dir`` itself.
        """
        path = os.path.join(self.resolve_dir(name), filename)
        return path if os.path.isfile(path) else None

    def gridmap_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``gridmap.pgm``, or None if absent."""
        return self._artifact_path(name, GRIDMAP_PGM)

    def gridmap_yaml_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``gridmap.yaml``, or None if absent.

        Absolute and free of ``~`` by construction — ``maps_dir`` is already
        expanded and ``resolve_dir`` returns a realpath — which is exactly what
        the consumer needs. ``syncai_map_server``'s ``loadMapYaml`` expands
        ``~/`` only to open the yaml, then resolves the yaml's *relative*
        ``image:`` key against ``dirname()`` of the string it was handed
        unexpanded, so a ``~``-prefixed url loads the metadata and then fails on
        the image with ``RESULT_INVALID_MAP_DATA`` — which reads like a corrupt
        map rather than a path bug. Deriving the path from here instead of from
        the INI's ``[map] map`` also sidesteps a second trap: that value is
        *relative* to the workspace root.
        """
        return self._artifact_path(name, GRIDMAP_YAML)

    def pointcloud_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``map.pcd``, or None if absent.

        The same file ``has_pointcloud`` reports on — this hands back the path
        so the REST layer can parse it, rather than making the caller rebuild it
        from ``resolve_dir`` and re-do the containment checks.
        """
        return self._artifact_path(name, POINTCLOUD_PCD)

    def octomap_layer_path(self, name: str, layer: OctomapLayer) -> Optional[str]:
        """Return the path of one 3D-map layer's PCD, or None if absent.

        The robot side writes both through a temp file + rename, so a path
        returned here always names a complete file -- what it does not promise
        is that the build that wrote it is the current one; that is the
        sidecar's job, and the router checks it first.
        """
        return self._artifact_path(name, layer.filename)

    def keepout_yaml_path(self, name: str) -> Optional[str]:
        """Return the path of the map's ``keepout.yaml``, or None if unloadable.

        None unless **both** the yaml and the ``keepout.pgm`` it names are there
        -- the same "the two agree" precondition ``_read_grid`` puts on the
        gridmap, because this path goes straight to ``filter_mask_server/
        load_map``, which reads the pgm through the yaml and answers
        ``RESULT_INVALID_MAP_DATA`` for a yaml whose image is missing. Absolute
        and ``~``-free for the reasons ``gridmap_yaml_path`` gives.
        """
        yaml_path = self._artifact_path(name, KEEPOUT_YAML)
        if yaml_path is None or self._artifact_path(name, KEEPOUT_PGM) is None:
            return None
        return yaml_path

    def read_keepout_zones(self, name: str) -> List[KeepoutZone]:
        """Return the forbidden zones recorded in ``keepout.json``, or ``[]``.

        ``[]`` for a map with no json at all -- never booted into, or only
        carrying the launch's blank mask, or a GIMP-drawn one -- and also for a
        json this build cannot make sense of (torn write, wrong ``version``,
        hand edit): logged as a warning, since unlike the recipe sidecar nothing
        writes this file in the background, and answered with "no zones" rather
        than an error, because the console's next save overwrites it anyway
        and a 500 on the read side would lock the operator out of doing so.
        Individual entries that are not ``{id, points: [{x, y}, ...]}`` are
        skipped one at a time for the same reason.
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

    # --- Listing ------------------------------------------------------------

    def list_maps(self) -> List[StoredMap]:
        """Return every map directory, by name.

        Directories only: the legacy loose ``warehouse.pgm`` / ``testmap.yaml``
        pairs that used to sit at the root of ``map/`` are not maps under the
        per-directory layout, and neither are the ``gridmap_raw.pgm`` backups.
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

        The service is the reason this exists at all: ``saveMapsCB`` demands
        the target directory already exist ("<path> IS NOT EXISTS!") and only
        ever creates ``patches/`` underneath it.

        An existing directory is a ``ConflictError``, never reused. save_maps
        overwrites ``map.pcd`` and *wipes* ``patches/`` before rewriting it, so
        pointing it at an existing map would destroy that map to store this
        one — an operator who wants that deletes the old map first, explicitly.
        ``makedirs`` with the default ``exist_ok=False`` backs the check up at
        the syscall, so two racing requests cannot both pass.
        """
        directory = self.resolve_dir(name)
        if os.path.exists(directory):
            raise ConflictError(f"A map named '{name}' already exists.")

        os.makedirs(directory)
        self.logger.info("[MapCatalogRepo] Created map directory", map=name)
        return directory

    def discard_empty_map_dir(self, name: str) -> None:
        """Remove a map directory only if it is empty — create_map_dir's unwind.

        For the path where the directory was created and save_maps then failed:
        without this, every failed save leaves a ghost entry that ``list_maps``
        reports as a map with nothing in it. ``rmdir``, deliberately not
        ``shutil.rmtree``: if the failure somehow happened *after* files were
        written, deleting data to tidy up a bookkeeping entry is the wrong
        trade, so a non-empty directory is left alone and logged.
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

        A rename is one ``os.rename`` of the directory and nothing else, and that
        rests on a property worth stating so nobody breaks it by accident:
        **no file inside a map directory names the map or holds an absolute
        path.** ``gridmap.yaml`` says ``image: gridmap.pgm`` (the relative
        basename, hand-formatted by ``helpers/pcd_to_gridmap.py`` precisely so
        map_server resolves it against the yaml's own directory), ``poses.txt``
        lists bare patch basenames, ``gridmap.recipe.json`` holds only
        measurements and parameters, ``keepout.yaml`` says ``image:
        keepout.pgm`` and ``keepout.json`` holds polygons in metres. The robot
        side's 3D map keeps to the same rule: ``octomap.bt``,
        ``octomap_road.pcd`` and ``octomap_occupied.pcd`` are data, and
        ``octomap.recipe.json`` holds parameters, measurements, timestamps and
        a path-free error sentence. If a future sidecar ever embeds the map's
        path, this method has to start rewriting it.

        What this does *not* touch, and the caller must: the ``map_vertices``
        and ``task_templates`` rows that key on the bare directory name — the
        filesystem is the catalogue, but the name is also a foreign key by
        convention in two tables with no constraint backing it.

        Both names go through ``resolve_dir``: the new one is a URL body, not a
        path segment, but it ends up on the filesystem all the same, and the
        pattern's comment already says it "has to hold when the write endpoint
        lands". Refusing an existing target is a check rather than a syscall
        guarantee — POSIX ``rename`` onto an existing *empty* directory quietly
        succeeds, so unlike ``create_map_dir`` there is no ``exist_ok=False`` to
        lean on. The check→act window is accepted, the same way
        ``write_gridmap`` accepts its own; two operators renaming onto the same
        name in the same millisecond is not the failure this fleet has.

        Refusing to rename the *active* map is the router's job, not this
        repo's: "which map is the stack running on" is INI state the repo only
        reads through ``active_name()``, and keeping the refusal beside the
        conversion-in-flight check puts every 409 for this route in one place.
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

        ``shutil.rmtree``, deliberately unlike ``discard_empty_map_dir``'s
        ``rmdir`` right above. That one is the unwind of a save that failed and
        must never destroy data it did not create, so it leaves a non-empty
        directory alone; this one *is* the destruction the operator asked for,
        and a map directory is never empty in practice — ``map.pcd``,
        ``poses.txt``, ``patches/`` and the gridmap family all have to go
        together. Half a map is not a state anything here can read.

        ``resolve_dir`` is what makes this safe to hang off a URL path segment:
        the name is regex-checked and the resolved path is confined under the
        maps root before an ``rmtree`` ever sees it.

        What this does *not* touch, and the caller must: the ``map_vertices``
        rows keyed on the bare directory name. ``task_templates.map_name`` is
        the caller's problem too — the router refuses the delete outright while
        a template still names the map, since a template pointing at a map that
        is gone can neither run nor be edited back into shape.

        Refusing to delete the *active* map is the router's job, not this
        repo's, for the reason ``rename_map_dir`` gives: "which map is the stack
        running on" is INI state this repo only reads through ``active_name()``,
        and every 409 for the route belongs in one place.
        """
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            raise NotFoundError(f"No map named '{name}' on this robot.")

        shutil.rmtree(path)
        self.logger.info("[MapCatalogRepo] Deleted map directory", map=name)

    # --- Importing ----------------------------------------------------------
    #
    # Four steps the router strings together, split so each one is a single
    # filesystem fact: ``begin_import`` makes the staging directory the archive
    # unpacks into, ``commit_import`` publishes it under the map's name (parking
    # the map it replaces), ``undo_import`` reverses that publish when the
    # database then refuses, and ``abort_import`` removes whichever hidden
    # directory is no longer wanted. Every path they touch is confined to the
    # ``IMPORT_STAGING_PREFIX`` family under ``maps_dir`` by ``_check_staging``,
    # so an ``rmtree`` here can never reach a real map, whatever the caller
    # hands in.

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

        ``name`` is only *validated* here — not reserved and not checked for a
        clash, because an import may replace an existing map; the router
        decides whether that is allowed. ``mkdtemp`` gives the directory a
        token nobody else can guess and 0o700, inside ``maps_dir`` for the
        same-filesystem reason the prefix's comment gives.
        """
        self.resolve_dir(name)
        os.makedirs(self.maps_dir, exist_ok=True)
        staging = tempfile.mkdtemp(prefix=IMPORT_STAGING_PREFIX, dir=self.maps_dir)
        self.logger.info("[MapCatalogRepo] Staging a map import", map=name, path=staging)
        return staging

    def commit_import(self, name: str, staging: str) -> Tuple[str, Optional[str]]:
        """Publish ``staging`` as ``map/<name>/``; return ``(path, displaced)``.

        When a map of that name exists it is moved aside first, to a fresh
        ``.import-old-<token>`` directory this returns as ``displaced`` so the
        caller can either remove it once the database has followed or hand it
        back to ``undo_import``. ``None`` when there was nothing to displace.

        Two renames rather than one when replacing — POSIX ``rename`` onto a
        non-empty directory fails, and onto an *empty* one quietly succeeds,
        so a park-then-publish is the only sequence that works for both. The
        map's name is absent for the instant between them; the router only
        gets here for a map that is not active and not converting, and no
        route can see a ``.import-*`` directory, so nothing observes it.
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

        The published directory is parked and removed; the displaced one, if
        any, goes back under the map's name. Each step that fails is logged
        and the next is still attempted — this runs while the caller is
        raising, and the operator needs its sentence, not a second traceback
        over the first.
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

        For a staging directory whose archive failed verification, and for the
        displaced previous map once the database has accepted its replacement.
        ``_check_staging`` is what makes an ``rmtree`` here safe to call with
        anything: a path outside the ``.import-*`` family is logged and left.
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

        The signal is ``gridmap_raw.pgm`` existing **and differing** from
        ``gridmap.pgm``. Existence alone is not enough: ``write_gridmap``
        snapshots the raw copy on the *first* save, so an operator who opened the
        editor and saved without changing a cell leaves a raw that is
        byte-identical to the live grid — no edit to protect. The comparison is a
        full read of two ~1-2 MB files, once, on an operator-initiated request;
        cheaper proxies (size, mtime) are exactly the ones ``_content_tag`` in
        the router documents as broken on this filesystem.

        Lives in the repo, not the re-convert endpoint, for the same reason
        ``write_gridmap`` owns the length check: "does this map hold hand edits"
        is a property of the store, true for any future caller.
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
            # A file that vanished or turned unreadable mid-check: claim edits.
            # The caller's next step on True is to refuse a destructive
            # overwrite, which is the safe answer to "cannot tell".
            self.logger.warning(
                "[MapCatalogRepo] Could not compare gridmap with its raw copy",
                map=name,
                error=str(exc),
            )
            return True

    def archive_gridmap(self, name: str) -> None:
        """Set the current gridmap aside as ``gridmap_prev.*`` before a re-convert.

        One ``_prev`` generation, overwritten by the next re-conversion — an undo
        level, deliberately not a version store. Three moves, each load-bearing:

        - ``gridmap.pgm`` is *copied* (not moved) to ``gridmap_prev.pgm`` so the
          map keeps serving its current grid until the new conversion atomically
          replaces it — a re-convert of the active map must not leave map_server
          a window with no file.
        - ``gridmap.yaml`` is copied along with it. A re-conversion can change
          the grid's extent and origin, and a prev pgm without the yaml that
          describes it is unloadable — restoring it would mean guessing geometry.
        - ``gridmap_raw.pgm`` is **moved** to ``gridmap_prev_raw.pgm``. This one
          cannot stay: ``write_gridmap`` snapshots to gridmap_raw.pgm only when
          absent, so a stale raw from the previous conversion would never be
          refreshed for the new grid and would sit there claiming to be the
          pristine copy of a file whose dimensions it may not even share.
        - ``gridmap.recipe.json`` is **moved** to ``gridmap_prev.recipe.json``,
          for the same reason as the raw and one more. The incoming conversion
          overwrites the sidecar with its own ``converting`` record before it
          does any work, so left in place the previous recipe record would be
          destroyed — and if the conversion then fails, the grid still being
          served is the archived one with nothing on disk saying which recipe
          and which parameters produced it.

        ``copy2`` for the copies, same as ``write_gridmap``'s raw snapshot: a
        fresh mtime would make the archive the newest file under ``_walk_stats``
        and drag the card's modified_at to now. A map with no gridmap yet (the
        POST /api/v1/maps path) is a no-op, not an error.
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

        ``data`` is one byte per cell in .pgm row order (row 0 is the top of the
        map, max y) and must be exactly ``width * height`` long — the extent is
        read back off the file being replaced, so this cannot resize a map. That
        check lives here rather than in the REST layer so "you cannot change a
        map's extent through this repo" is a property of the repo, true for any
        future caller, and not of one endpoint.

        Returns the whole file as written, for the REST layer's ETag.

        **``gridmap.yaml`` is never touched**, and that is not an omission. The
        body length pins the extent; ``resolution`` and ``origin`` are properties
        of the pcd → grid conversion, not of cell values; ``mode`` and the two
        thresholds are how the loader *interprets* bytes, and the editor writes
        values already in range for the existing ones. Rewriting it would be all
        downside: ``yaml.safe_dump`` reformats, losing the ``image: gridmap.pgm``
        spelling and the inline ``origin: [x, y, 0.0]`` that
        ``helpers/pcd_to_gridmap.py`` writes, and ``image:`` *must* stay relative
        because map_server resolves it against the yaml's own directory. A torn
        yaml is also the one failure the .pgm's atomic write cannot rescue — the
        map stops loading entirely.
        """
        directory = self.resolve_dir(name)
        path = os.path.join(directory, GRIDMAP_PGM)

        # Re-checked here even though the router already 404'd on a map without a
        # grid. Without it a race would *create* a gridmap.pgm in a directory that
        # has none, i.e. a pgm with no yaml — which _read_grid reports as having no
        # grid at all, so the map would look untouched while holding the edit.
        if not os.path.isfile(path):
            raise NotFoundError(f"Map '{name}' has no gridmap to overwrite.")

        # Only when absent, so gridmap_raw.pgm always holds the pristine
        # pcd_to_gridmap.py output. Copying on every save would, on the *second*
        # save, overwrite that with the first save's edit and destroy the only way
        # back to the conversion tool's result. copy2 rather than copy to keep the
        # original mtime: otherwise the backup becomes the newest file under
        # _walk_stats and drags the card's modified_at forward to now.
        raw_path = os.path.join(directory, GRIDMAP_RAW_PGM)
        if not os.path.exists(raw_path):
            shutil.copy2(path, raw_path)
            self.logger.info("[MapCatalogRepo] Kept the pre-edit gridmap", map=name, path=raw_path)

        try:
            width, height = read_pgm_size(path)
        except ValueError as exc:
            # Only reachable on a race: the router got its geometry from
            # _read_grid, which calls this same function.
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

        pgm before yaml so whoever finds the yaml finds the image it names
        (``filter_mask_server`` resolves ``image:`` against the yaml's own
        directory, and the launch reads the pair at boot). json *last*, so a
        crash between the writes leaves the robot enforcing more than the
        console shows rather than less -- the safe side of the two.

        ``zones`` may be empty: that writes the all-unknown mask, which is how
        zones are *cleared*. The files are never deleted, because deleting them
        would change nothing in a running KeepoutFilter (it holds the last mask
        it was sent) and the launch would only write a blank pair back at the
        next boot.

        ``grid`` comes from the caller rather than being re-read here because
        the router has already refused a map without one; a keepout needs the
        gridmap's geometry to line up with it cell for cell. Nothing written
        names the map or holds an absolute path, so ``rename_map_dir`` stays
        one ``os.rename``.
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
            has_octomap=all(
                os.path.isfile(os.path.join(path, layer.filename)) for layer in OctomapLayer
            ),
            octomap_record=self._read_octomap_record(name, path),
        )

    def _read_octomap_record(self, name: str, path: str) -> Optional[OctomapRecord]:
        """Read the robot-side 3D map build's sidecar, or None.

        The tolerance rules are ``_read_grid_record``'s, for a stronger version
        of its reason: this file is rewritten by a process in *another
        container* while the catalogue polls it, so a torn read, an unknown
        status or a non-object document is None -- "the files on disk are the
        only evidence" -- and never an exception. ``started_at`` and
        ``params.resolution`` are taken when well-formed and dropped otherwise;
        neither may cost the record its status.
        """
        sidecar = os.path.join(path, OCTOMAP_RECIPE_SIDECAR)
        try:
            with open(sidecar, "r", encoding="utf-8") as handle:
                document = json.load(handle)
            recorded_at = datetime.fromtimestamp(os.stat(sidecar).st_mtime, tz=timezone.utc)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as exc:
            # Debug, as for the gridmap sidecar: a mid-write read is expected
            # while a build is running and the console polls every few seconds.
            self.logger.debug(
                "[MapCatalogRepo] Unreadable octomap recipe sidecar",
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
        params = document.get("params")
        resolution = params.get("resolution") if isinstance(params, dict) else None
        if (
            isinstance(resolution, bool)
            or not isinstance(resolution, (int, float))
            or not resolution > 0
        ):
            resolution = None
        return OctomapRecord(
            status=status,
            error=error if isinstance(error, str) else None,
            started_at=_parse_utc(document.get("started_at")),
            recorded_at=recorded_at,
            resolution=float(resolution) if resolution is not None else None,
        )

    def _read_grid_record(self, name: str, path: str) -> Optional[GridRecord]:
        """Read how the last conversion ended off the sidecar, or None.

        None means "the sidecar says nothing usable", which covers five cases
        that all want the same treatment: no sidecar (a map saved before the
        status field existed, or one whose conversion never started), a sidecar
        written by that older code and so carrying no ``status``, one carrying a
        ``status`` this build does not know (see ``GridRecordStatus.parse``), an
        unreadable one, and a malformed one. In each, what the map *has* is the
        only evidence — a grid on disk or not.

        Never raises. This runs once per map on every catalogue listing, and a
        conversion is writing this exact file in another thread while it does;
        a half-written sidecar caught mid-write must degrade one card's status
        detail, not fail the listing that every screen depends on.
        """
        sidecar = os.path.join(path, GRIDMAP_RECIPE_SIDECAR)
        try:
            with open(sidecar, "r", encoding="utf-8") as handle:
                document = json.load(handle)
        except FileNotFoundError:
            return None
        except (OSError, ValueError) as exc:
            # ValueError covers json.JSONDecodeError, which is what a torn write
            # looks like. Logged at debug rather than warning: it is expected
            # while a conversion is in flight, and the poll behind this runs
            # every two seconds.
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

        A directory whose gridmap is unreadable is reported as having no grid
        rather than failing: one map caught mid-save must not take the whole
        catalogue listing down with it. The reason is logged, because "the card
        says no 2D grid but the files are right there" is otherwise a mystery.
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


def _parse_utc(value: object) -> Optional[datetime]:
    """An ISO-8601 timestamp from a sidecar, as an aware UTC datetime, or None.

    The robot side writes ``2026-10-08T03:12:45Z``; Python 3.10's
    ``fromisoformat`` does not take the ``Z``, hence the replace. A naive value
    is read as UTC rather than local time, since every writer here is UTC.
    """
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed


def _walk_stats(path: str) -> Tuple[int, float]:
    """Return total bytes and newest mtime under ``path``, dir itself included.

    ``patches/`` holds hundreds of small .pcd files, so this is a real walk
    rather than a stat of the directory entry — but ``map.pcd`` (~20 MB) is what
    the number is actually reporting.
    """
    total = 0
    newest = os.stat(path).st_mtime

    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                stats = os.stat(os.path.join(root, filename))
            except FileNotFoundError:
                # A file removed while we walked (a save in flight); skip it.
                continue
            total += stats.st_size
            newest = max(newest, stats.st_mtime)

    return total, newest


def init_map_catalog_repo(logger: structlog.stdlib.BoundLogger) -> MapCatalogRepo:
    return MapCatalogRepo(logger=logger)
