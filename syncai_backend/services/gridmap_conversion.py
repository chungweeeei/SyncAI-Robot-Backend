"""The pcd -> gridmap conversion behind POST /api/v1/maps and .../grid/convert.

Built once in main.py and injected (no module-level registry), so tests get a
fresh one. It owns the recipes, the registry of running threads and the sidecar
those threads write; the router only validates and maps refusals to statuses.

Two halves of one state:

- **the registry** is authoritative only while this process is up.
- **the sidecar** (GRIDMAP_RECIPE_SIDECAR, next to the pgm) survives, so it is
  written before any work starts: `converting` with no thread behind it is how
  the router's _grid_status detects `interrupted`.

Nothing here imports open3d at module scope (see the import in start).
"""

import json
import os
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Dict, NamedTuple, Optional

import numpy as np
import structlog

from syncai_backend.exceptions import ConflictError, UpstreamError
from syncai_backend.helpers.pcd_to_gridmap import (
    LocalFloorError,
    convert_pcd_to_gridmap,
    convert_traversable_to_gridmap,
    floor_level,
    read_poses_xyz,
)
from syncai_backend.helpers.pgm import write_text_atomic
from syncai_backend.helpers.pointcloud import read_pcd_xyz
from syncai_backend.repositories.map.catalog import (
    GRIDMAP_RECIPE_SIDECAR,
    MAP_CLEAN_SIDECAR,
    GridRecordStatus,
)


# Two recipes. **Every save converts with z-band** (GRIDMAP_RECIPE below);
# traversability runs only when an operator asks via .../grid/convert. They
# disagree about what an unknown cell means, so the choice is recorded on disk.
#
# **z-band** gives a trinary map. Unknown is recoverable (costmap_layer.cpp:90
# lets live observations overwrite NO_INFORMATION, never LETHAL) and cheap to
# finish by hand in the gridmap editor, which is how the fleet's maps are made.
#
# **traversability** (helpers.traversable: segment the floor by intensity,
# normal and height, repair, project) is for sites nobody hand-edits, and tells
# a drivable aisle from a kerb top or ramp, which z-banding cannot. Its output
# has **no unknown cells**: anything the floor cloud does not cover is occupied,
# permanently, since driving there cannot clear it.
#
# Do not reintroduce an automatic pick (a 3000 m² bbox threshold was removed
# 2026-09): glass inflated a conference hall's bbox past it and traversability
# turned the hall into an occupied blob. Floor area was rejected as the input
# too, since any threshold is calibrated on six clouds. A wrong z-band map is
# recoverable and a wrong traversability map is not, so the latter is a
# person's decision. Both areas are still recorded in the sidecar, as
# diagnostics.
#
# Empty: the helper's defaults are the tuned ones. Tune per site with the debug
# intermediates in hand via the re-convert endpoint (e.g. dp1f's gap_fill_size).
TRAVERSABLE_SEGMENT_RECIPE: Dict[str, object] = {}
TRAVERSABLE_REPAIR_RECIPE: Dict[str, object] = {}
TRAVERSABLE_GRID_RECIPE: Dict[str, object] = {}

# Cell size for measure_cloud's floor-area diagnostic. 0.5 m because map.pcd is
# voxel-downsampled and finer cells under-count the sparse floor.
FLOOR_AREA_CELL_M = 0.5

# GRIDMAP_RECIPE_SIDECAR records which recipe made the grid (the pgm cannot
# tell) and a ``status`` (converting / ok / failed). It is on disk next to the
# grid because it must outlive the thread and the process; without it a failed
# conversion looks like a map never converted.

# Where the segmentation's intermediates go with ``debug: true`` on re-convert.
# Off by default: MapCatalogRepo._walk_stats recurses, so they would inflate
# every catalogue card's size. The service's ``debug_subdir`` forces it on.
TRAVERSABLE_DEBUG_SUBDIR_NAME = "traversable_debug"


# The z-band recipe's non-band parameters (bands below).
#
# obstacle_close is 1, not 2: at 2 the closing sealed a doorway the robot had
# driven through. Raising it closes real doors; lower it before raising it.
GRIDMAP_RECIPE = dict(
    free_mode="floor",
    min_points=2,
    obstacle_close=1,
    free_close=5,
    despeckle_min_size=12,
    fill_holes_max_size=20000,
)

# The z-band recipe's bands, as offsets from the **floor**, not absolute z:
# LIO z=0 is the lidar's height at the start pose, so absolute bands moved with
# mount, chassis and slope (fleet floors span -0.36 to -0.73). They are the old
# absolute bands minus dp1f's floor (-0.66).
#
# Per-keyframe floor (``local``, pcd_to_gridmap.local_floor_levels) is the
# default because a map whose LIO drifted ~1 m in z (0917_TP1F_test1) had half
# its floor fall into the obstacle band under one global level. ``global`` is
# the fallback when there is no poses.txt.
GRIDMAP_BANDS_ABOVE_FLOOR = dict(
    floor_zmin=-0.29,
    floor_zmax=0.41,
    zmin=0.36,
    zmax=2.16,
)

# Where the z-band floor is measured (see above). The router's FloorReference
# enum mirrors it.
FLOOR_REFERENCES = ("local", "global")
FLOOR_REFERENCE_DEFAULT = "local"


class CloudMeasure(NamedTuple):
    """What measure_cloud reads off a map.pcd before a conversion."""

    footprint_m2: float
    floor_area_m2: float
    floor_z: float


def measure_cloud(
    logger: structlog.stdlib.BoundLogger, pcd_path: str
) -> CloudMeasure:
    """Measure the cloud: bbox footprint, covered floor area, floor level.

    ``floor_z`` recentres the z-band bands under the ``global`` reference; the
    areas are sidecar diagnostics only (never a recipe pick). ``floor_area_m2``
    counts FLOOR_AREA_CELL_M cells inside the z-band floor band; set against the
    bbox footprint it exposes a glass-inflated cloud.

    ``floor_z`` is the raw-cloud estimate; traversability measures its own.
    Reads the cloud a second time rather than threading an array through two
    conversions with different input types.
    """
    xyz = read_pcd_xyz(pcd_path)
    if len(xyz) == 0:
        raise ValueError(f"point cloud is empty: {pcd_path}")
    extent = xyz[:, :2].max(axis=0) - xyz[:, :2].min(axis=0)
    footprint = float(extent[0] * extent[1])
    floor_z = floor_level(xyz[:, 2].astype(np.float64))

    band = xyz[
        (xyz[:, 2] >= floor_z + GRIDMAP_BANDS_ABOVE_FLOOR["floor_zmin"])
        & (xyz[:, 2] <= floor_z + GRIDMAP_BANDS_ABOVE_FLOOR["floor_zmax"])
    ]
    if len(band):
        cells = np.unique(
            np.stack(
                [
                    np.floor(band[:, 0] / FLOOR_AREA_CELL_M).astype(np.int64),
                    np.floor(band[:, 1] / FLOOR_AREA_CELL_M).astype(np.int64),
                ],
                axis=1,
            ),
            axis=0,
        )
        floor_area = float(len(cells)) * FLOOR_AREA_CELL_M**2
    else:
        floor_area = 0.0

    logger.info(
        "measured map cloud",
        footprint_m2=round(footprint, 1),
        floor_area_m2=round(floor_area, 1),
        floor_z=round(floor_z, 3),
        points=len(xyz),
    )
    return CloudMeasure(footprint, floor_area, floor_z)


def iso_now() -> str:
    """UTC, ISO 8601, ``Z``-suffixed, like ``MapSummaryResponse.modified_at``."""
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def write_recipe_sidecar(
    logger: structlog.stdlib.BoundLogger, directory: str, payload: Dict[str, object]
) -> None:
    """Record the state of this map's conversion, next to the gridmap.

    Each call overwrites the last, so the file describes the latest attempt;
    ``MapCatalogRepo.archive_gridmap`` keeps the previous successful one as
    ``gridmap_prev.recipe.json``.

    Best-effort (logs, never raises): losing the diagnosis is acceptable,
    losing a converted grid is not.
    """
    path = os.path.join(directory, GRIDMAP_RECIPE_SIDECAR)
    try:
        # Atomic: a torn sidecar reads as None, and _grid_status would then
        # report `ok` for a grid whose rebuild had just failed.
        write_text_atomic(path, json.dumps(payload, indent=2, sort_keys=True) + "\n")
    except OSError as exc:
        logger.warning("Could not write gridmap recipe sidecar", path=path, error=str(exc))


# How long a conversion waits for the robot's map cleaning (MAP_CLEAN_SIDECAR).
# The longest clean measured is ~208 s, and cleans queue per host behind a
# flock; 15 min covers that and still bounds a hung clean.
MAP_CLEAN_TIMEOUT_S = 900.0
# One stat + a ~700 B read per poll; a clean runs for tens of seconds at least.
MAP_CLEAN_POLL_S = 2.0


def parse_iso_utc(value: object) -> Optional[datetime]:
    """``2026-10-12T05:47:05Z`` -> an aware datetime; None for anything else.

    Public because the map router ages the same sidecar's ``started_at`` for
    the 3D map's status, with the same reading of it.
    """
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed if parsed.tzinfo is not None else parsed.replace(tzinfo=timezone.utc)


def wait_for_map_clean(
    logger: structlog.stdlib.BoundLogger,
    directory: str,
    *,
    timeout_s: float = MAP_CLEAN_TIMEOUT_S,
    poll_s: float = MAP_CLEAN_POLL_S,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    wall_now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
) -> Dict[str, object]:
    """Block until the robot's map cleaning of ``directory`` is over; say how it ended.

    save_maps answers once the *raw* map.pcd is on disk, then a detached
    ``clean_map`` replaces it with a copy without people. Converting at once
    put people in the grid as obstacles. Waiting beats "re-convert when the
    clean lands", which converts twice and could overwrite a hand edit.

    Polls the sidecar: it is the clean's only surface (not a ROS node, and it
    outlives the mode switch).

    The result goes into the gridmap sidecar as ``map_clean``. Its ``status``:

    - ``absent``: no sidecar, nothing to wait for.
    - ``ok``: map.pcd is the cleaned cloud.
    - ``failed``: the clean gave up and left the raw cloud; its ``error`` is
      copied along.
    - ``stale``: ``converting`` but started over ``timeout_s`` ago (a clean
      killed hard never writes its outcome).
    - ``timed_out``: still ``converting`` after ``timeout_s`` of waiting here.

    Never raises: every outcome but ``ok`` converts the raw cloud. An
    unreadable sidecar is retried until the deadline, not taken as ``absent``,
    so a misread cannot bring the people back.
    """
    path = os.path.join(directory, MAP_CLEAN_SIDECAR)
    started = monotonic()

    def _waited() -> float:
        return round(monotonic() - started, 1)

    logged = False
    while True:
        record: object = None
        try:
            with open(path, encoding="utf-8") as stream:
                record = json.load(stream)
        except FileNotFoundError:
            return {"status": "absent"}
        except (OSError, ValueError):
            pass

        if isinstance(record, dict):
            status = record.get("status")
            if status == "ok":
                return {"status": "ok", "waited_s": _waited()}
            if status == "failed":
                logger.warning(
                    "map cleaning failed; converting the raw map.pcd",
                    error=record.get("error"),
                )
                return {
                    "status": "failed",
                    "waited_s": _waited(),
                    "error": record.get("error"),
                }
            clean_started = parse_iso_utc(record.get("started_at"))
            if (
                clean_started is not None
                and (wall_now() - clean_started).total_seconds() > timeout_s
            ):
                logger.warning(
                    "map cleaning never finished (killed?); converting the raw map.pcd",
                    started_at=record.get("started_at"),
                )
                return {
                    "status": "stale",
                    "waited_s": _waited(),
                    "started_at": record.get("started_at"),
                }

        if monotonic() - started >= timeout_s:
            logger.warning(
                "map cleaning still running at the deadline; converting the raw "
                "map.pcd -- re-convert once map_clean.recipe.json says ok",
                timeout_s=timeout_s,
            )
            return {"status": "timed_out", "waited_s": _waited()}

        if not logged:
            logger.info("waiting for the map cleaning before converting", sidecar=path)
            logged = True
        sleep(poll_s)


class GridmapConversionService:
    """Runs pcd -> gridmap conversions and tracks the ones in flight.

    One instance per process, built in main.py. ``debug_subdir`` forces the
    intermediates on for every conversion; None leaves it to the per-request
    ``debug`` flag. ``map_clean_*`` bound wait_for_map_clean.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        *,
        debug_subdir: Optional[str] = None,
        map_clean_timeout_s: float = MAP_CLEAN_TIMEOUT_S,
        map_clean_poll_s: float = MAP_CLEAN_POLL_S,
    ):
        self._logger = logger
        self._debug_subdir = debug_subdir
        self._map_clean_timeout_s = map_clean_timeout_s
        self._map_clean_poll_s = map_clean_poll_s

        # Maps with a conversion running, under the lock. Refuses a second
        # concurrent conversion of a map and backs the `converting` status.
        self._active: set = set()
        self._lock = threading.Lock()

    def is_converting(self, name: str) -> bool:
        """Whether a conversion for ``name`` is running in *this* process."""
        with self._lock:
            return name in self._active

    def start(
        self,
        name: str,
        directory: str,
        *,
        recipe_request: str = "z-band",
        floor_reference: str = FLOOR_REFERENCE_DEFAULT,
        band_offset_overrides: Optional[Dict[str, float]] = None,
        grid_overrides: Optional[Dict[str, object]] = None,
        debug: bool = False,
        override: Optional[Dict[str, object]] = None,
        archive: Optional[Callable[[], None]] = None,
        on_success: Optional[Callable[[], None]] = None,
    ) -> bool:
        """Start the pcd -> gridmap conversion on a daemon thread; report whether.

        In-process: the heavy passes release the GIL, and the helpers bound the
        grid size themselves.

        ``floor_reference`` (z-band only): ``local`` degrades to ``global``, with
        ``floor_reference_fallback`` in the sidecar, when poses.txt is unusable.
        For the re-convert endpoint: ``band_offset_overrides`` merges over
        GRIDMAP_BANDS_ABOVE_FLOOR, ``grid_overrides`` over
        TRAVERSABLE_GRID_RECIPE, ``debug`` writes the intermediates, ``override``
        is recorded verbatim. ``archive`` runs synchronously once the slot is
        held, so a 409'd concurrent request never archives a half-written grid;
        ``on_success`` runs in the thread after the sidecar (active-map reload).

        Raises ConflictError if this map is already converting; the slot is
        taken here, so there is no check-then-start race.

        True means only that the thread started; the outcome arrives through
        the sidecar, read back as ``grid_status``.
        """
        logger = self._logger

        pcd_path = os.path.join(directory, "map.pcd")
        if not os.path.isfile(pcd_path):
            logger.warning("Skipping gridmap conversion: no map.pcd", map=name)
            return False
        # A programming error (the router's enum narrows it), checked before
        # the slot is taken so it cannot archive a grid and wedge the map.
        if floor_reference not in FLOOR_REFERENCES:
            raise ValueError(f"unknown floor_reference: {floor_reference!r}")

        with self._lock:
            if name in self._active:
                raise ConflictError(
                    f"A gridmap conversion for '{name}' is already running.",
                    code="conversion_running",
                )
            self._active.add(name)

        def _release() -> None:
            with self._lock:
                self._active.discard(name)

        if archive is not None:
            try:
                archive()
            except OSError as exc:
                _release()
                logger.error("Could not archive the gridmap", map=name, error=str(exc))
                raise UpstreamError(f"Could not set the previous gridmap aside: {exc}")

        subdir = self._debug_subdir or (TRAVERSABLE_DEBUG_SUBDIR_NAME if debug else None)
        debug_dir = os.path.join(directory, subdir) if subdir else None
        bands_offsets = {**GRIDMAP_BANDS_ABOVE_FLOOR, **(band_offset_overrides or {})}
        traversable_grid = {**TRAVERSABLE_GRID_RECIPE, **(grid_overrides or {})}

        def _run() -> None:
            bound = logger.bind(map=name)
            basename = os.path.join(directory, "gridmap")
            started_at = iso_now()

            def _record(
                status: GridRecordStatus, **extra: object
            ) -> Dict[str, object]:
                """Build a sidecar payload with the keys every state shares.

                The enum is a ``str`` subclass, so ``json.dumps`` writes it bare.
                """
                payload: Dict[str, object] = {
                    "status": status,
                    "recipe": recipe_request,
                    "started_at": started_at,
                    **extra,
                }
                if override is not None:
                    payload["recipe_override"] = override
                if map_clean is not None:
                    payload["map_clean"] = map_clean
                return payload

            map_clean: Optional[Dict[str, object]] = None

            # Before any work: `interrupted` is derived from this record.
            write_recipe_sidecar(bound, directory, _record(GridRecordStatus.CONVERTING))

            # Every conversion waits, not just a save's: otherwise a re-convert
            # could read the raw cloud, or a different cloud in each pass. The
            # slot stays held, so rename / delete / activate keep refusing with
            # conversion_running (a rename would move the dir under clean_map).
            map_clean = wait_for_map_clean(
                bound,
                directory,
                timeout_s=self._map_clean_timeout_s,
                poll_s=self._map_clean_poll_s,
            )

            try:
                measure = measure_cloud(bound, pcd_path)
                if recipe_request == "traversability":
                    # Imported here, inside the try, on purpose: it pulls in
                    # open3d (~100 MB), which only this branch should pay for,
                    # and an ImportError must reach the handler below.
                    from syncai_backend.helpers.traversable import build_traversable_cloud

                    cloud = build_traversable_cloud(
                        bound,
                        pcd_path,
                        segment=TRAVERSABLE_SEGMENT_RECIPE,
                        repair=TRAVERSABLE_REPAIR_RECIPE,
                        debug_dir=debug_dir,
                    )
                    convert_traversable_to_gridmap(bound, cloud, basename, **traversable_grid)
                    params: Dict[str, object] = {
                        "segment": dict(TRAVERSABLE_SEGMENT_RECIPE),
                        "repair": dict(TRAVERSABLE_REPAIR_RECIPE),
                        "grid": dict(traversable_grid),
                    }
                else:
                    # poses.txt feeds the local floor and the pose-connectivity
                    # filter. Unusable poses degrade with a warning, never fail.
                    poses = None
                    poses_path = os.path.join(directory, "poses.txt")
                    try:
                        poses = read_poses_xyz(poses_path)
                    except (OSError, ValueError) as exc:
                        bound.warning(
                            "converting without the keyframe trajectory: no "
                            "pose-connectivity filter, global floor reference",
                            poses=poses_path,
                            error=str(exc),
                        )
                    reference = floor_reference
                    fallback: Optional[str] = None
                    if reference == "local" and poses is None:
                        reference = "global"
                        fallback = f"poses.txt unreadable: {poses_path}"

                    def _bands(reference: str) -> Dict[str, float]:
                        if reference == "local":
                            # As-is: the helper bands each point against the
                            # floor near it.
                            return dict(bands_offsets)
                        return {
                            key: round(offset + measure.floor_z, 3)
                            for key, offset in bands_offsets.items()
                        }

                    def _convert(reference: str) -> Dict[str, Dict[str, object]]:
                        bands = _bands(reference)
                        bound.info(
                            "z-band recipe bands",
                            floor_reference=reference,
                            floor_z=round(measure.floor_z, 3),
                            **bands,
                        )
                        return convert_pcd_to_gridmap(
                            bound,
                            pcd_path,
                            basename,
                            **GRIDMAP_RECIPE,
                            **bands,
                            pose_seed_xy=poses[:, :2] if poses is not None else None,
                            floor_reference_xyz=poses if reference == "local" else None,
                        )

                    try:
                        pass_stats = _convert(reference)
                    except LocalFloorError as exc:
                        # No keyframe has a floor under it: fall back to the
                        # global level and record why.
                        bound.warning(
                            "local floor reference unusable; converting against "
                            "the global floor level",
                            error=str(exc),
                        )
                        reference = "global"
                        fallback = str(exc)
                        pass_stats = _convert(reference)
                    # floor_z under ``local`` is a drift diagnostic, set against
                    # local_floor's floor_min/max.
                    params = {
                        **GRIDMAP_RECIPE,
                        **_bands(reference),
                        "floor_reference": reference,
                        "floor_z": round(measure.floor_z, 3),
                        **pass_stats,
                    }
                    if fallback is not None:
                        params["floor_reference_fallback"] = fallback
            # ValueError: a helper's diagnosis (empty cloud, no floor, oversized
            # grid). OSError: files gone. RuntimeError: open3d cannot read the cloud.
            except (ValueError, OSError, RuntimeError) as exc:
                hint = (
                    "for the traversability recipe, re-convert through "
                    "POST /api/v1/maps/{name}/grid/convert with debug: true and "
                    "read the intermediate clouds out of the map directory; the "
                    "z-band recipe is the same endpoint with recipe: 'z-band', "
                    "and floor_reference: 'global' skips the per-keyframe floor "
                    "measurement if that is what failed"
                )
                logger.error(
                    "Gridmap conversion failed",
                    map=name,
                    error=str(exc),
                    hint=hint,
                )
                # The log line's diagnosis, where the console can read it.
                write_recipe_sidecar(
                    bound,
                    directory,
                    _record(
                        GridRecordStatus.FAILED,
                        error=str(exc),
                        hint=hint,
                        finished_at=iso_now(),
                    ),
                )
                return
            # An environment fault, not a bad map; uncaught it would be a bare
            # traceback on stderr with no map attached.
            except ImportError as exc:
                hint = "pip3 install -r src/syncai_backend/requirements.txt in the container"
                logger.error(
                    "Gridmap conversion unavailable: open3d is missing",
                    map=name,
                    error=str(exc),
                    hint=hint,
                )
                # Verbatim: re-converting fails identically until the container
                # is fixed.
                write_recipe_sidecar(
                    bound,
                    directory,
                    _record(
                        GridRecordStatus.FAILED,
                        error=(
                            "the traversability recipe needs open3d, which is not "
                            f"installed ({exc})"
                        ),
                        hint=hint,
                        finished_at=iso_now(),
                    ),
                )
                return

            write_recipe_sidecar(
                bound,
                directory,
                _record(
                    GridRecordStatus.OK,
                    footprint_m2=round(measure.footprint_m2, 1),
                    floor_area_m2=round(measure.floor_area_m2, 1),
                    params=params,
                    finished_at=iso_now(),
                ),
            )
            logger.info("Gridmap conversion finished", map=name, recipe=recipe_request)

            if on_success is not None:
                # A failed reload must not read as a failed conversion.
                try:
                    on_success()
                except (ValueError, OSError, RuntimeError) as exc:
                    logger.error(
                        "Gridmap converted but the follow-up failed",
                        map=name,
                        error=str(exc),
                    )

        def _run_and_release() -> None:
            # Any uncaught exception must still free the slot, or the map is
            # unconvertible until a restart.
            try:
                _run()
            finally:
                _release()

        try:
            threading.Thread(
                target=_run_and_release, name=f"pcd-to-gridmap-{name}", daemon=True
            ).start()
        except BaseException:
            _release()
            raise
        return True


def init_gridmap_conversion_service(
    logger: structlog.stdlib.BoundLogger,
    *,
    debug_subdir: Optional[str] = None,
) -> GridmapConversionService:
    return GridmapConversionService(logger=logger, debug_subdir=debug_subdir)
