"""The pcd -> gridmap conversion, as a service the map router drives.

This is the work POST /api/v1/maps and POST /api/v1/maps/{name}/grid/convert
set going, and none of it is HTTP. It lived in the map router until it had
grown a recipe vocabulary, a thread, a process-wide registry and an on-disk
status protocol inside a module whose job is request parsing -- roughly 450
lines that no route could be read without. Worse, the registry was module
state, which the rest of this backend deliberately does not have: every other
collaborator is built in main.py and injected, and a module-level set cannot be
isolated between tests or reached by anything outside that one file.

So it is an object now, constructed once in main.py and handed to the router.
What it owns is the whole answer to "is this map being converted, and how did
the last attempt go": the in-memory registry of running threads, the recipes,
and the sidecar those threads write. The router keeps what it should have kept
all along -- validating a request, and turning a refusal into a status code.

Two halves of the same state, on purpose:

- **the registry** is authoritative only while this process is up, and that is
  not a caveat to work around. A set in memory cannot say anything about a
  conversion whose process is gone.
- **the sidecar** (GRIDMAP_RECIPE_SIDECAR, next to the pgm) is the half that
  survives, which is why it is written before any work starts. A record saying
  `converting` with no thread behind it is how an interrupted conversion is
  detected at all; MapCatalogRepo reads it back and the router's _grid_status
  turns that pairing into `interrupted`.

Nothing here imports open3d. The traversability recipe imports it inside the
thread body, which is load-bearing and explained at the import itself.
"""

import json
import os
import threading
from datetime import datetime, timezone
from typing import Callable, Dict, NamedTuple, Optional

import numpy as np
import structlog

from syncai_backend.exceptions import ConflictError, UpstreamError
from syncai_backend.helpers.pcd_to_gridmap import (
    convert_pcd_to_gridmap,
    convert_traversable_to_gridmap,
    floor_level,
    read_poses_xy,
    write_text_atomic,
)
from syncai_backend.helpers.pointcloud import read_pcd_xyz
from syncai_backend.repositories.map.catalog import (
    GRIDMAP_RECIPE_SIDECAR,
    GridRecordStatus,
)


# POST /api/v1/maps has two recipes. **Every save converts with the z-band one**
# (GRIDMAP_RECIPE below); the traversability one runs only when an operator
# explicitly asks for it through POST /api/v1/maps/{name}/grid/convert. Neither
# is a fallback for the other; they answer different questions and disagree
# about what an unknown cell means, which is exactly why the choice is recorded
# on disk rather than left implicit.
#
# **The default, z-band**, produces a genuine trinary map: walls where obstacles
# were observed, unknown where nothing was. Unknown is *recoverable* —
# costmap_layer.cpp:90 lets the obstacle layer's live observations overwrite a
# NO_INFORMATION master cell, while a LETHAL one can never be lowered, so a map
# that says "unknown" here can still grow as the robot drives and a map that
# says "wall" cannot. It is also cheap to finish by hand in the gridmap editor,
# which is how every gridmap on this fleet was actually made: dp2f still carries
# the pre-edit gridmap_raw.pgm next to the edited gridmap.pgm, and the edit
# lifted its largest connected free region from 89.1% to 94.4% of all free
# cells.
#
# **The opt-in, traversability** (helpers.traversable: segment the floor by
# lidar return intensity, surface normal and height, repair it, then project
# it), is for the site nobody hand-edits — dp1f is 6338 m² of bounding box — and
# sidesteps the z-band's real weakness: classifying a cell by *where* its points
# are in z cannot distinguish a drivable aisle from a kerb top or a ramp. What
# it costs is not small: the output has **no unknown cells**. The input cloud is
# taken as the whole of the drivable world, so every cell it does not cover
# comes out occupied — the padding ring included, and any real floor the
# segmentation wrongly rejected included. That is the safe direction for the
# planner, but it is unforgiving, and by the note above it is also permanent:
# unobserved area walled off this way can never be cleared by driving there.
#
# There used to be an automatic pick between the two, by bounding-box footprint
# against a 3000 m² threshold (LARGE_SITE_AREA_M2, removed 2026-09), and it is
# gone because it failed in the field three saves out of three: a MID360 sees
# through glass, so a 25x32 m conference hall came in at 3128-3512 m² of bbox —
# out-of-hall structure seen 2.5-6.5 m up — and was routed into the
# traversability recipe, whose no-ground-means-occupied inversion turned a
# furniture-occluded floor into a 55-59%-occupied blob with no wall geometry and
# no unknown cells. Measured floor area was considered as the replacement
# metric and rejected as the *decision* input: on this fleet it splits the six
# clouds 171-572 vs 926-1307 m², so any threshold in that gap is calibrated on
# exactly these six clouds, and the next venue (a different ceiling, glass, an
# outdoor lot) has no reason to land on the same side. What decides instead is
# the asymmetry of the failure directions: a wrongly-chosen z-band map is
# coarse but recoverable (unknown cells, hand-editable, drivable-clearable); a
# wrongly-chosen traversability map is permanently walled. So the recoverable
# recipe is the unconditional default and the unforgiving one is a decision a
# person makes. Both areas are still measured and recorded in the sidecar —
# diagnostics, not policy.
#
# The parameters stay empty because the helper's own defaults are now the tuned
# ones (its module docstring carries the measurement that moved them). A site
# that needs them changed needs them changed with the intermediate clouds in
# front of you, not guessed here — dp1f is the live example: it converts, and
# converts well, but its bottom aisle needs a wider gap_fill_size to bridge a
# doorway the floor sampling missed; the re-convert endpoint takes exactly that
# override.
TRAVERSABLE_SEGMENT_RECIPE: Dict[str, object] = {}
TRAVERSABLE_REPAIR_RECIPE: Dict[str, object] = {}
TRAVERSABLE_GRID_RECIPE: Dict[str, object] = {}

# Cell size for the measured-floor-area diagnostic in measure_cloud. 0.5 m
# because a pgo map.pcd is voxel-downsampled: at finer cells the sparse floor
# sampling under-counts (the same six clouds measure 119-998 m² at 0.2 m vs
# 171-1307 m² at 0.5 m), and a driven cell should count as floor even when only
# one return landed in it. Boundary over-count is bounded by perimeter x cell —
# ~30 m² on a 570 m² hall — noise for a diagnostic.
FLOOR_AREA_CELL_M = 0.5

# GRIDMAP_RECIPE_SIDECAR (imported above, named by the catalogue repo) is where
# a conversion records itself. Written next to the pgm because the catalogue
# would otherwise hold maps from two recipes with nothing on disk saying which
# produced what, and the two disagree about the meaning of an unknown cell — a
# map with no unknown cells at all is either a traversability output or a z-band
# output of a fully observed site, and there is no way to tell from the pgm.
# Tiny, so its effect on the size _walk_stats reports is noise.
#
# Since 2026-09 it also carries a ``status``, written at three points in the
# conversion (converting / ok / failed), and **that is what makes a conversion's
# outcome visible at all**. Before it, a failure logged and returned: the map
# came back with grid: null and grid_converting: false, i.e. exactly what a map
# that was never converted looks like, and the reason existed only in
# log/stack/<robot_id>/backend/current. A process-local registry could not fix
# that — the registry dies with the process, so a backend restarted
# mid-conversion left the same silence. The record has to be on disk next to the
# artefact it describes, because that is the only thing that outlives both the
# thread and the process.

# Where the segmentation's intermediate clouds go when a request asks for them
# (``debug: true`` on the re-convert endpoint). Off by default because
# MapCatalogRepo._walk_stats recurses, so five extra clouds would triple the
# size every catalogue card reports for a map. When a site's grid comes out
# wrong — the next outdoor lot, say — the per-request flag is the tuning
# interface: convert with debug, read the intermediates out of the map
# directory, adjust, re-convert.
#
# The process-wide override that used to sit beside this as a reassignable
# module constant is the service's ``debug_subdir`` argument now: same effect,
# but it is set where the service is built instead of by whoever reached in.
TRAVERSABLE_DEBUG_SUBDIR_NAME = "traversable_debug"


# The z-band recipe's non-band parameters. The bands themselves are below,
# because they are no longer constants.
#
# obstacle_close is 1, not the 2 this recipe shipped with. At 2 the 10 cm
# morphological closing sealed a doorway a robot had demonstrably driven through
# — 16 of that map's 212 keyframe poses landed on occupied cells, all within
# 5-14 cm of free space. At 1 that drops to 9, all of them poses that brushed a
# wall, and the walls and pillars survive intact. Raising it back closes real
# gaps at the cost of closing real doors; lower it before raising it.
GRIDMAP_RECIPE = dict(
    free_mode="floor",
    min_points=2,
    obstacle_close=1,
    free_close=5,
    despeckle_min_size=12,
    fill_holes_max_size=20000,
)

# The z-band recipe's bands, as offsets from the cloud's **measured** floor
# level rather than absolute z. These are the numbers every gridmap on the fleet
# before 2026-08 was built with (-0.95 / -0.25 / -0.3 / 1.5 absolute), minus the
# floor level of dp1f, the site they were picked on: -0.66. On dp1f they
# therefore resolve to exactly what they always were.
#
# They are offsets because absolute they were a per-site guess by construction —
# z=0 in a LIO map is the lidar mount height at the mapping start pose, so a
# different mount, a different chassis or a start pose on a slope moves the
# whole band while the floor stays where it is. The measured floor levels on
# this fleet span -0.36 to -0.73, and 37 cm is more than the floor band is wide.
# Held absolute on the shallowest of those sites, the bands put 26 of 212
# keyframe poses in unknown space and 35 on occupied cells — the robot's own
# path, walled off. Recentred, that is 0 and 10. dp1f is unchanged and dp2f
# moves by 4 cm, which is the raw-cloud estimate disagreeing with the
# flat-points one (see pcd_to_gridmap.floor_level) and well inside the band.
GRIDMAP_BANDS_ABOVE_FLOOR = dict(
    floor_zmin=-0.29,
    floor_zmax=0.41,
    zmin=0.36,
    zmax=2.16,
)


class CloudMeasure(NamedTuple):
    """What measure_cloud reads off a map.pcd before a conversion."""

    footprint_m2: float
    floor_area_m2: float
    floor_z: float


def measure_cloud(
    logger: structlog.stdlib.BoundLogger, pcd_path: str
) -> CloudMeasure:
    """Measure the cloud: bbox footprint, covered floor area, floor level.

    This is what remains of pick_recipe (removed 2026-09): the measurements
    survived the decision. The z-band conversion needs ``floor_z`` to recentre
    its bands, and the two areas go into the recipe sidecar as diagnostics —
    the module comment above GRIDMAP_RECIPE records why neither is allowed to
    *choose* the recipe any more. ``floor_area_m2`` counts FLOOR_AREA_CELL_M
    cells covered by points inside the z-band recipe's own floor band, so it
    reads as "the floor area that recipe would call observed"; footprint is the
    raw bbox, kept because comparing the two is what exposes a glass-inflated
    cloud at a glance (conference: 3512 m² of bbox over 572 m² of floor).

    The floor level here is the raw-cloud estimate. The traversability pipeline
    measures its own, from flat points, and ignores this one — the two agree to
    within 4 cm on this fleet, but the flat estimate is the better-founded of
    the two and that pipeline can afford it.

    Reads the cloud, which the conversion then reads again. That is one extra
    pass over ~20 MB inside a background thread that may be about to spend tens
    of seconds in open3d, against the alternative of threading an array through
    two conversions whose input types differ.
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
    """UTC, ISO 8601, ``Z``-suffixed — the timestamp spelling the REST layer uses.

    The same shape ``MapSummaryResponse.modified_at`` is serialised with, so the
    sidecar's timestamps and the catalogue's agree without the client having to
    parse two conventions.
    """
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def write_recipe_sidecar(
    logger: structlog.stdlib.BoundLogger, directory: str, payload: Dict[str, object]
) -> None:
    """Record the state of this map's conversion, next to the gridmap.

    Called three times per conversion — once on entry with ``converting`` and
    once on the way out with ``ok`` or ``failed`` — so each call overwrites the
    last, and the file always describes the most recent attempt rather than
    accumulating history. The one generation that is kept is the previous
    *successful* record, which ``MapCatalogRepo.archive_gridmap`` moves aside to
    ``gridmap_prev.recipe.json`` alongside the grid it describes.

    Best-effort: a sidecar that fails to write must not lose a gridmap that
    converted fine, so this logs and returns rather than raising into the
    conversion thread's handler. The cost of that on the ``converting`` write is
    worth naming — the interrupted-conversion state is derived from it, so a
    conversion whose first write failed and whose process then died reads as a
    map that was never converted. Losing the diagnosis is the acceptable half of
    the trade; losing the grid is not.
    """
    path = os.path.join(directory, GRIDMAP_RECIPE_SIDECAR)
    try:
        # Temp file + os.replace, the same way the yaml beside it is written.
        # This used to be a plain open("w"), the one non-atomic write in the
        # map directory: a listing that landed mid-write on the `failed`
        # record of a re-conversion saw a torn sidecar, which the reader
        # tolerates by returning None -- and _grid_status then fell through to
        # "the grid on disk decides", reporting `ok` for a grid whose rebuild
        # had just failed. A flicker rather than corruption, but a wrong answer
        # from the one place the operator looks.
        write_text_atomic(path, json.dumps(payload, indent=2, sort_keys=True) + "\n")
    except OSError as exc:
        logger.warning("Could not write gridmap recipe sidecar", path=path, error=str(exc))


class GridmapConversionService:
    """Runs pcd -> gridmap conversions and tracks the ones in flight.

    One instance per process, built in main.py. ``debug_subdir`` forces the
    segmentation's intermediate clouds on for *every* conversion; leave it None
    and the per-request ``debug`` flag decides. It is a constructor argument
    rather than the module constant it used to be so that turning it on is a
    wiring decision someone can see, not an attribute somebody reassigned.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        *,
        debug_subdir: Optional[str] = None,
    ):
        self._logger = logger
        self._debug_subdir = debug_subdir

        # Maps with a conversion thread currently running, guarded by the lock.
        # Membership is what refuses a second concurrent conversion of the same
        # map -- two threads writing the same gridmap.pgm would interleave their
        # outputs -- and what the catalogue's `converting` status reads.
        #
        # See the module docstring for why this half of the state is allowed to
        # be this forgetful, and what covers the rest.
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
        band_offset_overrides: Optional[Dict[str, float]] = None,
        grid_overrides: Optional[Dict[str, object]] = None,
        debug: bool = False,
        override: Optional[Dict[str, object]] = None,
        archive: Optional[Callable[[], None]] = None,
        on_success: Optional[Callable[[], None]] = None,
    ) -> bool:
        """Kick off the pcd -> gridmap conversion in the background; report whether.

        A daemon thread rather than the handler's own thread because the conversion
        takes tens of seconds on a large site and POST /api/v1/maps must answer as
        soon as the pcd is on disk. In-process (not a subprocess) since the pipeline
        moved into helpers: the heavy passes are numpy/scipy/open3d, which release
        the GIL, so the FastAPI threadpool keeps serving while it runs. The old
        600 s subprocess timeout went with it — the helpers bound the grid size
        themselves, so the pipeline cannot run away.

        ``recipe_request`` defaults to z-band for every caller — the module comment
        above GRIDMAP_RECIPE records why there is no automatic pick any more. The
        keyword-only extras exist for the re-convert endpoint: ``band_offset_
        overrides`` merges over GRIDMAP_BANDS_ABOVE_FLOOR (still as offsets from the
        measured floor), ``grid_overrides`` over TRAVERSABLE_GRID_RECIPE
        (gap_fill_size), ``debug`` writes the segmentation's intermediate clouds
        into the map directory, ``override`` is recorded verbatim in the sidecar,
        ``archive`` runs synchronously once the slot is held (setting the previous
        grid aside — synchronous so a 409'd concurrent request can never archive a
        half-written grid), and ``on_success`` runs in the thread after the sidecar
        (the active-map reload).

        Raises ConflictError while a conversion for this map is already running.
        Acquisition happens in here, before the thread starts, precisely so there is
        no check-then-start race for the endpoint to lose.

        The return value only says the thread started, never that the grid appeared:
        a segmentation that rejects the whole floor fails *after* this has answered
        True and the route has 200'd. What the caller reports is therefore "started",
        and the outcome arrives through the sidecar the thread writes — which the
        catalogue reads back as ``grid_status``, so an operator who set a conversion
        going watches it there rather than in the log.
        """
        logger = self._logger

        pcd_path = os.path.join(directory, "map.pcd")
        if not os.path.isfile(pcd_path):
            logger.warning("Skipping gridmap conversion: no map.pcd", map=name)
            return False

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
                """Build a sidecar payload, with the keys every state shares.

                The status goes in as the enum member, not ``.value``: it is a
                ``str`` subclass, so ``json.dumps`` writes the bare
                ``"converting"`` / ``"ok"`` / ``"failed"`` the reader expects.
                """
                payload: Dict[str, object] = {
                    "status": status,
                    "recipe": recipe_request,
                    "started_at": started_at,
                    **extra,
                }
                if override is not None:
                    payload["recipe_override"] = override
                return payload

            # Before any work, so that a process that dies mid-conversion leaves a
            # record saying so. This is the write the `interrupted` state is derived
            # from; without it the only trace of an abandoned conversion is a map
            # that looks like it was never converted at all.
            write_recipe_sidecar(bound, directory, _record(GridRecordStatus.CONVERTING))

            try:
                # Measured whichever recipe runs: z-band needs floor_z to place its
                # bands, and both areas go into the sidecar as diagnostics.
                measure = measure_cloud(bound, pcd_path)
                if recipe_request == "traversability":
                    # Imported here, not at module scope, and that is load-bearing:
                    # this would be the only module-level import of
                    # helpers.traversable in the backend, and it pulls in open3d
                    # (~100 MB). At module scope every backend start would pay that
                    # for a conversion that runs only when an operator asks for it.
                    #
                    # Inside the try, not just inside the function: an ImportError
                    # raised above it escapes _run entirely, and the handler below
                    # never sees it. Inside the branch as well, so the default
                    # z-band path never pays the import at all.
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
                    bands = {
                        key: round(offset + measure.floor_z, 3)
                        for key, offset in bands_offsets.items()
                    }
                    bound.info(
                        "z-band recipe bands", floor_z=round(measure.floor_z, 3), **bands
                    )
                    # The pose-connectivity filter needs the keyframe trajectory pgo
                    # writes next to the pcd. Missing or unreadable poses degrade to
                    # an unfiltered conversion with a warning, never to a failed one:
                    # the filter is a cleanup pass, and losing the whole gridmap to a
                    # malformed poses.txt would cost far more than the glass-leak
                    # speckle it removes.
                    pose_xy = None
                    poses_path = os.path.join(directory, "poses.txt")
                    try:
                        pose_xy = read_poses_xy(poses_path)
                    except (OSError, ValueError) as exc:
                        bound.warning(
                            "converting without the pose-connectivity filter",
                            poses=poses_path,
                            error=str(exc),
                        )
                    pose_stats = convert_pcd_to_gridmap(
                        bound,
                        pcd_path,
                        basename,
                        **GRIDMAP_RECIPE,
                        **bands,
                        pose_seed_xy=pose_xy,
                    )
                    params = {
                        **GRIDMAP_RECIPE,
                        **bands,
                        "floor_z": round(measure.floor_z, 3),
                    }
                    if pose_stats is not None:
                        params["pose_filter"] = pose_stats
            # ValueError is a helper's own diagnosis: an empty cloud, an intensity
            # window that selected no ground, no cluster large enough to be a floor,
            # an oversized grid. OSError is the pcd or the map directory going away
            # under it. RuntimeError is open3d's channel for a cloud it cannot read.
            except (ValueError, OSError, RuntimeError) as exc:
                hint = (
                    "for the traversability recipe, re-convert through "
                    "POST /api/v1/maps/{name}/grid/convert with debug: true and "
                    "read the intermediate clouds out of the map directory; the "
                    "z-band recipe is the same endpoint with recipe: 'z-band'"
                )
                logger.error(
                    "Gridmap conversion failed",
                    map=name,
                    error=str(exc),
                    hint=hint,
                )
                # The same diagnosis the log line carries, put where a client can
                # reach it. The log is the record for whoever is on the robot; this
                # is the one the operator console reads back onto the map's card,
                # and until it existed a failed conversion was indistinguishable
                # from a map nobody had converted.
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
            # Separate, and not folded into the tuple above: this one is an
            # environment fault, not a bad map. Without it the ImportError would kill
            # the thread and land as a bare traceback on stderr, where nothing
            # correlates it with the map that was being saved.
            except ImportError as exc:
                hint = "pip3 install -r src/syncai_backend/requirements.txt in the container"
                logger.error(
                    "Gridmap conversion unavailable: open3d is missing",
                    map=name,
                    error=str(exc),
                    hint=hint,
                )
                # Worth the operator seeing verbatim rather than as a generic
                # failure: nothing about this map or its cloud is wrong, and
                # re-converting it will fail identically until the container is
                # fixed. The sentence names the fix.
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
                # Guarded like the conversion itself: a failed active-map reload must
                # not read as a failed conversion — the grid is on disk either way.
                try:
                    on_success()
                except (ValueError, OSError, RuntimeError) as exc:
                    logger.error(
                        "Gridmap converted but the follow-up failed",
                        map=name,
                        error=str(exc),
                    )

        def _run_and_release() -> None:
            # try/finally around the whole body: an exception nothing above caught
            # must still free the slot, or the map is unconvertible until a backend
            # restart — a wedge no log line would explain.
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
