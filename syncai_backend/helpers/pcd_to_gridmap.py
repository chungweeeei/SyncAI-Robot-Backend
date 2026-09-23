"""The pcd -> gridmap conversion: a 3D point-cloud map into a 2D occupancy grid.

This is the in-process descendant of the retired ``tools/pcd_to_gridmap.py``
CLI (removed 2026-08; it lives on in git history). The map router used to
shell out to it (``sys.executable`` + ``~/robot_ws/tools/pcd_to_gridmap.py``)
after every map save, which coupled the backend to a file outside its own
package — a path that only resolved because the tool happened to be checked
out next to the install space — and meant the conversion's failure modes
arrived as a captured stderr tail instead of a Python exception.

Two of the CLI's affordances were deliberately not ported. ``--preview`` wrote
a PNG the catalogue ignores anyway (``helpers/pgm.py`` explains why).
``--stats`` printed the z histogram used to pick bands on a new site; when a
site needs that again, recover the tool from git history or eyeball the bands
off the 3D view — the backend itself always converts with the fixed recipe in
the map router.

This module holds two conversions, and they answer opposite questions.
``convert_pcd_to_gridmap`` classifies cells by height band — the recipe every
gridmap on the fleet was built with, described below.
``convert_traversable_to_gridmap`` takes an already-segmented traversable cloud
(from ``helpers/traversable.py``) and marks everything it does not cover as
occupied. They share this module because they share an output contract — one
``.pgm`` plus one ``.yaml`` under the same basename, in the pcd's own frame —
and because both are pure numpy/scipy: the segmentation pipeline that feeds the
second one needs open3d, and it stays out of here so the map router's
module-level import does not pull open3d into every backend start.

Cell classification for ``convert_pcd_to_gridmap`` (unchanged from the tool):
  occupied : >= ``min_points`` points inside the obstacle z-band [zmin, zmax]
  free     : an "observed" cell that is not occupied, where observed depends on
             ``free_mode`` — ``floor`` needs points in [floor_zmin, floor_zmax],
             ``any`` counts a point at any z, ``none`` marks nothing free
  unknown  : everything else

What "z" means in those bands is the one thing that has changed since the tool:
given the keyframe trajectory (``floor_reference_xyz``), z is each point's height
above the floor measured *near it* (``local_floor_levels``), so a site whose LIO
map drifted in z is banded correctly end to end. Without the trajectory z is the
cloud's own, as it always was.

Failures raise ``ValueError`` (bad bands, oversized grid) or propagate the
underlying ``OSError`` — the caller decides how to report them.
"""

import os
import tempfile
from typing import Dict, Optional, Tuple, Union

import numpy as np
import structlog
from scipy import ndimage
from scipy.spatial import cKDTree

from syncai_backend.helpers.pgm import write_pgm
from syncai_backend.helpers.pointcloud import read_pcd_xyz


def floor_level(z: np.ndarray, bin_size: float = 0.10, relative_peak: float = 0.20) -> float:
    """Estimate the floor's height from a histogram of point z.

    Lives here rather than next to either caller because both need it and this
    module is the open3d-free one. ``helpers.traversable`` measures the floor to
    tell a floor from a ceiling — aligning normals to +z leaves the two
    indistinguishable, since both come out nz≈+1 — and the map router measures it
    to recentre the z-band recipe's bands, which are otherwise a constant that
    only holds for the robot and start pose they were picked on.

    The **lowest** substantial peak, not the largest: the largest is the floor
    only when the floor is the best-sampled horizontal surface, and a hall with a
    big flat ceiling breaks that (one map on this fleet has 173 k flat points on
    a ceiling 8.4 m up against 7 k on its floor). Taking the lowest bin that
    clears ``relative_peak`` of the tallest survives that as long as the floor is
    *substantial*.

    Works on a raw cloud as well as on pre-filtered flat points, which is what
    lets the z-band recipe use it without estimating normals: a horizontal floor
    concentrates its points into one or two bins while walls spread across every
    bin, so the floor still wins its bin. Measured against the fleet's clouds the
    two agree to within 4 cm (raw vs flat: -0.66/-0.66, -0.73/-0.69,
    -0.36/-0.36). Callers should log the result — this is an estimate, and a site
    where it lands wrong needs to be visible rather than silent.
    """
    finite = z[np.isfinite(z)]
    if finite.size == 0:
        raise ValueError("cannot estimate a floor level from an empty cloud")
    lo, hi = float(finite.min()), float(finite.max())
    bins = max(1, int(np.ceil((hi - lo) / bin_size)))
    hist, edges = np.histogram(finite, bins=bins, range=(lo, lo + bins * bin_size))
    substantial = np.flatnonzero(hist >= relative_peak * hist.max())
    first = int(substantial[0])
    return float(0.5 * (edges[first] + edges[first + 1]))


def read_poses_xyz(path: str) -> np.ndarray:
    """Read the map-frame xyz of every keyframe from a pgo ``poses.txt``.

    Lives here rather than in ``helpers.pointcloud`` because its consumers are
    the two passes below — ``pointcloud`` is the pcd-wire-format module and a
    keyframe pose list is not a point cloud.

    The format is what ``pgo/save_maps`` writes next to ``map.pcd``: one line per
    keyframe, ``<N>.pcd x y z qw qx qy qz``, in trajectory order. Orientation is
    dropped; x/y feed the pose-connectivity filter and z the local floor
    estimate (``local_floor_levels``), which is why this reads three columns
    where its predecessor read two. Blank lines are tolerated (a trailing
    newline is normal); anything else malformed raises ``ValueError`` with the
    line number, and so does an empty file — a poses.txt with no poses means the
    save is broken in a way the caller should hear about rather than silently
    skip.
    """
    rows = []
    with open(path, "r", encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, 1):
            parts = line.split()
            if not parts:
                continue
            if len(parts) < 4:
                raise ValueError(
                    f"{path}:{lineno}: expected '<name> x y z qw qx qy qz', got {line!r}"
                )
            try:
                rows.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                raise ValueError(
                    f"{path}:{lineno}: could not parse x/y/z from {line!r}"
                )
    if not rows:
        raise ValueError(f"no poses in {path}")
    return np.asarray(rows, dtype=np.float64)


def read_poses_xy(path: str) -> np.ndarray:
    """The xy columns of ``read_poses_xyz`` — what the connectivity filter takes."""
    return read_poses_xyz(path)[:, :2]


class LocalFloorError(ValueError):
    """The local floor could not be measured from these poses and this cloud.

    Its own type so the conversion service can tell "the floor reference is
    unusable" (retry with the global one) from every other ValueError the
    conversion raises (an empty obstacle band, an oversized grid), which are
    failures of the map itself.
    """


def local_floor_levels(
    logger: structlog.stdlib.BoundLogger,
    xyz: np.ndarray,
    pose_xyz: np.ndarray,
    *,
    radius: float = 4.0,
    below: float = 3.0,
    min_points: int = 200,
    smooth: int = 5,
    max_deviation: float = 0.5,
) -> Tuple[np.ndarray, Dict[str, object]]:
    """Estimate the floor height under every keyframe, from the cloud around it.

    Exists because of 0917_TP1F_test1 (2026-09): a 110 x 117 m single-storey
    venue whose LIO trajectory drifted a metre in z across the site (keyframe z
    from -0.48 to +0.51, smoothly, no step) and was pulled back at loop closure.
    The z-band recipe placed its bands off ONE floor level for the whole cloud,
    so wherever the map had drifted up by more than the floor band's headroom
    the floor left the band, landed in the obstacle band, and half the venue
    came out as speckle over unknown — while the other half was fine. No single
    band setting fixes that, because the two halves need different ones; the
    floor has to be measured as a function of position, and the keyframe
    trajectory is the natural sample grid: the robot was on the floor at every
    one of them.

    Per keyframe, the points within ``radius`` (xy) that lie below the lidar and
    no more than ``below`` under it go through ``floor_level`` — the same lowest-
    substantial-peak estimate the global measurement uses, on a neighbourhood
    small enough that drift is negligible inside it. Restricting to points below
    the lidar keeps a ceiling from ever being the peak. A keyframe with fewer
    than ``min_points`` such points (a doorway dwell, a pose at the cloud's rim)
    gets no measurement of its own.

    The measured keyframes give the lidar's height above the floor as the median
    of ``pose_z - floor`` — a property of the robot, so it should be one number,
    and on this fleet it is (0.46-0.48 m across three maps). That constant fills
    the unmeasured keyframes and catches the outliers: a measurement more than
    ``max_deviation`` from ``pose_z - height`` has locked onto something that is
    not the floor the robot stands on (a pit seen through a railing, a lower
    level through glass) and is replaced. A ``smooth``-wide median filter along
    the trajectory then removes single-keyframe jitter. Both are safety rails
    around an estimate that is already right almost everywhere, not the
    estimate itself.

    Returns the per-keyframe floor heights (same order as ``pose_xyz``) and the
    stats the sidecar records — including ``pose_z_spread``, the p95-p5 of
    keyframe z, which is the number that says whether a map needed this at all
    (0.03 m on a small hall, 0.8 m on TP1F).
    """
    pose_xyz = np.asarray(pose_xyz, dtype=np.float64)
    if pose_xyz.ndim != 2 or pose_xyz.shape[1] < 3:
        raise LocalFloorError(f"expected (N, 3) poses, got shape {pose_xyz.shape}")
    if len(pose_xyz) == 0:
        raise LocalFloorError("no poses to measure the local floor at")

    tree = cKDTree(xyz[:, :2])
    floors = np.full(len(pose_xyz), np.nan)
    for k, (x, y, z) in enumerate(pose_xyz):
        idx = tree.query_ball_point([x, y], radius)
        if len(idx) < min_points:
            continue
        zz = xyz[idx, 2]
        zz = zz[(zz < z) & (zz > z - below)]
        if len(zz) < min_points:
            continue
        floors[k] = floor_level(zz)

    measured = np.isfinite(floors)
    if not measured.any():
        raise LocalFloorError(
            "no keyframe has enough cloud around it to measure a floor — "
            "poses.txt and map.pcd look mismatched"
        )
    lidar_height = float(np.median(pose_xyz[measured, 2] - floors[measured]))
    expected = pose_xyz[:, 2] - lidar_height
    outlier = measured & (np.abs(floors - expected) > max_deviation)
    floors = np.where(measured & ~outlier, floors, expected)
    if smooth > 1 and len(floors) >= smooth:
        floors = ndimage.median_filter(floors, size=smooth, mode="nearest")

    stats: Dict[str, object] = {
        "keyframes": int(len(pose_xyz)),
        "keyframes_measured": int(measured.sum()),
        "keyframes_filled": int((~measured).sum()),
        "keyframes_replaced": int(outlier.sum()),
        "lidar_height": round(lidar_height, 3),
        "floor_min": round(float(floors.min()), 3),
        "floor_max": round(float(floors.max()), 3),
        "pose_z_spread": round(
            float(np.percentile(pose_xyz[:, 2], 95) - np.percentile(pose_xyz[:, 2], 5)), 3
        ),
        "radius": radius,
    }
    logger.info("local floor", **stats)
    return floors, stats


def flatten_to_local_floor(
    xyz: np.ndarray,
    pose_xy: np.ndarray,
    floors: np.ndarray,
    *,
    neighbours: int = 4,
) -> np.ndarray:
    """Every point's height above the floor **near it**, not above one global floor.

    The floor under a point is the inverse-distance-weighted mean of the floor
    heights at its ``neighbours`` nearest keyframes (from ``local_floor_levels``).
    Weighted over several rather than copied from the nearest one because a
    corridor driven twice at different drift heights would otherwise carry a
    Voronoi seam down its middle, with the floor band jumping by the drift at
    the seam; blending the neighbours turns that into a ramp shorter than the
    band's headroom. Points far from every keyframe (structure seen through
    glass) take the floor of the nearest ones, which is the best available
    guess and is where the pose-connectivity filter takes over anyway.

    Returns ``z - floor(x, y)`` as float32 so the caller can substitute it for
    the cloud's z column and run the band classification unchanged.
    """
    pose_xy = np.asarray(pose_xy, dtype=np.float64)[:, :2]
    floors = np.asarray(floors, dtype=np.float64)
    k = min(neighbours, len(pose_xy))
    dist, nn = cKDTree(pose_xy).query(xyz[:, :2], k=k)
    if k == 1:
        return (xyz[:, 2] - floors[nn]).astype(np.float32)
    weights = 1.0 / (dist + 1e-3)
    floor_xy = (floors[nn] * weights).sum(axis=1) / weights.sum(axis=1)
    return (xyz[:, 2] - floor_xy).astype(np.float32)


def revert_unreachable_free(
    logger: structlog.stdlib.BoundLogger,
    grid: np.ndarray,
    pose_xy: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
    *,
    seed_radius: int = 3,
) -> Tuple[np.ndarray, Dict[str, int]]:
    """Turn free cells unreachable from the driven trajectory back to unknown.

    Exists because of the conference-hall maps (2026-09): a MID360 sees straight
    through glass, so the floor *outside* the hall gets floor-band returns and
    comes out free — on those three maps 5.4-6.5% of all free cells, spread over
    700-1100 speckle components the operator would otherwise erase by hand. The
    robot's own keyframe trajectory is ground truth for "reachable", and pgo
    already writes it next to the pcd, so seeding a connected-component pass from
    it removes exactly that leakage without touching anything the robot could
    actually drive to (free space beyond an undriven doorway stays connected,
    stays free).

    ``grid`` is in this module's internal orientation — ``[row, col]`` with row 0
    at **min y**, i.e. *before* ``_write_gridmap``'s flip — and so are the cell
    coordinates derived from ``pose_xy``. Feeding this a .pgm read back from disk
    without flipping it would mirror every pose across the map's horizontal
    midline.

    Reverted cells become 205 (unknown), not 0 (occupied), and that choice is
    load-bearing: costmap_layer.cpp:90 lets live observations overwrite a
    NO_INFORMATION master cell but never lower a LETHAL one, so unknown is the
    recoverable direction — glass leakage walled off as occupied would be
    permanent. 8-connectivity, matching ``_despeckle``'s labelling, so the two
    passes agree about what a component is.

    ``seed_radius`` widens each pose to a ``(2r+1)²`` cell window before looking
    up components. Not a nicety: the band-recentring measurements in the map
    router record 9-16 keyframe poses per map landing on *occupied* cells within
    5-14 cm of free space (poses that brushed a wall), and a radius-0 seed would
    silently drop the component those poses stand in. 3 cells is 15 cm at the
    fleet's 0.05 m resolution — just past that measured range.

    If no seed window touches any free component at all, the grid comes back
    **unchanged** (``applied: 0``) with an error in the log: poses that disagree
    with the map that badly mean the inputs are mismatched, and reverting every
    free cell to enforce the filter would destroy the map to satisfy a
    post-process. Skipping leaves exactly the behaviour the fleet had before the
    filter existed.
    """
    free = grid == 254
    labels, component_count = ndimage.label(free, structure=np.ones((3, 3)))
    height, width = grid.shape

    cols = ((pose_xy[:, 0] - origin_xy[0]) / resolution).astype(np.int64)
    rows = ((pose_xy[:, 1] - origin_xy[1]) / resolution).astype(np.int64)
    inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    if not inside.all():
        # Out-of-bounds poses are logged, not fatal: the grid's extent comes from
        # the *z-banded* slices, so a pose recorded while the lidar saw nothing in
        # band (a doorway dwell) can legitimately fall outside it.
        logger.warning(
            "pose-filter: poses outside the grid",
            outside=int((~inside).sum()),
            total=len(pose_xy),
        )
    rows, cols = rows[inside], cols[inside]

    seeded: set = set()
    for row, col in zip(rows, cols):
        r0, r1 = max(0, row - seed_radius), min(height, row + seed_radius + 1)
        c0, c1 = max(0, col - seed_radius), min(width, col + seed_radius + 1)
        seeded.update(np.unique(labels[r0:r1, c0:c1]).tolist())
    seeded.discard(0)

    stats = {
        "applied": 0,
        "poses": int(len(pose_xy)),
        "poses_in_grid": int(len(rows)),
        "components_total": int(component_count),
        "components_kept": len(seeded),
        "reverted_free_cells": 0,
    }
    if not seeded:
        logger.error(
            "pose-filter: no pose touches any free component; leaving the grid "
            "unfiltered — poses.txt and the grid look mismatched",
            poses_in_grid=len(rows),
            components_total=int(component_count),
        )
        return grid, stats

    keep = np.isin(labels, list(seeded))
    reverted = free & ~keep
    grid = grid.copy()
    grid[reverted] = 205
    stats["applied"] = 1
    stats["reverted_free_cells"] = int(reverted.sum())
    logger.info("pose-filter", **stats)
    return grid, stats


def _disk(radius: int) -> np.ndarray:
    """A disk-shaped structuring element for the morphological passes."""
    yy, xx = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    return (xx * xx + yy * yy) <= radius * radius


def _despeckle(
    logger: structlog.stdlib.BoundLogger, grid: np.ndarray, min_obstacle_size: int
) -> np.ndarray:
    """Remove occupied blobs smaller than ``min_obstacle_size`` cells (sensor
    noise, dynamic-object ghosting). Real walls/racks form large connected components
    and are untouched. Removed cells become free (they sit in observed space)."""
    occ = grid == 0
    labels, _ = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    small = sizes < min_obstacle_size
    small[0] = False
    removed = small[labels]
    grid = grid.copy()
    grid[removed] = 254
    logger.info(
        "despeckle",
        removed_cells=int(removed.sum()),
        removed_blobs=int(small.sum()),
        min_obstacle_size=min_obstacle_size,
    )
    return grid


def _fill_holes(
    logger: structlog.stdlib.BoundLogger, grid: np.ndarray, max_hole_size: int
) -> np.ndarray:
    """Turn small unknown pockets fully enclosed by free space into free
    (lidar ring-gap arcs). Unknown regions touching the border or bounded by
    obstacles (e.g. inside racks) are preserved."""
    unk = grid == 205
    labels, n = ndimage.label(unk, structure=np.ones((3, 3)))
    grid = grid.copy()
    border_labels = set(
        np.unique(np.concatenate([labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]]))
    ) - {0}
    filled_cells = 0
    filled_blobs = 0
    for lab in range(1, n + 1):
        if lab in border_labels:
            continue
        mask = labels == lab
        size = int(mask.sum())
        if size > max_hole_size:
            continue
        ring = ndimage.binary_dilation(mask, np.ones((3, 3))) & ~mask
        neigh = grid[ring]
        # Fill only if the pocket is (almost) entirely surrounded by free.
        if (neigh == 0).sum() <= 0.05 * len(neigh):
            grid[mask] = 254
            filled_cells += size
            filled_blobs += 1
    logger.info(
        "fill-holes",
        filled_cells=filled_cells,
        filled_blobs=filled_blobs,
        max_hole_size=max_hole_size,
    )
    return grid


def write_text_atomic(path: str, content: str) -> None:
    """Write a small text file via a temp file + rename, like ``write_pgm``.

    Same reader-race rationale: the catalogue parses gridmap.yaml (and the
    recipe sidecar) on every ``GET /api/v1/maps``, and a listing that lands
    mid-write would see a torn file. For the yaml that degrades the map to
    ``grid: None``; for the sidecar it makes a failed re-conversion read as
    ``ok`` for the width of the write. The yaml is written *after* the .pgm by
    the caller, so a reader that sees the yaml always finds the pgm it names.
    """
    directory = os.path.dirname(path) or "."
    handle = tempfile.NamedTemporaryFile(
        mode="w", dir=directory, prefix=".gridmap-", suffix=".tmp", delete=False
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


def _write_gridmap(
    logger: structlog.stdlib.BoundLogger,
    output_basename: str,
    grid: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
) -> None:
    """Write ``grid`` as ``<basename>.pgm`` plus its ``.yaml``, bottom row first.

    Shared by both conversions in this module so a map is indistinguishable
    whichever recipe produced it — same header, same yaml key order, same
    ``origin`` spelling. ``grid`` is indexed ``[row, col]`` with row 0 at
    **min y**; the flip to pgm order (row 0 = top = max y) happens here, in one
    place, because getting it wrong hands nav2 a mirrored map that localizes
    fine near the origin and diverges across the site.
    """
    height, width = grid.shape
    os.makedirs(os.path.dirname(output_basename) or ".", exist_ok=True)
    pgm_path = output_basename + ".pgm"
    yaml_path = output_basename + ".yaml"
    # write_pgm rather than a bare open(.., "wb"): the catalogue and map_server
    # can read a freshly converted map at any moment, and this also keeps the
    # header byte-identical to an edited-and-saved one.
    write_pgm(pgm_path, width, height, np.flipud(grid).tobytes())
    # Hand-formatted, not yaml.dump — same reason MapCatalogRepo.write_gridmap
    # never touches this file: image: must stay the relative basename.
    write_text_atomic(
        yaml_path,
        f"image: {os.path.basename(pgm_path)}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_xy[0]:.6f}, {origin_xy[1]:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n",
    )
    logger.info("wrote gridmap", pgm=pgm_path, yaml=yaml_path)


def convert_pcd_to_gridmap(
    logger: structlog.stdlib.BoundLogger,
    pcd_path: str,
    output_basename: str,
    *,
    resolution: float = 0.05,
    zmin: float = 0.0,
    zmax: float = 1.5,
    free_mode: str = "any",
    floor_zmin: Optional[float] = None,
    floor_zmax: Optional[float] = None,
    min_points: int = 2,
    min_floor_points: int = 1,
    obstacle_close: int = 0,
    free_close: int = 0,
    despeckle_min_size: Optional[int] = None,
    fill_holes_max_size: Optional[int] = None,
    pose_seed_xy: Optional[np.ndarray] = None,
    floor_reference_xyz: Optional[np.ndarray] = None,
) -> Dict[str, Dict[str, object]]:
    """Convert a 3D point-cloud map into a 2D occupancy grid (pgm + yaml).

    Writes ``<output_basename>.pgm`` and ``<output_basename>.yaml``. The grid
    is expressed in the SAME frame as the pcd (the SLAM ``map`` frame), so
    localization TF lines up with the produced map without extra alignment:
    the yaml's origin is simply the grid's lower-left corner in pcd
    coordinates.

    Parameters mirror the retired CLI's flags; defaults are its defaults, not
    the router's recipe — the recipe stays at the call site where its z-band
    rationale lives. ``despeckle_min_size`` / ``fill_holes_max_size`` are None
    for off, replacing the CLI's flag-plus-size pairs.

    ``pose_seed_xy`` — (N, 2) map-frame keyframe positions (see
    ``read_poses_xy``) — enables ``revert_unreachable_free`` as the final pass.
    The CLI had no such flag; it postdates the CLI by a year of glass-walled
    venues.

    ``floor_reference_xyz`` — (N, 3) keyframe positions (``read_poses_xyz``) —
    makes the bands **relative to the local floor**: the cloud's z is replaced
    by its height above the floor measured around the nearest keyframes
    (``local_floor_levels`` + ``flatten_to_local_floor``) before any band is
    applied, so ``zmin``/``zmax``/``floor_zmin``/``floor_zmax`` are then
    offsets from the floor wherever the point is, not absolute z. Without it the
    bands are absolute and the output is bit-identical to what this function
    always produced.

    Returns the stats of the passes that ran, keyed ``pose_filter`` and
    ``local_floor``; an empty dict when neither was enabled. Callers spread it
    into the recipe sidecar.
    """
    if free_mode not in ("floor", "any", "none"):
        raise ValueError(f"unknown free_mode: {free_mode!r}")
    if free_mode == "floor" and (floor_zmin is None or floor_zmax is None):
        raise ValueError("free_mode 'floor' needs floor_zmin/floor_zmax")

    # float32, not read_pcd_xyz's float64: the pcd stores float32 fields, so
    # the wider type adds no information — and the retired CLI computed in
    # float32, so this is what kept the port bit-identical to it on the same
    # input (verified on map/dp1f: float64 shifts the origin by 1e-6 m and a
    # handful of boundary cells with it). Every gridmap on the fleet was
    # produced with float32 arithmetic; keep it that way so a re-conversion
    # reproduces the map it replaces.
    xyz = read_pcd_xyz(pcd_path).astype(np.float32)
    stats: Dict[str, Dict[str, object]] = {}

    if floor_reference_xyz is not None:
        # Only z changes. xy stays the pcd's own, so the grid's origin and
        # extent — and therefore its registration against the SLAM map frame —
        # are exactly what the absolute-band conversion would have produced.
        poses = np.asarray(floor_reference_xyz, dtype=np.float64)
        floors, stats["local_floor"] = local_floor_levels(logger, xyz, poses)
        xyz = xyz.copy()
        xyz[:, 2] = flatten_to_local_floor(xyz, poses[:, :2], floors)

    obst = xyz[(xyz[:, 2] >= zmin) & (xyz[:, 2] <= zmax)]
    if free_mode == "floor":
        observed = xyz[(xyz[:, 2] >= floor_zmin) & (xyz[:, 2] <= floor_zmax)]
    elif free_mode == "any":
        observed = xyz
    else:
        observed = xyz[:0]

    if len(obst) == 0:
        raise ValueError(
            "no points in obstacle z-band — check zmin/zmax against the site's floor height"
        )
    logger.info(
        "converting pcd to gridmap",
        pcd=pcd_path,
        obstacle_points=len(obst),
        observed_points=len(observed),
        free_mode=free_mode,
    )

    # Grid bounds from the union of both slices, small padding.
    used = np.vstack([obst[:, :2], observed[:, :2]]) if len(observed) else obst[:, :2]
    min_xy = used.min(axis=0) - resolution
    max_xy = used.max(axis=0) + resolution
    width = int(np.ceil((max_xy[0] - min_xy[0]) / resolution))
    height = int(np.ceil((max_xy[1] - min_xy[1]) / resolution))
    # Also the reason this is safe to run in-process without the subprocess
    # timeout the router used to set: every pass below is linear-ish in the
    # cell count, and the cell count is bounded right here.
    if width * height > 200_000_000:
        raise ValueError(f"grid {width}x{height} too large — wrong bands or resolution?")
    logger.info(
        "gridmap geometry",
        width=width,
        height=height,
        resolution=resolution,
        origin_x=round(float(min_xy[0]), 3),
        origin_y=round(float(min_xy[1]), 3),
    )

    def bincount2d(pts: np.ndarray) -> np.ndarray:
        ix = ((pts[:, 0] - min_xy[0]) / resolution).astype(np.int64).clip(0, width - 1)
        iy = ((pts[:, 1] - min_xy[1]) / resolution).astype(np.int64).clip(0, height - 1)
        return np.bincount(iy * width + ix, minlength=width * height).reshape(height, width)

    obst_cnt = bincount2d(obst)
    obs_cnt = bincount2d(observed) if len(observed) else np.zeros((height, width), np.int64)

    occ_mask = obst_cnt >= min_points
    if obstacle_close > 0:
        # A pgo map.pcd is voxel-downsampled (LIO scan_resolution), so at 0.05 m
        # cells a wall is sampled as a DASHED line: on the dp1f map the median
        # hit cell holds 2 points and only half the cells hit at all. Dashes are
        # worse than a thick wall — NavFn happily threads a path through a
        # one-cell hole, so the planner returns paths straight through walls.
        # Closing (dilate then erode by the same disk) bridges gaps up to ~2*r
        # cells along the wall without thickening it, because the erode undoes
        # the dilation everywhere the gap was not filled.
        #
        # Keep r small: the same operation also seals real openings narrower
        # than ~2*r cells. At r=2 (0.05 m/px) that is 0.2 m, well below any
        # doorway the robot could drive through anyway.
        closed = ndimage.binary_closing(occ_mask, structure=_disk(obstacle_close))
        logger.info(
            "obstacle-close",
            radius=obstacle_close,
            added_cells=int(closed.sum() - occ_mask.sum()),
        )
        occ_mask = closed

    # nav2 pgm convention: 0=occupied(black), 254=free(white), 205=unknown(gray)
    grid = np.full((height, width), 205, dtype=np.uint8)
    free_mask = obs_cnt >= min_floor_points
    if free_close > 0:
        # The lidar samples the floor as sparse rings, so per-cell floor hits
        # are speckled. A morphological closing bridges those sub-radius gaps
        # into a solid drivable area WITHOUT growing the outer boundary — so
        # unknown outside the walls stays unknown.
        closed = ndimage.binary_closing(free_mask, structure=_disk(free_close))
        logger.info(
            "free-close",
            radius=free_close,
            added_cells=int(closed.sum() - free_mask.sum()),
        )
        free_mask = closed
    grid[free_mask] = 254
    grid[occ_mask] = 0

    if despeckle_min_size is not None:
        grid = _despeckle(logger, grid, despeckle_min_size)
    if fill_holes_max_size is not None:
        grid = _fill_holes(logger, grid, fill_holes_max_size)

    # Last, after _despeckle and _fill_holes, and the ordering is load-bearing:
    # _fill_holes turns small enclosed unknown pockets back into free, so run
    # before it the filter's reverted cells would be resurrected as exactly the
    # leakage it just removed (a glass-leak blob ringed by free is precisely a
    # "hole" to that pass).
    if pose_seed_xy is not None:
        grid, pose_stats = revert_unreachable_free(
            logger, grid, np.asarray(pose_seed_xy, dtype=np.float64), min_xy, resolution
        )
        stats["pose_filter"] = dict(pose_stats)

    occ = int((grid == 0).sum())
    fre = int((grid == 254).sum())
    logger.info(
        "gridmap cells",
        occupied=occ,
        free=fre,
        unknown=width * height - occ - fre,
    )

    _write_gridmap(logger, output_basename, grid, min_xy, resolution)
    return stats


def convert_traversable_to_gridmap(
    logger: structlog.stdlib.BoundLogger,
    cloud: Union[str, np.ndarray],
    output_basename: str,
    *,
    resolution: float = 0.05,
    padding: float = 1.0,
    gap_fill_size: float = 0.60,
) -> None:
    """Project an already-segmented traversable cloud into a 2D grid.

    The counterpart to ``convert_pcd_to_gridmap`` and the third stage of the
    pipeline in ``helpers/traversable.py`` (read that module's docstring for why
    there are two recipes). It inverts this one's question: rather than asking
    which cells hold an obstacle, it takes the input cloud as the definitive
    statement of where the robot may drive and marks **everything else
    occupied**.

    That inversion is why the output has no unknown cells. It is the safe
    direction — unobserved area comes out as wall rather than as free space the
    planner will route through — but it is also unforgiving: area the
    segmentation wrongly rejected becomes a wall the robot will not cross, and
    the ``padding`` ring around the map is solid black by construction. Feed
    this a repaired ground cloud, not a raw floor slice.

    ``cloud`` is a path to a pcd or an (N, 3) array — the latter is what
    ``build_traversable_cloud`` returns, and taking it keeps this module free of
    open3d.

    ``gap_fill_size`` (metres) is the one parameter a site normally needs tuned,
    and the offline tool's ReadMe says so. It is the widest hole in the input
    cloud that gets bridged, applied as a morphological closing. Too small and a
    sparsely sampled aisle reads as a field of obstacles; too large and a real
    obstacle standing in the middle of the floor gets closed over and
    disappears — which is the failure to watch for, so lower it if the grid
    loses obstacles the pcd clearly shows.
    """
    xyz = read_pcd_xyz(cloud).astype(np.float32) if isinstance(cloud, str) else cloud
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] < 2:
        raise ValueError(f"expected an (N, 3) cloud, got shape {xyz.shape}")
    if len(xyz) == 0:
        raise ValueError("traversable cloud is empty — the segmentation rejected everything")

    min_xy = xyz[:, :2].min(axis=0) - padding
    max_xy = xyz[:, :2].max(axis=0) + padding
    width = int(np.ceil((max_xy[0] - min_xy[0]) / resolution))
    height = int(np.ceil((max_xy[1] - min_xy[1]) / resolution))
    if width * height > 200_000_000:
        raise ValueError(f"grid {width}x{height} too large — wrong resolution?")
    logger.info(
        "traversable gridmap geometry",
        points=len(xyz),
        width=width,
        height=height,
        resolution=resolution,
        origin_x=round(float(min_xy[0]), 3),
        origin_y=round(float(min_xy[1]), 3),
    )

    ix = ((xyz[:, 0] - min_xy[0]) / resolution).astype(np.int64).clip(0, width - 1)
    iy = ((xyz[:, 1] - min_xy[1]) / resolution).astype(np.int64).clip(0, height - 1)
    free_mask = np.zeros((height, width), dtype=bool)
    free_mask[iy, ix] = True

    if gap_fill_size > 0:
        # A disk of this radius, not the offline tool's cv2.MORPH_ELLIPSE of
        # diameter ceil(gap_fill_size/resolution): the two are the same
        # structuring element to within a cell, and _disk keeps this module on
        # scipy alone. opencv is in the backend's requirements, but only the PNG
        # encoder in helpers/pgm.py needs it and this pass has no reason to
        # widen that.
        radius = max(1, int(np.ceil(gap_fill_size / resolution)) // 2)
        closed = ndimage.binary_closing(free_mask, structure=_disk(radius))
        logger.info(
            "traversable gap fill",
            gap_fill_size=gap_fill_size,
            radius=radius,
            added_cells=int(closed.sum() - free_mask.sum()),
        )
        free_mask = closed

    # Same nav2 pgm convention as the z-band recipe, so the gridmap editor, the
    # thumbnail renderer and syncai_map_server all read this map unchanged: 0 =
    # occupied, 254 = free. 254 and not the offline tool's 255 — 255 is what
    # helpers/occupancy_grid.py renders a *free* OccupancyGrid cell as, but every
    # .pgm on this stack writes 254, and read-modify-write through the editor
    # would otherwise rewrite the values anyway.
    grid = np.zeros((height, width), dtype=np.uint8)
    grid[free_mask] = 254
    free_cells = int(free_mask.sum())
    logger.info(
        "traversable gridmap cells",
        free=free_cells,
        occupied=width * height - free_cells,
    )

    _write_gridmap(logger, output_basename, grid, min_xy, resolution)
