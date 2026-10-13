"""The pcd -> gridmap conversion: a 3D point-cloud map into a 2D occupancy grid.

In-process replacement for the retired ``tools/pcd_to_gridmap.py`` CLI (in git
history); its ``--preview`` and ``--stats`` flags were not ported.

Two conversions with one output contract (``.pgm`` + ``.yaml`` under one
basename, in the pcd's own frame), both pure numpy/scipy. Keep open3d out of
this module: the map router imports it at startup.

* ``convert_pcd_to_gridmap`` classifies cells by height band:

  occupied : >= ``min_points`` points in the obstacle band [zmin, zmax]
  free     : an observed, unoccupied cell; observed per ``free_mode`` —
             ``floor`` needs points in [floor_zmin, floor_zmax], ``any`` any z,
             ``none`` marks nothing free
  unknown  : everything else

  With the keyframe trajectory (``floor_reference_xyz``), z is height above
  the floor measured near each point (``local_floor_levels``), so a map that
  drifted in z is banded correctly; without it z is the cloud's own.
* ``convert_traversable_to_gridmap`` takes a segmented traversable cloud (from
  ``helpers/traversable.py``) and marks everything it does not cover occupied.

Failures raise ``ValueError`` or propagate ``OSError``.
"""

import os
from typing import Dict, Optional, Tuple, Union

import numpy as np
import structlog
from scipy import ndimage
from scipy.spatial import cKDTree

from syncai_backend.helpers.pgm import write_pgm, write_text_atomic
from syncai_backend.helpers.pointcloud import read_pcd_xyz


def floor_level(z: np.ndarray, bin_size: float = 0.10, relative_peak: float = 0.20) -> float:
    """Estimate the floor's height from a histogram of point z.

    Used by ``helpers.traversable`` (to tell floor from ceiling, both nz≈+1)
    and by the map router (to recentre the z-band recipe's bands).

    Takes the **lowest** bin reaching ``relative_peak`` of the tallest, not the
    tallest: a big flat ceiling can out-sample the floor (one fleet map: 173 k
    ceiling points vs 7 k floor). Works on raw clouds too, since walls spread
    over every bin (raw and flat-only agree within 4 cm on the fleet). Callers
    should log the result, as it is an estimate.
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

    ``pgo/save_maps`` writes one line per keyframe, ``<N>.pcd x y z qw qx qy
    qz``, in trajectory order; orientation is dropped. Blank lines are skipped;
    a malformed line or an empty file raises ``ValueError``.
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

    Its own type so the conversion service can retry with the global floor
    instead of failing, unlike other ValueErrors which are map failures.
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

    Needed when LIO drifts in z (0917_TP1F_test1: ~1 m across a 110 x 117 m
    venue): one global floor level put half the floor in the obstacle band. The
    robot stood on the floor at every keyframe, so measure it there.

    Per keyframe, points within ``radius`` (xy), below the lidar and at most
    ``below`` under it, go through ``floor_level``; staying below the lidar
    excludes ceilings. Fewer than ``min_points`` gives no measurement.

    The median ``pose_z - floor`` is the lidar height (a robot constant,
    0.46-0.48 m on the fleet). It fills unmeasured keyframes and replaces
    measurements more than ``max_deviation`` off (a pit, a level seen through
    glass); a ``smooth``-wide median filter then removes jitter.

    Returns per-keyframe floor heights (``pose_xyz`` order) and sidecar stats;
    ``pose_z_spread`` (p95-p5 of keyframe z) shows whether a map needed this.
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

    The floor is the inverse-distance-weighted mean over the ``neighbours``
    nearest keyframes, not the nearest alone: a corridor driven twice at
    different drift heights would otherwise get a step at the Voronoi seam.

    Returns ``z - floor(x, y)`` as float32, a drop-in for the z column.
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

    A MID360 sees through glass, so floor outside a hall comes out free (5-6%
    of free cells on the conference-hall maps). Free components touched by the
    keyframe trajectory are kept; the rest revert.

    ``grid`` is ``[row, col]`` with row 0 at **min y** (before
    ``_write_gridmap``'s flip); an unflipped .pgm would mirror the poses.

    Reverted cells become 205, not 0: live observations can overwrite
    NO_INFORMATION but never lower LETHAL (costmap_layer.cpp:90), so unknown
    is recoverable. 8-connectivity, as in ``_despeckle``.

    ``seed_radius`` widens each pose to a ``(2r+1)²`` window: 9-16 poses per map
    land on occupied cells within 5-14 cm of free space; 3 cells (15 cm at
    0.05 m) covers that.

    If no pose touches a free component, the grid is returned **unchanged**
    (``applied: 0``) with an error log: the inputs are mismatched, and
    reverting everything would destroy the map.
    """
    free = grid == 254
    labels, component_count = ndimage.label(free, structure=np.ones((3, 3)))
    height, width = grid.shape

    cols = ((pose_xy[:, 0] - origin_xy[0]) / resolution).astype(np.int64)
    rows = ((pose_xy[:, 1] - origin_xy[1]) / resolution).astype(np.int64)
    inside = (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)
    if not inside.all():
        # Not fatal: the extent comes from the banded slices, so a pose can
        # legitimately fall outside it.
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


def _write_gridmap(
    logger: structlog.stdlib.BoundLogger,
    output_basename: str,
    grid: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
) -> None:
    """Write ``grid`` as ``<basename>.pgm`` plus its ``.yaml``, bottom row first.

    Shared by both conversions so their output is identical in form. ``grid``
    has row 0 at **min y**; the flip to pgm order (row 0 = max y) happens only
    here, since a mirrored map localizes near the origin and diverges elsewhere.
    """
    height, width = grid.shape
    os.makedirs(os.path.dirname(output_basename) or ".", exist_ok=True)
    pgm_path = output_basename + ".pgm"
    yaml_path = output_basename + ".yaml"
    # Atomic, and the same header as an edited-and-saved map.
    write_pgm(pgm_path, width, height, np.flipud(grid).tobytes())
    # Hand-formatted, not yaml.dump: image: must stay the relative basename.
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

    The grid is in the pcd's own (SLAM ``map``) frame; the yaml origin is its
    lower-left corner. Defaults are the retired CLI's, not the router's recipe,
    which lives at the call site. ``despeckle_min_size`` /
    ``fill_holes_max_size``: None for off.

    ``pose_seed_xy`` — (N, 2) keyframe xy — enables ``revert_unreachable_free``
    as the final pass. ``floor_reference_xyz`` — (N, 3) keyframes — makes all
    bands offsets from the local floor instead of absolute z.

    Returns the stats of the passes that ran (``pose_filter``,
    ``local_floor``), for the recipe sidecar.
    """
    if free_mode not in ("floor", "any", "none"):
        raise ValueError(f"unknown free_mode: {free_mode!r}")
    if free_mode == "floor" and (floor_zmin is None or floor_zmax is None):
        raise ValueError("free_mode 'floor' needs floor_zmin/floor_zmax")

    # float32 on purpose: every fleet gridmap was built in float32, and float64
    # shifts the origin and some boundary cells, so a re-conversion would not
    # reproduce the map it replaces.
    xyz = read_pcd_xyz(pcd_path).astype(np.float32)
    stats: Dict[str, Dict[str, object]] = {}

    if floor_reference_xyz is not None:
        # Only z changes, so the grid stays registered to the SLAM map frame.
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

    used = np.vstack([obst[:, :2], observed[:, :2]]) if len(observed) else obst[:, :2]
    min_xy = used.min(axis=0) - resolution
    max_xy = used.max(axis=0) + resolution
    width = int(np.ceil((max_xy[0] - min_xy[0]) / resolution))
    height = int(np.ceil((max_xy[1] - min_xy[1]) / resolution))
    # Bounds the cell count, which every pass below is roughly linear in.
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
        # A voxel-downsampled map.pcd samples walls as dashed lines at 0.05 m,
        # and NavFn paths through one-cell holes. Closing bridges gaps up to
        # ~2*r cells without thickening the wall. Keep r small: it also seals
        # real openings under ~2*r (0.2 m at r=2).
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
        # Bridges the gaps between lidar floor rings without growing the outer
        # boundary.
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

    # Must run after _fill_holes, which would otherwise refill the reverted
    # glass-leak pockets as "holes".
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

    The last stage of ``helpers/traversable.py``'s pipeline. The cloud is where
    the robot may drive; **everything else is occupied**, so there are no
    unknown cells. Safe but unforgiving: wrongly rejected floor becomes wall,
    and the ``padding`` ring is solid black. Feed it a repaired ground cloud.

    ``cloud`` is a pcd path or an (N, 3) array (``build_traversable_cloud``'s
    output, keeping this module open3d-free).

    ``gap_fill_size`` (metres), the parameter a site usually tunes, is the
    widest hole bridged by closing. Too small: sparse aisles read as obstacles;
    too large: real obstacles vanish — lower it if the grid loses obstacles.
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
        # Within a cell of the offline tool's cv2.MORPH_ELLIPSE, on scipy alone.
        radius = max(1, int(np.ceil(gap_fill_size / resolution)) // 2)
        closed = ndimage.binary_closing(free_mask, structure=_disk(radius))
        logger.info(
            "traversable gap fill",
            gap_fill_size=gap_fill_size,
            radius=radius,
            added_cells=int(closed.sum() - free_mask.sum()),
        )
        free_mask = closed

    # 0 = occupied, 254 = free, as every .pgm on this stack (not the offline
    # tool's 255).
    grid = np.zeros((height, width), dtype=np.uint8)
    grid[free_mask] = 254
    free_cells = int(free_mask.sum())
    logger.info(
        "traversable gridmap cells",
        free=free_cells,
        occupied=width * height - free_cells,
    )

    _write_gridmap(logger, output_basename, grid, min_xy, resolution)
