"""Intensity-based traversability segmentation and ground repair.

The second pcd -> grid recipe. ``helpers/pcd_to_gridmap.py`` slices by height
and cannot tell whether the observed "floor" is drivable (a ramp, a kerb top
and an aisle share a band). This one segments by what the surface *is*,
ported from the offline ``SyncAI-Robot-Pointcloud-ProcessAndConversion`` tool:

  1. ``segment_traversable`` — split into ground / ramp / obstacle by return
     intensity, normal z, and height above the measured floor.
  2. ``repair_ground`` — merge coplanar fragments, fill their holes, stitch
     across gaps and single steps.
  3. ``convert_traversable_to_gridmap`` (in ``pcd_to_gridmap.py``) — project
     the ground; no traversable point means occupied.

**This is the only module that imports open3d (~100 MB); nothing imports it at
startup.** Import it inside the function that needs it. Stage 3 lives in
``pcd_to_gridmap`` to keep that module open3d-free.

Runs only on request (``POST /api/v1/maps/{name}/grid/convert``); see
``GRIDMAP_RECIPE`` in ``services/gridmap_conversion.py`` for why it is not
picked automatically. By hand, ``debug_dir`` shows which stage rejected the
floor:

    from syncai_backend.helpers.traversable import build_traversable_cloud
    from syncai_backend.helpers.pcd_to_gridmap import convert_traversable_to_gridmap
    cloud = build_traversable_cloud(logger, "map/dp2f/map.pcd",
                                    debug_dir="map/dp2f/traversable_debug")
    convert_traversable_to_gridmap(logger, cloud, "map/dp2f/gridmap")

Stage 2's defaults are the offline tool's ``__main__`` values (its documented
recommendation), not its function signature's.

Stage 1's ground-density defaults are **not** the tool's: its 3 neighbours in
0.05 m (~500 points/m²) kept only 6-20% of ground candidates on the fleet's
maps, too few for DBSCAN. Hence radius 0.25, eps 0.50 and
``ground_min_cluster_points`` 200 (was 1000). Failures raise ``ValueError``.
"""

import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import open3d as o3d
import structlog
from scipy import ndimage
from scipy.spatial import Delaunay, QhullError, cKDTree

# Shared numpy code lives in pcd_to_gridmap, which must stay open3d-free.
from syncai_backend.helpers.pcd_to_gridmap import floor_level


# Intensity field names; FAST-LIO writes "intensity", other exporters the rest.
_INTENSITY_KEYS = ("intensity", "scalar_intensity", "i", "reflectivity")


@dataclass
class SegmentedCloud:
    """What ``segment_traversable`` split the map into.

    Only ``ground`` is repaired and projected; ``slope`` and the obstacle
    clouds are diagnostics (see ``debug_dir``).
    """

    ground: o3d.geometry.PointCloud
    slope: o3d.geometry.PointCloud
    obstacle: o3d.geometry.PointCloud
    obstacle_raw: o3d.geometry.PointCloud


def _load_cloud(
    logger: structlog.stdlib.BoundLogger, pcd_path: str, normal_radius: float, normal_max_nn: int
) -> Tuple[o3d.geometry.PointCloud, np.ndarray, np.ndarray]:
    """Read a PCD and return it with per-point normals and intensity.

    Normals are estimated when missing (the FAST-LIO case) **on the returned
    cloud**: the offline tool estimated them on a throwaway copy, and the
    obstacle pass then silently produced an empty cloud.
    """
    tensor_pcd = o3d.t.io.read_point_cloud(pcd_path)
    pcd = tensor_pcd.to_legacy()
    points = np.asarray(pcd.points)
    if len(points) == 0:
        raise ValueError(f"point cloud is empty: {pcd_path}")

    has_normals = pcd.has_normals() and np.any(np.asarray(pcd.normals) != 0)
    if not has_normals:
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=normal_max_nn)
        )
        # PCA normals are sign-ambiguous, which would split the floor at every
        # nz threshold. After this nz ∈ [0, 1], so a ceiling reads nz≈+1 like a
        # floor: never infer "faces up" from nz (hence the height gating), and
        # any ``nz < <negative>`` test is dead.
        pcd.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))
    normals = np.asarray(pcd.normals)

    intensity = None
    for key in _INTENSITY_KEYS:
        if key in tensor_pcd.point:
            intensity = tensor_pcd.point[key].numpy().reshape(-1).astype(np.float64)
            break
    if intensity is None:
        # Not fatal: degrades to normals only, losing the rejection of painted
        # markings and wet patches.
        logger.warning("pcd carries no intensity field; segmenting on normals only", pcd=pcd_path)
        intensity = np.zeros(len(points))

    logger.info(
        "loaded cloud for segmentation",
        pcd=pcd_path,
        points=len(points),
        normals_estimated=not has_normals,
        has_intensity=bool(np.any(intensity)),
    )
    return pcd, normals, intensity


def _clean_ground(
    logger: structlog.stdlib.BoundLogger,
    ground: o3d.geometry.PointCloud,
    *,
    ror_nb_points: int,
    ror_radius: float,
    dbscan_eps: float,
    dbscan_min_points: int,
    min_cluster_points: int,
) -> o3d.geometry.PointCloud:
    """Drop sparse flyers and disconnected crumbs from the raw ground slice.

    A radius filter removes isolated flyers; DBSCAN removes dense islands too
    small to be floor (table tops, ceiling patches). **Every** cluster reaching
    ``min_cluster_points`` is kept, not just the largest: two aisles behind an
    unseen doorway are two clusters.
    """
    if len(ground.points) == 0:
        raise ValueError(
            "the intensity / normal / height gate selected no ground points — check the "
            "logged 'ground height band' against this site's cloud, then the intensity "
            "percentile window and ground_nz"
        )

    _, keep = ground.remove_radius_outlier(nb_points=ror_nb_points, radius=ror_radius)
    dense = ground.select_by_index(keep)
    logger.info(
        "ground radius-outlier filter",
        before=len(ground.points),
        after=len(dense.points),
        nb_points=ror_nb_points,
        radius=ror_radius,
    )
    if len(dense.points) == 0:
        return dense

    labels = np.asarray(
        dense.cluster_dbscan(eps=dbscan_eps, min_points=dbscan_min_points, print_progress=False)
    )
    if labels.size == 0 or labels.max() < 0:
        # All noise to DBSCAN: still return a floor, just unclustered.
        logger.warning("ground DBSCAN found no cluster; keeping the density-filtered cloud")
        return dense

    counts = np.bincount(labels[labels >= 0])
    valid = np.flatnonzero(counts >= min_cluster_points)
    if valid.size == 0:
        raise ValueError(
            f"no ground cluster reaches min_cluster_points={min_cluster_points} "
            f"(largest is {int(counts.max())}) — lower it or widen dbscan_eps"
        )
    final = dense.select_by_index(np.flatnonzero(np.isin(labels, valid)))
    logger.info(
        "ground DBSCAN",
        clusters=int(counts.size),
        kept_clusters=int(valid.size),
        after=len(final.points),
        min_cluster_points=min_cluster_points,
    )
    return final


def _clean_obstacles(
    logger: structlog.stdlib.BoundLogger,
    obstacles: o3d.geometry.PointCloud,
    *,
    floor_z: float,
    ceiling_height: float,
    ceiling_nz: float,
    scatter_knn: int,
    max_scattering: float,
    ror_nb_points: int,
    ror_radius: float,
) -> o3d.geometry.PointCloud:
    r"""Reduce everything-that-is-not-floor to the solid parts a robot can hit.

    1. **Ceilings**: near-horizontal (\|nz\| >= ``ceiling_nz``) *and* above
       ``floor_z + ceiling_height``; either test alone would cut rack tops or
       the floor. Do not go back to a signed ``nz < ceiling_nz`` test: normals
       are aligned to +z, so it removed nothing.
    2. **Foliage**: PCA scattering λ3/Σλ is near 0 on a plane and large in a
       canopy. The expensive pass.
    3. **Motion residue**: a generous radius filter removes people's trails.

    May return empty (a corridor can be all floor); this cloud is diagnostic.
    """
    if len(obstacles.points) == 0:
        return obstacles

    normals = np.asarray(obstacles.normals)
    points = np.asarray(obstacles.points)
    ceiling = (points[:, 2] > floor_z + ceiling_height) & (np.abs(normals[:, 2]) >= ceiling_nz)
    kept = obstacles.select_by_index(np.flatnonzero(~ceiling))
    logger.info(
        "obstacle ceiling cut",
        before=len(obstacles.points),
        after=len(kept.points),
        cut_above_z=round(floor_z + ceiling_height, 2),
        flatness=ceiling_nz,
    )
    if len(kept.points) == 0:
        return kept

    kept.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(knn=scatter_knn))
    # eigvalsh is ascending: column 0 is the normal direction's eigenvalue.
    eigenvalues = np.linalg.eigvalsh(np.asarray(kept.covariances))
    scattering = eigenvalues[:, 0] / (eigenvalues.sum(axis=1) + 1e-8)
    kept = kept.select_by_index(np.flatnonzero(scattering < max_scattering))
    logger.info("obstacle foliage cut", after=len(kept.points), max_scattering=max_scattering)
    if len(kept.points) == 0:
        return kept

    _, keep = kept.remove_radius_outlier(nb_points=ror_nb_points, radius=ror_radius)
    kept = kept.select_by_index(keep)
    logger.info("obstacle radius-outlier filter", after=len(kept.points), radius=ror_radius)
    return kept


def segment_traversable(
    logger: structlog.stdlib.BoundLogger,
    pcd_path: str,
    *,
    intensity_percentiles: Tuple[float, float] = (8.0, 60.0),
    ground_nz: float = 0.99,
    slope_nz: Tuple[float, float] = (0.5, 0.75),
    normal_radius: float = 0.5,
    normal_max_nn: int = 30,
    ground_z_range: Optional[Tuple[float, float]] = None,
    ground_z_margin: Tuple[float, float] = (0.5, 0.5),
    floor_bin_size: float = 0.10,
    floor_relative_peak: float = 0.20,
    ground_ror_nb_points: int = 3,
    ground_ror_radius: float = 0.25,
    ground_dbscan_eps: float = 0.50,
    ground_dbscan_min_points: int = 20,
    ground_min_cluster_points: int = 200,
    obstacle_ceiling_height: float = 1.8,
    obstacle_ceiling_nz: float = 0.9,
    obstacle_scatter_knn: int = 25,
    obstacle_max_scattering: float = 0.06,
    obstacle_ror_nb_points: int = 12,
    obstacle_ror_radius: float = 0.4,
) -> SegmentedCloud:
    """Split a LIO map into traversable ground, ramps and obstacles.

    Ground needs intensity **and** normal **and** height: intensity alone
    accepts same-material walls, the normal alone every flat surface including
    the ceiling; height (``floor_level``) separates floor from ceiling.

    ``ground_z_range`` (absolute) is normally None: the band is then the
    measured floor ± ``ground_z_margin``. Override only when the logged
    ``floor_z`` is wrong.

    ``intensity_percentiles`` is a window over this cloud's non-zero
    intensities, since absolute intensity varies by lidar, surface and range;
    it cuts zero-return noise and retroreflective signage.

    ``ground_nz`` is strict (0.99, ~8°) because stage 2 recovers what it
    rejects and would otherwise stitch across slopes. Ramps (``slope_nz``,
    ~40-60°) are **not** merged into ``ground``: the planar nav stack would
    drive them as flat free space at full speed.
    """
    pcd, normals, intensity = _load_cloud(logger, pcd_path, normal_radius, normal_max_nn)
    nz = normals[:, 2]

    non_zero = intensity[intensity > 0]
    if non_zero.size:
        i_min, i_max = np.percentile(non_zero, list(intensity_percentiles))
        intensity_mask = (intensity >= i_min) & (intensity <= i_max)
        logger.info(
            "intensity window",
            i_min=round(float(i_min), 2),
            i_max=round(float(i_max), 2),
            percentiles=list(intensity_percentiles),
            passed=int(intensity_mask.sum()),
        )
    else:
        intensity_mask = np.ones(len(nz), dtype=bool)

    flat_mask = intensity_mask & (nz >= ground_nz)
    # Measured on flat candidates, not the whole cloud, which walls dominate.
    points = np.asarray(pcd.points)
    if ground_z_range is None:
        if not flat_mask.any():
            raise ValueError(
                "intensity/normal gate selected no flat points — check the intensity "
                "percentile window and ground_nz against this site's cloud"
            )
        floor_z = floor_level(points[flat_mask][:, 2], floor_bin_size, floor_relative_peak)
        z_lo, z_hi = floor_z - ground_z_margin[0], floor_z + ground_z_margin[1]
    else:
        z_lo, z_hi = ground_z_range
        floor_z = 0.5 * (z_lo + z_hi)
    ground_mask = flat_mask & (points[:, 2] >= z_lo) & (points[:, 2] <= z_hi)
    logger.info(
        "ground height band",
        floor_z=round(floor_z, 3),
        z_lo=round(z_lo, 3),
        z_hi=round(z_hi, 3),
        measured=ground_z_range is None,
        flat=int(flat_mask.sum()),
        in_band=int(ground_mask.sum()),
    )
    # Not height-gated: that would cut ramp tops, and `slope` is diagnostic only.
    slope_mask = intensity_mask & (nz >= slope_nz[0]) & (nz < slope_nz[1])
    # Taken before denoising: points a filter drops are noise, not obstacles.
    other_mask = ~(ground_mask | slope_mask)

    obstacle_raw = pcd.select_by_index(np.flatnonzero(other_mask))
    result = SegmentedCloud(
        ground=_clean_ground(
            logger,
            pcd.select_by_index(np.flatnonzero(ground_mask)),
            ror_nb_points=ground_ror_nb_points,
            ror_radius=ground_ror_radius,
            dbscan_eps=ground_dbscan_eps,
            dbscan_min_points=ground_dbscan_min_points,
            min_cluster_points=ground_min_cluster_points,
        ),
        slope=pcd.select_by_index(np.flatnonzero(slope_mask)),
        obstacle=_clean_obstacles(
            logger,
            obstacle_raw,
            floor_z=floor_z,
            ceiling_height=obstacle_ceiling_height,
            ceiling_nz=obstacle_ceiling_nz,
            scatter_knn=obstacle_scatter_knn,
            max_scattering=obstacle_max_scattering,
            ror_nb_points=obstacle_ror_nb_points,
            ror_radius=obstacle_ror_radius,
        ),
        obstacle_raw=obstacle_raw,
    )
    logger.info(
        "segmentation done",
        ground=len(result.ground.points),
        slope=len(result.slope.points),
        obstacle=len(result.obstacle.points),
    )
    return result


def _find_root(parent: dict, node: int) -> int:
    """Union-find root with path compression, iteratively.

    Iterative: thousands of fragments can exceed the recursion limit.
    """
    root = node
    while parent[root] != root:
        root = parent[root]
    while parent[node] != root:
        parent[node], node = root, parent[node]
    return root


def _row_stride(points: np.ndarray, origin: np.ndarray, grid_res: float, dilation: int) -> int:
    """The row pitch that packs a (cell x, cell y) pair into one int64 key.

    Must come from **all** the points compared, not one fragment: keys from
    different strides compare unrelated numbers, silently.
    """
    rows = int((points[:, 1].max() - origin[1]) / grid_res)
    # A cell's dilated key reaches iy + 2 * dilation (see _footprint_keys).
    return rows + 2 * dilation + 1


def _footprint_keys(
    points: np.ndarray, origin: np.ndarray, grid_res: float, dilation: int, stride: int
) -> np.ndarray:
    """The cluster's 2D cell footprint, dilated, as a sorted array of int64 keys.

    Dilation lets fragments that merely touch count as overlapping. Packed
    int64 keys make the dilation one broadcast and the O(clusters²) overlap
    test an ``intersect1d``.
    """
    ix = ((points[:, 0] - origin[0]) / grid_res).astype(np.int64)
    iy = ((points[:, 1] - origin[1]) / grid_res).astype(np.int64)

    offsets = np.arange(-dilation, dilation + 1, dtype=np.int64)
    dx, dy = (mesh.ravel() for mesh in np.meshgrid(offsets, offsets, indexing="ij"))
    # +dilation keeps indices non-negative, so the packing is one-to-one.
    kx = (ix[:, None] + dx[None, :]).ravel() + dilation
    ky = (iy[:, None] + dy[None, :]).ravel() + dilation
    return np.unique(kx * stride + ky)


def _merge_coplanar(
    logger: structlog.stdlib.BoundLogger,
    points: np.ndarray,
    labels: np.ndarray,
    fragments: list,
    *,
    grid_res: float,
    same_plane_z: float,
    dilation: int,
) -> list:
    """Group DBSCAN fragments that are the same physical plane.

    Merged when footprints touch *and* heights match (either alone merges a
    step with its floor, or two level but separate aisles); union-find makes
    it transitive. Height is the median z, robust to points on a wall.
    """
    footprints = {}
    heights = {}
    origin = points.min(axis=0)
    stride = _row_stride(points, origin, grid_res, dilation)
    for fragment in fragments:
        pts = points[labels == fragment]
        heights[fragment] = float(np.median(pts[:, 2]))
        footprints[fragment] = _footprint_keys(pts, origin, grid_res, dilation, stride)

    parent = {fragment: fragment for fragment in fragments}
    for i, first in enumerate(fragments):
        for second in fragments[i + 1:]:
            if abs(heights[first] - heights[second]) >= same_plane_z:
                continue
            if not np.intersect1d(
                footprints[first], footprints[second], assume_unique=True
            ).size:
                continue
            root_a, root_b = _find_root(parent, first), _find_root(parent, second)
            if root_a != root_b:
                parent[root_b] = root_a

    groups = {}
    for fragment in fragments:
        groups.setdefault(_find_root(parent, fragment), []).append(fragment)
    logger.info("merged coplanar fragments", fragments=len(fragments), planes=len(groups))
    return list(groups.values())


def _fill_and_outline(
    points: np.ndarray, grid_res: float, max_cells: int
) -> Tuple[np.ndarray, np.ndarray]:
    """Fill a plane's interior holes and return (filled points, boundary points).

    Only fully enclosed pockets are filled; one reaching the edge may be an
    obstacle footprint. Both outputs sit at the plane's mean height (safe, as
    fragments are within ``same_plane_z``), so the stitching step test does not
    depend on which edge point was sampled.
    """
    min_p = points.min(axis=0)
    span = points.max(axis=0) - min_p
    cols = int(span[0] / grid_res) + 3
    rows = int(span[1] / grid_res) + 3
    if cols * rows > max_cells:
        raise ValueError(
            f"plane rasterises to {cols}x{rows} cells at grid_res={grid_res} — "
            "raise grid_res or check the segmentation for a plane spanning the whole site"
        )

    # +1: a one-cell empty margin, so fill_holes and erosion see background on
    # every side.
    ix = ((points[:, 0] - min_p[0]) / grid_res).astype(np.int64) + 1
    iy = ((points[:, 1] - min_p[1]) / grid_res).astype(np.int64) + 1

    occupied = np.zeros((cols, rows), dtype=bool)
    occupied[ix, iy] = True
    mean_z = float(points[:, 2].mean())

    def to_world(cx: np.ndarray, cy: np.ndarray) -> np.ndarray:
        return np.column_stack(
            [
                min_p[0] + (cx - 1) * grid_res,
                min_p[1] + (cy - 1) * grid_res,
                np.full(cx.size, mean_z),
            ]
        )

    holes = ndimage.binary_fill_holes(occupied) & ~occupied
    filled = to_world(*np.nonzero(holes))
    # Boundary of the repaired plane, not of its holes.
    occupied |= holes
    boundary = to_world(*np.nonzero(occupied & ~ndimage.binary_erosion(occupied)))
    return filled, boundary


def _stitch(
    boundary_a: np.ndarray,
    boundary_b: np.ndarray,
    *,
    max_gap: float,
    density: float,
    rng: np.random.Generator,
) -> Optional[np.ndarray]:
    """Sample points across the gap between two plane boundaries.

    Only boundary points within ``max_gap`` of the other plane take part, and
    only Delaunay triangles with a vertex on each side are kept. Triangles with
    an edge over ``1.5 * max_gap`` are dropped: Delaunay fills the convex hull
    around the planes too.
    """
    tree_b = cKDTree(boundary_b[:, :2])
    near_a = boundary_a[tree_b.query(boundary_a[:, :2], distance_upper_bound=max_gap)[0] < np.inf]
    tree_a = cKDTree(boundary_a[:, :2])
    near_b = boundary_b[tree_a.query(boundary_b[:, :2], distance_upper_bound=max_gap)[0] < np.inf]
    if len(near_a) < 2 or len(near_b) < 2:
        return None

    combined = np.vstack([near_a, near_b])
    try:
        simplices = Delaunay(combined[:, :2]).simplices
    except QhullError:
        # Collinear seam points: skip this seam rather than abort the conversion.
        return None

    corners = combined[simplices]
    from_b = simplices >= len(near_a)
    edges = np.stack(
        [
            np.linalg.norm(corners[:, 1, :2] - corners[:, 0, :2], axis=1),
            np.linalg.norm(corners[:, 2, :2] - corners[:, 1, :2], axis=1),
            np.linalg.norm(corners[:, 0, :2] - corners[:, 2, :2], axis=1),
        ],
        axis=1,
    )
    area = 0.5 * np.linalg.norm(
        np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]), axis=1
    )
    keep = (
        from_b.any(axis=1)
        & (~from_b).any(axis=1)
        & (edges.max(axis=1) <= max_gap * 1.5)
        & (area > 0)
    )
    if not keep.any():
        return None

    corners, area = corners[keep], area[keep]
    counts = np.maximum(1, (area * density).astype(np.int64))
    which = np.repeat(np.arange(len(counts)), counts)
    # Uniform barycentric sampling (reflect u + v > 1 back into the triangle).
    u = rng.random((which.size, 1))
    v = rng.random((which.size, 1))
    outside = (u + v > 1).ravel()
    u[outside] = 1.0 - u[outside]
    v[outside] = 1.0 - v[outside]
    origin_v = corners[which, 0]
    return origin_v + u * (corners[which, 1] - origin_v) + v * (corners[which, 2] - origin_v)


def repair_ground(
    logger: structlog.stdlib.BoundLogger,
    ground: o3d.geometry.PointCloud,
    *,
    grid_res: float = 0.04,
    max_gap: float = 0.80,
    min_step: float = 0.07,
    max_step: float = 0.31,
    cluster_eps: float = 0.08,
    cluster_min_points: int = 60,
    min_fragment_points: int = 70,
    same_plane_z: float = 0.05,
    footprint_dilation: int = 2,
    fill_density: float = 1.5,
    max_plane_cells: int = 200_000_000,
    seed: int = 0,
) -> o3d.geometry.PointCloud:
    """Close the gaps a LIO map leaves in a floor that is continuous in reality.

    * **Holes inside a plane**: enclosed pockets filled.
    * **Seams between coplanar fragments**: stitched after merging.
    * **Single steps**: stitched when the height difference is in
      ``[min_step, max_step]``, so a crossable kerb gets a surface and a deep
      drop does not. ``min_step`` is low (7 cm) because the lidar understates
      step height.

    Returns the input in grey plus generated points in **red** (for eyeballing
    in a viewer), the latter voxel-downsampled to just under the grid pitch.
    Sampling is seeded so a re-conversion reproduces the map.
    """
    points = np.asarray(ground.points)
    if len(points) == 0:
        raise ValueError("ground cloud is empty; nothing to repair")

    original = o3d.geometry.PointCloud(ground)
    if not original.has_colors():
        original.paint_uniform_color([0.6, 0.6, 0.6])

    labels = np.asarray(
        ground.cluster_dbscan(eps=cluster_eps, min_points=cluster_min_points, print_progress=False)
    )
    fragments = [
        label
        for label in range(labels.max() + 1)
        if np.count_nonzero(labels == label) > min_fragment_points
    ]
    if not fragments:
        logger.warning(
            "no plane fragment survived clustering; returning the ground unchanged",
            cluster_eps=cluster_eps,
            cluster_min_points=cluster_min_points,
        )
        return original

    planes = _merge_coplanar(
        logger,
        points,
        labels,
        fragments,
        grid_res=grid_res,
        same_plane_z=same_plane_z,
        dilation=footprint_dilation,
    )

    generated = []
    boundaries = []
    for plane in planes:
        filled, boundary = _fill_and_outline(
            points[np.isin(labels, plane)], grid_res, max_plane_cells
        )
        if len(filled):
            generated.append(filled)
        boundaries.append(boundary)
    logger.info(
        "filled plane interiors",
        planes=len(planes),
        points=int(sum(len(chunk) for chunk in generated)),
    )

    # Points per m², about the plane's own raster density.
    density = fill_density / (grid_res * grid_res)
    rng = np.random.default_rng(seed)
    seams = 0
    for i, boundary_a in enumerate(boundaries):
        if not len(boundary_a):
            continue
        for boundary_b in boundaries[i + 1:]:
            if not len(boundary_b):
                continue
            z_diff = abs(boundary_a[:, 2].mean() - boundary_b[:, 2].mean())
            if not (z_diff < same_plane_z or min_step <= z_diff <= max_step):
                continue
            patch = _stitch(
                boundary_a, boundary_b, max_gap=max_gap, density=density, rng=rng
            )
            if patch is not None:
                generated.append(patch)
                seams += 1
    logger.info("stitched seams", seams=seams)

    if not generated:
        logger.warning("nothing to repair: no interior hole and no stitchable seam")
        return original

    patch_cloud = o3d.geometry.PointCloud()
    patch_cloud.points = o3d.utility.Vector3dVector(np.vstack(generated))
    patch_cloud.paint_uniform_color([1.0, 0.0, 0.0])
    patch_cloud = patch_cloud.voxel_down_sample(voxel_size=grid_res * 0.8)
    logger.info("repair done", generated=len(patch_cloud.points), original=len(original.points))
    return original + patch_cloud


def build_traversable_cloud(
    logger: structlog.stdlib.BoundLogger,
    pcd_path: str,
    *,
    segment: Optional[dict] = None,
    repair: Optional[dict] = None,
    debug_dir: Optional[str] = None,
) -> np.ndarray:
    """Run segmentation then repair, and return the traversable cloud as (N, 3).

    An ndarray, so ``pcd_to_gridmap`` stays open3d-free. ``debug_dir``
    (opt-in, ~4x the map's size) writes the intermediate clouds and the
    repaired ground.
    """
    segmented = segment_traversable(logger, pcd_path, **(segment or {}))
    repaired = repair_ground(logger, segmented.ground, **(repair or {}))

    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)
        for filename, cloud in (
            ("ground.pcd", segmented.ground),
            ("slope.pcd", segmented.slope),
            ("obstacle.pcd", segmented.obstacle),
            ("obstacle_raw.pcd", segmented.obstacle_raw),
            ("ground_repaired.pcd", repaired),
        ):
            if len(cloud.points) == 0:
                continue
            path = os.path.join(debug_dir, filename)
            o3d.io.write_point_cloud(path, cloud)
            logger.info("wrote segmentation debug cloud", path=path, points=len(cloud.points))

    return np.asarray(repaired.points)
