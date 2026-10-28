"""Rasterise forbidden zones into a 2D keepout mask (for nav2's KeepoutFilter).

No route calls this since 2026-10-28, when the stack went 3D-only and the
backend's 2D gridmap surface was removed; kept as a library.

The mask is a map-server yaml + PGM pair (``keepout.yaml`` + ``keepout.pgm``)
beside ``gridmap.*``, in the same pixel geometry so the two line up cell for
cell. Zones come in as polygons in map-frame metres.

**Background is 205 (unknown), never 254 (free).** ``KeepoutFilter::process``
skips a NO_INFORMATION mask cell, but a FREE mask cell *overwrites* a
NO_INFORMATION costmap cell (same as upstream nav2), so a white background
would make every unexplored cell plannable. The nav session's blank mask uses
205 for the same reason; clearing all zones here produces the same file.

**Pixel convention.** Row 0 is max y, as ``pcd_to_gridmap.py`` writes and the
console maps a click (``lib/map/view.ts``). The console snaps corners to cell
centres, so ``floor`` lands on that cell; yaw is ignored on both sides. The
fill is boundary-inclusive (conservative for a keepout). Off-map polygons are
clipped, not rejected; geometry validation (>= 3 finite points) is the
router's.

Open3d-free on purpose, like ``pcd_to_gridmap.py``.
"""

from typing import Sequence, Tuple

import cv2
import numpy as np

# Trinary unknown: loads as -1 under negate 0 / 0.65 / 0.196
# ((255 - 205) / 255 = 0.196 is *not* < free_thresh), which the filter skips.
KEEPOUT_UNKNOWN = 205
# Black: occupied (100) in the OccupancyGrid, lethal in the costmap.
KEEPOUT_OCCUPIED = 0

# Clip before the int32 cast so an absurd coordinate cannot wrap into an index
# inside the map; fillPoly clips anything this far out.
_INDEX_CLIP = 2**30

Point = Tuple[float, float]


def rasterize_zones(
    zones: Sequence[Sequence[Point]],
    *,
    width: int,
    height: int,
    resolution: float,
    origin_xy: Point,
) -> np.ndarray:
    """Paint ``zones`` (polygons in map-frame metres) into a ``height x width`` mask.

    Returns uint8 in .pgm row order (row 0 = max y): ``KEEPOUT_OCCUPIED``
    inside a zone, ``KEEPOUT_UNKNOWN`` elsewhere; empty ``zones`` clears the
    mask. A zone with fewer than three points is skipped rather than raising,
    so one degenerate polygon cannot take the whole mask with it.
    """
    mask = np.full((height, width), KEEPOUT_UNKNOWN, dtype=np.uint8)
    ox, oy = origin_xy
    for points in zones:
        if len(points) < 3:
            continue
        xy = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        cols = np.floor((xy[:, 0] - ox) / resolution)
        rows = np.floor(height - (xy[:, 1] - oy) / resolution)
        cells = np.stack([cols, rows], axis=1)
        cells = np.clip(cells, -_INDEX_CLIP, _INDEX_CLIP).astype(np.int32)
        cv2.fillPoly(mask, [cells.reshape(-1, 1, 2)], KEEPOUT_OCCUPIED)
    return mask


def keepout_yaml_text(*, resolution: float, origin_xy: Point, image: str) -> str:
    """The ``keepout.yaml`` for a mask of this geometry, hand-formatted.

    Same keys and layout as ``pcd_to_gridmap._write_gridmap``'s
    ``gridmap.yaml``, and not ``yaml.dump`` for the same reason: ``image:``
    stays a relative basename so a map rename stays one ``os.rename``.
    """
    ox, oy = origin_xy
    return (
        f"image: {image}\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{ox:.6f}, {oy:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
