"""Rasterise forbidden zones into the keepout mask ``filter_mask_server`` serves.

A keepout mask is an ordinary map-server yaml + PGM pair (``keepout.yaml`` +
``keepout.pgm``) living beside ``gridmap.*`` in a map directory. The planner's
global costmap runs ``syncai_costmap_2d::KeepoutFilter`` over the OccupancyGrid
``filter_mask_server`` publishes from it, and the operator draws the zones as
polygons in map-frame metres on the console. This module is the step between
the two: polygons in, one byte per cell out, in the same pixel convention as
``gridmap.pgm`` so the two files line up cell for cell.

**Background is 205 (trinary unknown), never 254 (free).** ``KeepoutFilter::
process`` skips a mask cell that decodes to NO_INFORMATION, but a mask cell
that decodes to FREE *overwrites* a costmap cell that is NO_INFORMATION
(``data > old_data || old_data == NO_INFORMATION`` -- same as upstream nav2).
An all-white background would therefore turn every unexplored cell inside the
map's bounding box into plannable free space. All-grey is a true no-op: only
the black (0 -> occupied -> lethal) cells the zones paint change anything. The
nav session's ``costmap_filter_info.launch.py`` writes its blank mask with the
same 205 for the same reason; a clear (no zones) from here produces the same
file it would.

**Pixel convention.** Row 0 is the top of the image, i.e. max y, exactly as
``helpers/pcd_to_gridmap.py`` flips its grid before writing and as the console
maps a click (``lib/map/view.ts``: ``px = (wx - ox) / res``,
``py = height - (wy - oy) / res``). The console snaps every polygon corner to a
cell *centre*, so ``floor`` of those expressions lands on that cell; the yaw in
``origin[2]`` is ignored on both sides. The fill is boundary-inclusive
(``cv2.fillPoly`` on integer vertices paints the edge cells), which is the
conservative direction for a keepout. Polygons partly or wholly off the map are
clipped by the rasteriser, not rejected -- a zone hugging the map edge is
normal, and the geometry check that matters (>= 3 finite points) is the
router's.

Open3d-free on purpose, like ``pcd_to_gridmap.py``: cv2 and numpy are already
imported at startup by the map router.
"""

from typing import Sequence, Tuple

import cv2
import numpy as np

# Trinary "unknown", what map_saver writes for -1 cells and what the launch's
# blank mask is made of. Loads as -1 under negate: 0 / 0.65 / 0.196
# ((255 - 205) / 255 = 0.196 is *not* < free_thresh), which the filter skips.
KEEPOUT_UNKNOWN = 205
# Black: occupied (100) in the OccupancyGrid, lethal in the costmap.
KEEPOUT_OCCUPIED = 0

# Cell indices are clipped to this before the int32 cast. A coordinate of 1e12 m
# is nonsense but must not overflow into a wrapped index that lands *inside* the
# map; anything past this is far outside every real map and fillPoly clips it.
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

    Returns a uint8 array in .pgm row order (row 0 = max y): ``KEEPOUT_UNKNOWN``
    everywhere a zone does not cover, ``KEEPOUT_OCCUPIED`` where one does. An
    empty ``zones`` gives the all-unknown mask, which is how zones are cleared
    -- see the module docstring for why that is not all-free.

    A zone with fewer than three points paints nothing rather than raising;
    validating the polygon is the REST layer's job, and a degenerate one that
    slips through must not take the whole mask with it.
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

    Same seven keys, order and spelling as ``pcd_to_gridmap._write_gridmap``
    writes ``gridmap.yaml`` with, for the same reason it is not ``yaml.dump``:
    ``image:`` must stay the relative basename so map_server resolves it against
    the yaml's own directory and a map rename stays one ``os.rename``. Yaw is
    written as the literal ``0.0`` -- the gridmap's is always 0.0 too, and the
    console ignores it.
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
