"""Convert a nav_msgs/OccupancyGrid into a base64-encoded PNG.

numpy + OpenCV rather than a lighter image library so overlays (laser, pose,
path) can later be drawn on the same image server-side.

No route calls this since 2026-10-28, when the stack went 3D-only and the
backend's 2D gridmap surface was removed; kept as a library.
"""

import base64

import cv2
import numpy as np

from nav_msgs.msg import OccupancyGrid

# nav2 / .pgm greyscale convention.
_UNKNOWN_GREY = 205


def occupancy_grid_to_png(grid: OccupancyGrid) -> np.ndarray:
    """Render the grid as an upright single-channel greyscale image.

    Free (0) -> 255, occupied (100) -> 0, unknown (-1) -> 205, linear in
    between; flipped vertically so row 0 is the top, as in a .pgm / PNG.
    """
    height = grid.info.height
    width = grid.info.width

    data = np.array(grid.data, dtype=np.int16).reshape(height, width)

    img = np.full((height, width), _UNKNOWN_GREY, dtype=np.uint8)
    known = data >= 0
    img[known] = np.round((100 - data[known]) * 255.0 / 100.0).astype(np.uint8)

    return cv2.flip(img, 0)


def occupancy_grid_to_png_base64(grid: OccupancyGrid) -> str:
    """Return the grid as a ``data:image/png;base64,...`` URI for an <img> src."""
    img = occupancy_grid_to_png(grid)

    ok, buffer = cv2.imencode(".png", img)
    if not ok:
        raise ValueError("Failed to encode OccupancyGrid as PNG")

    encoded = base64.b64encode(buffer.tobytes()).decode("ascii")
    return f"data:image/png;base64,{encoded}"
