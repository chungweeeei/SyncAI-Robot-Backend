"""A 6-DOF pose's heading on the floor.

A waypoint is planar (x, y, theta) and a detected landmark is not: a marker on
a wall comes back with whatever tilt it was mounted and measured with. Its own
module so the router that needs it does not import a gateway's maths.
"""

import math
from typing import Optional

from syncai_backend.helpers.pointcloud import quat_to_rotation_matrix

# The +x axis must lie within 60 degrees of the floor for its projection to be
# a heading; past that a small measurement error swings the yaw anywhere.
MIN_HORIZONTAL = 0.5


def planar_yaw(x: float, y: float, z: float, w: float) -> Optional[float]:
    """Yaw (radians) of the pose's +x axis projected onto the floor.

    None when that axis points too nearly up or down to have one.
    """
    rotation = quat_to_rotation_matrix(x, y, z, w)
    ahead_x, ahead_y = float(rotation[0, 0]), float(rotation[1, 0])
    if math.hypot(ahead_x, ahead_y) < MIN_HORIZONTAL:
        return None
    return math.atan2(ahead_y, ahead_x)
