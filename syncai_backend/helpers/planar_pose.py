"""A 6-DOF pose's heading on the floor.

A waypoint is planar (x, y, theta) and a detected landmark is not: a marker on
a wall comes back with whatever tilt it was mounted and measured with. Its own
module so the router that needs it does not import a gateway's maths.
"""

import math
from typing import Optional

import numpy as np

from syncai_backend.helpers.pointcloud import quat_to_rotation_matrix

# The +x axis must lie within 60 degrees of the floor for its projection to be
# a heading; past that a small measurement error swings the yaw anywhere.
MIN_HORIZONTAL = 0.5


def planar_yaw(x: float, y: float, z: float, w: float) -> Optional[float]:
    """Floor yaw (radians) of a quaternion pose's +x axis; None when it points up or down."""
    return planar_yaw_of_rotation(quat_to_rotation_matrix(x, y, z, w))


def planar_yaw_of_rotation(rotation: np.ndarray) -> Optional[float]:
    """Floor yaw (radians) of a rotation matrix's +x axis; None when it points up or down."""
    # The first column is where the +x axis points in the outer frame.
    ahead_x, ahead_y = float(rotation[0, 0]), float(rotation[1, 0])
    if math.hypot(ahead_x, ahead_y) < MIN_HORIZONTAL:
        return None
    return math.atan2(ahead_y, ahead_x)
