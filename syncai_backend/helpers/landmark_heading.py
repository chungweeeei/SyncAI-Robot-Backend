"""A landmark the camera saw, as plain data on the map, and the waypoint it makes: no ROS, no IO.

A planar marker's pose has two solutions with the same position whose faces
mirror each other about the line of sight (the camera to the marker), each
``off_axis`` from it; at a metre the image cannot tell them apart (the
detector's ``ambiguous``). The line of sight is then the heading kept: wrong
by exactly ``off_axis``, which is measurable without knowing which solution is
true, so it is kept only up to ``MAX_OFF_AXIS_RAD``. Measured on robot01
(2026-11-17): the pairs mirror the line of sight to 0.1 deg, and the
detector's own pose alternates between fits of 0.23 px and 0.95 px whose
``off_axis`` read 8 and 17 deg, hence the best fit's is the one judged.
"""

import math
import statistics
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from syncai_backend.helpers.planar_pose import planar_yaw_of_rotation
from syncai_backend.helpers.pointcloud import quat_to_rotation_matrix

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]

# The line-of-sight heading's error bound: 10 deg moves a staging pose 1.3 m out 0.23 m.
MAX_OFF_AXIS_RAD = math.radians(10.0)


@dataclass(frozen=True)
class LandmarkSighting:
    """One sighting in the map: the landmark, the camera that saw it, the detector's verdict."""

    x: float
    y: float
    z: float
    # Out of the landmark's face, towards a robot in front of it (radians); None facing up/down.
    yaw: Optional[float]
    camera_x: float
    camera_y: float
    camera_z: float
    ambiguous: bool
    consecutive: int
    age: float
    reprojection_error_px: float


def sighting_in_map(
    tag_position: Vector3,
    tag_orientation: Quaternion,
    camera_translation: Vector3,
    camera_rotation: Quaternion,
    *,
    ambiguous: bool,
    consecutive: int,
    age: float,
    reprojection_error_px: float,
) -> LandmarkSighting:
    """A sighting in the camera's frame, carried into the map by map <- camera at its stamp."""
    rotation_map_from_camera = quat_to_rotation_matrix(*camera_rotation)
    camera_in_map = np.asarray(camera_translation, dtype=float)
    # The centre: rotated into the map's axes, then moved to where the camera stood.
    tag_in_map = rotation_map_from_camera @ np.asarray(tag_position, dtype=float) + camera_in_map
    # The face: the camera's orientation followed by the landmark's relative to it.
    rotation_map_from_tag = rotation_map_from_camera @ quat_to_rotation_matrix(*tag_orientation)
    return LandmarkSighting(
        x=float(tag_in_map[0]),
        y=float(tag_in_map[1]),
        z=float(tag_in_map[2]),
        yaw=planar_yaw_of_rotation(rotation_map_from_tag),
        camera_x=float(camera_in_map[0]),
        camera_y=float(camera_in_map[1]),
        camera_z=float(camera_in_map[2]),
        ambiguous=ambiguous,
        consecutive=consecutive,
        age=age,
        reprojection_error_px=reprojection_error_px,
    )


@dataclass(frozen=True)
class LandmarkWaypoint:
    """Where a waypoint for the landmark goes, and how far its heading can be trusted."""

    x: float
    y: float
    z: float
    # Out of the landmark's face (radians): the detector's when no sighting was ambiguous,
    # else the line of sight, off by at most ``off_axis``.
    yaw: float
    ambiguous: bool
    # The best-fitting sighting's angle between the face and the line of sight (radians).
    off_axis: float
    consecutive: int
    age: float
    reprojection_error_px: float

    @property
    def too_far_off_axis(self) -> bool:
        """True when the heading came from the line of sight and may be off past the bound."""
        return self.ambiguous and self.off_axis > MAX_OFF_AXIS_RAD


def landmark_waypoint(sightings: Sequence[LandmarkSighting]) -> Optional[LandmarkWaypoint]:
    """The waypoint a run of sightings of one still landmark makes; None if none has a heading."""
    # A landmark facing up or down has no heading to keep or to judge.
    usable = [sighting for sighting in sightings if sighting.yaw is not None]
    if not usable:
        return None
    # Both solutions share the position, so every sighting's counts.
    x = statistics.median(sighting.x for sighting in usable)
    y = statistics.median(sighting.y for sighting in usable)
    z = statistics.median(sighting.z for sighting in usable)
    camera_x = statistics.median(sighting.camera_x for sighting in usable)
    camera_y = statistics.median(sighting.camera_y for sighting in usable)
    line_of_sight = math.atan2(camera_y - y, camera_x - x)
    # The face is judged on the fit that matches the image best.
    best = min(usable, key=lambda sighting: sighting.reprojection_error_px)
    face_from_sight = best.yaw - line_of_sight
    off_axis = abs(math.atan2(math.sin(face_from_sight), math.cos(face_from_sight)))
    ambiguous = any(sighting.ambiguous for sighting in usable)
    return LandmarkWaypoint(
        x=x,
        y=y,
        z=z,
        yaw=line_of_sight if ambiguous else best.yaw,
        ambiguous=ambiguous,
        off_axis=off_axis,
        consecutive=best.consecutive,
        age=best.age,
        reprojection_error_px=best.reprojection_error_px,
    )
