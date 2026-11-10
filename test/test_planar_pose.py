"""Tests for helpers/planar_pose: a 6-DOF pose's heading on the floor."""

import math

import pytest

pytest.importorskip("numpy")

from syncai_backend.helpers.planar_pose import planar_yaw  # noqa: E402


def _quat(roll: float, pitch: float, yaw: float):
    """ZYX Euler angles to (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


@pytest.mark.parametrize("yaw", [0.0, 1.0, math.pi / 2, -2.5, math.pi])
def test_a_level_pose_keeps_its_yaw(yaw):
    assert planar_yaw(*_quat(0.0, 0.0, yaw)) == pytest.approx(yaw)


def test_tilt_does_not_change_the_heading():
    # A marker mounted leaning back and rolled a little still faces where its
    # +x axis points on the floor.
    assert planar_yaw(*_quat(0.2, 0.3, 1.2)) == pytest.approx(1.2)


def test_an_axis_pointing_up_has_no_heading():
    assert planar_yaw(*_quat(0.0, -math.pi / 2, 0.7)) is None
    # 70 degrees off the floor: past the 60 the helper accepts.
    assert planar_yaw(*_quat(0.0, math.radians(70), 0.7)) is None
