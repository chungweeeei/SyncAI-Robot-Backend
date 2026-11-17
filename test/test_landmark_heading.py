"""Tests for helpers/landmark_heading: a camera-frame sighting carried into the map."""

import math

import pytest

np = pytest.importorskip("numpy")

from syncai_backend.helpers.landmark_heading import sighting_in_map  # noqa: E402


def _quat_from_matrix(rotation):
    """A rotation matrix as (x, y, z, w)."""
    m = np.asarray(rotation, dtype=float)

    def half_root(value):
        return math.sqrt(max(0.0, value)) / 2.0

    w = half_root(1.0 + m[0, 0] + m[1, 1] + m[2, 2])
    x = math.copysign(half_root(1.0 + m[0, 0] - m[1, 1] - m[2, 2]), m[2, 1] - m[1, 2])
    y = math.copysign(half_root(1.0 - m[0, 0] + m[1, 1] - m[2, 2]), m[0, 2] - m[2, 0])
    z = math.copysign(half_root(1.0 - m[0, 0] - m[1, 1] + m[2, 2]), m[1, 0] - m[0, 1])
    return (x, y, z, w)


def _yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# An optical frame (z ahead, x right, y down) looking along the map's +x.
OPTICAL_IN_MAP = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# A landmark facing the camera: its +x (out of the face) is the optical -z, its +z the optical -y.
LANDMARK_FACING_CAMERA = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])


def _carry(camera_yaw=0.0, camera_at=(2.0, 3.0, 0.1), ahead=1.2, landmark=LANDMARK_FACING_CAMERA,
           scale=1.0):
    camera_rotation = _yaw_matrix(camera_yaw) @ OPTICAL_IN_MAP
    return sighting_in_map(
        (0.0, 0.0, ahead),
        tuple(scale * c for c in _quat_from_matrix(landmark)),
        camera_at,
        tuple(scale * c for c in _quat_from_matrix(camera_rotation)),
        ambiguous=True,
        consecutive=7,
        age=0.03,
        reprojection_error_px=0.2,
    )


def test_a_landmark_straight_ahead_faces_back_at_the_camera():
    sighting = _carry()

    assert (sighting.x, sighting.y, sighting.z) == pytest.approx((3.2, 3.0, 0.1))
    assert abs(sighting.yaw) == pytest.approx(math.pi)
    assert (sighting.camera_x, sighting.camera_y, sighting.camera_z) == (2.0, 3.0, 0.1)


@pytest.mark.parametrize("camera_yaw", [-2.5, -0.7, 0.3, 1.6, 3.0])
def test_the_camera_turning_turns_the_landmark_with_it(camera_yaw):
    sighting = _carry(camera_yaw=camera_yaw)

    assert sighting.x == pytest.approx(2.0 + 1.2 * math.cos(camera_yaw))
    assert sighting.y == pytest.approx(3.0 + 1.2 * math.sin(camera_yaw))
    expected = camera_yaw + math.pi
    assert math.cos(sighting.yaw - expected) == pytest.approx(1.0)


def test_the_detectors_verdict_is_kept():
    sighting = _carry()

    assert sighting.ambiguous is True
    assert (sighting.consecutive, sighting.age, sighting.reprojection_error_px) == (7, 0.03, 0.2)


def test_a_quaternion_that_is_not_unit_length_is_read_as_its_rotation():
    # The localizer has sent a map -> odom quaternion of length 1.03.
    assert _carry(scale=1.03).yaw == pytest.approx(_carry().yaw)
    assert _carry(scale=1.03).x == pytest.approx(_carry().x)


def test_a_landmark_facing_the_ceiling_has_no_heading():
    # Its +x is the optical -y: straight up in the map.
    facing_up = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

    assert _carry(landmark=facing_up).yaw is None


# --- landmark_waypoint: the heading kept, and how far it can be trusted -----------------

from syncai_backend.helpers.landmark_heading import (  # noqa: E402
    MAX_OFF_AXIS_RAD,
    LandmarkSighting,
    landmark_waypoint,
)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _seen(true_yaw, off_axis, mirrored=False, ambiguous=True, error=0.2, distance=1.2,
          landmark=(2.0, -14.0)):
    """A sighting of a landmark facing true_yaw, seen from off_axis off its face's axis.

    The camera stands on the line at true_yaw - off_axis from the landmark, so the line of
    sight is that angle and the mirrored solution faces 2 * line_of_sight - true_yaw.
    """
    line_of_sight = true_yaw - off_axis
    yaw = _wrap(2.0 * line_of_sight - true_yaw) if mirrored else true_yaw
    return LandmarkSighting(
        x=landmark[0],
        y=landmark[1],
        z=0.1,
        yaw=yaw,
        camera_x=landmark[0] + distance * math.cos(line_of_sight),
        camera_y=landmark[1] + distance * math.sin(line_of_sight),
        camera_z=0.09,
        ambiguous=ambiguous,
        consecutive=10,
        age=0.03,
        reprojection_error_px=error,
    )


def test_ambiguous_sightings_take_the_line_of_sight_and_measure_the_angle_off_it():
    true_yaw, off_axis = math.radians(-86.0), math.radians(6.0)

    waypoint = landmark_waypoint([_seen(true_yaw, off_axis), _seen(true_yaw, off_axis, True)])

    assert waypoint.ambiguous
    assert waypoint.yaw == pytest.approx(true_yaw - off_axis)
    assert waypoint.off_axis == pytest.approx(off_axis)
    assert (waypoint.x, waypoint.y, waypoint.z) == pytest.approx((2.0, -14.0, 0.1))
    assert not waypoint.too_far_off_axis


def test_the_mirrored_solution_measures_the_same_angle():
    true_yaw, off_axis = math.radians(30.0), math.radians(7.0)

    straight = landmark_waypoint([_seen(true_yaw, off_axis)])
    mirrored = landmark_waypoint([_seen(true_yaw, off_axis, mirrored=True)])

    assert straight.off_axis == pytest.approx(mirrored.off_axis)
    assert straight.yaw == pytest.approx(mirrored.yaw)


def test_unambiguous_sightings_keep_the_best_fitting_heading():
    true_yaw = math.radians(120.0)

    waypoint = landmark_waypoint([
        _seen(true_yaw + 0.05, math.radians(20.0), ambiguous=False, error=0.4),
        _seen(true_yaw, math.radians(20.0), ambiguous=False, error=0.1),
    ])

    assert not waypoint.ambiguous
    assert waypoint.yaw == pytest.approx(true_yaw)
    # Far off the axis, but the detector resolved the face: nothing to refuse.
    assert not waypoint.too_far_off_axis
    assert waypoint.reprojection_error_px == 0.1


def test_one_ambiguous_sighting_is_enough_to_distrust_the_detectors_heading():
    # A single noisy frame can look decisive; the rest of the run is what the image can tell.
    true_yaw, off_axis = math.radians(-86.0), math.radians(4.0)

    waypoint = landmark_waypoint([
        _seen(true_yaw, off_axis, mirrored=True, ambiguous=False, error=0.05),
        _seen(true_yaw, off_axis, ambiguous=True),
    ])

    assert waypoint.ambiguous
    assert waypoint.yaw == pytest.approx(true_yaw - off_axis)


def test_past_the_bound_off_the_axis_is_too_far():
    true_yaw = math.radians(10.0)

    waypoint = landmark_waypoint([_seen(true_yaw, MAX_OFF_AXIS_RAD + math.radians(1.0))])

    assert waypoint.too_far_off_axis


def test_the_angle_is_judged_on_the_best_fit_not_a_bad_one():
    # robot01, 2026-11-17: fits of 0.23 px and 0.95 px alternating, reading 8 and 17 deg.
    true_yaw = math.radians(-86.0)
    good = _seen(true_yaw, math.radians(8.0), error=0.23)
    bad = _seen(true_yaw, math.radians(8.0), error=0.95)
    bad = LandmarkSighting(**{**bad.__dict__, "yaw": _wrap(bad.yaw + math.radians(9.0))})

    waypoint = landmark_waypoint([bad, good, bad, bad, good])

    assert math.degrees(waypoint.off_axis) == pytest.approx(8.0)
    assert not waypoint.too_far_off_axis
    assert waypoint.reprojection_error_px == 0.23


def test_the_heading_holds_across_the_half_turn():
    true_yaw = math.radians(178.0)

    waypoint = landmark_waypoint([_seen(true_yaw, math.radians(-5.0))])

    assert math.degrees(waypoint.off_axis) == pytest.approx(5.0)
    assert math.cos(waypoint.yaw - math.radians(183.0)) == pytest.approx(1.0)


def test_sightings_without_a_heading_are_left_out_and_none_leaves_no_waypoint():
    facing_up = LandmarkSighting(**{**_seen(0.0, 0.0).__dict__, "yaw": None})

    assert landmark_waypoint([]) is None
    assert landmark_waypoint([facing_up]) is None
    assert landmark_waypoint([facing_up, _seen(0.5, 0.02)]).yaw == pytest.approx(0.48)


def test_stress_the_kept_heading_is_never_off_by_more_than_the_angle_it_reports():
    """10^5 random scenes: any mix of true and mirrored, ambiguous sightings."""
    rng = np.random.default_rng(20261117)
    worst_excess = 0.0
    for _ in range(100_000):
        true_yaw = rng.uniform(-math.pi, math.pi)
        off_axis = rng.uniform(-math.radians(40.0), math.radians(40.0))
        landmark = tuple(rng.uniform(-50.0, 50.0, size=2))
        distance = rng.uniform(0.5, 2.0)
        count = int(rng.integers(1, 6))
        sightings = [
            _seen(true_yaw, off_axis, mirrored=bool(rng.integers(0, 2)),
                  error=float(rng.uniform(0.02, 1.2)), distance=distance, landmark=landmark)
            for _ in range(count)
        ]

        waypoint = landmark_waypoint(sightings)

        heading_error = abs(_wrap(waypoint.yaw - true_yaw))
        worst_excess = max(worst_excess, heading_error - waypoint.off_axis)
        assert waypoint.too_far_off_axis == (abs(off_axis) > MAX_OFF_AXIS_RAD + 1e-9) or (
            abs(abs(off_axis) - MAX_OFF_AXIS_RAD) < 1e-9
        )
        if not waypoint.too_far_off_axis:
            assert heading_error <= MAX_OFF_AXIS_RAD + 1e-9
    assert worst_excess <= 1e-9
