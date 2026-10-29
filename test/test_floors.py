"""helpers/floors.py: storeys found from where the robot walked."""

from syncai_backend.helpers.floors import (
    FLOOR_MIN_POSES,
    detect_floors,
    floor_name,
    read_poses,
)


def _walk(*segments):
    """Poses for a run: ``(level, keyframes, x0, x1)`` per segment, walked
    along x from ``x0`` to ``x1`` at that level, a small bob on each."""
    poses = []
    for level, count, x0, x1 in segments:
        for i in range(count):
            x = x0 + (x1 - x0) * i / max(1, count - 1)
            poses.append((x, 0.0, level + 0.02 * ((i % 5) - 2)))
    return poses


def _slope(z0, z1, count, x0, x1):
    """A slope walked evenly from ``(x0, z0)`` to ``(x1, z1)``."""
    return [
        (x0 + (x1 - x0) * i / (count - 1), 0.0, z0 + (z1 - z0) * i / (count - 1))
        for i in range(count)
    ]


def _levels(poses):
    return [floor.level for floor in detect_floors(poses)]


def test_two_storeys_one_over_the_other_are_two_floors():
    # The shape of a real two-storey run: most of it on each floor, the two
    # over the same ground, a handful of keyframes on each flight and landing.
    poses = _walk(
        (0.0, 900, 0, 40),
        (1.5, 30, 40, 42),
        (3.0, 30, 42, 44),
        (4.2, 30, 44, 46),
        (5.3, 600, 0, 40),
    )

    assert _levels(poses) == [0.0, 5.3]


def test_a_storey_spans_the_heights_walked_on_it_and_not_the_stair():
    poses = _walk((0.0, 900, 0, 40), (1.5, 30, 40, 42), (3.0, 30, 42, 44), (5.3, 600, 0, 40))

    lower, upper = detect_floors(poses)
    assert (lower.low, lower.high) == (-0.04, 0.04)
    assert (upper.low, upper.high) == (5.26, 5.34)


def test_a_long_slope_is_one_floor_from_its_foot_to_its_top():
    # outdoor_1027: down a slope and back, each end walked in its own place.
    # Its two ends are storey heights apart, but nothing is over anything.
    poses = _walk((0.0, 200, 0, 20)) + _slope(0.0, -3.0, 100, 20, 80) + _walk((-3.0, 200, 80, 100))

    floors = detect_floors(poses)
    assert len(floors) == 1
    assert (floors[0].low, floors[0].high) == (-3.04, 0.04)


def test_a_ramp_down_to_a_bay_joins_the_floor_it_leaves_from():
    # dp_1028: a ramp from the ground floor down to a lower bay, walked less
    # than the floor, and a storey over the ground floor. The bay is not
    # under anything, so it is the ground floor's, which keeps its level.
    poses = (
        _walk((0.0, 1200, 0, 40))
        + _slope(0.0, -2.5, 80, 40, 60)
        + _walk((-2.5, 300, 60, 80))
        + _walk((4.0, 600, 0, 40))
    )

    floors = detect_floors(poses)
    assert [floor.level for floor in floors] == [0.0, 4.0]
    assert floors[0].low == -2.54


def test_a_part_of_a_floor_walked_less_still_widens_it():
    # The low end of a sloping deck: under a storey's share, over a stair's.
    poses = _walk((0.0, 900, 0, 40), (4.0, 500, 0, 40), (3.6, 60, 0, 10))

    upper = detect_floors(poses)[1]
    assert upper.level == 4.0
    assert upper.low == 3.56


def test_one_storey_is_one_floor():
    assert _levels(_walk((0.0, 500, 0, 40))) == [0.0]


def test_a_level_on_a_bin_edge_is_still_one_floor():
    # Half the keyframes either side of an edge, both bins kept: one storey.
    poses = [(i * 0.1, 0.0, 0.24) for i in range(100)] + [(i * 0.1, 0.0, 0.26) for i in range(100)]

    assert len(detect_floors(poses)) == 1


def test_levels_closer_than_a_storey_collapse_into_the_one_walked_more():
    poses = _walk((0.0, 300, 0, 40), (1.0, 500, 0, 40))

    assert _levels(poses) == [1.0]


def test_a_short_run_detects_nothing():
    assert detect_floors([(0.0, 0.0, 0.0)] * (FLOOR_MIN_POSES - 1)) == []


def test_floor_names_count_up_from_1f():
    assert [floor_name(i) for i in range(3)] == ["1F", "2F", "3F"]


def test_read_poses_skips_lines_it_cannot_read(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text(
        "0.pcd 0 0 0.1 1 0 0 0\n"
        "\n"
        "1.pcd 0 0 nope 1 0 0 0\n"
        "short line\n"
        "2.pcd 1 2 5.5 1 0 0 0\n"
    )

    assert read_poses(str(path)) == [(0.0, 0.0, 0.1), (1.0, 2.0, 5.5)]
