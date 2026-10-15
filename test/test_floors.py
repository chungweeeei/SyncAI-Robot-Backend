"""helpers/floors.py: storeys found from the heights the robot walked at."""

from syncai_backend.helpers.floors import (
    FLOOR_MIN_POSES,
    detect_floor_levels,
    floor_name,
    read_pose_heights,
)


def _walk(*segments):
    """Heights for a run: ``(level, keyframes)`` per segment, a small bob on each."""
    heights = []
    for level, count in segments:
        heights.extend(level + 0.02 * ((i % 5) - 2) for i in range(count))
    return heights


def test_two_storeys_with_a_stair_between_are_two_floors():
    # The shape of a real two-storey run: most of it on each floor, a
    # handful of keyframes on each flight and landing.
    heights = _walk((0.0, 900), (1.5, 30), (3.0, 30), (4.2, 30), (5.3, 600))

    assert detect_floor_levels(heights) == [0.0, 5.3]


def test_one_storey_is_one_level():
    assert detect_floor_levels(_walk((0.0, 500))) == [0.0]


def test_a_level_on_a_bin_edge_is_still_one_floor():
    # Half the keyframes either side of an edge, both bins kept: one storey.
    heights = [0.24] * 100 + [0.26] * 100

    assert len(detect_floor_levels(heights)) == 1


def test_levels_closer_than_a_storey_collapse_into_the_one_walked_more():
    heights = _walk((0.0, 300), (1.0, 500))

    assert detect_floor_levels(heights) == [1.0]


def test_a_short_run_detects_nothing():
    assert detect_floor_levels([0.0] * (FLOOR_MIN_POSES - 1)) == []


def test_floor_names_count_up_from_1f():
    assert [floor_name(i) for i in range(3)] == ["1F", "2F", "3F"]


def test_read_pose_heights_skips_lines_it_cannot_read(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text(
        "0.pcd 0 0 0.1 1 0 0 0\n"
        "\n"
        "1.pcd 0 0 nope 1 0 0 0\n"
        "short line\n"
        "2.pcd 1 2 5.5 1 0 0 0\n"
    )

    assert read_pose_heights(str(path)) == [0.1, 5.5]
