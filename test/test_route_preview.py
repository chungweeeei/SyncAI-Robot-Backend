"""Tests for helpers/route_preview.py -- what POST /api/v1/route_preview answers.

Pure, so the rules are pinned here once: the router suite only pins that it
hands the gateway's answer through, and the gateway suite that it turns the
planner's result into one of these.
"""

import math

from syncai_backend.helpers.route_preview import (
    MAX_PATH_POINTS,
    planned,
    refused,
    thin_points,
)


def test_a_planned_route_is_reachable_with_its_length():
    preview = planned([(0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (3.0, 4.0, 0.0)])

    assert preview.reachable
    assert preview.points == ((0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (3.0, 4.0, 0.0))
    assert preview.length_m == 7.0
    assert preview.code is None and preview.message == ""


def test_a_stairs_rise_counts_in_the_length():
    # Up a flight: 3 m along, 1.5 m up. The operator compares routes by it,
    # and one that climbs is longer than its footprint.
    preview = planned([(0.0, 0.0, 0.0), (3.0, 0.0, 1.5)])

    assert preview.length_m == round(math.hypot(3.0, 1.5), 2)


def test_the_length_is_the_whole_paths_not_the_thinned_ones():
    # A zigzag thinned to every other pose would read as a straight line.
    zigzag = [(i * 0.1, 0.1 if i % 2 else 0.0, 0.0) for i in range(2 * MAX_PATH_POINTS + 1)]
    preview = planned(zigzag)

    assert len(preview.points) < len(zigzag)
    expected = sum(math.dist(a, b) for a, b in zip(zigzag, zigzag[1:]))
    assert preview.length_m == round(expected, 2)


def test_thinning_keeps_the_goal_an_uneven_stride_would_drop():
    points = [(float(i), 0.0, 0.0) for i in range(3 * MAX_PATH_POINTS + 1)]

    thinned = thin_points(points)

    assert thinned[-1] == (float(3 * MAX_PATH_POINTS), 0.0, 0.0)
    assert len(thinned) <= MAX_PATH_POINTS + 1


def test_thinning_drops_duplicates_but_not_a_step_straight_up():
    # A repeated pose would make a zero-length segment, which blanks the
    # console's ribbon; a point over the last one is a stair, not a repeat.
    thinned = thin_points([(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.2)])

    assert thinned == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2))


def test_no_path_thins_to_nothing():
    assert thin_points([]) == ()


def test_a_refusal_says_why_in_the_task_runners_words():
    preview = refused(223, "goal_in_keepout: the goal lies inside keepout zone 'z1'")

    assert not preview.reachable
    assert preview.points == () and preview.length_m == 0.0
    assert preview.code == "goal_in_keepout"
    # The console translates the task runner's sentences by their prefix, so
    # a preview must open with the same one a failed MOVE would.
    assert preview.message == (
        "Planning failed: the goal is inside a keepout zone "
        "(goal_in_keepout: the goal lies inside keepout zone 'z1')"
    )


def test_a_refusal_with_no_detail_has_no_empty_parentheses():
    assert refused(220, "  ").message == "Planning failed: no map is loaded"


def test_a_code_this_table_does_not_know_keeps_the_planners_detail():
    # The interface may add a code before this table does.
    preview = refused(230, "something new: a reason")

    assert preview.code == "plan_failed"
    assert preview.message == "Planning failed: no path to the goal (something new: a reason)"
