"""Tests for helpers/move_guard.py -- the one rule dispatch and execute_move share.

Pure, so the rule is pinned here once; the router and activity suites only
pin that each of them asks it and turns its answer into their own refusal.
"""

from syncai_backend.helpers.move_guard import (
    CONVERSION_RUNNING,
    MAP_MISMATCH,
    move_refusal,
)


def test_a_job_on_the_loaded_map_may_drive():
    assert move_refusal("lab", "lab", False) is None


def test_an_unknown_map_is_not_refused():
    # MCP, curl and schedules older than TaskMap say nothing about the map;
    # refusing them would break every caller that predates the field.
    assert move_refusal(None, "lab", False) is None


def test_a_job_planned_on_another_map_is_refused_naming_both():
    refusal = move_refusal("warehouse", "lab", False)

    assert refusal.code == MAP_MISMATCH
    assert "'warehouse'" in refusal.message and "'lab'" in refusal.message


def test_no_loaded_map_is_a_mismatch_too():
    refusal = move_refusal("warehouse", None, False)

    assert refusal.code == MAP_MISMATCH
    assert "no map loaded" in refusal.message


def test_a_rebuilding_floor_plan_refuses_even_an_unlabelled_job():
    # The planner's grid is about to be replaced whoever dispatched.
    refusal = move_refusal(None, "lab", True)

    assert refusal.code == CONVERSION_RUNNING


def test_the_mismatch_wins_over_the_rebuild():
    # Switching maps is the fix that applies; waiting for the rebuild is not.
    assert move_refusal("warehouse", "lab", True).code == MAP_MISMATCH
