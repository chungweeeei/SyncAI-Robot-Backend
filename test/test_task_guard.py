"""Tests for helpers/task_guard.py -- the robot-state rule dispatch and the
navigation activities read off the latest robot_state sample.

Pure, so the rule is pinned here once; the router and activity suites only
pin that each of them asks it and turns its answer into their own refusal.
The authoritative gate is the robot's own begin_task (test_workflows.py).
"""

import pytest

from syncai_backend.helpers.task_guard import (
    ERROR_LATCHED,
    NOT_INITIALIZED,
    ROBOT_BUSY,
    ROBOT_STATE_REFUSED,
    ROBOT_STATE_STALE,
    ROBOT_STATE_UNAVAILABLE,
    ROBOT_STATE_UNREACHABLE,
    STATUS_ERROR,
    STATUS_UNINITIALIZED,
    UNREACHABLE_MESSAGE,
    goal_refusal,
    start_refusal,
    state_silence,
)
from syncai_backend.tasks.schema import (
    MoveParams,
    Step,
    StepStatus,
    StepType,
    needs_localization,
)


# --- start_refusal: may a task start ------------------------------------------


@pytest.mark.parametrize("needs", [True, False])
def test_no_fresh_sample_is_never_refused_at_the_start(needs):
    # None is what RobotRepo.status() answers for a robot never heard from and
    # for one whose last sample aged out. begin_task answers for the run (and
    # fails it closed if the state service is gone), so nothing is guessed
    # here -- unlike before a goal, below.
    assert start_refusal(None, needs) is None


@pytest.mark.parametrize("needs", [True, False])
def test_error_refuses_every_task(needs):
    # Standing up or speaking included: ERROR is a robot nobody has looked at
    # since a task failed on it.
    refusal = start_refusal(STATUS_ERROR, needs)

    assert refusal.code == ERROR_LATCHED
    assert "ERROR" in refusal.message


def test_error_ends_on_the_robots_own_reason():
    refusal = start_refusal(
        STATUS_ERROR, False, "Task morning-round failed: no route to the goal."
    )

    assert refusal.message.endswith("Task morning-round failed: no route to the goal.")


def test_error_with_no_reason_is_still_a_whole_sentence():
    refusal = start_refusal(STATUS_ERROR, False, "   ")

    assert refusal.message.endswith("releases it on the console.")


def test_an_unconfirmed_robot_refuses_a_task_that_drives():
    refusal = start_refusal(STATUS_UNINITIALIZED, True)

    assert refusal.code == NOT_INITIALIZED
    assert "confirm" in refusal.message


def test_an_unconfirmed_robot_takes_a_task_that_does_not_drive():
    # Standing up needs no position; refusing it would leave an unconfirmed
    # robot lying where it booted.
    assert start_refusal(STATUS_UNINITIALIZED, False) is None


@pytest.mark.parametrize("status", ["IDLE", "RUNNING", "CHARGING", "WARNING", "UNKNOWN"])
@pytest.mark.parametrize("needs", [True, False])
def test_a_confirmed_robot_is_not_refused(status, needs):
    # The rollup is ERROR > UNINITIALIZED > the rest, so any other name means
    # confirmed and unlatched. RUNNING included: whether another run holds the
    # robot is begin_task's to say (robot_busy), and Temporal's at dispatch.
    assert start_refusal(status, needs) is None


# --- goal_refusal: may a navigation goal go out --------------------------------


def test_a_goal_is_refused_once_the_confirmation_is_withdrawn():
    refusal = goal_refusal(STATUS_UNINITIALIZED)

    assert refusal.code == NOT_INITIALIZED
    assert "no longer confirmed" in refusal.message
    # An operator confirms a position; five seconds do not.
    assert refusal.retryable is False


@pytest.mark.parametrize("status", ["IDLE", "RUNNING", "CHARGING", "WARNING", "ERROR"])
def test_a_goal_goes_out_on_a_fresh_confirmed_state(status):
    # ERROR: not this check's -- the run was admitted, so it is news from
    # elsewhere.
    assert goal_refusal(status) is None


def test_a_goal_is_held_back_when_the_robot_has_gone_quiet():
    # The last sample aged out: nobody is vouching for the position, and a
    # goal sent is a robot driving. Unlike at the start, nothing stands behind
    # this check.
    refusal = goal_refusal(None, silent_for_s=7.4)

    assert refusal.code == ROBOT_STATE_STALE
    assert "has not reported its state for 7 seconds" in refusal.message
    # ...but retryably: a few seconds without a frame is far more often a
    # hiccup than a dead node, and the step's retry policy outlasts one.
    assert refusal.retryable is True


def test_a_goal_is_held_back_when_the_robot_was_never_heard_from():
    refusal = goal_refusal(None, silent_for_s=None)

    assert refusal.code == ROBOT_STATE_STALE
    assert "has not reported its state yet" in refusal.message
    assert refusal.retryable is True


def test_a_start_refusal_is_never_retryable():
    assert start_refusal(STATUS_ERROR, False).retryable is False
    assert start_refusal(STATUS_UNINITIALIZED, True).retryable is False


@pytest.mark.parametrize(
    "silent_for_s,clause",
    [
        (None, "The robot has not reported its state yet"),
        (3.2, "The robot has not reported its state for 3 seconds"),
        (61.9, "The robot has not reported its state for 61 seconds"),
    ],
)
def test_the_silence_clause_counts_whole_seconds(silent_for_s, clause):
    assert state_silence(silent_for_s) == clause


# --- the vocabulary -----------------------------------------------------------


def test_no_sentence_names_a_node():
    sentences = [
        start_refusal(STATUS_ERROR, False, "").message,
        start_refusal(STATUS_UNINITIALIZED, True).message,
        goal_refusal(STATUS_UNINITIALIZED).message,
        goal_refusal(None, 5.0).message,
        goal_refusal(None, None).message,
        UNREACHABLE_MESSAGE,
    ]

    for sentence in sentences:
        for internal in ("robot_state", "begin_task", "syncai", "localizer"):
            assert internal not in sentence, sentence


def test_the_codes_are_distinct_wire_strings():
    codes = [
        ERROR_LATCHED, NOT_INITIALIZED, ROBOT_BUSY,
        ROBOT_STATE_UNREACHABLE, ROBOT_STATE_REFUSED,
        ROBOT_STATE_STALE, ROBOT_STATE_UNAVAILABLE,
    ]

    assert codes == [
        "error_latched", "not_initialized", "robot_busy",
        "robot_state_unreachable", "robot_state_refused",
        "robot_state_stale", "robot_state_unavailable",
    ]


def test_the_status_names_are_the_repos():
    # task_guard spells the two names itself to stay free of syncai_common;
    # they must be the ones RobotRepo.status() answers.
    pytest.importorskip("syncai_common")
    from syncai_common.msg import RobotStatus

    from syncai_backend.repositories.robot.robot import status_name

    assert status_name(RobotStatus.ERROR) == STATUS_ERROR
    assert status_name(RobotStatus.UNINITIALIZED) == STATUS_UNINITIALIZED


# --- needs_localization: which runs the rule is about --------------------------


def _step(step_type, status=StepStatus.PENDING):
    params = MoveParams(x=0.0, y=0.0, theta=0.0) if step_type is StepType.MOVE else None
    return Step(id=step_type.value, type=step_type, params=params, status=status)


def test_a_run_with_a_navigation_step_needs_a_position():
    assert needs_localization([_step(StepType.STANDUP), _step(StepType.MOVE)]) is True


def test_a_run_of_postures_does_not():
    assert needs_localization([_step(StepType.STANDUP), _step(StepType.LIEDOWN)]) is False


def test_a_skipped_navigation_step_does_not_count():
    steps = [_step(StepType.MOVE, StepStatus.SKIPPED), _step(StepType.LIEDOWN)]

    assert needs_localization(steps) is False


def test_a_request_step_without_a_status_counts():
    # The routers hand in request models, which carry no status: all run.
    class _RequestStep:
        type = StepType.PATROL

    assert needs_localization([_RequestStep()]) is True
