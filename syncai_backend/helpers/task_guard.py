"""Whether the robot's own state lets a task start, or a goal go out.

syncai_robot_state owns the robot's state machine and is the judge: every run
asks it ``begin_task`` before its first step (RobotWorkflow._begin_task), and
that answer is the one that counts. This module is the same rule read off the
latest ``robot_state`` sample, for the two places that cannot ask the judge:

- ``POST /api/v1/tasks`` (and a continuation) answers an ADVISORY 409 before
  it dispatches, so the operator who pressed Start reads why at once instead
  of finding a run that failed in its first second. A sample can be a second
  old, so no fresh sample, or a state that changes in between, is let through:
  begin_task decides.
- ``RobotActivities`` asks again before every navigation goal. begin_task was
  answered when the run started; the confirmation can be withdrawn while it
  runs (the localizer restarted, a pose set from outside this backend), and
  the next MOVE would then plan from a pose nobody has looked at.

The two differ on what NO FRESH SAMPLE means (never received, or aged out:
RobotRepo believes a sample for ROBOT_STATE_MAX_AGE_S). At dispatch it passes,
because begin_task stands behind it and fails closed when the state service is
gone. Before a goal nothing stands behind it -- a goal sent is a robot driving
-- so the goal is held back; but as a RETRYABLE refusal, since a few seconds
without a frame is far more often a hiccup than a dead node, and the step's
retry policy then gives it about 15 s to come back before the step fails.

The codes are the wire's -- ConflictError.code on REST, ApplicationError.type
on a failed run -- and the sentences operator prose, shown verbatim by the
console and the history row. They name no node: an operator does not know
what syncai_robot_state is. Pure, so the rule is tested here without a repo
or a worker; ``status`` is RobotStatus by name (RobotRepo.status()), so this
imports without syncai_common.
"""

from dataclasses import dataclass
from typing import Optional


# The RobotStatus names this rule reads (repositories/robot/robot.py owns the
# table; test_task_guard.py pins that the two agree). The rollup is ERROR >
# UNINITIALIZED > RUNNING > CHARGING > WARNING > IDLE, so any other name means
# the position is confirmed and nothing is latched.
STATUS_ERROR = "ERROR"
STATUS_UNINITIALIZED = "UNINITIALIZED"

# ERROR is latched (a task failed): every task is refused, whatever its steps,
# until an operator releases it (POST /api/v1/robot/state).
ERROR_LATCHED = "error_latched"
# No operator has confirmed where the robot stands, or the confirmation was
# withdrawn by a map switch or a new pose. Only for a task that drives.
NOT_INITIALIZED = "not_initialized"
# The robot counts another run as RUNNING. begin_task's alone: a sample cannot
# tell this run's RUNNING from another's, and POST /api/v1/tasks already asks
# Temporal that question (WorkflowGateway._require_idle).
ROBOT_BUSY = "robot_busy"
# begin_task got no answer in its retries. The run fails closed: a task that
# starts without the robot knowing is one nothing would refuse or count.
ROBOT_STATE_UNREACHABLE = "robot_state_unreachable"
# A begin_task refusal with a code this backend has no name for (a newer
# interface). The same word POST /api/v1/robot/state answers a refusal it has
# no finer code for (gateways/failure.py's ROBOT_STATE_REFUSED).
ROBOT_STATE_REFUSED = "robot_state_refused"
# The robot stopped reporting its state: the last sample aged out. On a
# navigation goal (retryable, and for a robot never heard from too), and
# GET /api/v1/robot/state's 404 for a sample that went stale.
ROBOT_STATE_STALE = "robot_state_stale"
# GET /api/v1/robot/state's 404 before the first sample ever arrives.
ROBOT_STATE_UNAVAILABLE = "robot_state_unavailable"

# What the run fails with when begin_task was not answered. Here beside the
# other sentences, so every reason a task did not start reads as one voice.
UNREACHABLE_MESSAGE = (
    "The robot's state service did not answer, so the task was not started. "
    "Check that the robot's software is running and start the task again."
)


@dataclass(frozen=True)
class TaskRefusal:
    code: str
    message: str
    # Whether asking again in a few seconds can get another answer. False for
    # everything an operator has to act on; True only for a robot that has
    # gone quiet, which usually speaks again by itself.
    retryable: bool = False


def state_silence(silent_for_s: Optional[float]) -> str:
    """How long the robot has not reported its state, as a clause shared by
    the navigation refusal and GET /api/v1/robot/state's 404.

    ``silent_for_s`` is the age of the last sample (RobotRepo.sample_age());
    None is a robot never heard from since this backend started. Whole
    seconds: the precision of a 1 Hz topic, and a number an operator reads.
    """
    if silent_for_s is None:
        return "The robot has not reported its state yet"
    return f"The robot has not reported its state for {int(silent_for_s)} seconds"


def start_refusal(
    status: Optional[str],
    needs_localization: bool,
    state_message: str = "",
) -> Optional[TaskRefusal]:
    """Why a task must not start on a robot reading ``status``, or None.

    ``status`` is the fresh sample's RobotStatus name, None when there is none
    -- never received, or aged out (never refused: begin_task answers for the
    run, and fails it closed if the robot's state service is gone).
    ``state_message`` is the robot's own sentence for its state, which for
    ERROR says what failed; it ends the refusal when there is one.
    """
    if status == STATUS_ERROR:
        why = f" {state_message.strip()}" if state_message.strip() else ""
        return TaskRefusal(
            code=ERROR_LATCHED,
            message=(
                "The robot is locked in ERROR and starts no task until an "
                f"operator releases it on the console.{why}"
            ),
        )
    if status == STATUS_UNINITIALIZED and needs_localization:
        return TaskRefusal(
            code=NOT_INITIALIZED,
            message=(
                "The robot's position has not been confirmed, so it cannot "
                "start a task that drives. Check where the map shows the "
                "robot, set its pose if that is wrong, and confirm the "
                "position on the console first."
            ),
        )
    return None


def goal_refusal(
    status: Optional[str], silent_for_s: Optional[float] = None
) -> Optional[TaskRefusal]:
    """Why a navigation goal must not go out on a robot reading ``status``.

    ``status`` None is no fresh sample, with ``silent_for_s`` the age of the
    last one (None: never received): the goal is held back, RETRYABLY
    (robot_state_stale). Unlike start_refusal, which lets that through --
    there the robot's begin_task still answers; here the next thing that
    happens is the robot driving on a confirmation nobody is vouching for.
    Retryable because the usual cause is a hiccup (a dropped frame, the state
    node restarting), and the step's retry policy outlasts one.

    A fresh UNINITIALIZED is the non-retryable not_initialized: an operator
    confirms a position, five seconds do not. Nothing else refuses: the run
    was accepted, so ERROR here would be news from another source than this
    run, and RUNNING is this run itself.
    """
    if status is None:
        return TaskRefusal(
            code=ROBOT_STATE_STALE,
            message=(
                f"{state_silence(silent_for_s)}, so it was not sent a "
                "navigation goal. Check that the robot's software is running "
                "and start the task again."
            ),
            retryable=True,
        )
    if status == STATUS_UNINITIALIZED:
        return TaskRefusal(
            code=NOT_INITIALIZED,
            message=(
                "The robot's position is no longer confirmed (the map was "
                "switched, or its pose was set again), so it will not drive. "
                "Confirm the position on the console and start the task again."
            ),
        )
    return None
