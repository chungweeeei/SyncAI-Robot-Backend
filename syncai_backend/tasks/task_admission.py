"""What a run and the robot's state machine say to each other, as the activity
arguments that carry it (RobotWorkflow._begin_task / _end_task).

syncai_robot_state owns the robot's state (UNINITIALIZED / IDLE / RUNNING /
ERROR) and is the one judge of whether a task may start: every run asks it
``begin_task`` before its first step and, once accepted, reports ``end_task``
exactly once. SyncAI-Robot-Interface's BeginTask.srv / EndTask.srv are the
contract; these models are its shape on the Temporal side.

Here, in the task model's layer, because the workflow, the activities and the
gateway all read them and none of those may import another.
"""

from enum import Enum

from pydantic import Field

from syncai_backend.tasks.schema import BaseSchema


class TaskOutcome(str, Enum):
    """How an accepted run ended (EndTask.outcome, by name).

    Only FAILED changes the robot: it latches ERROR until an operator releases
    it. A cancel -- the operator's, or the safety lock's -- is CANCELED and
    latches nothing.
    """

    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class TaskAdmissionRequest(BaseSchema):
    task_id: str = Field(
        ...,
        description=(
            "The run's Temporal workflow id: unique among runs open at once, "
            "and the id its end_task must carry."
        ),
    )
    needs_localization: bool = Field(
        ...,
        description=(
            "Whether a step the run will execute drives on the map (MOVE / "
            "PATROL / CHARGE); an unconfirmed robot refuses those only."
        ),
    )


class TaskAdmission(BaseSchema):
    """The robot's answer to begin_task. A refusal is an answer, not an error:
    the activity returns it, so Temporal never retries one."""

    accepted: bool = Field(
        ..., description="True: the robot counts this run as RUNNING from now on."
    )
    code: str = Field(
        default="",
        description=(
            "On a refusal, helpers/task_guard.py's code for it (error_latched / "
            "not_initialized / robot_busy); empty when accepted."
        ),
    )
    message: str = Field(
        default="",
        description=(
            "The robot's sentence, written for an operator; on a refusal the "
            "run's first step fails with it."
        ),
    )


class TaskEnd(BaseSchema):
    task_id: str = Field(..., description="The id begin_task was accepted with.")
    outcome: TaskOutcome = Field(..., description="How the run ended.")
    reason: str = Field(
        default="",
        description=(
            "For FAILED, the failed step's error: what the console shows while "
            "the robot is locked in ERROR. Ignored for the other two."
        ),
    )
