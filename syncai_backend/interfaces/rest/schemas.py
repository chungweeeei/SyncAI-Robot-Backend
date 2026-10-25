"""REST schemas more than one router speaks.

A router imports another router's module for nothing: shared request and
response bodies live here, and each router keeps the ones only it uses.
"""

from typing import Optional

from pydantic import BaseModel, Field, model_validator

from syncai_backend.tasks.schema import (
    GROUP_DESCRIPTION,
    ScheduleTrigger,
    Step,
    StepParams,
    StepType,
    validate_step_params,
)


# POST /api/v1/tasks, POST /api/v1/schedules and the task-template bodies.
class StepRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: Optional[StepParams] = Field(
        default=None,
        description=(
            "Parameters for the step, which vary based on the step type. "
            "Omitted for STANDUP/LIEDOWN/UNDOCK, required for MOVE, DOCK "
            "(x/y/theta: the dock pose), SPEAK and WAIT"
        ),
    )
    group: Optional[str] = Field(default=None, description=GROUP_DESCRIPTION)

    # Same check as Step, so a mismatched body is a 422 here, not a 500 later.
    @model_validator(mode="after")
    def _check_params(self) -> "StepRequest":
        validate_step_params(self.type, self.params)
        return self

    def to_step(self) -> Step:
        """The task model's step, as dispatched or frozen into a schedule."""
        return Step(id=self.id, type=self.type, params=self.params, group=self.group)


# POST / PATCH /api/v1/schedules and POST /api/v1/task_templates/{id}/schedule.
class ScheduleTriggerRequest(BaseModel):
    cron: Optional[str] = Field(
        default=None,
        description="Cron expression. Provide exactly one of cron or interval_seconds.",
        examples=["*/3 * * * *"],
    )
    interval_seconds: Optional[int] = Field(
        default=None,
        gt=0,
        description="Fire every N seconds. Provide exactly one of cron or interval_seconds.",
        examples=[1800],
    )
    timezone: Optional[str] = Field(
        default=None,
        description="IANA timezone applied to cron (ignored for interval).",
        examples=["Asia/Taipei"],
    )

    @model_validator(mode="after")
    def _exactly_one(self) -> "ScheduleTriggerRequest":
        if bool(self.cron) == bool(self.interval_seconds):
            raise ValueError("Provide exactly one of 'cron' or 'interval_seconds'")
        return self

    def to_trigger(self) -> ScheduleTrigger:
        """The task model's trigger, for every route that registers or edits one."""
        return ScheduleTrigger(
            cron=self.cron,
            interval_seconds=self.interval_seconds,
            timezone=self.timezone,
        )


class ScheduleResponse(BaseModel):
    id: str = Field(..., description="Unique identifier of the schedule")
    message: str = Field(..., description="Additional information about the schedule")
