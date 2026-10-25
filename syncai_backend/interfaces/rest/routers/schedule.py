import structlog
from datetime import datetime
from typing import List, Optional
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_backend.tasks.schema import (
    ScheduleTask,
    WorkflowTaskDefinition,
    step_group_problem,
)
from syncai_backend.gateways.workflow.workflow import ScheduleView, WorkflowGateway
from syncai_backend.interfaces.rest.schemas import (
    ScheduleResponse,
    ScheduleTriggerRequest,
    StepRequest,
)


class ScheduleRequest(BaseModel):
    id: str = Field(
        ...,
        description="Unique identifier of the schedule",
        examples=["robot01-daily-move"],
    )
    trigger: ScheduleTriggerRequest = Field(..., description="When the schedule fires")
    steps: List[StepRequest] = Field(
        ..., description="Steps executed on each trigger", examples=[]
    )
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "Map whose frame the MOVE / DOCK coordinates are in, carried "
            "through to the schedule memo so a client can tell whether this schedule still "
            "belongs to the loaded map. A display label only, and deliberately "
            "unvalidated: this router has no MapCatalogRepo, and threading one in "
            "for a label is not worth it. POST /api/v1/task_templates/{id}/schedule "
            "is the validated path."
        ),
    )


class ScheduleUpdateRequest(BaseModel):
    trigger: ScheduleTriggerRequest = Field(
        ...,
        description=(
            "The new firing rule; replaces the old one whole (a cron schedule "
            "may become an interval one and back). Steps, map_name, provenance "
            "and the paused state are left as they are."
        ),
    )


class ScheduleTriggerResponse(BaseModel):
    cron: Optional[str] = None
    interval_seconds: Optional[int] = None
    timezone: Optional[str] = None


class ScheduleStateResponse(BaseModel):
    id: str = Field(..., description="Unique identifier of the schedule")
    trigger: ScheduleTriggerResponse = Field(..., description="When the schedule fires")
    paused: bool = Field(..., description="Whether the schedule is paused")
    next_run_times: List[datetime] = Field(
        ..., description="Upcoming trigger times (UTC)"
    )
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "Map whose frame this schedule's MOVE / DOCK coordinates "
            "are in."
        ),
    )
    task_template_id: Optional[str] = Field(
        default=None,
        description="The task template this schedule was frozen from, if any.",
    )
    task_template_name: Optional[str] = Field(
        default=None, description="That template's name at registration time."
    )
    # StepRequest reused as the element: a second identical model could drift.
    steps: List[StepRequest] = Field(
        default_factory=list,
        description=(
            "The frozen step list. Populated by GET /api/v1/schedules/{id} only; "
            "the collection endpoint always answers [], because Temporal's "
            "schedule *list* API does not carry the start-workflow arguments. "
            "Frozen at registration: later vertex edits do not reach a scheduled "
            "run, so a client comparing these against their source template is "
            "how staleness becomes visible. Only the trigger is editable "
            "(PATCH /api/v1/schedules/{id}); it never touches these."
        ),
    )


def init_schedule_router(
    logger: structlog.stdlib.BoundLogger, workflow_gw: WorkflowGateway
) -> APIRouter:
    schedule_router = APIRouter(prefix="", tags=["Schedule"])

    @schedule_router.post("/api/v1/schedules", response_model=ScheduleResponse)
    async def create_schedule(req: ScheduleRequest):
        problem = step_group_problem(req.steps)
        if problem is not None:
            raise problem.refusal()
        schedule_task = ScheduleTask(
            id=req.id,
            trigger=req.trigger.to_trigger(),
            definition=WorkflowTaskDefinition(
                steps=[step.to_step() for step in req.steps],
            ),
            map_name=req.map_name,
        )

        await workflow_gw.create_schedule(schedule=schedule_task)

        return ScheduleResponse(
            id=req.id,
            message=f"Schedule {req.id} has been created.",
        )

    def _state_response(
        view: ScheduleView, steps: Optional[List[StepRequest]] = None
    ) -> ScheduleStateResponse:
        # `steps` None on the list path: see the field's description.
        return ScheduleStateResponse(
            id=view.id,
            trigger=ScheduleTriggerResponse(
                cron=view.trigger.cron,
                interval_seconds=view.trigger.interval_seconds,
                timezone=view.trigger.timezone,
            ),
            paused=view.paused,
            next_run_times=view.next_run_times,
            map_name=view.map_name,
            task_template_id=view.task_template_id,
            task_template_name=view.task_template_name,
            steps=steps or [],
        )

    @schedule_router.get(
        "/api/v1/schedules", response_model=List[ScheduleStateResponse]
    )
    async def list_schedules():
        return [_state_response(view) for view in await workflow_gw.list_schedules()]

    @schedule_router.get(
        "/api/v1/schedules/{id}",
        response_model=ScheduleStateResponse,
        # Load-bearing: BaseSchema's to_camel aliases would otherwise answer
        # multi-word Step fields in camelCase; the frontend is snake_case-only.
        response_model_by_alias=False,
    )
    async def get_schedule(id: str):
        view = await workflow_gw.get_schedule(schedule_id=id)

        return _state_response(
            view,
            steps=[
                StepRequest(
                    id=step.id, type=step.type, params=step.params, group=step.group
                )
                for step in view.steps
            ],
        )

    @schedule_router.patch("/api/v1/schedules/{id}", response_model=ScheduleResponse)
    async def update_schedule(id: str, req: ScheduleUpdateRequest):
        """Change when an existing schedule fires, in place.

        Id, frozen steps, provenance and paused state are kept; the id cannot
        be renamed (that is a delete and a create).
        """
        await workflow_gw.update_schedule_trigger(
            schedule_id=id,
            trigger=req.trigger.to_trigger(),
        )

        return ScheduleResponse(
            id=id,
            message=f"Schedule {id} trigger has been updated.",
        )

    @schedule_router.delete("/api/v1/schedules/{id}", response_model=ScheduleResponse)
    async def delete_schedule(id: str):
        await workflow_gw.delete_schedule(schedule_id=id)

        return ScheduleResponse(
            id=id,
            message=f"Schedule {id} has been deleted.",
        )

    @schedule_router.post(
        "/api/v1/schedules/{id}/pause", response_model=ScheduleResponse
    )
    async def pause_schedule(id: str):
        await workflow_gw.pause_schedule(schedule_id=id)

        return ScheduleResponse(
            id=id,
            message=f"Schedule {id} has been paused.",
        )

    @schedule_router.post(
        "/api/v1/schedules/{id}/resume", response_model=ScheduleResponse
    )
    async def resume_schedule(id: str):
        await workflow_gw.resume_schedule(schedule_id=id)

        return ScheduleResponse(
            id=id,
            message=f"Schedule {id} has been resumed.",
        )

    return schedule_router
