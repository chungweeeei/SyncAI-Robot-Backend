import base64
import binascii
import structlog
from datetime import datetime, timezone
from typing import List, Optional
from fastapi import APIRouter, Query
from pydantic import BaseModel, Field, model_validator
from enum import Enum

from syncai_backend.exceptions import BadRequestError

from syncai_backend.gateways.workflow.config import (
    TASK_HISTORY_PAGE_SIZE_DEFAULT,
    TASK_HISTORY_PAGE_SIZE_MAX,
)
from syncai_backend.gateways.workflow.schema import (
    Step,
    StepType,
    StepStatus,
    StepParams,
    TaskSource,
    WorkflowTask,
    WorkflowTaskDefinition,
    validate_step_params,
)
from syncai_backend.gateways.workflow.workflow import WorkflowGateway


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    # Only ever answered by DELETE /tasks/{id}, never by the status mapping:
    # a delete is a cancel *request*, and Temporal is free to let the workflow
    # finish COMPLETED before the cancellation lands. The old response said
    # CANCELED outright — a lie the frontend had already noticed and was
    # discarding (see its task.ts).
    CANCELING = "CANCELING"


class StepRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    type: StepType = Field(..., description="Type of the step", examples=["MOVE"])
    params: Optional[StepParams] = Field(
        default=None,
        description=(
            "Parameters for the step, which vary based on the step type. "
            "Omitted for STANDUP/LIEDOWN, required for MOVE and SPEAK"
        ),
    )

    # Same check as Step, repeated here so a mismatched body is rejected at the
    # request boundary (422) instead of raising a ValidationError inside the
    # handler when the Step is built (500).
    @model_validator(mode="after")
    def _check_params(self) -> "StepRequest":
        validate_step_params(self.type, self.params)
        return self


# No `timestamp` field: the old required one was received and then never read
# by anything, which made every client invent a value to satisfy validation.
# Existing callers (console, MCP) still send it and pydantic ignores unknown
# fields by default, so dropping it is backward compatible.
class TaskRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    steps: List[StepRequest] = Field(
        ..., description="List of steps to be executed", examples=[]
    )


class TaskResponse(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    status: TaskStatus = Field(
        ..., description="Current status of the task", examples=["PENDING"]
    )
    message: str = Field(
        ...,
        description="Additional information about the task",
        examples=["Task is pending execution."],
    )


class StepState(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the step", examples=["step1"]
    )
    status: StepStatus = Field(
        ..., description="Current status of the step", examples=["IN_PROGRESS"]
    )
    error_msg: str = Field(
        default="",
        description="Error message if the step failed",
    )


class TaskStateResponse(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    status: TaskStatus = Field(
        ..., description="Overall status of the task", examples=["IN_PROGRESS"]
    )
    steps: List[StepState] = Field(..., description="Per-step state of the task")


class ActiveTaskResponse(BaseModel):
    id: str = Field(
        ...,
        description="Workflow id, i.e. the id GET/DELETE /api/v1/tasks/{id} takes",
        examples=["robot01-goal-1782786519-3"],
    )
    run_id: str = Field(..., description="Temporal run id")
    status: TaskStatus = Field(..., examples=["IN_PROGRESS"])
    started_at: datetime = Field(..., description="Execution start time (UTC)")
    source: TaskSource = Field(
        ..., description="DIRECT (someone called POST /api/v1/tasks) or SCHEDULE"
    )
    schedule_id: Optional[str] = Field(
        default=None, description="The schedule that started it, if any"
    )


class ActiveTasksResponse(BaseModel):
    tasks: List[ActiveTaskResponse] = Field(
        ..., description="Executions running on this robot's task queue"
    )
    as_of: datetime = Field(
        ...,
        description=(
            "When the snapshot was read. Elapsed time should be computed "
            "against this, not against the client's clock — they are two "
            "different clocks and the answer is served from a short cache."
        ),
    )


class TaskHistoryStatus(str, Enum):
    """The TaskStatus values a finished run can have — the history filter."""

    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class TaskHistoryEntryResponse(BaseModel):
    id: str = Field(
        ...,
        description="Workflow id, i.e. the id GET /api/v1/tasks/{id} takes",
        examples=["robot01-task-001"],
    )
    run_id: str = Field(..., description="Temporal run id")
    status: TaskHistoryStatus = Field(..., examples=["COMPLETED"])
    started_at: datetime = Field(..., description="Execution start time (UTC)")
    closed_at: Optional[datetime] = Field(
        default=None, description="Execution close time (UTC)"
    )
    source: TaskSource = Field(
        ..., description="DIRECT (someone called POST /api/v1/tasks) or SCHEDULE"
    )
    schedule_id: Optional[str] = Field(
        default=None, description="The schedule that started it, if any"
    )


class TaskHistoryResponse(BaseModel):
    tasks: List[TaskHistoryEntryResponse] = Field(
        ..., description="Finished executions on this robot, newest close first"
    )
    next_page_token: Optional[str] = Field(
        default=None,
        description=(
            "Pass back as page_token, with the same status/since, for the next "
            "page. Absent on the last page."
        ),
    )


# Temporal's page token is opaque bytes; the REST surface carries it as
# unpadded base64url so it survives a query string untouched.
def _encode_page_token(token: Optional[bytes]) -> Optional[str]:
    if not token:
        return None
    return base64.urlsafe_b64encode(token).decode("ascii").rstrip("=")


def _decode_page_token(token: Optional[str]) -> Optional[bytes]:
    if not token:
        return None
    # validate=True: without it characters outside the alphabet are silently
    # dropped, and a mangled token would quietly restart at page one.
    try:
        return base64.b64decode(
            token + "=" * (-len(token) % 4), altchars=b"-_", validate=True
        )
    except (binascii.Error, ValueError):
        raise BadRequestError("Invalid page token")


def init_task_router(
    logger: structlog.stdlib.BoundLogger, workflow_gw: WorkflowGateway
) -> APIRouter:
    task_router = APIRouter(prefix="", tags=["Task"])

    @task_router.post("/api/v1/tasks", response_model=TaskResponse)
    async def trigger_task(req: TaskRequest):

        workflow_task = WorkflowTask(
            id=req.id,
            definition=WorkflowTaskDefinition(
                steps=[
                    Step(
                        id=step.id,
                        type=step.type,
                        params=step.params,
                    )
                    for step in req.steps
                ],
            ),
        )

        await workflow_gw.start_task(request=workflow_task)

        return TaskResponse(
            id=req.id,
            status=TaskStatus.PENDING,
            message=f"Task {req.id} accepted and queued for execution.",
        )

    # Not /api/v1/tasks/active. FastAPI matches by declaration order, so that
    # path only works while it is declared above /api/v1/tasks/{id} — the day
    # someone reorders these decorators it silently becomes a lookup for a task
    # literally named "active" and answers 404. The same collision is why
    # /api/v1/task_templates was chosen over /api/v1/tasks/templates; see the
    # note in interfaces/rest/server.py.
    #
    # Plural, and a list, because "one robot does one thing" is not an invariant
    # this endpoint can rely on: ScheduleOverlapPolicy.SKIP constrains a single
    # schedule against itself, and a direct POST /api/v1/tasks bypasses it
    # entirely, so an operator dispatch during a scheduled run leaves two
    # executions Running. Two is not an error, and reporting one of them would
    # be the lie.
    #
    # Never 404s: an empty list is a valid answer. "Nothing is running" is not a
    # missing resource.
    @task_router.get("/api/v1/active_tasks", response_model=ActiveTasksResponse)
    async def list_active_tasks():
        tasks, as_of = await workflow_gw.list_active_tasks()

        return ActiveTasksResponse(
            tasks=[
                ActiveTaskResponse(
                    id=task.id,
                    run_id=task.run_id,
                    status=TaskStatus(task.status),
                    started_at=task.started_at,
                    source=task.source,
                    schedule_id=task.schedule_id,
                )
                for task in tasks
            ],
            as_of=as_of,
        )

    # Not /api/v1/tasks/history, for the same reason as active_tasks above.
    #
    # Straight from Temporal's visibility index — the database stores no runs —
    # so it reaches back exactly as far as the namespace retention, and a run
    # older than that is not here. Per-step detail for a row is
    # GET /api/v1/tasks/{id}, as for a running task.
    @task_router.get("/api/v1/task_history", response_model=TaskHistoryResponse)
    async def list_task_history(
        page_size: int = Query(
            TASK_HISTORY_PAGE_SIZE_DEFAULT, ge=1, le=TASK_HISTORY_PAGE_SIZE_MAX
        ),
        page_token: Optional[str] = Query(
            None, description="next_page_token from the previous page"
        ),
        status: Optional[TaskHistoryStatus] = Query(
            None, description="Only runs that finished with this status"
        ),
        since: Optional[datetime] = Query(
            None,
            description="Only runs that closed at or after this time; naive is UTC",
        ),
    ):
        if since is not None and since.tzinfo is None:
            since = since.replace(tzinfo=timezone.utc)

        entries, next_token = await workflow_gw.list_task_history(
            page_size=page_size,
            next_page_token=_decode_page_token(page_token),
            status=status.value if status is not None else None,
            since=since,
        )

        return TaskHistoryResponse(
            tasks=[
                TaskHistoryEntryResponse(
                    id=entry.id,
                    run_id=entry.run_id,
                    status=TaskHistoryStatus(entry.status),
                    started_at=entry.started_at,
                    closed_at=entry.closed_at,
                    source=entry.source,
                    schedule_id=entry.schedule_id,
                )
                for entry in entries
            ],
            next_page_token=_encode_page_token(next_token),
        )

    @task_router.get("/api/v1/tasks/{id}", response_model=TaskStateResponse)
    async def get_task_state(id: str):
        state = await workflow_gw.get_task_state(task_id=id)

        return TaskStateResponse(
            id=state.id,
            status=TaskStatus(state.status),
            steps=[
                StepState(
                    id=step.id,
                    status=step.status,
                    error_msg=step.error_msg or "",
                )
                for step in state.steps
            ],
        )

    @task_router.delete("/api/v1/tasks/{id}", response_model=TaskResponse)
    async def cancel_task(id: str):
        await workflow_gw.cancel_task(task_id=id)

        # CANCELING, not CANCELED — see the note on the enum member.
        return TaskResponse(
            id=id,
            status=TaskStatus.CANCELING,
            message=(
                f"Cancellation of task {id} requested; poll "
                f"GET /api/v1/tasks/{id} for the final state."
            ),
        )

    return task_router
