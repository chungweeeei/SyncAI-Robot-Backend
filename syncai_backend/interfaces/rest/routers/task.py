import base64
import binascii
import structlog
from datetime import datetime, timezone
from typing import List, Optional
from fastapi import APIRouter, Query
from pydantic import BaseModel, Field, field_validator, model_validator
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
    TaskKind,
    TaskProvenance,
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
    # Both optional, so the MCP server and curl keep dispatching unchanged;
    # such a run is simply unlabelled in the history.
    kind: Optional[TaskKind] = Field(
        default=None,
        description=(
            "How the task was started: goal / standup / liedown / task. Recorded "
            "on the run for GET /api/v1/task_history and its /stats. 'schedule' "
            "is reserved for runs a schedule starts."
        ),
        examples=["goal"],
    )
    name: Optional[str] = Field(
        default=None,
        min_length=1,
        max_length=255,
        description="The task template this dispatch came from, if any.",
        examples=["Morning patrol"],
    )

    @field_validator("kind")
    @classmethod
    def _direct_kind_only(cls, value: Optional[TaskKind]) -> Optional[TaskKind]:
        if value is TaskKind.SCHEDULE:
            raise ValueError("kind 'schedule' is set by the backend for scheduled runs")
        return value


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
    kind: Optional[TaskKind] = Field(
        default=None,
        description="How it was started; null for a run dispatched without saying",
    )
    name: Optional[str] = Field(
        default=None, description="The task template it was dispatched from, if any"
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
    kind: Optional[TaskKind] = Field(
        default=None,
        description=(
            "How it was started: goal / standup / liedown / task / schedule; "
            "null for a run dispatched without saying"
        ),
    )
    name: Optional[str] = Field(
        default=None, description="The task template it was dispatched from, if any"
    )


class TaskHistoryResponse(BaseModel):
    tasks: List[TaskHistoryEntryResponse] = Field(
        ..., description="Finished executions on this robot, newest close first"
    )
    next_page_token: Optional[str] = Field(
        default=None,
        description=(
            "Pass back as page_token, with the same status/since/until/kind/name, "
            "for the next page. Absent on the last page."
        ),
    )


class TaskKindCountResponse(BaseModel):
    kind: Optional[TaskKind] = Field(
        default=None,
        description="null for runs that carry no kind (dispatched without one)",
    )
    total: int
    completed: int
    failed: int
    canceled: int


class TaskHistoryStatsResponse(BaseModel):
    as_of: datetime = Field(..., description="When the counts were taken (UTC)")
    total: int = Field(..., description="Finished runs matching the filter")
    by_status: dict[TaskHistoryStatus, int] = Field(
        ..., description="The same runs, by how they ended"
    )
    success_rate: Optional[float] = Field(
        default=None,
        description="COMPLETED over total; null when nothing finished",
    )
    by_kind: List[TaskKindCountResponse] = Field(
        ...,
        description=(
            "Every kind in a fixed order plus the null row, or exactly one row "
            "when the filter names a kind"
        ),
    )


def _utc(moment: Optional[datetime]) -> Optional[datetime]:
    """A naive query datetime is UTC, not the server's local time."""
    if moment is not None and moment.tzinfo is None:
        return moment.replace(tzinfo=timezone.utc)
    return moment


def _check_window(since: Optional[datetime], until: Optional[datetime]) -> None:
    # Cross-field, so not a pydantic bound: the house rule is a 400 with a
    # sentence (see CLAUDE.md), and an empty window is a caller mistake, not
    # a query to send.
    if since is not None and until is not None and until < since:
        raise BadRequestError("until must not be before since")


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

        await workflow_gw.start_task(
            request=workflow_task,
            provenance=TaskProvenance(kind=req.kind, name=req.name),
        )

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
                    kind=task.kind,
                    name=task.name,
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
        until: Optional[datetime] = Query(
            None,
            description="Only runs that closed at or before this time; naive is UTC",
        ),
        kind: Optional[TaskKind] = Query(
            None,
            description="Only runs started this way: goal / standup / liedown / task / schedule",
        ),
        name: Optional[str] = Query(
            None,
            min_length=1,
            max_length=255,
            description="Only runs dispatched from the task template with this exact name",
        ),
    ):
        since, until = _utc(since), _utc(until)
        _check_window(since, until)

        entries, next_token = await workflow_gw.list_task_history(
            page_size=page_size,
            next_page_token=_decode_page_token(page_token),
            status=status.value if status is not None else None,
            since=since,
            until=until,
            kind=kind,
            name=name,
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
                    kind=entry.kind,
                    name=entry.name,
                )
                for entry in entries
            ],
            next_page_token=_encode_page_token(next_token),
        )

    # The same filter as task_history, counted instead of listed: the numbers
    # the history dashboard shows above its list. Declared as its own static
    # path -- `task_history` has no path parameter, so nothing shadows it.
    @task_router.get(
        "/api/v1/task_history/stats", response_model=TaskHistoryStatsResponse
    )
    async def task_history_stats(
        status: Optional[TaskHistoryStatus] = Query(
            None, description="Only runs that finished with this status"
        ),
        since: Optional[datetime] = Query(
            None,
            description="Only runs that closed at or after this time; naive is UTC",
        ),
        until: Optional[datetime] = Query(
            None,
            description="Only runs that closed at or before this time; naive is UTC",
        ),
        kind: Optional[TaskKind] = Query(
            None,
            description="Only runs started this way: goal / standup / liedown / task / schedule",
        ),
        name: Optional[str] = Query(
            None,
            min_length=1,
            max_length=255,
            description="Only runs dispatched from the task template with this exact name",
        ),
    ):
        since, until = _utc(since), _utc(until)
        _check_window(since, until)

        stats = await workflow_gw.task_history_stats(
            status=status.value if status is not None else None,
            since=since,
            until=until,
            kind=kind,
            name=name,
        )

        return TaskHistoryStatsResponse(
            as_of=stats.as_of,
            total=stats.total,
            by_status={
                TaskHistoryStatus.COMPLETED: stats.completed,
                TaskHistoryStatus.FAILED: stats.failed,
                TaskHistoryStatus.CANCELED: stats.canceled,
            },
            # Computed here, once, so no client has to agree with another about
            # what a rate of nothing is: it is null, never 0.
            success_rate=stats.completed / stats.total if stats.total else None,
            by_kind=[
                TaskKindCountResponse(
                    kind=row.kind,
                    total=row.total,
                    completed=row.completed,
                    failed=row.failed,
                    canceled=row.canceled,
                )
                for row in stats.by_kind
            ],
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
