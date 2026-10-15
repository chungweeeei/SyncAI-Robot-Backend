import base64
import binascii
import structlog
from datetime import datetime, timezone
from typing import List, Optional, Protocol
from fastapi import APIRouter, Query
from pydantic import BaseModel, Field, field_validator, model_validator
from enum import Enum

from syncai_backend.exceptions import BadRequestError, ConflictError

from syncai_backend.gateways.workflow.config import (
    TASK_HISTORY_PAGE_SIZE_DEFAULT,
    TASK_HISTORY_PAGE_SIZE_MAX,
)
from syncai_backend.gateways.workflow.schema import (
    GROUP_DESCRIPTION,
    NAV_STEP_TYPES,
    POSE_STEP_TYPES,
    Step,
    StepType,
    StepStatus,
    StepParams,
    TaskKind,
    TaskProvenance,
    TaskSource,
    WorkflowTask,
    WorkflowTaskDefinition,
    step_group_problem,
    validate_step_params,
)
from syncai_backend.gateways.workflow.workflow import WorkflowGateway

from syncai_backend.helpers.move_guard import move_refusal
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.services.gridmap_conversion import GridmapConversionService


class TaskStatus(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    # The run is holding. Not terminal: a held run is still running to
    # Temporal, so /active_tasks lists it as IN_PROGRESS; only GET /tasks/{id}
    # sees the hold.
    PAUSED = "PAUSED"
    # Only answered by POST /tasks/{id}/pause: a pause is a request, and an
    # uninterruptible step (SPEAK, a posture) finishes before the run holds.
    PAUSING = "PAUSING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"
    # Only answered by DELETE /tasks/{id}: a cancel is a request, and the run
    # may still finish COMPLETED before it lands. Do not answer CANCELED.
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


# No `timestamp` field: it was never read. Callers that still send it are fine,
# pydantic ignores unknown fields.
class TaskRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    steps: List[StepRequest] = Field(
        ..., description="List of steps to be executed", examples=[]
    )
    # Optional, so the MCP server and curl dispatch unchanged (unlabelled).
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
    # The caller's expectation, not the record: a run is stamped with the map
    # loaded at dispatch; this only gets it refused if that has changed.
    map_name: Optional[str] = Field(
        default=None,
        min_length=1,
        description=(
            "The map the MOVE positions were planned on. Refused with 409 "
            "map_mismatch when it is not the map the robot is using."
        ),
        examples=["lab"],
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
    message: str = Field(
        default="",
        description=(
            "What the robot is doing for a running MOVE step, in the robot's "
            "own words (its navigator's status sentence); empty for every "
            "other step and state. Free text for display only"
        ),
        examples=["Path blocked; replanning"],
    )
    group: Optional[str] = Field(
        default=None,
        description="The step's group, as dispatched: steps sharing one ran side by side",
    )


class MoveMessageSource(Protocol):
    """The one thing the task-state read needs from the robot gateway.

    A Protocol so this router stays free of rclpy and testable without ROS.
    """

    def get_move_message(self) -> str: ...


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
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "The map this run holds while it runs -- its positions are on it, "
            "and the map routes refuse to change it until the run ends. Null "
            "for a run that drives nowhere (stand, lie down)."
        ),
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
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "The map its positions were on; null for a run with no MOVE step "
            "or one older than the field."
        ),
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


def _utc(moment: Optional[datetime]) -> Optional[datetime]:
    """A naive query datetime is UTC, not the server's local time."""
    if moment is not None and moment.tzinfo is None:
        return moment.replace(tzinfo=timezone.utc)
    return moment


def _check_window(since: Optional[datetime], until: Optional[datetime]) -> None:
    # Cross-field, so a 400 with a sentence rather than a pydantic bound.
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
    logger: structlog.stdlib.BoundLogger,
    workflow_gw: WorkflowGateway,
    map_catalog_repo: MapCatalogRepo,
    conversion_svc: GridmapConversionService,
    robot_gw: MoveMessageSource,
) -> APIRouter:
    task_router = APIRouter(prefix="", tags=["Task"])

    @task_router.post("/api/v1/tasks", response_model=TaskResponse)
    async def trigger_task(req: TaskRequest):
        # A sentence, not a 422: see step_group_problem.
        problem = step_group_problem(req.steps)
        if problem is not None:
            raise problem.refusal()

        # A job that drives to a pose is stamped with the loaded map (what the
        # map routes lock on and the history shows). One that does not -- an
        # UNDOCK backs off blind, from wherever the robot stands -- holds no map
        # and is neither stamped nor refused.
        map_name: Optional[str] = None
        if any(step.type in POSE_STEP_TYPES for step in req.steps):
            active = map_catalog_repo.active_name()
            refusal = move_refusal(
                req.map_name,
                active,
                active is not None and conversion_svc.is_converting(active),
            )
            if refusal is not None:
                raise ConflictError(refusal.message, code=refusal.code)
            map_name = active

        workflow_task = WorkflowTask(
            id=req.id,
            definition=WorkflowTaskDefinition(
                steps=[
                    Step(
                        id=step.id,
                        type=step.type,
                        params=step.params,
                        group=step.group,
                    )
                    for step in req.steps
                ],
            ),
        )

        await workflow_gw.start_task(
            request=workflow_task,
            provenance=TaskProvenance(kind=req.kind, name=req.name, map_name=map_name),
        )

        return TaskResponse(
            id=req.id,
            status=TaskStatus.PENDING,
            message=f"Task {req.id} accepted and queued for execution.",
        )

    # Not /api/v1/tasks/active: /api/v1/tasks/{id} would shadow it depending on
    # declaration order (see interfaces/rest/server.py).
    #
    # A list: SKIP only stops a schedule overlapping itself, so a direct dispatch
    # during a scheduled run leaves two executions running. Never 404s.
    @task_router.get("/api/v1/active_tasks", response_model=ActiveTasksResponse)
    async def list_active_tasks():
        tasks, as_of = await workflow_gw.list_active_tasks()
        # Only read when some run lacks the attribute (see map_in_use), so the
        # console's 2 s poll usually costs no INI read.
        active = (
            map_catalog_repo.active_name()
            if any(not task.map_name for task in tasks)
            else None
        )

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
                    map_name=task.map_in_use(active),
                )
                for task in tasks
            ],
            as_of=as_of,
        )

    # Not /api/v1/tasks/history, as for active_tasks. Read from Temporal's
    # visibility index (no runs are stored here), so it reaches back only as
    # far as namespace retention.
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
                    map_name=entry.map_name,
                )
                for entry in entries
            ],
            next_page_token=_encode_page_token(next_token),
        )

    # The task_history filter, counted instead of listed (one count RPC).
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
            # Null, never 0, when nothing finished.
            success_rate=stats.completed / stats.total if stats.total else None,
        )

    def _step_message(step: Step) -> str:
        # Read live from the gateway: the workflow cannot see an activity's
        # progress. Only a running MOVE / DOCK / UNDOCK has a goal; between retry
        # attempts the gateway answers "".
        if step.type in NAV_STEP_TYPES and step.status is StepStatus.IN_PROGRESS:
            return robot_gw.get_move_message()
        return ""

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
                    message=_step_message(step),
                    group=step.group,
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

    # The hold: both verbs are workflow signals, so like DELETE they answer
    # what was asked (PAUSING); GET reports PAUSED once it holds. A MOVE stops
    # at once and is re-sent on resume, a WAIT freezes, SPEAK and postures
    # finish first. 409 `task_not_running` once the run is closed.
    @task_router.post("/api/v1/tasks/{id}/pause", response_model=TaskResponse)
    async def pause_task(id: str):
        await workflow_gw.pause_task(task_id=id)

        return TaskResponse(
            id=id,
            status=TaskStatus.PAUSING,
            message=(
                f"Pause of task {id} requested; a MOVE or WAIT stops now, any "
                f"other step finishes first. Poll GET /api/v1/tasks/{id} for PAUSED."
            ),
        )

    @task_router.post("/api/v1/tasks/{id}/resume", response_model=TaskResponse)
    async def resume_task(id: str):
        await workflow_gw.resume_task(task_id=id)

        return TaskResponse(
            id=id,
            status=TaskStatus.IN_PROGRESS,
            message=(
                f"Resume of task {id} requested; poll GET /api/v1/tasks/{id} "
                f"for the step picking back up."
            ),
        )

    return task_router
