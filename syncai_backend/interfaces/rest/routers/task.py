import base64
import binascii
import csv
import io
import math
import structlog
from collections.abc import Iterable, Iterator
from datetime import datetime, timezone, tzinfo
from typing import Any, List, Optional, Protocol, Tuple
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError
from fastapi import APIRouter, Depends, Query
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field, field_validator
from enum import Enum

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError

from syncai_backend.tasks.catalogue import StepCatalogue, step_catalogue
from syncai_backend.tasks.config import (
    TASK_HISTORY_PAGE_SIZE_DEFAULT,
    TASK_HISTORY_PAGE_SIZE_MAX,
)
from syncai_backend.tasks.schema import (
    ActiveTask,
    MoveParams,
    NAV_STEP_TYPES,
    POSE_STEP_TYPES,
    Step,
    StepStatus,
    StepType,
    TaskKind,
    TaskProvenance,
    TaskSource,
    WorkflowTask,
    WorkflowTaskDefinition,
    continuation_steps,
    step_group_problem,
)
from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.helpers.route_preview import RoutePreview
from syncai_backend.interfaces.rest.schemas import StepRequest

from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.task.task_record import TaskRecordRepo, as_utc
from syncai_backend.database.models import TaskRecordRow
from syncai_backend.services.map_usage import MapUsagePolicy


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


# No `timestamp` field: it was never read. Callers that still send it are fine,
# pydantic ignores unknown fields.
class TaskRequest(BaseModel):
    id: str = Field(
        ..., description="Unique identifier of the task", examples=["robot01-task-001"]
    )
    # Exactly one of steps / continue_from: checked in trigger_task, as a 400.
    steps: Optional[List[StepRequest]] = Field(
        default=None,
        description="List of steps to be executed. Omitted with continue_from",
        examples=[],
    )
    continue_from: Optional[str] = Field(
        default=None,
        min_length=1,
        description=(
            "A FAILED task to continue: this task runs its steps again from the "
            "failed one (all of its group, a PATROL past the waypoints it had "
            "passed); the ones before it read SKIPPED. Instead of steps and "
            "map_name; kind and name default to that task's"
        ),
        examples=["robot01-task-001"],
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
    continued_from: Optional[str] = Field(
        default=None,
        description="The failed task this one continues; null unless continue_from was sent",
    )
    from_step_id: Optional[str] = Field(
        default=None,
        description=(
            "The step a continuing task starts at (the failed step, or the first "
            "of its group); null unless continue_from was sent"
        ),
    )


class ChargerPose(BaseModel):
    x: float = Field(..., description="Marker x in the map frame, metres", examples=[1.0])
    y: float = Field(..., description="Marker y in the map frame, metres", examples=[2.0])
    theta: float = Field(
        ...,
        description="Marker heading in degrees, CCW from +x, out of its face",
        examples=[90.0],
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
    charge: Optional[ChargerPose] = Field(
        default=None,
        description=(
            "The charger waypoint a CHARGE step walks onto, as dispatched (map "
            "frame): the marker's centre, theta out of its face; null for every "
            "other step. A console derives the staging pose from it (1.3 m out "
            "along theta, facing back) to draw the approach of a running charge"
        ),
    )
    started_at: Optional[datetime] = Field(
        default=None,
        description=(
            "When the step started (UTC); null until it has, and for a run "
            "older than the field"
        ),
    )
    ended_at: Optional[datetime] = Field(
        default=None,
        description="When the step ended, however it ended (UTC); null until it has",
    )
    paused_seconds: float = Field(
        default=0.0,
        description=(
            "How long the run was held while this step was under way. The time "
            "the step itself took is ended_at - started_at - paused_seconds"
        ),
    )
    waypoints_passed: int = Field(
        default=0,
        description=(
            "How many of a PATROL's waypoints the robot has passed; 0 for every "
            "other step"
        ),
    )


class MoveMessageSource(Protocol):
    """What this router needs from the robot gateway: the running MOVE's
    status sentence for the task-state read, and the route preview.

    A Protocol so this router stays free of rclpy and testable without ROS.
    """

    def get_move_message(self) -> str: ...

    def preview_route(
        self, x: float, y: float, yaw: float, z: float = 0.0
    ) -> Tuple[bool, str, Optional[RoutePreview]]: ...


class RoutePreviewRequest(MoveParams):
    """A MOVE's goal (x, y, theta in degrees, z the ground of its floor), and
    the map it was aimed on."""

    map_name: Optional[str] = Field(
        default=None,
        min_length=1,
        description=(
            "The map the goal was aimed on. Refused with 409 map_mismatch when "
            "it is not the map the robot is using, as POST /api/v1/tasks is."
        ),
        examples=["lab"],
    )


class RoutePreviewResponse(BaseModel):
    reachable: bool = Field(
        ...,
        description=(
            "Whether the planner found a route. False is an answer, not an "
            "error: code and message say why"
        ),
    )
    points: List[Tuple[float, float, float]] = Field(
        ...,
        description=(
            "The route from the robot to the goal, (x, y, z) in the map frame, "
            "z the ground it stands on; at most ~512 points. Empty when not reachable"
        ),
    )
    length_m: float = Field(
        ..., description="The route's length in metres, a stair's rise included"
    )
    code: Optional[str] = Field(
        default=None,
        description=(
            "Why there is no route, machine-readable (goal_in_keepout, "
            "no_valid_path, map_not_loaded, ...); null when reachable"
        ),
        examples=["goal_in_keepout"],
    )
    message: str = Field(
        default="",
        description=(
            "Why there is no route, as the job that drove there would have "
            "failed with it; empty when reachable"
        ),
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
    ended_at: Optional[datetime] = Field(
        default=None,
        description="Execution close time (UTC); null in `tasks`, set in `recently_ended`",
    )
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
    recently_ended: List[ActiveTaskResponse] = Field(
        default_factory=list,
        description=(
            "Executions on this queue that closed within the last few seconds "
            "(RECENTLY_ENDED_WINDOW_S), newest close first. A run that starts and "
            "fails between two polls is never in `tasks`; this is how a console "
            "that did not start it can still say how it ended. Not running: "
            "nothing here holds a map or makes the robot busy."
        ),
    )
    as_of: datetime = Field(
        ...,
        description=(
            "When the snapshot was read. Elapsed time should be computed "
            "against this, not against the client's clock — they are two "
            "different clocks and the answer is served from a short cache."
        ),
    )


def _active_task_response(
    task: ActiveTask, map_name: Optional[str]
) -> ActiveTaskResponse:
    """One row of either list; ``map_name`` is the caller's, since a running
    run and a closed one read it differently (see list_active_tasks)."""
    return ActiveTaskResponse(
        id=task.id,
        run_id=task.run_id,
        status=TaskStatus(task.status),
        started_at=task.started_at,
        ended_at=task.ended_at,
        source=task.source,
        schedule_id=task.schedule_id,
        kind=task.kind,
        name=task.name,
        map_name=map_name,
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


# One row per step, so a spreadsheet can filter and pivot on either level; the
# run's columns repeat on each of its steps. Only ever append a column: a
# customer's sheet reads them by position.
EXPORT_COLUMNS = (
    "task_id",
    "run_id",
    "task_name",
    "kind",
    "schedule_id",
    "map_name",
    "task_status",
    "task_started_at",
    "task_closed_at",
    "task_duration_s",
    "step_index",
    "step_id",
    "step_type",
    "step_group",
    "step_status",
    "step_started_at",
    "step_ended_at",
    "step_wall_s",
    "step_paused_s",
    "step_active_s",
    "error_msg",
    "timezone",
)

# Rows per chunk written to the response.
_EXPORT_CHUNK_ROWS = 200


def _parse_time(value: Any) -> Optional[datetime]:
    """A stored step's timestamp (pydantic's JSON: ISO 8601, ``Z`` for UTC).

    Python 3.10's fromisoformat does not take ``Z``.
    """
    if not value:
        return None
    return as_utc(datetime.fromisoformat(str(value).replace("Z", "+00:00")))


def _format_time(moment: Optional[datetime], zone: tzinfo) -> str:
    # Not isoformat: Excel reads "2026-11-01 08:00:00" as a date, not the T/offset form.
    if moment is None:
        return ""
    return moment.astimezone(zone).strftime("%Y-%m-%d %H:%M:%S")


def _format_seconds(seconds: Optional[float]) -> str:
    return "" if seconds is None else f"{seconds:.1f}"


def _export_rows(records: Iterable[TaskRecordRow], zone: tzinfo) -> Iterator[list]:
    """EXPORT_COLUMNS' rows for ``records``: one per step, in step order."""
    zone_name = str(zone)
    for record in records:
        started, closed = as_utc(record.started_at), as_utc(record.closed_at)
        task = [
            record.task_id,
            record.run_id,
            record.name or "",
            record.kind or "",
            record.schedule_id or "",
            record.map_name or "",
            record.status,
            _format_time(started, zone),
            _format_time(closed, zone),
            _format_seconds((closed - started).total_seconds()),
        ]
        for index, step in enumerate(record.steps, start=1):
            step_started = _parse_time(step.get("started_at"))
            step_ended = _parse_time(step.get("ended_at"))
            paused = float(step.get("paused_seconds") or 0.0)
            wall = (
                (step_ended - step_started).total_seconds()
                if step_started is not None and step_ended is not None
                else None
            )
            yield task + [
                index,
                step.get("id", ""),
                step.get("type", ""),
                step.get("group") or "",
                step.get("status", ""),
                _format_time(step_started, zone),
                _format_time(step_ended, zone),
                _format_seconds(wall),
                # Blank for a step that never ran, rather than a measured zero.
                _format_seconds(paused if step_started is not None else None),
                _format_seconds(max(0.0, wall - paused) if wall is not None else None),
                step.get("error_msg") or "",
                zone_name,
            ]


def _csv_chunks(rows: Iterable[list]) -> Iterator[str]:
    """``rows`` as CSV text, a header first, a few hundred rows per chunk.

    Starts with a UTF-8 BOM: without it Excel reads the file as the system's
    code page and garbles any non-ASCII (an error message, a template name).
    """
    buffer = io.StringIO()
    writer = csv.writer(buffer)

    def drain() -> str:
        text = buffer.getvalue()
        buffer.seek(0)
        buffer.truncate(0)
        return text

    buffer.write("\ufeff")
    writer.writerow(EXPORT_COLUMNS)
    for count, row in enumerate(rows, start=1):
        writer.writerow(row)
        if count % _EXPORT_CHUNK_ROWS == 0:
            yield drain()
    yield drain()


def _export_zone(tz: Optional[str]) -> tzinfo:
    if not tz:
        return timezone.utc
    try:
        return ZoneInfo(tz)
    except (ZoneInfoNotFoundError, ValueError):
        raise BadRequestError(f"Unknown time zone: {tz}")


def _export_filename(since: Optional[datetime], until: Optional[datetime]) -> str:
    parts = [f"{moment:%Y%m%d}" for moment in (since, until) if moment is not None]
    return "-".join(["task-report", *parts]) + ".csv"


def _utc(moment: Optional[datetime]) -> Optional[datetime]:
    """A naive query datetime is UTC, not the server's local time."""
    if moment is not None and moment.tzinfo is None:
        return moment.replace(tzinfo=timezone.utc)
    return moment


def _check_window(since: Optional[datetime], until: Optional[datetime]) -> None:
    # Cross-field, so a 400 with a sentence rather than a pydantic bound.
    if since is not None and until is not None and until < since:
        raise BadRequestError("until must not be before since")


class _HistoryFilter:
    """The filters GET /task_history and /task_history/stats share, as one
    dependency: declared once, so the two cannot drift, and checked once.

    A class dependency, so OpenAPI still lists the five query parameters.
    Naive datetimes become UTC and an inverted window is the 400 sentence
    (raised from here, it goes through the same exception handlers).
    """

    def __init__(
        self,
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
        self.status = status.value if status is not None else None
        self.since = since
        self.until = until
        self.kind = kind
        self.name = name

    def as_kwargs(self) -> dict:
        """What the gateway's history verbs take."""
        return {
            "status": self.status,
            "since": self.since,
            "until": self.until,
            "kind": self.kind,
            "name": self.name,
        }


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
    map_usage: MapUsagePolicy,
    robot_gw: MoveMessageSource,
    task_record_repo: TaskRecordRepo,
) -> APIRouter:
    task_router = APIRouter(prefix="", tags=["Task"])

    def stamp_map(steps, expected_map: Optional[str]) -> Optional[str]:
        """The map a job is stamped with: the loaded one, if it drives to a pose.

        Refused (409, move_refusal's code) when that is not ``expected_map``. A
        job that does not (postures, SPEAK, WAIT only) holds no map and is
        neither stamped nor refused.
        """
        if not any(step.type in POSE_STEP_TYPES for step in steps):
            return None
        active = map_catalog_repo.active_name()
        refusal = map_usage.move_refusal(expected_map, active)
        if refusal is not None:
            raise ConflictError(refusal.message, code=refusal.code)
        return active

    @task_router.post("/api/v1/tasks", response_model=TaskResponse)
    async def trigger_task(req: TaskRequest):
        if (req.steps is None) == (req.continue_from is None):
            raise BadRequestError("Provide exactly one of steps or continue_from.")
        if req.continue_from is not None:
            return await continue_task(req)

        # A sentence, not a 422: see step_group_problem.
        problem = step_group_problem(req.steps)
        if problem is not None:
            raise problem.refusal()

        map_name = stamp_map(req.steps, req.map_name)

        workflow_task = WorkflowTask(
            id=req.id,
            definition=WorkflowTaskDefinition(
                steps=[step.to_step() for step in req.steps],
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

    async def continue_task(req: TaskRequest) -> TaskResponse:
        """A new run of a FAILED task's steps from the failed one on (continuation_steps).

        The positions are the failed run's, so its map is the expectation; the
        caller sends none. A scheduled run's continuation is the operator's
        dispatch, so it is a TASK, not a SCHEDULE.
        """
        if req.map_name is not None:
            raise BadRequestError(
                "map_name is not taken with continue_from: the task continued "
                "carries its own."
            )

        failed = await workflow_gw.get_failed_run(req.continue_from)
        continued = continuation_steps(failed.steps)
        if continued is None:
            raise ConflictError(
                f"Task {req.continue_from} has no failed step to continue from",
                code="no_failed_step",
            )
        steps, from_step_id = continued

        map_name = stamp_map(
            [step for step in steps if step.status is StepStatus.PENDING],
            failed.map_name,
        )

        kind = req.kind or failed.kind
        if kind is TaskKind.SCHEDULE:
            kind = TaskKind.TASK

        await workflow_gw.start_task(
            request=WorkflowTask(
                id=req.id, definition=WorkflowTaskDefinition(steps=steps)
            ),
            provenance=TaskProvenance(
                kind=kind, name=req.name or failed.name, map_name=map_name
            ),
        )

        return TaskResponse(
            id=req.id,
            status=TaskStatus.PENDING,
            message=(
                f"Task {req.id} accepted: continues task {req.continue_from} "
                f"from step {from_step_id}."
            ),
            continued_from=req.continue_from,
            from_step_id=from_step_id,
        )

    # Not /api/v1/tasks/preview, for the reason active_tasks gives below. Plain
    # def: it waits on the planner (seconds), so FastAPI runs it off the loop.
    # Nothing is dispatched, recorded or locked: a preview moves nothing, which
    # is also why it is not refused while a job runs.
    @task_router.post("/api/v1/route_preview", response_model=RoutePreviewResponse)
    def preview_route(req: RoutePreviewRequest):
        """The route the robot would drive from where it stands to this goal,
        planned exactly as a MOVE's would be but never sent to the robot.

        200 with ``reachable: false`` when there is no route (the planner's
        reason in ``code`` / ``message``); 409 ``map_mismatch`` when the goal
        was aimed on another map; 409 ``preview_superseded`` when a newer
        preview replaced this one; 502 when there was no planner to ask.
        """
        refusal = map_usage.move_refusal(req.map_name, map_catalog_repo.active_name())
        if refusal is not None:
            raise ConflictError(refusal.message, code=refusal.code)

        success, message, preview = robot_gw.preview_route(
            x=req.x, y=req.y, yaw=math.radians(req.theta), z=req.z
        )
        if not success or preview is None:
            if failure_code(message) is Failure.PREVIEW_SUPERSEDED:
                raise ConflictError(message, code=Failure.PREVIEW_SUPERSEDED.value)
            logger.warning("Route preview failed", message=message)
            raise UpstreamError(message)

        return RoutePreviewResponse(
            reachable=preview.reachable,
            points=list(preview.points),
            length_m=preview.length_m,
            code=preview.code,
            message=preview.message,
        )

    # Built once: it is read off the params models, which do not change while
    # the process runs.
    catalogue = step_catalogue()

    # Not /api/v1/tasks/steps, for the reason active_tasks gives below. No
    # gateway or disk behind it, so the console can read it on every editor
    # open without a cost to anything else.
    @task_router.get("/api/v1/task_steps", response_model=StepCatalogue)
    async def list_task_steps():
        return catalogue

    # Not /api/v1/tasks/active: /api/v1/tasks/{id} would shadow it depending on
    # declaration order (see interfaces/rest/server.py).
    #
    # A list: SKIP only stops a schedule overlapping itself, so a direct dispatch
    # during a scheduled run leaves two executions running. Never 404s.
    @task_router.get("/api/v1/active_tasks", response_model=ActiveTasksResponse)
    async def list_active_tasks():
        tasks, ended, as_of = await workflow_gw.list_active_tasks()
        # Only read when some run lacks the attribute (see map_in_use), so the
        # console's 2 s poll usually costs no INI read.
        active = (
            map_catalog_repo.active_name()
            if any(not task.map_name for task in tasks)
            else None
        )

        return ActiveTasksResponse(
            tasks=[
                _active_task_response(task, map_name=task.map_in_use(active))
                for task in tasks
            ],
            # The map as stamped: a closed run holds none, so map_in_use's
            # fallback to the loaded map would over-claim.
            recently_ended=[
                _active_task_response(task, map_name=task.map_name)
                for task in sorted(ended, key=lambda t: t.ended_at, reverse=True)
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
        filters: _HistoryFilter = Depends(),
    ):
        entries, next_token = await workflow_gw.list_task_history(
            page_size=page_size,
            next_page_token=_decode_page_token(page_token),
            **filters.as_kwargs(),
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
    async def task_history_stats(filters: _HistoryFilter = Depends()):
        stats = await workflow_gw.task_history_stats(**filters.as_kwargs())

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

    # The report: every finished run in the filter, one CSV row per step. From
    # task_record, not Temporal, so it reaches back TASK_RECORD_RETENTION_DAYS
    # (30) rather than the namespace's 5 -- and holds only runs that closed
    # since the table existed. Plain def: the rows come off psycopg2, streamed
    # in batches from the threadpool so a month of soak test is never in memory.
    @task_router.get(
        "/api/v1/task_history/export",
        response_class=StreamingResponse,
        responses={200: {"content": {"text/csv": {}}}},
    )
    def export_task_history(
        filters: _HistoryFilter = Depends(),
        tz: Optional[str] = Query(
            None,
            description=(
                "IANA time zone the times are written in, e.g. Asia/Taipei; "
                "UTC when omitted. The filter's since/until are unaffected"
            ),
        ),
    ):
        zone = _export_zone(tz)
        records = task_record_repo.iter_records(**filters.as_kwargs())
        return StreamingResponse(
            _csv_chunks(_export_rows(records, zone)),
            media_type="text/csv; charset=utf-8",
            headers={
                "Content-Disposition": (
                    f'attachment; filename="{_export_filename(filters.since, filters.until)}"'
                )
            },
        )

    def _step_message(step: Step) -> str:
        # Read live from the gateway: the workflow cannot see an activity's
        # progress. Only a running MOVE / PATROL / CHARGE has a goal; between retry
        # attempts the gateway answers "".
        if step.type in NAV_STEP_TYPES and step.status is StepStatus.IN_PROGRESS:
            return robot_gw.get_move_message()
        return ""

    def _step_charge(step: Step) -> Optional[ChargerPose]:
        # The params as dispatched, which a template already resolved from its
        # vertex: the charger the robot is walking onto, not the vertex as it
        # may have been edited since. The waypoint as stored (the marker, the
        # heading out of its face) is also what is sent.
        if step.type is StepType.CHARGE and isinstance(step.params, MoveParams):
            return ChargerPose(x=step.params.x, y=step.params.y, theta=step.params.theta)
        return None

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
                    charge=_step_charge(step),
                    started_at=step.started_at,
                    ended_at=step.ended_at,
                    paused_seconds=step.paused_seconds,
                    waypoints_passed=step.waypoints_passed,
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
