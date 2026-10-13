"""REST surface for the operator's library of re-dispatchable task templates.

``POST /api/v1/tasks`` persists nothing, and Temporal keeps closed workflows for
one day, so templates are what lets an operator run a step list again.

**Vertex resolution happens on the read.** A MOVE step stores a ``vertex_id``
and a ``params`` snapshot; every read reports ``resolved_params`` -- the
vertex's current pose, or the snapshot once it is deleted -- and a client
dispatches by sending that through ``POST /api/v1/tasks``, the single dispatch
path (no server-side ``/dispatch``: the operator sees the resolved numbers and
the "vertex was deleted" warning first). Temporal gets **a label, not a
reference**: the template's name (``TaskName``) and, for a schedule, its id in
the memo, never a ``vertex_id``, so nothing there goes stale on a vertex edit.

**The path is ``/api/v1/task_templates``, not ``/api/v1/tasks/templates``**:
``GET /api/v1/tasks/{id}`` takes any ``str``, so a static sub-path would be
shadowed by it or steal the id ``"templates"``, depending on include order.
"""

import uuid
import structlog

from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple, Union

from fastapi import APIRouter
from fastapi.concurrency import run_in_threadpool
from pydantic import BaseModel, Field, field_validator, model_validator

from syncai_backend.database.models import MapPoint, TaskTemplate
from syncai_backend.exceptions import BadRequestError, NotFoundError
from syncai_backend.gateways.workflow.schema import (
    MoveParams,
    ScheduleTask,
    ScheduleTrigger,
    Step,
    StepParams,
    StepType,
    WorkflowTaskDefinition,
    step_group_problem,
)
from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.interfaces.rest.routers.schedule import (
    ScheduleResponse,
    ScheduleTriggerRequest,
)
from syncai_backend.interfaces.rest.routers.task import StepRequest
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.task.task_template import TaskTemplateRepo


# --- Request models ---------------------------------------------------------


class TemplateStepRequest(StepRequest):
    """A step as it is *stored*: a StepRequest plus where its numbers came from.

    Subclassed so ``validate_step_params`` stays the one per-type params check.
    """

    vertex_id: Optional[uuid.UUID] = Field(
        default=None,
        description=(
            "The map vertex this MOVE step's coordinates were taken from. "
            "`params` is kept as a snapshot either way: a dispatch prefers the "
            "vertex's current pose and falls back to the snapshot once the "
            "vertex has been deleted."
        ),
    )

    # Not named _check_params: that would override the inherited validator and
    # silently drop the params check.
    @model_validator(mode="after")
    def _check_vertex_ref(self) -> "TemplateStepRequest":
        if self.vertex_id is not None and self.type is not StepType.MOVE:
            raise ValueError(f"{self.type.value} step cannot reference a vertex")
        return self


class _StoredStep(TemplateStepRequest):
    """The persisted shape: a TemplateStepRequest plus the server-owned label.

    ``vertex_name`` is resolved at save time as part of the snapshot, so a
    deleted vertex can still be named. It is never used to re-find a vertex
    (``vertex_id`` is the only reference), and it is not on the request model
    so a client cannot send a name that disagrees with its vertex.
    """

    vertex_name: Optional[str] = Field(default=None)


class TaskTemplateRequest(BaseModel):
    name: str = Field(..., min_length=1, max_length=255)
    description: str = Field(default="", max_length=1000)
    map_name: Optional[str] = Field(
        default=None,
        description=(
            "The map whose frame the MOVE coordinates are in. Required when the "
            "template has any MOVE step; must be omitted when it has none."
        ),
    )
    # min_length=1, unlike TaskRequest: an empty template could never be run.
    steps: List[TemplateStepRequest] = Field(..., min_length=1)

    @field_validator("name")
    @classmethod
    def _strip_name(cls, value: str) -> str:
        # min_length=1 alone lets " " through.
        stripped = value.strip()
        if not stripped:
            raise ValueError("name must not be blank")
        return stripped


class TaskTemplateUpdateRequest(BaseModel):
    """Fields to change; omitted ones are left alone.

    ``steps`` replaces the whole list; there is no per-step patch.
    """

    name: Optional[str] = Field(default=None, min_length=1, max_length=255)
    description: Optional[str] = Field(default=None, max_length=1000)
    map_name: Optional[str] = None
    steps: Optional[List[TemplateStepRequest]] = Field(default=None, min_length=1)

    @field_validator("name")
    @classmethod
    def _strip_name(cls, value: Optional[str]) -> Optional[str]:
        if value is None:
            return None
        stripped = value.strip()
        if not stripped:
            raise ValueError("name must not be blank")
        return stripped


class TaskTemplateScheduleRequest(BaseModel):
    id: str = Field(
        ..., min_length=1, description="Temporal schedule id to register"
    )
    trigger: ScheduleTriggerRequest = Field(..., description="When the schedule fires")


# --- Response models --------------------------------------------------------


class VertexRefStatus(str, Enum):
    NONE = "NONE"
    """The step references no vertex; its coordinates were typed by hand."""

    CURRENT = "CURRENT"
    """The vertex exists; `resolved_params` and `vertex_name` come from it."""

    MISSING = "MISSING"
    """The vertex is gone; `resolved_params` is the snapshot taken at save time."""


class TemplateStepResponse(BaseModel):
    id: str
    type: StepType
    params: Optional[StepParams] = Field(
        default=None, description="The snapshot, exactly as it was saved."
    )
    vertex_id: Optional[uuid.UUID] = None
    vertex_name: Optional[str] = Field(
        default=None,
        description=(
            "The vertex's CURRENT name when vertex_status is CURRENT, the "
            "snapshot label when MISSING, null otherwise. vertex_status is what "
            "says which of those you are reading."
        ),
    )
    group: Optional[str] = None
    vertex_status: VertexRefStatus
    resolved_params: Optional[StepParams] = Field(
        default=None,
        description=(
            "What a dispatch should send right now. Equal to `params` for a "
            "non-MOVE step, a hand-typed MOVE, or a MISSING vertex; the vertex's "
            "live pose when CURRENT."
        ),
    )


class TaskTemplateResponse(BaseModel):
    """One stored template.

    ``id`` is the row's UUID, **not** a task id: ``GET /api/v1/tasks/{id}``
    wants the Temporal workflow id of a run.
    """

    id: uuid.UUID
    name: str
    description: str
    map_name: Optional[str]
    steps: List[TemplateStepResponse]
    map_matches_active: bool = Field(
        ...,
        description=(
            "True when this template's map is the one the robot is on, or the "
            "template is map-independent and so has nothing to mismatch. A "
            "client's dispatch guard: coordinates from another map's frame point "
            "somewhere else in the loaded one."
        ),
    )
    missing_vertex_count: int = Field(
        ..., description="How many steps have vertex_status MISSING."
    )
    created_at: datetime
    updated_at: datetime


class DeleteResponse(BaseModel):
    message: str


# --- Validation helpers -----------------------------------------------------
#
# Handler-side checks answering 400 with a sentence, not model_validators: a
# 422's detail is an array the console cannot show, and a PUT's body must be
# checked merged with the stored row, which a request validator cannot see.


def _check_step_ids(steps: List[TemplateStepRequest]) -> None:
    """Reject duplicate step ids (a durable ambiguity in a template)."""
    seen: set[str] = set()
    duplicates: list[str] = []
    for step in steps:
        if step.id in seen and step.id not in duplicates:
            duplicates.append(step.id)
        seen.add(step.id)
    if duplicates:
        raise ValueError(f"duplicate step ids: {duplicates}")


def _check_step_groups(steps: List[TemplateStepRequest]) -> None:
    # Raised as the 400 itself: a ValueError would drop the step it names.
    problem = step_group_problem(steps)
    if problem is not None:
        raise problem.refusal()


def _check_map_scope(map_name: Optional[str], steps: List[TemplateStepRequest]) -> None:
    """Any MOVE step means a map name is required; none means it is forbidden.

    Any MOVE, not just a vertex reference: hand-typed coordinates are in a
    map's frame too.
    """
    has_move = any(step.type is StepType.MOVE for step in steps)
    if has_move and not map_name:
        raise ValueError(
            "a template with MOVE steps must name the map its coordinates are in"
        )
    if not has_move and map_name:
        raise ValueError(
            "a template with no MOVE step is map-independent; omit map_name"
        )


def init_task_template_router(
    logger: structlog.stdlib.BoundLogger,
    task_template_repo: TaskTemplateRepo,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    workflow_gw: WorkflowGateway,
) -> APIRouter:
    task_template_router = APIRouter(prefix="", tags=["Task template"])

    # --- Helpers ------------------------------------------------------------

    def _require(task_id: uuid.UUID) -> TaskTemplate:
        row = task_template_repo.get_task_template(task_id=task_id)
        if row is None:
            raise NotFoundError(f"Task template {task_id} was not found.")
        return row

    def _require_map(map_name: Optional[str]) -> None:
        """400, not 404, when a body names a map this robot does not have.

        The body is wrong, not the addressed resource.
        """
        if map_name is None:
            return
        if map_catalog_repo.get_map(map_name) is None:
            raise BadRequestError(f"No map named '{map_name}' on this robot.")

    def _referenced_ids(
        steps: Sequence[Union[TemplateStepRequest, _StoredStep]]
    ) -> List[uuid.UUID]:
        """The distinct vertex ids of request or stored steps, first-seen order."""
        seen: Dict[uuid.UUID, None] = {}
        for step in steps:
            if step.vertex_id is not None:
                seen.setdefault(step.vertex_id, None)
        return list(seen)

    def _vertices(
        map_name: Optional[str], vertex_ids: Sequence[uuid.UUID]
    ) -> Dict[uuid.UUID, MapPoint]:
        """Resolve just the referenced vertices, keyed by id.

        Only the named ids, not the whole map (which may hold thousands).
        Absence means "not a vertex of ``map_name``": ``get_vertices`` filters
        on the map as well as the ids.
        """
        if map_name is None:
            return {}
        return {v.id: v for v in map_repo.get_vertices(map=map_name, vertex_ids=vertex_ids)}

    def _require_vertex_refs(
        map_name: Optional[str],
        steps: List[TemplateStepRequest],
        vertices: Dict[uuid.UUID, MapPoint],
    ) -> None:
        """Every referenced vertex must exist and belong to this template's map.

        "Missing" and "on another map" are reported separately, unlike the map
        router's ``_require_vertex``: here the body names both, so there is no
        URL scope to protect, and the specific sentence is what makes it fixable.
        """
        for step in steps:
            if step.vertex_id is None:
                continue
            if step.vertex_id in vertices:
                continue
            other = map_repo.get_vertex(vertex_id=step.vertex_id)
            if other is None:
                raise BadRequestError(
                    f"Step '{step.id}' references vertex {step.vertex_id}, "
                    "which does not exist."
                )
            raise BadRequestError(
                f"Step '{step.id}' references vertex {step.vertex_id}, which is "
                f"on map '{other.map}', not '{map_name}'."
            )

    def _check(map_name: Optional[str], steps: List[TemplateStepRequest]) -> None:
        try:
            _check_step_ids(steps)
            _check_map_scope(map_name, steps)
            _check_step_groups(steps)
        except ValueError as exc:
            raise BadRequestError(str(exc))

    def _stored_steps(
        steps: List[TemplateStepRequest], vertices: Dict[uuid.UUID, MapPoint]
    ) -> list[dict]:
        """Freeze the request's steps into the JSON column's element shape.

        Snapshot coordinates are stored verbatim as posted, never replaced with
        the vertex's pose: reads resolve against the live vertex anyway.

        ``mode="json"``: uuid.UUID is not JSON-serialisable. No
        ``exclude_none``: explicit nulls keep a *missing* key meaning "written by
        an older backend". No ``by_alias``: keeps the blob snake_case.
        """
        stored: list[dict] = []
        for step in steps:
            vertex = vertices.get(step.vertex_id) if step.vertex_id else None
            stored.append(
                _StoredStep(
                    id=step.id,
                    type=step.type,
                    params=step.params,
                    group=step.group,
                    vertex_id=step.vertex_id,
                    vertex_name=vertex.name if vertex else None,
                ).model_dump(mode="json")
            )
        return stored

    def _read_steps(row: TaskTemplate) -> List[_StoredStep]:
        return [_StoredStep.model_validate(entry) for entry in row.steps]

    def _step_response(
        stored: _StoredStep, vertices: Dict[uuid.UUID, MapPoint]
    ) -> TemplateStepResponse:
        if stored.vertex_id is None:
            return TemplateStepResponse(
                id=stored.id,
                type=stored.type,
                params=stored.params,
                group=stored.group,
                vertex_id=None,
                vertex_name=None,
                vertex_status=VertexRefStatus.NONE,
                resolved_params=stored.params,
            )

        vertex = vertices.get(stored.vertex_id)
        if vertex is None:
            return TemplateStepResponse(
                id=stored.id,
                type=stored.type,
                params=stored.params,
                group=stored.group,
                vertex_id=stored.vertex_id,
                # The snapshot label: the only human handle for what is gone.
                vertex_name=stored.vertex_name,
                vertex_status=VertexRefStatus.MISSING,
                resolved_params=stored.params,
            )

        return TemplateStepResponse(
            id=stored.id,
            type=stored.type,
            params=stored.params,
            group=stored.group,
            vertex_id=stored.vertex_id,
            # The vertex's *current* name, so a rename shows through.
            vertex_name=vertex.name,
            vertex_status=VertexRefStatus.CURRENT,
            # Moving a vertex moves every template that references it.
            resolved_params=MoveParams(x=vertex.x, y=vertex.y, theta=vertex.theta),
        )

    def _response(
        row: TaskTemplate,
        active_name: Optional[str],
        vertices: Dict[uuid.UUID, MapPoint],
        stored_steps: List[_StoredStep],
    ) -> TaskTemplateResponse:
        """``stored_steps`` is ``_read_steps(row)``, passed in to parse it once."""
        steps = [_step_response(stored, vertices) for stored in stored_steps]
        return TaskTemplateResponse(
            id=row.id,
            name=row.name,
            description=row.description,
            map_name=row.map_name,
            steps=steps,
            # A map-independent template has nothing to mismatch.
            map_matches_active=row.map_name is None or row.map_name == active_name,
            missing_vertex_count=sum(
                1 for step in steps if step.vertex_status is VertexRefStatus.MISSING
            ),
            created_at=row.created_at,
            updated_at=row.updated_at,
        )

    # --- Routes -------------------------------------------------------------
    #
    # `response_model_by_alias=False` is load-bearing on every route returning
    # Step objects: BaseSchema's to_camel aliases would otherwise answer
    # multi-word fields in camelCase, and the frontend is snake_case-only.
    #
    # Plain `def` except /schedule: psycopg2 blocks, so they run in the
    # threadpool.

    @task_template_router.post(
        "/api/v1/task_templates",
        response_model=TaskTemplateResponse,
        response_model_by_alias=False,
    )
    def create_task_template(req: TaskTemplateRequest):
        _check(req.map_name, req.steps)
        _require_map(req.map_name)
        vertices = _vertices(req.map_name, _referenced_ids(req.steps))
        _require_vertex_refs(req.map_name, req.steps, vertices)

        row = task_template_repo.create_task_template(
            name=req.name,
            description=req.description,
            map_name=req.map_name,
            steps=_stored_steps(req.steps, vertices),
        )
        return _response(row, map_catalog_repo.active_name(), vertices, _read_steps(row))

    @task_template_router.get(
        "/api/v1/task_templates",
        response_model=List[TaskTemplateResponse],
        response_model_by_alias=False,
    )
    def list_task_templates(map_name: Optional[str] = None):
        """Every template, or one map's plus the map-independent ones.

        The console fetches everything and filters client-side, so it can say
        how many rows it hid; ``map_name`` is for other consumers.
        """
        rows = task_template_repo.list_task_templates(map_name=map_name)
        active_name = map_catalog_repo.active_name()

        steps_by_row: List[List[_StoredStep]] = [_read_steps(row) for row in rows]

        # One vertex query per distinct map, for just the referenced ids.
        ids_by_map: Dict[Optional[str], Dict[uuid.UUID, None]] = {}
        for row, stored_steps in zip(rows, steps_by_row):
            referenced = ids_by_map.setdefault(row.map_name, {})
            for vertex_id in _referenced_ids(stored_steps):
                referenced.setdefault(vertex_id, None)

        by_map: Dict[Optional[str], Dict[uuid.UUID, MapPoint]] = {
            name: _vertices(name, list(ids)) for name, ids in ids_by_map.items()
        }

        return [
            _response(row, active_name, by_map[row.map_name], stored_steps)
            for row, stored_steps in zip(rows, steps_by_row)
        ]

    @task_template_router.get(
        "/api/v1/task_templates/{id}",
        response_model=TaskTemplateResponse,
        response_model_by_alias=False,
    )
    def get_task_template(id: uuid.UUID):
        row = _require(id)
        stored_steps = _read_steps(row)
        return _response(
            row,
            map_catalog_repo.active_name(),
            _vertices(row.map_name, _referenced_ids(stored_steps)),
            stored_steps,
        )

    @task_template_router.put(
        "/api/v1/task_templates/{id}",
        response_model=TaskTemplateResponse,
        response_model_by_alias=False,
    )
    def update_task_template(id: uuid.UUID, req: TaskTemplateUpdateRequest):
        row = _require(id)
        changes = req.model_dump(exclude_unset=True)

        # Merged before checking: the map/steps invariant spans body and stored
        # row, and re-checking stored steps against a new map keeps them from
        # pointing at the old map's vertices.
        merged_map = changes["map_name"] if "map_name" in changes else row.map_name
        merged_steps: List[TemplateStepRequest] = (
            req.steps if req.steps is not None else list(_read_steps(row))
        )

        _check(merged_map, merged_steps)
        _require_map(merged_map)
        vertices = _vertices(merged_map, _referenced_ids(merged_steps))
        _require_vertex_refs(merged_map, merged_steps, vertices)

        if "steps" in changes and req.steps is not None:
            changes["steps"] = _stored_steps(req.steps, vertices)

        updated = task_template_repo.update_task_template(task_id=id, **changes)
        if updated is None:
            raise NotFoundError(f"Task template {id} was not found.")
        return _response(
            updated, map_catalog_repo.active_name(), vertices, _read_steps(updated)
        )

    @task_template_router.delete(
        "/api/v1/task_templates/{id}", response_model=DeleteResponse
    )
    def delete_task_template(id: uuid.UUID):
        _require(id)
        task_template_repo.delete_task_template(task_id=id)
        return DeleteResponse(message=f"Task template {id} has been deleted.")

    @task_template_router.post(
        "/api/v1/task_templates/{id}/schedule", response_model=ScheduleResponse
    )
    async def schedule_task_template(id: uuid.UUID, req: TaskTemplateScheduleRequest):
        """Freeze this template's current resolution into a Temporal schedule.

        ``async`` for the create_schedule gRPC; the psycopg2 work goes to the
        threadpool in one hop so a wedged Postgres cannot stall the event loop.

        The schedule stores concrete steps, so later vertex edits do not reach
        it. Because a scheduled run is unattended, it is refused (400) for
        another map than the loaded one or a deleted vertex, which a hand
        dispatch only warns about.
        """

        def _freeze() -> Tuple[TaskTemplate, List[Step]]:
            row = _require(id)
            active_name = map_catalog_repo.active_name()

            if row.map_name is not None and row.map_name != active_name:
                raise BadRequestError(
                    f"Template '{row.name}' is for map '{row.map_name}', but the "
                    f"robot has {active_name or 'no map'} loaded. A scheduled run "
                    "is unattended, so it will not be registered against another "
                    "map's coordinate frame."
                )

            stored_steps = _read_steps(row)
            vertices = _vertices(row.map_name, _referenced_ids(stored_steps))
            steps: List[Step] = []
            for stored in stored_steps:
                resolved = _step_response(stored, vertices)
                if resolved.vertex_status is VertexRefStatus.MISSING:
                    raise BadRequestError(
                        f"Step '{stored.id}' references vertex "
                        f"'{stored.vertex_name or stored.vertex_id}', which no "
                        "longer exists. A scheduled run is unattended, so it will "
                        "not be registered against a stale snapshot — re-point or "
                        "remove the step first."
                    )
                # vertex_id / vertex_name never enter the Temporal step schema;
                # the template's name does, via create_schedule.
                steps.append(
                    Step(
                        id=stored.id,
                        type=stored.type,
                        params=resolved.resolved_params,
                        group=stored.group,
                    )
                )
            return row, steps

        row, steps = await run_in_threadpool(_freeze)

        await workflow_gw.create_schedule(
            schedule=ScheduleTask(
                id=req.id,
                trigger=ScheduleTrigger(
                    cron=req.trigger.cron,
                    interval_seconds=req.trigger.interval_seconds,
                    timezone=req.trigger.timezone,
                ),
                definition=WorkflowTaskDefinition(steps=steps),
                map_name=row.map_name,
                task_template_id=str(row.id),
                task_template_name=row.name,
            )
        )

        return ScheduleResponse(
            id=req.id,
            message=(
                f"Schedule {req.id} runs '{row.name}' ({len(steps)} steps), "
                "frozen at the vertices' current positions."
            ),
        )

    return task_template_router
