"""A task template's steps resolved against its map's vertices.

One rule, read by the template routes (what a dispatch would send now) and by a
scheduled run as it starts (what it sends): a step taken from a vertex goes
where the vertex is now, and to the snapshot saved with it once the vertex is
gone. Here, in the task model's layer, because the worker runs it too and must
not import a router.

Vertices are duck-typed (``x`` / ``y`` / ``theta`` / ``z``): the ORM's MapPoint
in the backend, anything shaped like it in a test.
"""

import uuid

from enum import Enum
from typing import Any, List, Mapping, Optional, Protocol, Sequence, Tuple

from pydantic import Field

from syncai_backend.tasks.schema import (
    BaseSchema,
    MoveParams,
    PatrolParams,
    Step,
    StepType,
)


class VertexRefStatus(str, Enum):
    NONE = "NONE"
    """The step references no vertex; its coordinates were typed by hand."""

    CURRENT = "CURRENT"
    """The vertex exists; `resolved_params` and `vertex_name` come from it."""

    MISSING = "MISSING"
    """The vertex is gone; `resolved_params` is the snapshot taken at save time."""


class VertexPose(Protocol):
    x: float
    y: float
    theta: float
    z: float


def resolve_pose(
    snapshot: MoveParams,
    vertex_id: Optional[uuid.UUID],
    vertices: Mapping[uuid.UUID, VertexPose],
) -> Tuple[MoveParams, VertexRefStatus]:
    """Where one pose taken from ``vertex_id`` points now, and why."""
    if vertex_id is None:
        return snapshot, VertexRefStatus.NONE
    vertex = vertices.get(vertex_id)
    if vertex is None:
        return snapshot, VertexRefStatus.MISSING
    # Moving a vertex moves every template that references it.
    return (
        MoveParams(x=vertex.x, y=vertex.y, theta=vertex.theta, z=vertex.z),
        VertexRefStatus.CURRENT,
    )


def _uuid(value: Any) -> Optional[uuid.UUID]:
    if value is None or isinstance(value, uuid.UUID):
        return value
    return uuid.UUID(str(value))


def _patrol_vertex_ids(entry: Mapping[str, Any], count: int) -> List[Optional[uuid.UUID]]:
    # Omitted on a PATROL saved without any vertex: every waypoint a free point.
    ids = entry.get("vertex_ids")
    if ids is None:
        return [None] * count
    return [_uuid(vid) for vid in ids]


def stored_vertex_ids(entries: Sequence[Mapping[str, Any]]) -> List[uuid.UUID]:
    """The distinct vertex ids a stored step list names, first-seen order."""
    seen: dict[uuid.UUID, None] = {}
    for entry in entries:
        if entry.get("type") == StepType.PATROL.value:
            waypoints = (entry.get("params") or {}).get("waypoints") or []
            ids = _patrol_vertex_ids(entry, len(waypoints))
        else:
            ids = [_uuid(entry.get("vertex_id"))]
        for vid in ids:
            if vid is not None:
                seen.setdefault(vid, None)
    return list(seen)


def resolve_stored_steps(
    entries: Sequence[Mapping[str, Any]],
    vertices: Mapping[uuid.UUID, VertexPose],
) -> List[Step]:
    """A template's stored steps (its JSON column) as the steps a run executes.

    A missing vertex is not refused here: a scheduled run is unattended, and
    the snapshot is where the operator last saw the step go.
    """
    steps: List[Step] = []
    for entry in entries:
        step = Step(
            id=entry["id"],
            type=entry["type"],
            params=entry.get("params"),
            group=entry.get("group"),
        )
        if step.type is StepType.PATROL:
            ids = _patrol_vertex_ids(entry, len(step.params.waypoints))
            step.params = PatrolParams(
                waypoints=[
                    resolve_pose(waypoint, vid, vertices)[0]
                    for waypoint, vid in zip(step.params.waypoints, ids)
                ]
            )
        elif isinstance(step.params, MoveParams):
            step.params, _ = resolve_pose(
                step.params, _uuid(entry.get("vertex_id")), vertices
            )
        steps.append(step)
    return steps


class ResolvedTemplate(BaseSchema):
    """What a scheduled run executes, read from its template as it starts."""

    name: str = Field(..., description="The template's name now (the run's TaskName).")
    map_name: Optional[str] = Field(
        default=None, description="The map its positions are in now (the run's TaskMap)."
    )
    steps: List[Step] = Field(..., description="Its steps, resolved against that map.")
    refusal: Optional[str] = Field(
        default=None,
        description=(
            "Why the run must not go ahead (another map is loaded), as the "
            "operator reads it; the run fails with it as its first step's error."
        ),
    )
