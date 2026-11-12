"""The one rule a template's steps resolve by, for the routes and a scheduled run.

What is pinned: a step taken from a vertex goes where the vertex is now, to its
snapshot once the vertex is gone, and a hand-typed one stays as typed; a PATROL
resolves each waypoint that way; the vertex ids are read off the stored JSON in
the shapes it has been written in (a PATROL saved without ``vertex_ids``).
"""

import uuid
from types import SimpleNamespace

from syncai_backend.tasks.schema import MoveParams, StepType
from syncai_backend.tasks.template_resolution import (
    VertexRefStatus,
    resolve_pose,
    resolve_stored_steps,
    stored_vertex_ids,
)


SNAPSHOT = MoveParams(x=0.0, y=0.0, theta=0.0)
DOCK = uuid.UUID("0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55")
GONE = uuid.UUID("11111111-2222-3333-4444-555555555555")
VERTICES = {DOCK: SimpleNamespace(x=3.0, y=-1.5, theta=90.0, z=2.5)}


def test_a_pose_resolves_to_the_vertex_its_snapshot_or_itself():
    assert resolve_pose(SNAPSHOT, DOCK, VERTICES) == (
        MoveParams(x=3.0, y=-1.5, theta=90.0, z=2.5),
        VertexRefStatus.CURRENT,
    )
    assert resolve_pose(SNAPSHOT, GONE, VERTICES) == (SNAPSHOT, VertexRefStatus.MISSING)
    assert resolve_pose(SNAPSHOT, None, VERTICES) == (SNAPSHOT, VertexRefStatus.NONE)


def test_stored_steps_resolve_into_the_steps_a_run_executes():
    snapshot = SNAPSHOT.model_dump()
    steps = resolve_stored_steps(
        [
            {"id": "1-move", "type": "MOVE", "params": snapshot, "vertex_id": str(DOCK)},
            {"id": "2-dock", "type": "DOCK", "params": snapshot, "vertex_id": str(GONE)},
            {"id": "3-patrol", "type": "PATROL",
             "params": {"waypoints": [snapshot, snapshot]},
             "vertex_ids": [None, str(DOCK)]},
            # Saved before PATROL had vertex_ids: every waypoint a free point.
            {"id": "4-patrol", "type": "PATROL",
             "params": {"waypoints": [snapshot, snapshot]}},
            {"id": "5-speak", "type": "SPEAK", "params": {"text": "Hi"}, "group": "g",
             "vertex_id": None, "vertex_name": None},
            {"id": "6-standup", "type": "STANDUP", "params": None, "group": "g"},
        ],
        VERTICES,
    )

    assert [s.type for s in steps] == [
        StepType.MOVE, StepType.DOCK, StepType.PATROL, StepType.PATROL,
        StepType.SPEAK, StepType.STANDUP,
    ]
    assert steps[0].params == MoveParams(x=3.0, y=-1.5, theta=90.0, z=2.5)
    assert steps[1].params == SNAPSHOT
    assert [(w.x, w.z) for w in steps[2].params.waypoints] == [(0.0, 0.0), (3.0, 2.5)]
    assert steps[3].params.waypoints == [SNAPSHOT, SNAPSHOT]
    assert (steps[4].params.text, steps[4].group) == ("Hi", "g")
    assert steps[5].params is None


def test_the_vertex_ids_are_read_once_each_in_first_seen_order():
    snapshot = SNAPSHOT.model_dump()
    assert stored_vertex_ids(
        [
            {"id": "1", "type": "MOVE", "params": snapshot, "vertex_id": str(GONE)},
            {"id": "2", "type": "PATROL", "params": {"waypoints": [snapshot, snapshot]},
             "vertex_ids": [str(DOCK), str(GONE)]},
            {"id": "3", "type": "PATROL", "params": {"waypoints": [snapshot, snapshot]}},
            {"id": "4", "type": "STANDUP"},
        ]
    ) == [GONE, DOCK]
