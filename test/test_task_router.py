"""Tests for /api/v1/tasks, /active_tasks and /task_history — the REST projection only.

Same pattern as ``test_schedule_router.py``: a stub gateway records what the
router hands it and answers canned views, so these tests pin the boundary
(request validation, response shapes, exception mapping) without any Temporal.
The gateway's own behaviour lives in ``test_workflow_gateway.py``.
"""

import math
from datetime import datetime, timezone

import pytest

pytest.importorskip("httpx")
pytest.importorskip("temporalio")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.exceptions import (  # noqa: E402
    BadRequestError,
    ConflictError,
    NotFoundError,
)
from syncai_backend.tasks.schema import (  # noqa: E402
    ActiveTask,
    MoveParams,
    PatrolParams,
    SpeakParams,
    Step,
    StepStatus,
    StepType,
    TaskHistoryEntry,
    TaskHistoryStats,
    TaskKind,
    TaskSource,
    TaskState,
    WaitParams,
    continuation_steps,
)
from syncai_backend.gateways.failure import Failure, fail  # noqa: E402
from syncai_backend.gateways.workflow.workflow import FailedRun  # noqa: E402
from syncai_backend.helpers.route_preview import planned, refused  # noqa: E402
from syncai_backend.interfaces.rest.routers.task import init_task_router  # noqa: E402
from syncai_backend.services.map_usage import MapUsagePolicy  # noqa: E402
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


def _failed_run(**overrides):
    """A failed run of four steps: s1 done, s2 + s3 a group whose MOVE failed
    (the SPEAK beside it canceled), s4 never started."""
    move = MoveParams(x=1.0, y=2.0, theta=90.0)
    steps = [
        Step(id="s1", type=StepType.STANDUP, status=StepStatus.COMPLETED),
        Step(id="s2", type=StepType.SPEAK, group="g",
             params=SpeakParams(text="Moving"), status=StepStatus.CANCELED,
             error_msg="Canceled: step s3 failed"),
        Step(id="s3", type=StepType.MOVE, group="g", params=move,
             status=StepStatus.FAILED, error_msg="Move ended in aborted",
             started_at=datetime(2026, 11, 5, 8, 0, tzinfo=timezone.utc)),
        Step(id="s4", type=StepType.WAIT, params=WaitParams(seconds=5)),
    ]
    fields = dict(steps=steps, kind=TaskKind.TASK, name="Morning patrol", map_name="lab")
    fields.update(overrides)
    return FailedRun(**fields)


class _StubWorkflowGateway:
    def __init__(self):
        self.started = []
        self.cancelled = []
        self.paused = []
        self.resumed = []
        self.history_calls = []
        self.stats_calls = []
        self.failed_reads = []
        self.failed = _failed_run()
        self.history = (
            [
                TaskHistoryEntry(
                    id="robot01-task-001",
                    run_id="run-1",
                    status="COMPLETED",
                    started_at=datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc),
                    closed_at=datetime(2026, 8, 10, 8, 5, tzinfo=timezone.utc),
                    source=TaskSource.DIRECT,
                    kind=TaskKind.TASK,
                    name="Morning patrol",
                )
            ],
            b"\xfftoken",
        )
        self.stats = TaskHistoryStats(
            as_of=datetime(2026, 8, 10, 9, 0, tzinfo=timezone.utc),
            total=4,
            completed=3,
            failed=1,
            canceled=0,
        )
        self.state = TaskState(
            id="robot01-task-001",
            status="IN_PROGRESS",
            steps=[
                Step(
                    id="step1",
                    type=StepType.MOVE,
                    params=MoveParams(x=1.0, y=2.0, theta=90.0),
                    status=StepStatus.IN_PROGRESS,
                    error_msg=None,
                )
            ],
        )
        self.active = (
            [
                ActiveTask(
                    id="robot01-sched-001-2026-08-10T09:00:00Z",
                    run_id="run-1",
                    status="IN_PROGRESS",
                    started_at=datetime(2026, 8, 10, 9, 0, tzinfo=timezone.utc),
                    source=TaskSource.SCHEDULE,
                    schedule_id="robot01-sched-001",
                )
            ],
            [],
            datetime(2026, 8, 10, 9, 0, 30, tzinfo=timezone.utc),
        )

    async def start_task(self, request, provenance=None):
        self.started.append((request, provenance))

    async def get_task_state(self, task_id):
        return self.state

    async def get_failed_run(self, task_id):
        self.failed_reads.append(task_id)
        return self.failed

    async def cancel_task(self, task_id):
        self.cancelled.append(task_id)

    async def pause_task(self, task_id):
        self.paused.append(task_id)

    async def resume_task(self, task_id):
        self.resumed.append(task_id)

    async def list_active_tasks(self):
        return self.active

    async def list_task_history(self, **kwargs):
        self.history_calls.append(kwargs)
        return self.history

    async def task_history_stats(self, **kwargs):
        self.stats_calls.append(kwargs)
        return self.stats


class _StubCatalog:
    def __init__(self):
        self.active = "lab"
        self.reads = 0

    def active_name(self):
        self.reads += 1
        return self.active


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


class _StubRobotGateway:
    """The router's robot reads: the running MOVE's status sentence, and the
    route preview, answered with whatever the test sets in ``preview``."""

    def __init__(self):
        self.message = ""
        self.calls = 0
        self.preview = (True, "", planned([(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]))
        self.previews = []

    def get_move_message(self):
        self.calls += 1
        return self.message

    def preview_route(self, x, y, yaw, z=0.0):
        self.previews.append({"x": x, "y": y, "yaw": yaw, "z": z})
        return self.preview


@pytest.fixture
def catalog():
    return _StubCatalog()


@pytest.fixture
def robot_gw():
    return _StubRobotGateway()


@pytest.fixture
def client(logger, workflow_gw, catalog, robot_gw, task_record_repo):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_task_router(
            logger=logger,
            workflow_gw=workflow_gw,
            map_catalog_repo=catalog,
            map_usage=MapUsagePolicy(
                map_catalog_repo=catalog,
                task_template_repo=None,
            ),
            robot_gw=robot_gw,
            task_record_repo=task_record_repo,
        )
    )
    return TestClient(app)


def _stand_task(task_id="robot01-standup-001"):
    return {"id": task_id, "kind": "standup", "steps": [{"id": "s1", "type": "STANDUP"}]}


def _move_task(task_id="robot01-task-001"):
    # `timestamp` was dropped from TaskRequest but deployed clients (console,
    # MCP) still send it; keeping it in this body pins that pydantic goes on
    # ignoring the unknown field instead of 422ing old callers.
    return {
        "id": task_id,
        "timestamp": 1782786519,
        "steps": [
            {"id": "step1", "type": "MOVE", "params": {"x": 1.0, "y": 2.0, "theta": 90.0}}
        ],
    }


def _patrol_task(task_id="robot01-patrol-001"):
    return {
        "id": task_id,
        "steps": [
            {
                "id": "s1",
                "type": "PATROL",
                "params": {
                    "waypoints": [
                        {"x": 1.0, "y": 2.0, "theta": 0.0},
                        {"x": 4.0, "y": 2.0, "theta": 90.0, "z": 3.1},
                    ]
                },
            }
        ],
    }


class TestTriggerTask:
    def test_post_builds_the_workflow_task(self, client, workflow_gw):
        body = client.post("/api/v1/tasks", json=_move_task()).json()

        assert body["status"] == "PENDING"
        task, provenance = workflow_gw.started[0]
        assert task.id == "robot01-task-001"
        step = task.definition.steps[0]
        assert step.type is StepType.MOVE
        assert step.params == MoveParams(x=1.0, y=2.0, theta=90.0)
        # A caller that says nothing about the run's origin (the MCP server,
        # curl) still dispatches; the run is simply unlabelled.
        assert (provenance.kind, provenance.name) == (None, None)

    def test_post_forwards_kind_and_name_as_provenance(self, client, workflow_gw):
        task = {**_move_task(), "kind": "goal", "name": "Morning patrol"}

        assert client.post("/api/v1/tasks", json=task).status_code == 200
        _, provenance = workflow_gw.started[0]
        assert (provenance.kind, provenance.name) == (TaskKind.GOAL, "Morning patrol")

    @pytest.mark.parametrize(
        "extra",
        [
            # The backend's own kind: a direct dispatch may not pose as a
            # scheduled run, or the dashboard's "Scheduled" row stops meaning it.
            {"kind": "schedule"},
            {"kind": "teleport"},
            {"name": ""},
            {"name": "x" * 256},
        ],
    )
    def test_a_bad_kind_or_name_is_a_422(self, client, workflow_gw, extra):
        response = client.post("/api/v1/tasks", json={**_move_task(), **extra})

        assert response.status_code == 422
        assert workflow_gw.started == []

    def test_a_mismatched_step_body_is_a_422(self, client, workflow_gw):
        # STANDUP takes no params; StepRequest's validator must reject this at
        # the boundary rather than let it 500 inside the handler.
        task = _move_task()
        task["steps"] = [
            {"id": "s1", "type": "STANDUP", "params": {"x": 0.0, "y": 0.0, "theta": 0.0}}
        ]

        assert client.post("/api/v1/tasks", json=task).status_code == 422
        assert workflow_gw.started == []

    @pytest.mark.parametrize("repeat, expected", [(None, 1), (3, 3), (50, 50)])
    def test_a_speak_step_carries_its_repeat_through(
        self, client, workflow_gw, repeat, expected
    ):
        # Omitted is once, so every SPEAK stored before the field speaks as it did.
        params = {"text": "hi"} if repeat is None else {"text": "hi", "repeat": repeat}
        task = _move_task()
        task["steps"] = [{"id": "s1", "type": "SPEAK", "params": params}]

        assert client.post("/api/v1/tasks", json=task).status_code == 200
        started, _ = workflow_gw.started[0]
        assert started.definition.steps[0].params.repeat == expected

    @pytest.mark.parametrize("repeat", [0, 51])
    def test_a_speak_repeat_out_of_range_is_a_422(self, client, workflow_gw, repeat):
        task = _move_task()
        task["steps"] = [
            {"id": "s1", "type": "SPEAK", "params": {"text": "hi", "repeat": repeat}}
        ]

        assert client.post("/api/v1/tasks", json=task).status_code == 422
        assert workflow_gw.started == []

    def test_a_wait_step_carries_its_seconds_through(self, client, workflow_gw):
        task = _move_task()
        task["steps"] = [{"id": "s1", "type": "WAIT", "params": {"seconds": 12.5}}]

        assert client.post("/api/v1/tasks", json=task).status_code == 200
        step = workflow_gw.started[0][0].definition.steps[0]
        assert step.type is StepType.WAIT
        assert step.params == WaitParams(seconds=12.5)

    @pytest.mark.parametrize(
        "params",
        [None, {"seconds": 0}, {"seconds": -1}, {"seconds": 3601}, {"x": 0, "y": 0, "theta": 0}],
    )
    def test_a_bad_wait_step_is_a_422(self, client, workflow_gw, params):
        task = _move_task()
        task["steps"] = [{"id": "s1", "type": "WAIT", "params": params}]

        assert client.post("/api/v1/tasks", json=task).status_code == 422
        assert workflow_gw.started == []

    def test_a_group_is_carried_onto_the_workflow_steps(self, client, workflow_gw):
        task = _move_task()
        task["steps"][0]["group"] = "a"
        task["steps"].append(
            {"id": "step2", "type": "SPEAK", "params": {"text": "hi"}, "group": "a"}
        )

        assert client.post("/api/v1/tasks", json=task).status_code == 200
        steps = workflow_gw.started[0][0].definition.steps
        assert [step.group for step in steps] == ["a", "a"]

    @pytest.mark.parametrize(
        "steps, phrase, step_id",
        [
            # Two body steps in one group would queue on the one body slot.
            (
                [
                    {
                        "id": "m1",
                        "type": "MOVE",
                        "params": {"x": 0, "y": 0, "theta": 0},
                        "group": "a",
                    },
                    {"id": "s1", "type": "STANDUP", "group": "a"},
                ],
                "more than one",
                # The later body step, the one the console marks.
                "s1",
            ),
            # A group is one place in the sequence.
            (
                [
                    {"id": "w1", "type": "WAIT", "params": {"seconds": 1}, "group": "a"},
                    {"id": "s1", "type": "STANDUP"},
                    {"id": "w2", "type": "WAIT", "params": {"seconds": 1}, "group": "a"},
                ],
                "adjacent",
                "w2",
            ),
        ],
    )
    def test_a_bad_grouping_is_a_400_naming_the_step(
        self, client, workflow_gw, steps, phrase, step_id
    ):
        response = client.post("/api/v1/tasks", json={**_move_task(), "steps": steps})

        assert response.status_code == 400
        body = response.json()
        assert phrase in body["detail"]
        assert body["code"] == "invalid_step_group"
        assert body["step_id"] == step_id
        assert workflow_gw.started == []

    def test_a_duplicate_id_surfaces_as_400(self, client, workflow_gw):
        async def _raise(request, provenance=None):
            raise BadRequestError(f"Task {request.id} already exists")

        workflow_gw.start_task = _raise

        response = client.post("/api/v1/tasks", json=_move_task())

        assert response.status_code == 400
        assert "already exists" in response.json()["detail"]

    def test_a_busy_robot_surfaces_as_409(self, client, workflow_gw):
        # The gateway's one-task-at-a-time gate; the router only translates.
        async def _raise(request, provenance=None):
            raise ConflictError("Robot is busy: task robot01-task-000 is running")

        workflow_gw.start_task = _raise

        response = client.post("/api/v1/tasks", json=_move_task())

        assert response.status_code == 409
        assert "robot01-task-000" in response.json()["detail"]


class TestJobMap:
    """Which map a dispatched job is stamped with, and when it may not drive."""

    def test_a_job_that_drives_is_stamped_with_the_loaded_map(
        self, client, workflow_gw
    ):
        assert client.post("/api/v1/tasks", json=_move_task()).status_code == 200

        _, provenance = workflow_gw.started[0]
        assert provenance.map_name == "lab"

    def test_a_patrol_job_is_stamped_like_a_move(self, client, workflow_gw):
        response = client.post("/api/v1/tasks", json=_patrol_task())
        assert response.status_code == 200, response.text

        task, provenance = workflow_gw.started[0]
        assert provenance.map_name == "lab"
        waypoints = task.definition.steps[0].params.waypoints
        assert [(w.x, w.z) for w in waypoints] == [(1.0, 0.0), (4.0, 3.1)]

    def test_a_patrol_from_another_map_is_refused(self, client, workflow_gw):
        task = {**_patrol_task(), "map_name": "warehouse"}

        response = client.post("/api/v1/tasks", json=task)

        assert response.status_code == 409
        assert response.json()["code"] == "map_mismatch"
        assert workflow_gw.started == []

    def test_a_patrol_of_one_waypoint_is_refused(self, client, workflow_gw):
        task = _patrol_task()
        del task["steps"][0]["params"]["waypoints"][1]

        assert client.post("/api/v1/tasks", json=task).status_code == 422
        assert workflow_gw.started == []

    def test_a_patrol_in_a_group_with_a_move_is_refused(self, client, workflow_gw):
        task = _patrol_task()
        task["steps"][0]["group"] = "g"
        task["steps"].append(
            {"id": "s2", "type": "MOVE", "group": "g",
             "params": {"x": 1.0, "y": 2.0, "theta": 90.0}}
        )

        response = client.post("/api/v1/tasks", json=task)

        assert response.status_code == 400
        assert response.json()["step_id"] == "s2"

    @pytest.mark.parametrize("step_type", ["DOCK", "UNDOCK"])
    def test_the_removed_dock_steps_are_refused(self, client, workflow_gw, step_type):
        # Removed on 2026-11-12 with the robot stack's dock trees.
        task = {"id": "robot01-dock-001", "steps": [{"id": "s1", "type": step_type}]}

        assert client.post("/api/v1/tasks", json=task).status_code == 422
        assert workflow_gw.started == []

    def test_a_posture_job_holds_no_map(self, client, workflow_gw):
        assert client.post("/api/v1/tasks", json=_stand_task()).status_code == 200

        _, provenance = workflow_gw.started[0]
        assert provenance.map_name is None

    def test_the_callers_map_is_checked_not_recorded(self, client, workflow_gw):
        task = {**_move_task(), "map_name": "lab"}

        assert client.post("/api/v1/tasks", json=task).status_code == 200
        assert workflow_gw.started[0][1].map_name == "lab"

    def test_positions_from_another_map_are_refused(self, client, workflow_gw):
        task = {**_move_task(), "map_name": "warehouse"}

        response = client.post("/api/v1/tasks", json=task)

        assert response.status_code == 409
        assert response.json()["code"] == "map_mismatch"
        assert "'warehouse'" in response.json()["detail"]
        assert workflow_gw.started == []


class TestTaskState:
    def test_get_projects_step_status_only(self, client):
        body = client.get("/api/v1/tasks/robot01-task-001").json()

        assert body["status"] == "IN_PROGRESS"
        # StepState is status/error/message (and the group it ran in, and
        # its times) — the definition (params) stays out. No feedback from the
        # robot yet reads as an empty message; an ungrouped step reads group
        # null, and one without times reads them null.
        assert body["steps"] == [
            {
                "id": "step1",
                "status": "IN_PROGRESS",
                "error_msg": "",
                "message": "",
                "group": None,
                "started_at": None,
                "ended_at": None,
                "paused_seconds": 0.0,
                "waypoints_passed": 0,
            }
        ]

    def test_a_running_move_carries_the_robots_status(self, client, robot_gw):
        robot_gw.message = "Path blocked; replanning"

        body = client.get("/api/v1/tasks/robot01-task-001").json()

        assert body["steps"][0]["message"] == "Path blocked; replanning"

    def test_a_running_patrol_carries_the_progress_sentence(
        self, client, workflow_gw, robot_gw
    ):
        robot_gw.message = "Patrolling: 1 of 2 waypoints left, 3.0 m to go"
        workflow_gw.state = TaskState(
            id="robot01-patrol-001",
            status="IN_PROGRESS",
            steps=[
                Step(
                    id="s1",
                    type=StepType.PATROL,
                    params=PatrolParams(
                        waypoints=[
                            MoveParams(x=1.0, y=2.0, theta=0.0),
                            MoveParams(x=4.0, y=2.0, theta=90.0),
                        ]
                    ),
                    status=StepStatus.IN_PROGRESS,
                ),
            ],
        )

        body = client.get("/api/v1/tasks/robot01-patrol-001").json()

        assert body["steps"][0]["message"] == robot_gw.message

    def test_a_step_carries_its_times(self, client, workflow_gw):
        started = datetime(2026, 11, 1, 8, 0, tzinfo=timezone.utc)
        ended = datetime(2026, 11, 1, 8, 1, 30, tzinfo=timezone.utc)
        workflow_gw.state = TaskState(
            id="robot01-task-003",
            status="IN_PROGRESS",
            steps=[
                Step(
                    id="1-wait",
                    type=StepType.WAIT,
                    params=WaitParams(seconds=60),
                    status=StepStatus.COMPLETED,
                    started_at=started,
                    ended_at=ended,
                    paused_seconds=30.0,
                ),
                Step(id="2-stand", type=StepType.STANDUP),
            ],
        )

        steps = client.get("/api/v1/tasks/robot01-task-003").json()["steps"]

        assert steps[0]["started_at"] == "2026-11-01T08:00:00Z"
        assert steps[0]["ended_at"] == "2026-11-01T08:01:30Z"
        assert steps[0]["paused_seconds"] == 30.0
        # Not started: no times, nothing held.
        assert (steps[1]["started_at"], steps[1]["ended_at"], steps[1]["paused_seconds"]) == (
            None,
            None,
            0.0,
        )

    def test_only_a_running_move_asks_the_robot(self, client, workflow_gw, robot_gw):
        # A paused MOVE has cancelled its goal, and a STANDUP never had one:
        # neither may pick up whatever navigation the gateway is tracking.
        robot_gw.message = "Following the path"
        workflow_gw.state = TaskState(
            id="robot01-task-001",
            status="PAUSED",
            steps=[
                Step(id="step1", type=StepType.STANDUP, status=StepStatus.IN_PROGRESS),
                Step(
                    id="step2",
                    type=StepType.MOVE,
                    params=MoveParams(x=1.0, y=2.0, theta=90.0),
                    status=StepStatus.PAUSED,
                ),
            ],
        )

        body = client.get("/api/v1/tasks/robot01-task-001").json()

        assert [step["message"] for step in body["steps"]] == ["", ""]
        assert robot_gw.calls == 0

    def test_get_unknown_task_is_404(self, client, workflow_gw):
        async def _raise(task_id):
            raise NotFoundError(f"Task {task_id} not found")

        workflow_gw.get_task_state = _raise

        assert client.get("/api/v1/tasks/missing").status_code == 404

    def test_get_projects_a_held_run_as_paused(self, client, workflow_gw):
        # The gateway already folded the held step into the task status; the
        # router passes both through, which is the one place the console can
        # see a hold (/active_tasks keeps saying IN_PROGRESS).
        workflow_gw.state = TaskState(
            id="robot01-task-001",
            status="PAUSED",
            steps=[
                Step(id="step1", type=StepType.STANDUP, status=StepStatus.COMPLETED),
                Step(
                    id="step2",
                    type=StepType.MOVE,
                    params=MoveParams(x=1.0, y=2.0, theta=90.0),
                    status=StepStatus.PAUSED,
                ),
            ],
        )

        body = client.get("/api/v1/tasks/robot01-task-001").json()

        assert body["status"] == "PAUSED"
        assert [step["status"] for step in body["steps"]] == ["COMPLETED", "PAUSED"]


class TestTaskSteps:
    def test_lists_the_steps_an_editor_builds_from_without_a_gateway(self, client, workflow_gw):
        res = client.get("/api/v1/task_steps")
        assert res.status_code == 200
        steps = res.json()["steps"]
        assert [step["type"] for step in steps] == [
            "MOVE", "PATROL", "SPEAK", "WAIT", "STANDUP", "LIEDOWN",
        ]
        assert [step["while_walking"] for step in steps] == [
            False, False, True, False, False, False,
        ]
        assert steps[0]["params"] == [
            {"kind": "waypoint", "key": "target", "label": "Waypoint", "filter": None}
        ]
        # Not a /tasks/{id} read under another name: nothing reaches Temporal.
        assert workflow_gw.started == [] and workflow_gw.history_calls == []


class TestActiveTasks:
    def test_list_carries_provenance_and_as_of(self, client):
        body = client.get("/api/v1/active_tasks").json()

        assert body["as_of"] == "2026-08-10T09:00:30Z"
        task = body["tasks"][0]
        assert task["source"] == "SCHEDULE"
        assert task["schedule_id"] == "robot01-sched-001"

    def test_a_run_reports_the_map_it_was_stamped_with(
        self, client, workflow_gw, catalog
    ):
        workflow_gw.active[0][0].map_name = "warehouse"

        task = client.get("/api/v1/active_tasks").json()["tasks"][0]

        assert task["map_name"] == "warehouse"
        # Stamped runs need no INI read on the console's 2 s poll.
        assert catalog.reads == 0

    def test_a_run_older_than_the_stamp_holds_the_loaded_map(self, client):
        # The fixture's scheduled run carries no map: it can only be driving
        # on the map loaded now, because a switch is refused while it runs.
        task = client.get("/api/v1/active_tasks").json()["tasks"][0]

        assert task["map_name"] == "lab"

    def test_an_unstamped_posture_run_holds_no_map(self, client, workflow_gw):
        workflow_gw.active[0][0].kind = TaskKind.STANDUP

        task = client.get("/api/v1/active_tasks").json()["tasks"][0]

        assert task["map_name"] is None

    def test_nothing_running_is_an_empty_list_not_a_404(self, client, workflow_gw):
        workflow_gw.active = ([], [], datetime(2026, 8, 10, tzinfo=timezone.utc))

        response = client.get("/api/v1/active_tasks")

        assert response.status_code == 200
        assert response.json()["tasks"] == []
        assert response.json()["recently_ended"] == []

    def test_a_run_that_just_closed_is_listed_apart_newest_first(
        self, client, workflow_gw, catalog
    ):
        def closed(task_id, status, second):
            return ActiveTask(
                id=task_id,
                run_id="run-1",
                status=status,
                started_at=datetime(2026, 8, 10, 9, 0, tzinfo=timezone.utc),
                ended_at=datetime(2026, 8, 10, 9, 0, second, tzinfo=timezone.utc),
                source=TaskSource.DIRECT,
                kind=TaskKind.GOAL,
            )

        workflow_gw.active = (
            [],
            [closed("older", "COMPLETED", 1), closed("refused", "FAILED", 5)],
            datetime(2026, 8, 10, 9, 0, 6, tzinfo=timezone.utc),
        )

        body = client.get("/api/v1/active_tasks").json()

        # Nothing running: a run that just closed holds no map and keeps no
        # one from dispatching, so it never reaches `tasks`.
        assert body["tasks"] == []
        assert [(t["id"], t["status"]) for t in body["recently_ended"]] == [
            ("refused", "FAILED"),
            ("older", "COMPLETED"),
        ]
        assert body["recently_ended"][0]["ended_at"] == "2026-08-10T09:00:05Z"
        # As stamped (none here): the loaded map is not claimed for it.
        assert body["recently_ended"][0]["map_name"] is None
        assert catalog.reads == 0


class TestTaskHistory:
    def test_projects_rows_and_an_opaque_next_token(self, client, workflow_gw):
        body = client.get("/api/v1/task_history").json()

        task = body["tasks"][0]
        assert (task["id"], task["status"], task["source"]) == (
            "robot01-task-001",
            "COMPLETED",
            "DIRECT",
        )
        assert task["closed_at"] == "2026-08-10T08:05:00Z"
        assert (task["kind"], task["name"]) == ("task", "Morning patrol")
        # History has no fallback: a closed run's map is whatever it stamped.
        assert task["map_name"] is None
        # base64url, unpadded: safe to put straight back into a query string.
        assert body["next_page_token"] == "_3Rva2Vu"
        assert workflow_gw.history_calls == [
            {
                "page_size": 20,
                "next_page_token": None,
                "status": None,
                "since": None,
                "until": None,
                "kind": None,
                "name": None,
            }
        ]

    def test_the_token_round_trips_with_the_filters(self, client, workflow_gw):
        client.get(
            "/api/v1/task_history",
            params={
                "page_token": "_3Rva2Vu",
                "page_size": 5,
                "status": "FAILED",
                "since": "2026-08-10T00:00:00",
                "until": "2026-08-11T00:00:00",
                "kind": "schedule",
                "name": "Morning patrol",
            },
        )

        call = workflow_gw.history_calls[0]
        assert call["next_page_token"] == b"\xfftoken"
        assert (call["page_size"], call["status"]) == (5, "FAILED")
        # A naive `since` / `until` is read as UTC, not as the server's local time.
        assert call["since"] == datetime(2026, 8, 10, tzinfo=timezone.utc)
        assert call["until"] == datetime(2026, 8, 11, tzinfo=timezone.utc)
        assert (call["kind"], call["name"]) == (TaskKind.SCHEDULE, "Morning patrol")

    def test_an_empty_window_is_a_400_with_a_sentence(self, client, workflow_gw):
        response = client.get(
            "/api/v1/task_history",
            params={"since": "2026-08-11T00:00:00", "until": "2026-08-10T00:00:00"},
        )

        assert response.status_code == 400
        assert response.json()["detail"] == "until must not be before since"
        assert workflow_gw.history_calls == []

    def test_last_page_has_no_token(self, client, workflow_gw):
        workflow_gw.history = ([], None)

        body = client.get("/api/v1/task_history").json()

        assert body == {"tasks": [], "next_page_token": None}

    def test_a_mangled_token_is_a_400(self, client, workflow_gw):
        response = client.get("/api/v1/task_history", params={"page_token": "no!pe"})

        assert response.status_code == 400
        assert workflow_gw.history_calls == []

    @pytest.mark.parametrize(
        "params",
        [
            {"status": "IN_PROGRESS"},
            {"page_size": 0},
            {"page_size": 101},
            {"kind": "teleport"},
            {"name": ""},
        ],
    )
    def test_out_of_range_params_are_rejected(self, client, workflow_gw, params):
        response = client.get("/api/v1/task_history", params=params)

        assert response.status_code == 422
        assert workflow_gw.history_calls == []


class TestTaskHistoryStats:
    def test_projects_counts_and_a_success_rate(self, client, workflow_gw):
        body = client.get("/api/v1/task_history/stats").json()

        assert body["as_of"] == "2026-08-10T09:00:00Z"
        assert body["total"] == 4
        assert body["by_status"] == {"COMPLETED": 3, "FAILED": 1, "CANCELED": 0}
        assert body["success_rate"] == 0.75
        assert "by_kind" not in body
        assert workflow_gw.stats_calls == [
            {"status": None, "since": None, "until": None, "kind": None, "name": None}
        ]

    def test_a_rate_of_nothing_is_null_not_zero(self, client, workflow_gw):
        workflow_gw.stats = TaskHistoryStats(
            as_of=datetime(2026, 8, 10, tzinfo=timezone.utc),
            total=0,
            completed=0,
            failed=0,
            canceled=0,
        )

        body = client.get("/api/v1/task_history/stats").json()

        assert body["success_rate"] is None
        assert body["total"] == 0

    def test_forwards_the_same_filters_as_the_list(self, client, workflow_gw):
        client.get(
            "/api/v1/task_history/stats",
            params={
                "status": "CANCELED",
                "since": "2026-08-10T00:00:00",
                "until": "2026-08-11T00:00:00+08:00",
                "kind": "goal",
                "name": "Morning patrol",
            },
        )

        call = workflow_gw.stats_calls[0]
        assert call["status"] == "CANCELED"
        assert call["since"] == datetime(2026, 8, 10, tzinfo=timezone.utc)
        assert call["until"] == datetime(2026, 8, 10, 16, tzinfo=timezone.utc)
        assert (call["kind"], call["name"]) == (TaskKind.GOAL, "Morning patrol")

    def test_an_empty_window_is_a_400(self, client, workflow_gw):
        response = client.get(
            "/api/v1/task_history/stats",
            params={"since": "2026-08-11T00:00:00", "until": "2026-08-10T00:00:00"},
        )

        assert response.status_code == 400
        assert workflow_gw.stats_calls == []

    def test_a_bad_kind_is_a_422(self, client, workflow_gw):
        assert client.get("/api/v1/task_history/stats", params={"kind": "x"}).status_code == 422
        assert workflow_gw.stats_calls == []


class TestCancelTask:
    def test_delete_answers_canceling_not_canceled(self, client, workflow_gw):
        # A delete is a cancel *request*; the workflow may still finish
        # COMPLETED. The response must not claim the outcome.
        body = client.delete("/api/v1/tasks/robot01-task-001").json()

        assert workflow_gw.cancelled == ["robot01-task-001"]
        assert body["status"] == "CANCELING"


class TestHoldTask:
    def test_pause_answers_pausing_not_paused(self, client, workflow_gw):
        # Like DELETE, a pause is a *request*: a SPEAK or posture step finishes
        # before the run holds, so the ack must not claim PAUSED. The console
        # discards this body and reads the hold back from GET.
        response = client.post("/api/v1/tasks/robot01-task-001/pause")

        assert response.status_code == 200
        assert workflow_gw.paused == ["robot01-task-001"]
        assert response.json()["status"] == "PAUSING"

    def test_resume_answers_in_progress(self, client, workflow_gw):
        response = client.post("/api/v1/tasks/robot01-task-001/resume")

        assert response.status_code == 200
        assert workflow_gw.resumed == ["robot01-task-001"]
        assert response.json()["status"] == "IN_PROGRESS"

    def test_unknown_task_is_404(self, client, workflow_gw):
        async def _raise(task_id):
            raise NotFoundError(f"Task {task_id} not found")

        workflow_gw.pause_task = _raise
        workflow_gw.resume_task = _raise

        assert client.post("/api/v1/tasks/missing/pause").status_code == 404
        assert client.post("/api/v1/tasks/missing/resume").status_code == 404

    def test_a_closed_run_is_409_task_not_running(self, client, workflow_gw):
        # A console whose last poll still showed the run open sends a pause a
        # moment after it finished: a conflict with the state, with the code
        # beside the sentence so nothing has to match on prose.
        async def _raise(task_id):
            raise ConflictError(f"Task {task_id} is not running", code="task_not_running")

        workflow_gw.pause_task = _raise

        response = client.post("/api/v1/tasks/robot01-task-001/pause")

        assert response.status_code == 409
        assert response.json()["code"] == "task_not_running"


class TestContinuationSteps:
    def test_starts_at_the_failed_steps_group_and_keeps_every_step(self):
        steps, from_step_id = continuation_steps(_failed_run().steps)

        assert from_step_id == "s2"
        assert [(s.id, s.status) for s in steps] == [
            ("s1", StepStatus.SKIPPED),
            ("s2", StepStatus.PENDING),
            ("s3", StepStatus.PENDING),
            ("s4", StepStatus.PENDING),
        ]

    def test_every_step_is_fresh(self):
        steps, _ = continuation_steps(_failed_run().steps)

        s3 = steps[2]
        assert (s3.error_msg, s3.started_at, s3.ended_at) == ("", None, None)
        assert s3.params == MoveParams(x=1.0, y=2.0, theta=90.0)
        assert [s.group for s in steps] == [None, "g", "g", None]

    def test_a_lone_failed_step_is_its_own_start(self):
        steps = [
            Step(id="a", type=StepType.STANDUP, status=StepStatus.COMPLETED),
            Step(id="b", type=StepType.STANDUP, status=StepStatus.FAILED),
        ]

        continued, from_step_id = continuation_steps(steps)

        assert from_step_id == "b"
        assert [s.status for s in continued] == [StepStatus.SKIPPED, StepStatus.PENDING]

    def test_a_failed_first_step_skips_nothing(self):
        steps = [
            Step(id="a", type=StepType.STANDUP, status=StepStatus.FAILED),
            Step(id="b", type=StepType.LIEDOWN),
        ]

        continued, from_step_id = continuation_steps(steps)

        assert from_step_id == "a"
        assert all(s.status is StepStatus.PENDING for s in continued)

    def test_a_patrol_keeps_the_waypoints_it_had_passed(self):
        patrol = PatrolParams(waypoints=[
            MoveParams(x=0.0, y=0.0, theta=0.0),
            MoveParams(x=1.0, y=0.0, theta=0.0),
            MoveParams(x=2.0, y=0.0, theta=0.0),
        ])
        steps = [Step(id="p", type=StepType.PATROL, params=patrol,
                      status=StepStatus.FAILED, waypoints_passed=2)]

        continued, _ = continuation_steps(steps)

        assert continued[0].waypoints_passed == 2
        assert len(continued[0].params.waypoints) == 3

    def test_nothing_failed_is_none(self):
        steps = [Step(id="a", type=StepType.STANDUP, status=StepStatus.CANCELED)]

        assert continuation_steps(steps) is None


class TestContinueTask:
    def _continue(self, client, **extra):
        body = {"id": "robot01-task-002", "continue_from": "robot01-task-001", **extra}
        return client.post("/api/v1/tasks", json=body)

    def test_dispatches_the_failed_runs_steps_from_the_failed_group(
        self, client, workflow_gw
    ):
        response = self._continue(client)

        assert response.status_code == 200, response.text
        body = response.json()
        assert body["status"] == "PENDING"
        assert (body["id"], body["continued_from"], body["from_step_id"]) == (
            "robot01-task-002", "robot01-task-001", "s2",
        )
        assert workflow_gw.failed_reads == ["robot01-task-001"]
        task, provenance = workflow_gw.started[0]
        assert task.id == "robot01-task-002"
        assert [(s.id, s.status) for s in task.definition.steps] == [
            ("s1", StepStatus.SKIPPED),
            ("s2", StepStatus.PENDING),
            ("s3", StepStatus.PENDING),
            ("s4", StepStatus.PENDING),
        ]
        assert (provenance.kind, provenance.name, provenance.map_name) == (
            TaskKind.TASK, "Morning patrol", "lab",
        )

    def test_a_plain_dispatch_answers_no_continuation(self, client):
        body = client.post("/api/v1/tasks", json=_move_task()).json()

        assert (body["continued_from"], body["from_step_id"]) == (None, None)

    def test_kind_and_name_from_the_request_win(self, client, workflow_gw):
        assert self._continue(client, kind="goal", name="Other").status_code == 200

        _, provenance = workflow_gw.started[0]
        assert (provenance.kind, provenance.name) == (TaskKind.GOAL, "Other")

    def test_a_scheduled_runs_continuation_is_a_task(self, client, workflow_gw):
        workflow_gw.failed = _failed_run(kind=TaskKind.SCHEDULE)

        assert self._continue(client).status_code == 200

        assert workflow_gw.started[0][1].kind is TaskKind.TASK

    @pytest.mark.parametrize(
        "body",
        [
            {"id": "t"},
            {"id": "t", "continue_from": "robot01-task-001",
             "steps": [{"id": "s1", "type": "STANDUP"}]},
            {"id": "t", "continue_from": "robot01-task-001", "map_name": "lab"},
        ],
    )
    def test_a_malformed_request_is_a_400_sentence(self, client, workflow_gw, body):
        response = client.post("/api/v1/tasks", json=body)

        assert response.status_code == 400
        assert workflow_gw.started == []

    def test_a_run_that_has_not_failed_is_409(self, client, workflow_gw):
        async def _raise(task_id):
            raise ConflictError(f"Task {task_id} has not failed", code="task_not_failed")

        workflow_gw.get_failed_run = _raise

        response = self._continue(client)

        assert response.status_code == 409
        assert response.json()["code"] == "task_not_failed"
        assert workflow_gw.started == []

    def test_a_run_with_no_failed_step_is_409(self, client, workflow_gw):
        workflow_gw.failed = _failed_run(
            steps=[Step(id="s1", type=StepType.STANDUP, status=StepStatus.PENDING)]
        )

        response = self._continue(client)

        assert response.status_code == 409
        assert response.json()["code"] == "no_failed_step"
        assert workflow_gw.started == []

    def test_an_unknown_task_is_404(self, client, workflow_gw):
        async def _raise(task_id):
            raise NotFoundError(f"Task {task_id} not found")

        workflow_gw.get_failed_run = _raise

        assert self._continue(client).status_code == 404

    def test_a_run_on_another_map_is_refused(self, client, workflow_gw, catalog):
        catalog.active = "warehouse"

        response = self._continue(client)

        assert response.status_code == 409
        assert response.json()["code"] == "map_mismatch"
        assert workflow_gw.started == []

    def test_only_the_steps_still_to_run_hold_a_map(self, client, workflow_gw, catalog):
        # The MOVE was before the failure: what is left drives nowhere, so the
        # map switched since is no reason to refuse.
        move = MoveParams(x=1.0, y=2.0, theta=90.0)
        workflow_gw.failed = _failed_run(steps=[
            Step(id="s1", type=StepType.MOVE, params=move, status=StepStatus.COMPLETED),
            Step(id="s2", type=StepType.STANDUP, status=StepStatus.FAILED),
        ])
        catalog.active = "warehouse"

        assert self._continue(client).status_code == 200
        assert workflow_gw.started[0][1].map_name is None


class TestTaskHistoryExport:
    """GET /api/v1/task_history/export: task_record as CSV, one row per step."""

    @staticmethod
    def _record(task_repo, run_id="run-1", status="COMPLETED", closed_minute=10, **extra):
        from syncai_backend.tasks.schema import TaskRecord

        start = datetime(2026, 11, 1, 8, 0, tzinfo=timezone.utc)
        task_repo.insert(
            TaskRecord(
                task_id=f"robot01-task-{run_id}",
                run_id=run_id,
                status=status,
                started_at=start,
                closed_at=start.replace(minute=closed_minute),
                steps=[
                    Step(
                        id="1-wait",
                        type=StepType.WAIT,
                        params=WaitParams(seconds=60),
                        status=StepStatus.COMPLETED,
                        started_at=start,
                        ended_at=start.replace(minute=2),
                        paused_seconds=30.0,
                    ),
                    Step(
                        id="2-stand",
                        type=StepType.STANDUP,
                        status=StepStatus.FAILED,
                        error_msg="馬達異常",
                        started_at=start.replace(minute=2),
                        ended_at=start.replace(minute=3),
                    ),
                    Step(id="3-lie", type=StepType.LIEDOWN),
                ],
                **extra,
            )
        )

    @staticmethod
    def _rows(response):
        import csv
        import io

        return list(csv.DictReader(io.StringIO(response.content.decode("utf-8-sig"))))

    def test_one_row_per_step_with_its_times(self, client, task_record_repo):
        self._record(task_record_repo, kind=TaskKind.TASK, name="Patrol", map_name="lab")

        response = client.get("/api/v1/task_history/export")

        assert response.status_code == 200
        assert response.headers["content-type"].startswith("text/csv")
        assert response.headers["content-disposition"] == (
            'attachment; filename="task-report.csv"'
        )
        # The BOM, so Excel reads the file as UTF-8.
        assert response.content.startswith("\ufeff".encode("utf-8"))
        rows = self._rows(response)
        assert [row["step_id"] for row in rows] == ["1-wait", "2-stand", "3-lie"]
        wait, stand, lie = rows
        assert (wait["task_name"], wait["kind"], wait["map_name"]) == ("Patrol", "task", "lab")
        assert wait["task_started_at"] == "2026-11-01 08:00:00"
        assert wait["task_closed_at"] == "2026-11-01 08:10:00"
        assert wait["task_duration_s"] == "600.0"
        assert wait["step_index"] == "1"
        assert wait["step_wall_s"] == "120.0"
        assert wait["step_paused_s"] == "30.0"
        assert wait["step_active_s"] == "90.0"
        assert wait["timezone"] == "UTC"
        assert stand["error_msg"] == "馬達異常"
        assert stand["step_active_s"] == "60.0"
        # Never ran: blanks, not zeroes.
        assert lie["step_status"] == "PENDING"
        assert (lie["step_started_at"], lie["step_wall_s"], lie["step_paused_s"]) == ("", "", "")

    def test_times_are_written_in_the_asked_zone(self, client, task_record_repo):
        self._record(task_record_repo)

        rows = self._rows(client.get("/api/v1/task_history/export?tz=Asia/Taipei"))

        assert rows[0]["task_started_at"] == "2026-11-01 16:00:00"
        assert rows[0]["step_ended_at"] == "2026-11-01 16:02:00"
        assert rows[0]["timezone"] == "Asia/Taipei"

    def test_an_unknown_zone_is_a_400(self, client):
        response = client.get("/api/v1/task_history/export?tz=Mars/Olympus")

        assert response.status_code == 400
        assert "Mars/Olympus" in response.json()["detail"]

    def test_the_history_filters_apply(self, client, task_record_repo):
        self._record(task_record_repo, run_id="a", closed_minute=5)
        self._record(task_record_repo, run_id="b", status="FAILED", closed_minute=20)
        self._record(task_record_repo, run_id="c", closed_minute=40)

        def runs(query):
            response = client.get(f"/api/v1/task_history/export?{query}")
            return sorted({row["run_id"] for row in self._rows(response)})

        assert runs("status=COMPLETED") == ["a", "c"]
        assert runs("since=2026-11-01T08:10:00Z&until=2026-11-01T08:30:00Z") == ["b"]
        assert runs("") == ["a", "b", "c"]

    def test_the_file_is_named_for_the_window(self, client):
        response = client.get(
            "/api/v1/task_history/export?since=2026-10-01T00:00:00Z&until=2026-10-31T00:00:00Z"
        )

        assert response.headers["content-disposition"] == (
            'attachment; filename="task-report-20261001-20261031.csv"'
        )

    def test_an_inverted_window_is_a_400(self, client):
        response = client.get(
            "/api/v1/task_history/export?since=2026-11-02T00:00:00Z&until=2026-11-01T00:00:00Z"
        )

        assert response.status_code == 400

    def test_nothing_recorded_is_a_header_only(self, client):
        rows = self._rows(client.get("/api/v1/task_history/export"))

        assert rows == []


class TestRoutePreview:
    def _goal(self, **overrides):
        return {"x": 3.0, "y": 4.0, "theta": 90.0, "z": 2.8, "map_name": "lab", **overrides}

    def test_a_reachable_goal_answers_the_route_and_its_length(self, client, robot_gw):
        response = client.post("/api/v1/route_preview", json=self._goal())

        assert response.status_code == 200
        assert response.json() == {
            "reachable": True,
            "points": [[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]],
            "length_m": 5.0,
            "code": None,
            "message": "",
        }

    def test_the_goal_reaches_the_planner_as_a_move_would(self, client, robot_gw):
        client.post("/api/v1/route_preview", json=self._goal())

        # theta in degrees on the wire, yaw in radians to the robot, and the
        # z that picks the floor passed through.
        assert robot_gw.previews == [
            {"x": 3.0, "y": 4.0, "yaw": pytest.approx(math.pi / 2), "z": 2.8}
        ]

    def test_no_route_is_an_answer_not_an_error(self, client, robot_gw):
        robot_gw.preview = (True, "", refused(223, "goal_in_keepout: zone 'z1'"))

        response = client.post("/api/v1/route_preview", json=self._goal())

        assert response.status_code == 200
        body = response.json()
        assert body["reachable"] is False
        assert body["points"] == []
        assert body["code"] == "goal_in_keepout"
        assert body["message"].startswith("Planning failed: the goal is inside a keepout zone")

    def test_a_goal_aimed_on_another_map_is_refused_before_the_planner(
        self, client, catalog, robot_gw
    ):
        catalog.active = "warehouse"

        response = client.post("/api/v1/route_preview", json=self._goal())

        assert response.status_code == 409
        assert response.json()["code"] == "map_mismatch"
        assert robot_gw.previews == []

    def test_a_preview_a_newer_one_replaced_is_a_409(self, client, robot_gw):
        robot_gw.preview = (
            False,
            fail(Failure.PREVIEW_SUPERSEDED, "A newer route preview replaced this one."),
            None,
        )

        response = client.post("/api/v1/route_preview", json=self._goal())

        assert response.status_code == 409
        assert response.json()["code"] == "preview_superseded"

    def test_no_planner_to_ask_is_a_502_with_the_gateways_sentence(self, client, robot_gw):
        robot_gw.preview = (False, "Route preview is not available.", None)

        response = client.post("/api/v1/route_preview", json=self._goal())

        assert response.status_code == 502
        assert response.json()["detail"] == "Route preview is not available."

    def test_a_heading_outside_a_moves_range_is_refused(self, client, robot_gw):
        response = client.post("/api/v1/route_preview", json=self._goal(theta=-180.0))

        assert response.status_code == 422
        assert robot_gw.previews == []
