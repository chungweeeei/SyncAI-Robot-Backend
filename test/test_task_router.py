"""Tests for /api/v1/tasks, /active_tasks and /task_history — the REST projection only.

Same pattern as ``test_schedule_router.py``: a stub gateway records what the
router hands it and answers canned views, so these tests pin the boundary
(request validation, response shapes, exception mapping) without any Temporal.
The gateway's own behaviour lives in ``test_workflow_gateway.py``.
"""

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
    Step,
    StepStatus,
    StepType,
    TaskHistoryEntry,
    TaskHistoryStats,
    TaskKind,
    TaskSource,
    TaskState,
    DemoParams,
    WaitParams,
)
from syncai_backend.interfaces.rest.routers.task import init_task_router  # noqa: E402
from syncai_backend.services.map_usage import MapUsagePolicy  # noqa: E402
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubWorkflowGateway:
    def __init__(self):
        self.started = []
        self.cancelled = []
        self.paused = []
        self.resumed = []
        self.history_calls = []
        self.stats_calls = []
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
            datetime(2026, 8, 10, 9, 0, 30, tzinfo=timezone.utc),
        )

    async def start_task(self, request, provenance=None):
        self.started.append((request, provenance))

    async def get_task_state(self, task_id):
        return self.state

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
    """The router's one robot read: the running MOVE's status sentence."""

    def __init__(self):
        self.message = ""
        self.calls = 0

    def get_move_message(self):
        self.calls += 1
        return self.message


@pytest.fixture
def catalog():
    return _StubCatalog()


@pytest.fixture
def robot_gw():
    return _StubRobotGateway()


@pytest.fixture
def client(logger, workflow_gw, catalog, robot_gw):
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


def _dock_task(task_id="robot01-dock-001"):
    return {
        "id": task_id,
        "steps": [
            {"id": "s1", "type": "DOCK", "params": {"x": 3.0, "y": 1.0, "theta": 180.0}}
        ],
    }


def _undock_task(task_id="robot01-undock-001"):
    return {"id": task_id, "steps": [{"id": "s1", "type": "UNDOCK"}]}


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

    def test_a_demo_step_carries_its_inspection_through(self, client, workflow_gw):
        task = _move_task()
        task["steps"] = [
            {"id": "s1", "type": "DEMO", "params": {"inspection": "door_open"}}
        ]

        assert client.post("/api/v1/tasks", json=task).status_code == 200
        step = workflow_gw.started[0][0].definition.steps[0]
        assert step.type is StepType.DEMO
        assert step.params == DemoParams(inspection="door_open")

    @pytest.mark.parametrize(
        "params", [None, {"inspection": "window_open"}, {"seconds": 10}]
    )
    def test_a_bad_demo_step_is_a_422(self, client, workflow_gw, params):
        task = _move_task()
        task["steps"] = [{"id": "s1", "type": "DEMO", "params": params}]

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

    def test_a_dock_job_is_stamped_like_a_move(self, client, workflow_gw):
        assert client.post("/api/v1/tasks", json=_dock_task()).status_code == 200
        assert workflow_gw.started[0][1].map_name == "lab"

    def test_an_undock_job_holds_no_map(self, client, workflow_gw):
        # It backs out from wherever the robot stands: no coordinates to be on a map.
        task = {**_undock_task(), "map_name": "warehouse"}

        assert client.post("/api/v1/tasks", json=task).status_code == 200
        assert workflow_gw.started[0][1].map_name is None

    def test_an_undock_with_a_pose_is_refused(self, client, workflow_gw):
        task = {
            "id": "robot01-undock-001",
            "steps": [
                {"id": "s1", "type": "UNDOCK", "params": {"x": 3.0, "y": 1.0, "theta": 0}}
            ],
        }

        assert client.post("/api/v1/tasks", json=task).status_code == 422
        assert workflow_gw.started == []

    def test_a_dock_from_another_map_is_refused(self, client, workflow_gw):
        task = {**_dock_task(), "map_name": "warehouse"}

        response = client.post("/api/v1/tasks", json=task)

        assert response.status_code == 409
        assert response.json()["code"] == "map_mismatch"
        assert workflow_gw.started == []

    def test_a_dock_without_a_pose_is_refused(self, client, workflow_gw):
        task = {"id": "robot01-dock-001", "steps": [{"id": "s1", "type": "DOCK"}]}

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
        # StepState is status/error/message (and the group it ran in) — the
        # definition (params) stays out, but for a DOCK's pose. No feedback
        # from the robot yet reads as an empty message; an ungrouped step
        # reads group null, and a step that is not a DOCK reads dock null.
        assert body["steps"] == [
            {
                "id": "step1",
                "status": "IN_PROGRESS",
                "error_msg": "",
                "message": "",
                "group": None,
                "dock": None,
            }
        ]

    def test_a_running_move_carries_the_robots_status(self, client, robot_gw):
        robot_gw.message = "Path blocked; replanning"

        body = client.get("/api/v1/tasks/robot01-task-001").json()

        assert body["steps"][0]["message"] == "Path blocked; replanning"

    def test_a_running_dock_carries_the_robots_status(
        self, client, workflow_gw, robot_gw
    ):
        robot_gw.message = "Docking: 0.42 m from the dock"
        workflow_gw.state = TaskState(
            id="robot01-dock-001",
            status="IN_PROGRESS",
            steps=[
                Step(
                    id="s1",
                    type=StepType.DOCK,
                    params=MoveParams(x=3.0, y=1.0, theta=180.0),
                    status=StepStatus.IN_PROGRESS,
                ),
            ],
        )

        body = client.get("/api/v1/tasks/robot01-dock-001").json()

        assert body["steps"][0]["message"] == "Docking: 0.42 m from the dock"

    def test_a_dock_step_carries_its_dock_pose(self, client, workflow_gw):
        # The console derives the staging pose from it to draw the approach,
        # so it is there in every state, not only while running -- and only
        # on the DOCK: a MOVE's goal is the route's end, which `plan` draws.
        workflow_gw.state = TaskState(
            id="robot01-task-002",
            status="IN_PROGRESS",
            steps=[
                Step(
                    id="1-move",
                    type=StepType.MOVE,
                    params=MoveParams(x=1.0, y=2.0, theta=90.0),
                    status=StepStatus.COMPLETED,
                ),
                Step(
                    id="2-dock",
                    type=StepType.DOCK,
                    params=MoveParams(x=3.0, y=-1.5, theta=180.0),
                    status=StepStatus.PENDING,
                ),
            ],
        )

        body = client.get("/api/v1/tasks/robot01-task-002").json()

        assert [step["dock"] for step in body["steps"]] == [
            None,
            {"x": 3.0, "y": -1.5, "theta": 180.0},
        ]

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
            "MOVE", "DOCK", "SPEAK", "WAIT", "STANDUP", "LIEDOWN",
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
        workflow_gw.active = ([], datetime(2026, 8, 10, tzinfo=timezone.utc))

        response = client.get("/api/v1/active_tasks")

        assert response.status_code == 200
        assert response.json()["tasks"] == []


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
