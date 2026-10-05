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
from syncai_backend.gateways.workflow.schema import (  # noqa: E402
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
    WaitParams,
)
from syncai_backend.interfaces.rest.routers.task import init_task_router  # noqa: E402
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


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


@pytest.fixture
def client(logger, workflow_gw):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(init_task_router(logger=logger, workflow_gw=workflow_gw))
    return TestClient(app)


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


class TestTaskState:
    def test_get_projects_step_status_only(self, client):
        body = client.get("/api/v1/tasks/robot01-task-001").json()

        assert body["status"] == "IN_PROGRESS"
        # StepState is status/error only — the definition (params) stays out.
        assert body["steps"] == [
            {"id": "step1", "status": "IN_PROGRESS", "error_msg": ""}
        ]

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


class TestActiveTasks:
    def test_list_carries_provenance_and_as_of(self, client):
        body = client.get("/api/v1/active_tasks").json()

        assert body["as_of"] == "2026-08-10T09:00:30Z"
        task = body["tasks"][0]
        assert task["source"] == "SCHEDULE"
        assert task["schedule_id"] == "robot01-sched-001"

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
