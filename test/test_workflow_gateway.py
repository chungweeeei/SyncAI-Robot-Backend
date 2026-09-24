"""Tests for WorkflowGateway's Temporal surface, mocked at ``Client.connect``.

Shaped after ``SyncAI-Device-CoreManager/tests/gateways/test_workflow_gateway.py``:
one class per gateway, an ``AsyncMock`` Temporal client injected by patching
``Client.connect`` (the same seam the gateway's lazy ``_get_client`` uses), and
every public method pinned on its success, downstream-failure and
connection-failure paths. Two deliberate departures, both forced by the robot
image this suite runs in (pytest 6.2.5, no plugins): plain ``assert`` instead of
assertpy, and ``asyncio.run`` inside sync tests instead of pytest-asyncio.

Ownership scoping (foreign task queues, memo robot_id filtering) is pinned
separately in ``test_workflow_gateway_scope.py``; here every described resource
belongs to this robot so the paths under test are reachable.
"""

import asyncio
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

pytest.importorskip("temporalio")

from temporalio.client import (  # noqa: E402
    Schedule,
    ScheduleActionStartWorkflow,
    ScheduleAlreadyRunningError,
    ScheduleCalendarSpec,
    ScheduleIntervalSpec,
    ScheduleOverlapPolicy,
    ScheduleSpec,
    ScheduleUpdate,
    WorkflowExecutionStatus,
)
from temporalio.exceptions import WorkflowAlreadyStartedError  # noqa: E402
from temporalio.service import RPCError, RPCStatusCode  # noqa: E402

from syncai_backend.exceptions import (  # noqa: E402
    BadRequestError,
    ConflictError,
    UpstreamError,
    NotFoundError,
)
from syncai_backend.gateways.workflow.config import WORKFLOW_TYPE_NAME  # noqa: E402
from syncai_backend.gateways.workflow.schema import (  # noqa: E402
    MoveParams,
    ScheduleTask,
    ScheduleTrigger,
    Step,
    StepType,
    TaskSource,
    WorkflowTask,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.workflow import (  # noqa: E402
    WorkflowGateway,
    _build_schedule_spec,
    _history_query,
    _read_trigger,
    _spec_to_trigger,
    init_workflow_gateway,
)


CONNECT = "syncai_backend.gateways.workflow.workflow.Client.connect"
OWN_QUEUE = "robot01.ROBOT_TASK_QUEUE"

MOVE_STEP = Step(id="step1", type=StepType.MOVE, params=MoveParams(x=1.0, y=2.0, theta=90.0))


def _not_found() -> RPCError:
    return RPCError("not found", RPCStatusCode.NOT_FOUND, b"")


def _task(task_id: str = "robot01-task-001") -> WorkflowTask:
    return WorkflowTask(
        id=task_id, definition=WorkflowTaskDefinition(steps=[MOVE_STEP])
    )


def _schedule(schedule_id: str = "robot01-sched-001") -> ScheduleTask:
    return ScheduleTask(
        id=schedule_id,
        trigger=ScheduleTrigger(cron="*/3 * * * *", timezone="Asia/Taipei"),
        definition=WorkflowTaskDefinition(steps=[MOVE_STEP]),
        map_name="full",
        task_template_id="0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55",
        task_template_name="Morning patrol",
    )


def _compiled_cron(cron: str) -> ScheduleCalendarSpec:
    """What Temporal hands back for a cron registered by _build_schedule_spec:
    a calendar with the ranges compiled (irrelevant here) and the string it was
    compiled from in ``comment``. A legacy cron compiles to the same with
    ``comment=None``."""
    return ScheduleCalendarSpec(comment=cron)


def _own_schedule_desc(memo: dict, spec: ScheduleSpec = None) -> SimpleNamespace:
    """A described schedule owned by this robot, in the shape the gateway reads."""
    return SimpleNamespace(
        id="robot01-sched-001",
        schedule=SimpleNamespace(
            action=ScheduleActionStartWorkflow(
                WORKFLOW_TYPE_NAME, args=[], id="robot01-sched-001", task_queue=OWN_QUEUE
            ),
            spec=spec if spec is not None else ScheduleSpec(),
            state=SimpleNamespace(paused=False),
        ),
        info=SimpleNamespace(
            next_action_times=[datetime(2026, 8, 10, 9, 0, tzinfo=timezone.utc)]
        ),
        memo=AsyncMock(return_value=memo),
    )


def _execution(
    task_id: str = "robot01-task-001",
    status=WorkflowExecutionStatus.RUNNING,
    schedule_id=None,
    close_time=None,
) -> SimpleNamespace:
    """One row of a visibility listing, as the gateway's list paths read it."""
    return SimpleNamespace(
        id=task_id,
        run_id="run-1",
        status=status,
        start_time=datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc),
        close_time=close_time,
        typed_search_attributes=SimpleNamespace(get=lambda key: schedule_id),
    )


class TestWorkflowGateway:
    @pytest.fixture
    def workflow_gw(self, logger) -> WorkflowGateway:
        return init_workflow_gateway(logger=logger, robot_id="robot01")

    @pytest.fixture
    def mock_client(self):
        client = AsyncMock()
        client.start_workflow = AsyncMock()
        client.create_schedule = AsyncMock()
        # Handle factories are synchronous on the real client.
        client.get_workflow_handle = MagicMock()
        client.get_schedule_handle = MagicMock()
        # Returns the iterator directly (NOT awaited) — see the gateway comment.
        client.list_workflows = MagicMock()
        # Coroutine returning the iterator — the OTHER shape, also pinned there.
        client.list_schedules = AsyncMock()
        return client

    def _workflow_handle(self, mock_client, describe=None, steps=None):
        handle = MagicMock()
        handle.describe = AsyncMock(return_value=describe)
        handle.query = AsyncMock(return_value=steps if steps is not None else [])
        handle.cancel = AsyncMock()
        mock_client.get_workflow_handle.return_value = handle
        return handle

    def _schedule_handle(self, mock_client, describe=None):
        handle = MagicMock()
        handle.describe = AsyncMock(return_value=describe)
        handle.delete = AsyncMock()
        handle.pause = AsyncMock()
        handle.unpause = AsyncMock()
        # The real update() describes, hands the description to the callback
        # and sends back what the callback returns; the stub does the same
        # minus the RPC, so a test can inspect the ScheduleUpdate.
        handle.updates = []

        async def _update(updater):
            update = updater(SimpleNamespace(description=describe))
            handle.updates.append(update)

        handle.update = AsyncMock(side_effect=_update)
        mock_client.get_schedule_handle.return_value = handle
        return handle

    # ==================== _get_client ====================

    def test_get_client_caches_the_connection(self, workflow_gw, mock_client):
        with patch(CONNECT, new_callable=AsyncMock) as connect:
            connect.return_value = mock_client

            async def _twice():
                return await workflow_gw._get_client(), await workflow_gw._get_client()

            first, second = asyncio.run(_twice())

        assert first is mock_client and second is mock_client
        connect.assert_called_once()

    # ==================== start_task ====================

    def test_start_task_enqueues_on_this_robots_queue(self, workflow_gw, mock_client):
        self._listing(mock_client, [])
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(workflow_gw.start_task(_task()))

        kwargs = mock_client.start_workflow.call_args[1]
        assert kwargs["workflow"] == WORKFLOW_TYPE_NAME
        assert kwargs["id"] == "robot01-task-001"
        assert kwargs["args"] == [_task()]
        assert kwargs["task_queue"] == OWN_QUEUE

    def test_start_task_maps_a_duplicate_id_to_bad_request(self, workflow_gw, mock_client):
        # Namespace-global ids: a re-post of this robot's task and a collision
        # with another robot's are rejected identically by Temporal. Either way
        # the request was well formed — 400 with the id, not the generic 502.
        self._listing(mock_client, [])
        mock_client.start_workflow.side_effect = WorkflowAlreadyStartedError(
            "robot01-task-001", WORKFLOW_TYPE_NAME
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(BadRequestError, match="already exists"):
                asyncio.run(workflow_gw.start_task(_task()))

    def test_start_task_maps_other_failures_to_internal(self, workflow_gw, mock_client):
        self._listing(mock_client, [])
        mock_client.start_workflow.side_effect = Exception("boom")
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="Start workflow failed"):
                asyncio.run(workflow_gw.start_task(_task()))

    def test_start_task_connection_failure(self, workflow_gw):
        with patch(CONNECT, new_callable=AsyncMock, side_effect=Exception("refused")):
            with pytest.raises(UpstreamError, match="connect to Temporal"):
                asyncio.run(workflow_gw.start_task(_task()))

    # ==================== start_task: one task at a time ====================
    #
    # The worker's max_workers=1 only serialises *activities*; two Running
    # workflows interleave step-by-step, so the task-level mutex has to live at
    # the dispatch entrance. Scheduled runs cannot be gated (Temporal starts
    # them itself) — what is pinned here is that a direct dispatch is refused
    # while anything, scheduled or direct, is already running.

    def test_start_task_refuses_while_anything_is_running(self, workflow_gw, mock_client):
        # A scheduled run is on the queue — the case SKIP cannot see.
        self._listing(
            mock_client,
            [_execution("robot01-sched-001-2026-08-10T09", schedule_id="sched-1")],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(ConflictError, match="robot01-sched-001"):
                asyncio.run(workflow_gw.start_task(_task()))

        mock_client.start_workflow.assert_not_called()

    def test_start_task_refuses_a_back_to_back_dispatch(self, workflow_gw, mock_client):
        # The visibility index is eventually consistent, so right after a start
        # the sweep still answers empty. The gate must catch the second dispatch
        # anyway, via a strongly-consistent describe of the task it just started.
        self._listing(mock_client, [])
        self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(status=WorkflowExecutionStatus.RUNNING),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):

            async def _twice():
                await workflow_gw.start_task(_task("robot01-task-001"))
                await workflow_gw.start_task(_task("robot01-task-002"))

            with pytest.raises(ConflictError, match="robot01-task-001"):
                asyncio.run(_twice())

        mock_client.start_workflow.assert_called_once()

    def test_start_task_forgets_a_finished_last_start(self, workflow_gw, mock_client):
        self._listing(mock_client, [])
        self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(status=WorkflowExecutionStatus.COMPLETED),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):

            async def _twice():
                await workflow_gw.start_task(_task("robot01-task-001"))
                await workflow_gw.start_task(_task("robot01-task-002"))

            asyncio.run(_twice())

        assert mock_client.start_workflow.call_count == 2
        assert workflow_gw._last_started_task_id == "robot01-task-002"

    def test_start_task_fails_closed_when_visibility_is_down(
        self, workflow_gw, mock_client
    ):
        # "Could not tell whether the robot is busy" must not become "assume
        # idle" on a machine that moves.
        mock_client.list_workflows.side_effect = RPCError(
            "unavailable", RPCStatusCode.UNAVAILABLE, b""
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="List active tasks failed"):
                asyncio.run(workflow_gw.start_task(_task()))

        mock_client.start_workflow.assert_not_called()

    # ==================== get_task_state ====================

    def test_get_task_state_maps_status_and_carries_steps(self, workflow_gw, mock_client):
        self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
            steps=[MOVE_STEP],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            state = asyncio.run(workflow_gw.get_task_state("robot01-task-001"))

        assert state.status == "IN_PROGRESS"
        assert [s.id for s in state.steps] == ["step1"]

    def test_get_task_state_folds_terminated_into_canceled(self, workflow_gw, mock_client):
        self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.TERMINATED, task_queue=OWN_QUEUE
            ),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            state = asyncio.run(workflow_gw.get_task_state("robot01-task-001"))

        assert state.status == "CANCELED"

    def test_get_task_state_degrades_a_failed_step_query(self, workflow_gw, mock_client):
        # The query can fail before the first workflow task runs, or with no
        # worker polling — the answer degrades to steps: [] rather than a 5xx.
        handle = self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
        )
        handle.query.side_effect = Exception("no worker")
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            state = asyncio.run(workflow_gw.get_task_state("robot01-task-001"))

        assert state.status == "IN_PROGRESS"
        assert state.steps == []

    def test_get_task_state_rejects_an_unmapped_status(self, workflow_gw, mock_client):
        self._workflow_handle(
            mock_client, describe=SimpleNamespace(status=None, task_queue=OWN_QUEUE)
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="Unknown workflow status"):
                asyncio.run(workflow_gw.get_task_state("robot01-task-001"))

    def test_get_task_state_not_found(self, workflow_gw, mock_client):
        handle = self._workflow_handle(mock_client)
        handle.describe.side_effect = _not_found()
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(NotFoundError, match="not found"):
                asyncio.run(workflow_gw.get_task_state("missing"))

    # ==================== cancel_task ====================

    def test_cancel_task_cancels_after_the_ownership_describe(
        self, workflow_gw, mock_client
    ):
        handle = self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(workflow_gw.cancel_task("robot01-task-001"))

        handle.describe.assert_awaited_once()
        handle.cancel.assert_awaited_once()

    def test_cancel_task_not_found(self, workflow_gw, mock_client):
        handle = self._workflow_handle(mock_client)
        handle.describe.side_effect = _not_found()
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(NotFoundError, match="not found"):
                asyncio.run(workflow_gw.cancel_task("missing"))
        handle.cancel.assert_not_awaited()

    def test_cancel_task_maps_a_failed_cancel_to_internal(self, workflow_gw, mock_client):
        handle = self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
        )
        handle.cancel.side_effect = RPCError(
            "unavailable", RPCStatusCode.UNAVAILABLE, b""
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="Cancel workflow failed"):
                asyncio.run(workflow_gw.cancel_task("robot01-task-001"))

    # ==================== list_active_tasks ====================

    def _listing(self, mock_client, executions, delay_s: float = 0.0):
        """Wire list_workflows to answer ``executions``, fresh per call."""

        def _factory(*args, **kwargs):
            async def _gen():
                if delay_s:
                    await asyncio.sleep(delay_s)
                for execution in executions:
                    yield execution

            return _gen()

        mock_client.list_workflows.side_effect = _factory

    def test_active_tasks_projects_provenance(self, workflow_gw, mock_client):
        self._listing(
            mock_client,
            [
                _execution("robot01-task-001"),
                _execution("robot01-sched-001-2026-08-10T09", schedule_id="sched-1"),
            ],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            tasks, as_of = asyncio.run(workflow_gw.list_active_tasks())

        assert as_of.tzinfo is not None
        direct, scheduled = tasks
        assert (direct.source, direct.schedule_id) == (TaskSource.DIRECT, None)
        assert (scheduled.source, scheduled.schedule_id) == (
            TaskSource.SCHEDULE,
            "sched-1",
        )

    def test_active_tasks_skips_an_unmapped_status_row(self, workflow_gw, mock_client):
        self._listing(
            mock_client,
            [_execution("odd", status=None), _execution("robot01-task-001")],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            tasks, _ = asyncio.run(workflow_gw.list_active_tasks())

        # One Temporal-side surprise must not cost the operator the whole answer.
        assert [t.id for t in tasks] == ["robot01-task-001"]

    def test_active_tasks_replays_the_snapshot_within_ttl(self, workflow_gw, mock_client):
        self._listing(mock_client, [_execution()])
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):

            async def _twice():
                return await workflow_gw.list_active_tasks(), (
                    await workflow_gw.list_active_tasks()
                )

            first, second = asyncio.run(_twice())

        assert first == second
        mock_client.list_workflows.assert_called_once()

    def test_active_tasks_coalesces_concurrent_callers(self, workflow_gw, mock_client):
        # N callers in the same tick must produce ONE RPC — the lock's re-check
        # exists precisely so the waiters replay instead of serialising N trips.
        self._listing(mock_client, [_execution()], delay_s=0.05)
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):

            async def _concurrent():
                return await asyncio.gather(
                    workflow_gw.list_active_tasks(), workflow_gw.list_active_tasks()
                )

            first, second = asyncio.run(_concurrent())

        assert first == second
        mock_client.list_workflows.assert_called_once()

    def test_active_tasks_caches_a_failure_too(self, workflow_gw, mock_client):
        # A dead Temporal must not be re-dialled by every polling tab: the
        # failure snapshot is replayed for a TTL just like a success.
        mock_client.list_workflows.side_effect = RPCError(
            "unavailable", RPCStatusCode.UNAVAILABLE, b""
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):

            async def _twice():
                for _ in range(2):
                    with pytest.raises(UpstreamError):
                        await workflow_gw.list_active_tasks()

            asyncio.run(_twice())

        mock_client.list_workflows.assert_called_once()

    # ==================== list_task_history ====================

    def _history_page(self, mock_client, executions, next_token=None, error=None):
        """Wire list_workflows to one page, the way list_task_history reads it."""
        pages = SimpleNamespace(
            fetch_next_page=AsyncMock(side_effect=error),
            current_page=executions,
            next_page_token=next_token,
        )
        mock_client.list_workflows.return_value = pages
        return pages

    def test_task_history_projects_one_page(self, workflow_gw, mock_client):
        closed = datetime(2026, 8, 10, 8, 5, tzinfo=timezone.utc)
        pages = self._history_page(
            mock_client,
            [
                _execution(
                    "robot01-task-001",
                    status=WorkflowExecutionStatus.TIMED_OUT,
                    close_time=closed,
                ),
                _execution(
                    "robot01-sched-001-2026-08-10T09",
                    status=WorkflowExecutionStatus.COMPLETED,
                    schedule_id="sched-1",
                    close_time=closed,
                ),
            ],
            next_token=b"page-2",
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            entries, token = asyncio.run(
                workflow_gw.list_task_history(page_size=2, next_page_token=b"page-1")
            )

        assert token == b"page-2"
        failed, done = entries
        assert (failed.status, failed.source, failed.closed_at) == (
            "FAILED",
            TaskSource.DIRECT,
            closed,
        )
        assert (done.status, done.schedule_id) == ("COMPLETED", "sched-1")
        # Exactly one page per request: fetched once, never iterated onward.
        pages.fetch_next_page.assert_awaited_once()
        kwargs = mock_client.list_workflows.call_args.kwargs
        assert (kwargs["page_size"], kwargs["next_page_token"]) == (2, b"page-1")

    def test_task_history_last_page_has_no_token(self, workflow_gw, mock_client):
        self._history_page(mock_client, [])
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            entries, token = asyncio.run(workflow_gw.list_task_history(page_size=20))

        assert (entries, token) == ([], None)

    def test_task_history_skips_an_unmapped_status_row(self, workflow_gw, mock_client):
        self._history_page(
            mock_client,
            [
                _execution("odd", status=None),
                _execution("ok", status=WorkflowExecutionStatus.CANCELED),
            ],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            entries, _ = asyncio.run(workflow_gw.list_task_history(page_size=20))

        assert [e.id for e in entries] == ["ok"]

    def test_task_history_bad_token_is_a_bad_request(self, workflow_gw, mock_client):
        self._history_page(
            mock_client,
            [],
            error=RPCError("bad token", RPCStatusCode.INVALID_ARGUMENT, b""),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(BadRequestError, match="page token"):
                asyncio.run(
                    workflow_gw.list_task_history(page_size=20, next_page_token=b"x")
                )

    def test_task_history_rejected_query_is_upstream(self, workflow_gw, mock_client):
        # No token was sent, so INVALID_ARGUMENT is about the query itself
        # (standard visibility) — the server's problem, not the caller's.
        self._history_page(
            mock_client,
            [],
            error=RPCError("bad query", RPCStatusCode.INVALID_ARGUMENT, b""),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError):
                asyncio.run(workflow_gw.list_task_history(page_size=20))

    def test_task_history_unavailable_is_upstream(self, workflow_gw, mock_client):
        self._history_page(
            mock_client,
            [],
            error=RPCError("unavailable", RPCStatusCode.UNAVAILABLE, b""),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError):
                asyncio.run(workflow_gw.list_task_history(page_size=20))

    # ==================== create_schedule ====================

    def test_create_schedule_freezes_action_policy_and_memo(
        self, workflow_gw, mock_client
    ):
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(workflow_gw.create_schedule(_schedule()))

        args, kwargs = mock_client.create_schedule.call_args
        assert args[0] == "robot01-sched-001"
        schedule: Schedule = args[1]
        assert isinstance(schedule.action, ScheduleActionStartWorkflow)
        assert schedule.action.task_queue == OWN_QUEUE
        # One robot does one thing at a time: a trigger must never overlap the
        # run the previous trigger started.
        assert schedule.policy.overlap == ScheduleOverlapPolicy.SKIP
        # The cron rides in the spec with itself as the `#` comment: Temporal
        # keeps the comment on the compiled calendar, which is how get/list
        # echo the string back after the memo stopped carrying it.
        assert schedule.spec.cron_expressions == ["*/3 * * * * # */3 * * * *"]
        assert schedule.spec.time_zone_name == "Asia/Taipei"
        # The memo is the list path's only readable channel for provenance and
        # (since the multi-robot scope work) the owning robot -- and nothing
        # else: a memo cannot be rewritten by an update, the trigger can.
        assert kwargs["memo"] == {
            "robot_id": "robot01",
            "map_name": "full",
            "task_template_id": "0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55",
            "task_template_name": "Morning patrol",
        }

    def test_create_schedule_maps_a_duplicate_to_bad_request(
        self, workflow_gw, mock_client
    ):
        mock_client.create_schedule.side_effect = ScheduleAlreadyRunningError()
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(BadRequestError, match="already exists"):
                asyncio.run(workflow_gw.create_schedule(_schedule()))

    def test_create_schedule_maps_other_failures_to_internal(
        self, workflow_gw, mock_client
    ):
        mock_client.create_schedule.side_effect = Exception("boom")
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="Create schedule failed"):
                asyncio.run(workflow_gw.create_schedule(_schedule()))

    # ==================== get_schedule ====================

    def test_get_schedule_reads_the_cron_from_the_calendar_comment(
        self, workflow_gw, mock_client
    ):
        # Temporal compiles cron_expressions into calendar specs and forgets the
        # string, but keeps the `# comment` -- that, not the memo, is what
        # round-trips the registered cron.
        self._schedule_handle(
            mock_client,
            describe=_own_schedule_desc(
                {"map_name": "full"},
                spec=ScheduleSpec(
                    calendars=[_compiled_cron("*/3 * * * *")],
                    time_zone_name="Asia/Taipei",
                ),
            ),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            view = asyncio.run(workflow_gw.get_schedule("robot01-sched-001"))

        assert view.trigger.cron == "*/3 * * * *"
        assert view.trigger.timezone == "Asia/Taipei"
        assert view.map_name == "full"
        assert view.paused is False
        assert view.next_run_times[0].tzinfo is not None

    def test_get_schedule_reads_a_legacy_trigger_from_the_memo(
        self, workflow_gw, mock_client
    ):
        """A cron registered before the calendar comment existed compiled to a
        comment-less calendar, so the memo's copy is the only string left."""
        self._schedule_handle(
            mock_client,
            describe=_own_schedule_desc(
                {"cron": "*/3 * * * *", "timezone": "Asia/Taipei"},
                spec=ScheduleSpec(calendars=[ScheduleCalendarSpec()]),
            ),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            view = asyncio.run(workflow_gw.get_schedule("robot01-sched-001"))

        assert view.trigger.cron == "*/3 * * * *"
        assert view.trigger.timezone == "Asia/Taipei"

    def test_get_schedule_reads_legacy_provenance_memo_keys(
        self, workflow_gw, mock_client
    ):
        """Schedules registered before the TaskTemplate rename carry
        saved_task_* memo keys. A memo is immutable on the server -- nothing
        can rewrite the ones already in Temporal -- so the view must keep
        reading them forever."""
        self._schedule_handle(
            mock_client,
            describe=_own_schedule_desc(
                {
                    "cron": "*/3 * * * *",
                    "saved_task_id": "0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55",
                    "saved_task_name": "Morning patrol",
                }
            ),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            view = asyncio.run(workflow_gw.get_schedule("robot01-sched-001"))

        assert view.task_template_id == "0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55"
        assert view.task_template_name == "Morning patrol"

    def test_get_schedule_falls_back_to_the_spec_without_a_memo(
        self, workflow_gw, mock_client
    ):
        desc = _own_schedule_desc({})
        desc.schedule.spec = ScheduleSpec(
            intervals=[ScheduleIntervalSpec(every=timedelta(seconds=1800))]
        )
        self._schedule_handle(mock_client, describe=desc)
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            view = asyncio.run(workflow_gw.get_schedule("robot01-sched-001"))

        assert view.trigger.interval_seconds == 1800

    def test_get_schedule_not_found(self, workflow_gw, mock_client):
        handle = self._schedule_handle(mock_client)
        handle.describe.side_effect = _not_found()
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(NotFoundError, match="not found"):
                asyncio.run(workflow_gw.get_schedule("missing"))

    # ==================== pause / resume / delete ====================

    def test_schedule_verbs_act_after_the_ownership_describe(
        self, workflow_gw, mock_client
    ):
        handle = self._schedule_handle(mock_client, describe=_own_schedule_desc({}))
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(workflow_gw.pause_schedule("robot01-sched-001"))
            asyncio.run(workflow_gw.resume_schedule("robot01-sched-001"))
            asyncio.run(workflow_gw.delete_schedule("robot01-sched-001"))

        handle.pause.assert_awaited_once()
        handle.unpause.assert_awaited_once()
        handle.delete.assert_awaited_once()
        assert handle.describe.await_count == 3

    def test_schedule_verbs_not_found(self, workflow_gw, mock_client):
        handle = self._schedule_handle(mock_client)
        handle.describe.side_effect = _not_found()
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            for verb in (
                workflow_gw.pause_schedule,
                workflow_gw.resume_schedule,
                workflow_gw.delete_schedule,
            ):
                with pytest.raises(NotFoundError, match="not found"):
                    asyncio.run(verb("missing"))

        handle.pause.assert_not_awaited()
        handle.unpause.assert_not_awaited()
        handle.delete.assert_not_awaited()

    # ==================== update_schedule_trigger ====================

    def test_update_schedule_trigger_swaps_only_the_spec(
        self, workflow_gw, mock_client
    ):
        """Cron -> interval, in place: the callback hands back the described
        schedule with a new spec and everything else -- the frozen action,
        the SKIP policy, the paused state -- exactly as described."""
        desc = _own_schedule_desc(
            {"map_name": "full"},
            spec=ScheduleSpec(calendars=[_compiled_cron("*/3 * * * *")]),
        )
        desc.schedule.state = SimpleNamespace(paused=True, note="keep me")
        desc.schedule.policy = SimpleNamespace(overlap=ScheduleOverlapPolicy.SKIP)
        action_before = desc.schedule.action
        handle = self._schedule_handle(mock_client, describe=desc)

        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(
                workflow_gw.update_schedule_trigger(
                    "robot01-sched-001", ScheduleTrigger(interval_seconds=1800)
                )
            )

        handle.update.assert_awaited_once()
        (update,) = handle.updates
        assert isinstance(update, ScheduleUpdate)
        assert update.schedule.spec.intervals[0].every == timedelta(seconds=1800)
        assert update.schedule.spec.calendars == []
        assert update.schedule.action is action_before
        assert update.schedule.state.paused is True
        assert update.schedule.state.note == "keep me"
        assert update.schedule.policy.overlap == ScheduleOverlapPolicy.SKIP
        # No search-attribute rewrite rides along.
        assert update.search_attributes is None

    def test_update_schedule_trigger_plants_the_cron_comment(
        self, workflow_gw, mock_client
    ):
        # The same `<cron> # <cron>` shape create uses, so the edited cron
        # echoes back from the calendar comment like a freshly registered one.
        handle = self._schedule_handle(mock_client, describe=_own_schedule_desc({}))

        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(
                workflow_gw.update_schedule_trigger(
                    "robot01-sched-001",
                    ScheduleTrigger(cron="0 8 * * 1-5", timezone="Asia/Taipei"),
                )
            )

        (update,) = handle.updates
        assert update.schedule.spec.cron_expressions == ["0 8 * * 1-5 # 0 8 * * 1-5"]
        assert update.schedule.spec.time_zone_name == "Asia/Taipei"

    def test_update_schedule_trigger_validates_before_any_rpc(
        self, workflow_gw, mock_client
    ):
        handle = self._schedule_handle(mock_client, describe=_own_schedule_desc({}))

        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(BadRequestError, match="either cron or intervalSeconds"):
                asyncio.run(
                    workflow_gw.update_schedule_trigger(
                        "robot01-sched-001", ScheduleTrigger()
                    )
                )

        handle.update.assert_not_awaited()

    def test_update_schedule_trigger_not_found(self, workflow_gw, mock_client):
        handle = self._schedule_handle(mock_client)
        handle.update.side_effect = _not_found()

        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(NotFoundError, match="not found"):
                asyncio.run(
                    workflow_gw.update_schedule_trigger(
                        "missing", ScheduleTrigger(interval_seconds=60)
                    )
                )

    def test_update_schedule_trigger_maps_other_failures_to_internal(
        self, workflow_gw, mock_client
    ):
        handle = self._schedule_handle(mock_client)
        handle.update.side_effect = RPCError("boom", RPCStatusCode.UNAVAILABLE, b"")

        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="Update schedule failed"):
                asyncio.run(
                    workflow_gw.update_schedule_trigger(
                        "robot01-sched-001", ScheduleTrigger(interval_seconds=60)
                    )
                )

    # ==================== list_schedules ====================

    def test_list_schedules_projects_owned_rows(self, workflow_gw, mock_client):
        item = SimpleNamespace(
            id="robot01-sched-001",
            schedule=SimpleNamespace(
                # The list shape: workflow type name only, no task queue.
                action=SimpleNamespace(workflow=WORKFLOW_TYPE_NAME),
                spec=ScheduleSpec(),
                state=SimpleNamespace(paused=True),
            ),
            info=SimpleNamespace(next_action_times=[]),
            memo=AsyncMock(
                return_value={"robot_id": "robot01", "interval_seconds": 1800}
            ),
        )

        async def _iterator():
            yield item

        mock_client.list_schedules.return_value = _iterator()
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            views = asyncio.run(workflow_gw.list_schedules())

        assert len(views) == 1
        assert views[0].trigger.interval_seconds == 1800
        assert views[0].paused is True
        # The list path cannot reach the frozen args; steps are describe-only.
        assert views[0].steps == []

    def test_list_schedules_maps_failures_to_internal(self, workflow_gw, mock_client):
        mock_client.list_schedules.side_effect = Exception("boom")
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="List schedules failed"):
                asyncio.run(workflow_gw.list_schedules())


class TestHistoryQuery:
    def test_scopes_to_this_robot_and_closed_runs(self):
        query = _history_query(OWN_QUEUE)

        assert f"WorkflowType = '{WORKFLOW_TYPE_NAME}'" in query
        assert f"TaskQueue = '{OWN_QUEUE}'" in query
        for status in (
            "Completed",
            "Failed",
            "TimedOut",
            "Canceled",
            "Terminated",
        ):
            assert f"'{status}'" in query
        assert "Running" not in query
        # SQL visibility rejects a custom ORDER BY.
        assert "ORDER BY" not in query

    def test_status_folds_every_temporal_status_it_reports_as(self):
        query = _history_query(OWN_QUEUE, status="CANCELED")

        assert "ExecutionStatus IN ('Canceled', 'Terminated')" in query
        assert "Completed" not in query

    def test_since_is_a_utc_close_time_bound(self):
        taipei = timezone(timedelta(hours=8))
        query = _history_query(
            OWN_QUEUE, since=datetime(2026, 8, 10, 17, 0, tzinfo=taipei)
        )

        assert query.endswith("AND CloseTime >= '2026-08-10T09:00:00+00:00'")


class TestScheduleTriggerMapping:
    """The pure trigger<->spec helpers, no client involved."""

    def test_build_spec_from_cron(self):
        spec = _build_schedule_spec(
            ScheduleTrigger(cron="0 9 * * 1-5", timezone="Asia/Taipei")
        )
        # The string twice: once for Temporal to compile, once as the comment it
        # keeps on the compiled calendar so get/list can echo it back.
        assert spec.cron_expressions == ["0 9 * * 1-5 # 0 9 * * 1-5"]
        assert spec.time_zone_name == "Asia/Taipei"

    @pytest.mark.parametrize(
        "cron, reason",
        [
            ("0 9 * * * # mine", "must not contain '#'"),
            ("CRON_TZ=UTC 0 9 * * *", "CRON_TZ=/TZ= prefix"),
            ("TZ=UTC 0 9 * * *", "CRON_TZ=/TZ= prefix"),
            ("@every 30m", "use intervalSeconds"),
        ],
    )
    def test_build_spec_refuses_crons_that_would_break_the_echo(self, cron, reason):
        with pytest.raises(BadRequestError, match=reason):
            _build_schedule_spec(ScheduleTrigger(cron=cron))

    def test_build_spec_from_interval(self):
        spec = _build_schedule_spec(ScheduleTrigger(interval_seconds=1800))
        assert spec.intervals[0].every == timedelta(seconds=1800)

    def test_build_spec_requires_a_trigger(self):
        with pytest.raises(BadRequestError, match="either cron or intervalSeconds"):
            _build_schedule_spec(ScheduleTrigger())

    def test_spec_round_trips_back_to_a_trigger(self):
        # As Temporal describes it: the compiled calendar carrying the comment.
        trigger = _spec_to_trigger(
            ScheduleSpec(
                calendars=[_compiled_cron("0 9 * * 1-5")], time_zone_name="Asia/Taipei"
            )
        )
        assert (trigger.cron, trigger.timezone) == ("0 9 * * 1-5", "Asia/Taipei")

        # As this process built it, before sending: the comment is stripped.
        trigger = _spec_to_trigger(
            _build_schedule_spec(ScheduleTrigger(cron="0 9 * * 1-5", timezone="Asia/Taipei"))
        )
        assert (trigger.cron, trigger.timezone) == ("0 9 * * 1-5", "Asia/Taipei")

        trigger = _spec_to_trigger(
            ScheduleSpec(intervals=[ScheduleIntervalSpec(every=timedelta(seconds=60))])
        )
        assert trigger.interval_seconds == 60

    def test_spec_without_a_comment_yields_no_cron(self):
        # A legacy cron, or one registered outside this API: the compiled
        # ranges cannot be turned back into a string.
        trigger = _spec_to_trigger(ScheduleSpec(calendars=[ScheduleCalendarSpec()]))
        assert trigger.cron is None and trigger.interval_seconds is None

    def test_spec_wins_over_a_stale_memo(self):
        # A legacy schedule whose cron was edited to an interval: the memo still
        # says cron, the spec says interval, and the spec is what fires.
        trigger = _read_trigger(
            {"cron": "*/3 * * * *", "timezone": "Asia/Taipei"},
            ScheduleSpec(intervals=[ScheduleIntervalSpec(every=timedelta(seconds=1800))]),
        )
        assert trigger.interval_seconds == 1800
        assert trigger.cron is None
        assert trigger.timezone is None

    def test_memo_fills_in_for_a_legacy_calendar_without_a_comment(self):
        trigger = _read_trigger(
            {"cron": "*/3 * * * *", "timezone": "Asia/Taipei"},
            ScheduleSpec(calendars=[ScheduleCalendarSpec()]),
        )
        assert (trigger.cron, trigger.timezone) == ("*/3 * * * *", "Asia/Taipei")
