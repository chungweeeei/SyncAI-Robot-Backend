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
    WorkflowExecutionCount,
    WorkflowExecutionCountAggregationGroup,
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
    StepStatus,
    StepType,
    TaskKind,
    TaskProvenance,
    TaskSource,
    WorkflowTask,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.search_attributes import (  # noqa: E402
    TASK_KIND_KEY,
    TASK_MAP_KEY,
    TASK_NAME_KEY,
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
    kind=None,
    name=None,
    map_name=None,
) -> SimpleNamespace:
    """One row of a visibility listing, as the gateway's list paths read it.

    The attribute stub answers by key name, the way the typed accessor does:
    the gateway reads four keys off a row and must not be handed the
    schedule id for all of them."""
    attributes = {
        "TemporalScheduledById": schedule_id,
        TASK_KIND_KEY.name: kind,
        TASK_NAME_KEY.name: name,
        TASK_MAP_KEY.name: map_name,
    }
    return SimpleNamespace(
        id=task_id,
        run_id="run-1",
        status=status,
        start_time=datetime(2026, 8, 10, 8, 0, tzinfo=timezone.utc),
        close_time=close_time,
        typed_search_attributes=SimpleNamespace(get=lambda key: attributes.get(key.name)),
    )


def _count(**groups: int) -> WorkflowExecutionCount:
    """A `GROUP BY ExecutionStatus` answer, keyed by Temporal's spellings."""
    return WorkflowExecutionCount(
        count=sum(groups.values()),
        groups=[
            WorkflowExecutionCountAggregationGroup(count=n, group_values=[status])
            for status, n in groups.items()
        ],
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
        handle.signal = AsyncMock()
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
        # Nothing said about the run's origin, nothing stamped on it.
        assert list(kwargs["search_attributes"]) == []

    def test_start_task_stamps_provenance_as_search_attributes(
        self, workflow_gw, mock_client
    ):
        # Kind, name and map travel as search attributes, not in the workflow
        # argument: the history filters and counts on them, and the map has
        # to reach scheduled runs too, which only attributes do.
        self._listing(mock_client, [])
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(
                workflow_gw.start_task(
                    _task(),
                    TaskProvenance(
                        kind=TaskKind.GOAL, name="Morning patrol", map_name="lab"
                    ),
                )
            )

        kwargs = mock_client.start_workflow.call_args[1]
        assert kwargs["args"] == [_task()]
        attributes = kwargs["search_attributes"]
        assert attributes.get(TASK_KIND_KEY) == "goal"
        assert attributes.get(TASK_NAME_KEY) == "Morning patrol"
        assert attributes.get(TASK_MAP_KEY) == "lab"

    def test_start_task_stamps_only_the_kind_when_there_is_no_name(
        self, workflow_gw, mock_client
    ):
        self._listing(mock_client, [])
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(
                workflow_gw.start_task(_task(), TaskProvenance(kind=TaskKind.STANDUP))
            )

        attributes = mock_client.start_workflow.call_args[1]["search_attributes"]
        assert attributes.get(TASK_KIND_KEY) == "standup"
        assert attributes.get(TASK_NAME_KEY) is None
        # A job that drives nowhere holds no map.
        assert attributes.get(TASK_MAP_KEY) is None

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

    def test_get_task_state_reads_paused_off_the_held_step(self, workflow_gw, mock_client):
        # Temporal has no paused status — a held run is RUNNING to it. The
        # workflow marks the step it is holding at PAUSED, and that one fact
        # is folded into the task status here, with no second query.
        self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
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
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            state = asyncio.run(workflow_gw.get_task_state("robot01-task-001"))

        assert state.status == "PAUSED"
        assert [s.status for s in state.steps] == [StepStatus.COMPLETED, StepStatus.PAUSED]

    def test_get_task_state_a_closed_run_is_never_paused(self, workflow_gw, mock_client):
        # Terminated while held: the step still reads PAUSED on replay, but
        # the run is over and the Temporal status wins.
        self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.TERMINATED, task_queue=OWN_QUEUE
            ),
            steps=[
                Step(
                    id="step1",
                    type=StepType.MOVE,
                    params=MoveParams(x=1.0, y=2.0, theta=90.0),
                    status=StepStatus.PAUSED,
                )
            ],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            state = asyncio.run(workflow_gw.get_task_state("robot01-task-001"))

        assert state.status == "CANCELED"

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

    # ==================== pause_task / resume_task ====================

    def test_pause_task_signals_after_the_ownership_describe(self, workflow_gw, mock_client):
        handle = self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(workflow_gw.pause_task("robot01-task-001"))

        handle.describe.assert_awaited_once()
        # By name, like the step-state query: the gateway never imports the
        # workflow class.
        handle.signal.assert_awaited_once_with("pause")

    def test_resume_task_signals_resume(self, workflow_gw, mock_client):
        handle = self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(workflow_gw.resume_task("robot01-task-001"))

        handle.signal.assert_awaited_once_with("resume")

    @pytest.mark.parametrize(
        "status",
        [
            WorkflowExecutionStatus.COMPLETED,
            WorkflowExecutionStatus.FAILED,
            WorkflowExecutionStatus.CANCELED,
            WorkflowExecutionStatus.TERMINATED,
        ],
    )
    def test_hold_verbs_refuse_a_closed_run_with_a_code(self, workflow_gw, mock_client, status):
        # A pause sent a moment after the run closed is a 409 with a stable
        # code — not the server's own "already completed" error as a 502.
        handle = self._workflow_handle(
            mock_client, describe=SimpleNamespace(status=status, task_queue=OWN_QUEUE)
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(ConflictError, match="not running") as exc_info:
                asyncio.run(workflow_gw.pause_task("robot01-task-001"))
            with pytest.raises(ConflictError):
                asyncio.run(workflow_gw.resume_task("robot01-task-001"))

        assert exc_info.value.code == "task_not_running"
        handle.signal.assert_not_awaited()

    def test_pause_task_not_found(self, workflow_gw, mock_client):
        handle = self._workflow_handle(mock_client)
        handle.describe.side_effect = _not_found()
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(NotFoundError, match="not found"):
                asyncio.run(workflow_gw.pause_task("missing"))
        handle.signal.assert_not_awaited()

    def test_pause_task_maps_a_failed_signal_to_upstream(self, workflow_gw, mock_client):
        handle = self._workflow_handle(
            mock_client,
            describe=SimpleNamespace(
                status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
            ),
        )
        handle.signal.side_effect = RPCError("unavailable", RPCStatusCode.UNAVAILABLE, b"")
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="Pause workflow failed"):
                asyncio.run(workflow_gw.pause_task("robot01-task-001"))

    def test_resume_task_names_its_own_verb_on_a_failed_describe(
        self, workflow_gw, mock_client
    ):
        # The shared describe helper reports the verb the operator asked for.
        handle = self._workflow_handle(mock_client)
        handle.describe.side_effect = RPCError("unavailable", RPCStatusCode.UNAVAILABLE, b"")
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="Resume workflow failed"):
                asyncio.run(workflow_gw.resume_task("robot01-task-001"))

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
                _execution("robot01-task-001", map_name="lab"),
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
        # The attribute as stamped, or None: filling in the loaded map for a
        # run older than it is the reader's call (ActiveTask.map_in_use).
        assert (direct.map_name, scheduled.map_name) == ("lab", None)

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

    # ==================== cancel_active_tasks ====================

    def _owned_running(self):
        return SimpleNamespace(
            status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
        )

    def test_cancel_active_tasks_bypasses_the_cache(self, workflow_gw, mock_client):
        handle = self._workflow_handle(mock_client, describe=self._owned_running())
        self._listing(mock_client, [_execution("robot01-task-001")])
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):

            async def _poll_then_cancel():
                await workflow_gw.list_active_tasks()
                return await workflow_gw.cancel_active_tasks()

            cancelled = asyncio.run(_poll_then_cancel())

        # A console poll a moment ago must not stand in for the sweep.
        assert mock_client.list_workflows.call_count == 2
        assert cancelled == ["robot01-task-001"]
        handle.cancel.assert_awaited_once()

    def test_cancel_active_tasks_includes_the_last_start(self, workflow_gw, mock_client):
        self._workflow_handle(mock_client, describe=self._owned_running())
        self._listing(mock_client, [])
        workflow_gw._last_started_task_id = "robot01-task-002"
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            cancelled = asyncio.run(workflow_gw.cancel_active_tasks())

        # Not in the index yet, still cancelled.
        assert cancelled == ["robot01-task-002"]

    def test_cancel_active_tasks_carries_on_past_a_failure(
        self, workflow_gw, mock_client
    ):
        handle = self._workflow_handle(mock_client, describe=self._owned_running())
        handle.cancel.side_effect = [
            RPCError("unavailable", RPCStatusCode.UNAVAILABLE, b""),
            _not_found(),
            None,
        ]
        self._listing(
            mock_client,
            [_execution("a"), _execution("b"), _execution("c")],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            cancelled = asyncio.run(workflow_gw.cancel_active_tasks())

        # One that failed is logged, one that closed meanwhile is fine; neither
        # stops the rest.
        assert cancelled == ["c"]
        assert handle.cancel.await_count == 3

    def test_cancel_active_tasks_raises_when_the_sweep_fails(
        self, workflow_gw, mock_client
    ):
        mock_client.list_workflows.side_effect = RPCError(
            "unavailable", RPCStatusCode.UNAVAILABLE, b""
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError):
                asyncio.run(workflow_gw.cancel_active_tasks())

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
                    kind="goal",
                ),
                _execution(
                    "robot01-sched-001-2026-08-10T09",
                    status=WorkflowExecutionStatus.COMPLETED,
                    schedule_id="sched-1",
                    close_time=closed,
                    kind="schedule",
                    name="Morning patrol",
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
        assert (failed.kind, failed.name) == (TaskKind.GOAL, None)
        assert (done.status, done.schedule_id) == ("COMPLETED", "sched-1")
        assert (done.kind, done.name) == (TaskKind.SCHEDULE, "Morning patrol")
        # Exactly one page per request: fetched once, never iterated onward.
        pages.fetch_next_page.assert_awaited_once()
        kwargs = mock_client.list_workflows.call_args.kwargs
        assert (kwargs["page_size"], kwargs["next_page_token"]) == (2, b"page-1")

    def test_task_history_reads_provenance_off_runs_that_predate_it(
        self, workflow_gw, mock_client
    ):
        # A schedule registered before the attributes existed still fires; its
        # runs are SCHEDULE off TemporalScheduledById with no name. A direct
        # run dispatched without saying is unlabelled, and a kind this build
        # does not know reads as none rather than failing the row.
        self._history_page(
            mock_client,
            [
                _execution("legacy-sched-2026-08-10T09", schedule_id="legacy"),
                _execution("robot01-task-002"),
                _execution("robot01-task-003", kind="teleport"),
            ],
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            entries, _ = asyncio.run(workflow_gw.list_task_history(page_size=3))

        assert [(e.kind, e.name, e.map_name) for e in entries] == [
            (TaskKind.SCHEDULE, None, None),
            (None, None, None),
            (None, None, None),
        ]

    def test_task_history_passes_every_filter_into_one_query(
        self, workflow_gw, mock_client
    ):
        self._history_page(mock_client, [])
        since = datetime(2026, 8, 10, tzinfo=timezone.utc)
        until = datetime(2026, 8, 11, tzinfo=timezone.utc)
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(
                workflow_gw.list_task_history(
                    page_size=20,
                    status="FAILED",
                    since=since,
                    until=until,
                    kind=TaskKind.TASK,
                    name="Morning patrol",
                )
            )

        query = mock_client.list_workflows.call_args.args[0]
        assert query == _history_query(
            OWN_QUEUE,
            status="FAILED",
            since=since,
            until=until,
            kind=TaskKind.TASK,
            name="Morning patrol",
        )

    # ==================== task_history_stats ====================

    def test_task_history_stats_is_one_grouped_count(self, workflow_gw, mock_client):
        mock_client.count_workflows = AsyncMock(
            return_value=_count(Completed=5, Failed=1, TimedOut=1, Canceled=2, Terminated=1)
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            stats = asyncio.run(
                workflow_gw.task_history_stats(kind=TaskKind.LIEDOWN, name="x")
            )

        # TimedOut folds into FAILED, Terminated into CANCELED, as the list does.
        assert (stats.total, stats.completed, stats.failed, stats.canceled) == (
            10, 5, 2, 3
        )
        mock_client.count_workflows.assert_awaited_once()
        query = mock_client.count_workflows.await_args.args[0]
        assert query.endswith(" GROUP BY ExecutionStatus")
        assert "TaskKind = 'liedown'" in query and "TaskName = 'x'" in query

    def test_task_history_stats_ignores_a_group_it_does_not_report(
        self, workflow_gw, mock_client
    ):
        mock_client.count_workflows = AsyncMock(return_value=_count(Running=4, Completed=1))
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            stats = asyncio.run(workflow_gw.task_history_stats())

        assert (stats.total, stats.completed) == (1, 1)

    def test_task_history_stats_maps_a_rejected_query_to_upstream(
        self, workflow_gw, mock_client
    ):
        # An unregistered search attribute answers INVALID_ARGUMENT; the
        # operator sees a 502, the log says which attribute to register.
        mock_client.count_workflows = AsyncMock(
            side_effect=RPCError(
                "not a valid search attribute", RPCStatusCode.INVALID_ARGUMENT, b""
            )
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            with pytest.raises(UpstreamError, match="stats failed"):
                asyncio.run(workflow_gw.task_history_stats())

    def test_task_history_stats_connection_failure(self, workflow_gw):
        with patch(CONNECT, new_callable=AsyncMock, side_effect=ConnectionError("down")):
            with pytest.raises(UpstreamError, match="connect"):
                asyncio.run(workflow_gw.task_history_stats())

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
        # What the *runs* carry, as opposed to the schedule: every run this
        # action starts is stamped SCHEDULE plus the template's name, which is
        # how the history counts them without a describe per row.
        attributes = schedule.action.typed_search_attributes
        assert attributes.get(TASK_KIND_KEY) == "schedule"
        assert attributes.get(TASK_NAME_KEY) == "Morning patrol"
        # And the map its frozen positions are on, which the memo cannot hand
        # a run: what lets a run that fires after a map switch refuse to drive.
        assert attributes.get(TASK_MAP_KEY) == "full"

    def test_create_schedule_without_a_template_stamps_only_the_kind(
        self, workflow_gw, mock_client
    ):
        bare = ScheduleTask(
            id="robot01-sched-002",
            trigger=ScheduleTrigger(interval_seconds=600),
            definition=WorkflowTaskDefinition(steps=[MOVE_STEP]),
        )
        with patch(CONNECT, new_callable=AsyncMock, return_value=mock_client):
            asyncio.run(workflow_gw.create_schedule(bare))

        attributes = mock_client.create_schedule.call_args.args[1].action.typed_search_attributes
        assert attributes.get(TASK_KIND_KEY) == "schedule"
        assert attributes.get(TASK_NAME_KEY) is None
        assert attributes.get(TASK_MAP_KEY) is None

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

    def test_until_is_a_utc_close_time_upper_bound(self):
        taipei = timezone(timedelta(hours=8))
        query = _history_query(
            OWN_QUEUE, until=datetime(2026, 8, 11, 8, 0, tzinfo=taipei)
        )

        assert query.endswith("AND CloseTime <= '2026-08-11T00:00:00+00:00'")

    def test_a_direct_kind_is_the_custom_attribute(self):
        query = _history_query(OWN_QUEUE, kind=TaskKind.GOAL)

        assert query.endswith("AND TaskKind = 'goal'")

    def test_the_scheduled_kind_keys_on_temporals_own_attribute(self):
        # So schedules registered before TaskKind existed still count.
        query = _history_query(OWN_QUEUE, kind=TaskKind.SCHEDULE)

        assert query.endswith("AND TemporalScheduledById IS NOT NULL")
        assert "TaskKind" not in query

    def test_a_name_is_quoted_for_the_filter_parser(self):
        query = _history_query(OWN_QUEUE, name="O'Brien's \\ round")

        assert query.endswith("AND TaskName = 'O\\'Brien\\'s \\\\ round'")

    def test_every_filter_composes_in_a_fixed_order(self):
        since = datetime(2026, 8, 10, tzinfo=timezone.utc)
        until = datetime(2026, 8, 11, tzinfo=timezone.utc)
        query = _history_query(
            OWN_QUEUE,
            status="FAILED",
            since=since,
            until=until,
            kind=TaskKind.TASK,
            name="patrol",
        )

        assert query == (
            f"WorkflowType = '{WORKFLOW_TYPE_NAME}' AND TaskQueue = '{OWN_QUEUE}' "
            "AND ExecutionStatus IN ('Failed', 'TimedOut') "
            "AND CloseTime >= '2026-08-10T00:00:00+00:00' "
            "AND CloseTime <= '2026-08-11T00:00:00+00:00' "
            "AND TaskKind = 'task' AND TaskName = 'patrol'"
        )
        assert "ORDER BY" not in query

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
