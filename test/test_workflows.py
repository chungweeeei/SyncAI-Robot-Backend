"""Tests for ``RobotWorkflow``, run against the time-skipping test server.

Unlike the activity tests (``test_activities.py``, ``ActivityEnvironment``)
and the gateway tests (client mocked at ``Client.connect``), these run the
*real* workflow code under ``WorkflowEnvironment.start_time_skipping()``: a
real Temporal test server, a real ``Worker``, the real retry policy. What is
mocked is only the activity layer — the workflow schedules activities by
function reference, but all that crosses the wire is the ``@activity.defn``
*name*, so a local ``async def`` registered under the same name is the exact
seam a production worker presents. Activity internals stay pinned in
``test_activities.py``; here they are stand-ins that record, gate, or fail.

What is pinned:

* Steps run strictly in order and their pydantic params survive the round
  trip through the pydantic data converter (the worker's converter is not
  the default one — forgetting it on the client is a real integration bug).
* ``get_step_states`` reports per-step status mid-run — a gated activity
  holds the workflow at a known point — and remains queryable after the
  workflow has closed, which is exactly how the REST layer reads the state
  of a finished task.
* A failed step fails the whole run: the failing step carries the cause in
  ``error_msg``, later steps stay PENDING and their activities never run.
  Both failure shapes are covered — the activity raising, and the activity
  returning ``success=False`` — because the workflow reports them with
  different messages.
* The retry policy is 3 attempts of the *same* step, not a re-run of the
  workflow. Time skipping is what makes the two 5 s backoffs free.
* ``handle.cancel()`` closes the run as canceled, and only the in-flight
  step is touched: it ends CANCELED / "Task canceled" whatever it was doing —
  mid-MOVE, mid-posture (where the cancel arrives wrapped in an
  ActivityError), held, or waiting — so the console never reads a cancel as
  a failure.
* The hold (``pause`` / ``resume`` signals): a MOVE is interrupted at once
  and re-sent as a fresh activity on resume; any other step finishes and the
  run holds before the next one; the held step reads PAUSED while it waits;
  a pause with nothing left to hold lets the run complete; a pause and a
  cancel landing together end the run canceled rather than held.
* Groups (``Step.group``): adjacent members run side by side and the next
  step waits for all of them; the first failure cancels the rest (CANCELED,
  naming the failed step) and fails the run; a run cancel reaches every
  member, a SPEAK included (it heartbeats, so it is stopped mid-utterance);
  a pause interrupts the group's MOVE while its SPEAK plays on.
  SPEAK is dispatched to the IO queue, so a stand-in registered only there
  is what runs it.
* PATROL is dispatched with its map, its step id and how many waypoints it
  has already passed: 0 at first, and after a pause whatever its activity
  reported on ``patrol_progress`` (never less), so a resume does not start the
  round over; a retry carries its count in the heartbeat details, which reach
  the next attempt; a cancel ends it CANCELED like a MOVE.
* WAIT is a workflow timer, not an activity: a pause freezes it and the
  resume starts a second timer for only the remainder (read off the run's
  history), and a cancel during it is CANCELED / "Task canceled".
* Timing and the record: every step that ran carries ``started_at`` /
  ``ended_at``, a hold while it was under way lands in its ``paused_seconds``
  (one before it started does not), and every run, however it ends, hands
  ``record_task`` its TaskRecord as it closes -- without that activity ever
  changing how the run ends.
* Admission (``begin_task`` / ``end_task``, the robot's own state machine):
  every run asks the robot before its first step -- a scheduled one after it
  has read its template, since what it asks depends on the steps it will run
  -- and an accepted run reports its end exactly once, however it ends, before
  its record. A refusal fails the run at once on the first step that would
  have run (never a SKIPPED one), with the robot's sentence and code, and
  reports nothing. No answer after three tries fails the run the same way,
  but is not a "no": the robot may have accepted and lost only its answer, so
  that case alone is followed by one best-effort ``end_task(CANCELED)``. The
  reason a FAILED report carries is the failure's sentence, without the
  error-type prefix a step's ``error_msg`` stores. A cancel while either
  call is out does not strand it: the answer to begin_task is waited for, and
  the report is still made. A paused run reports nothing. Histories recorded
  before the gate (``test/fixtures/workflow_histories/pre_task_admission``)
  still replay.

Deliberately not covered: the unknown-step-type guard in the dispatch table.
``Step`` validation rejects anything outside ``StepType`` before a task can
reach the workflow, so that branch is unreachable through the public surface;
pinning it would mean forging an invalid Step.

Mechanics, matching the register of the other suites: no pytest-asyncio
(pytest 6.2.5, no plugins), so each test is a sync function wrapping one
``asyncio.run``. Every test boots its own environment — the test-server
binary is downloaded once and cached, after which a boot costs ~0.1 s, which
is cheaper than sharing one event loop across tests would be worth.
"""

import asyncio
import glob
import os
from contextlib import asynccontextmanager
from datetime import timedelta
from typing import Optional

import pytest

pytest.importorskip("temporalio")

from temporalio import activity  # noqa: E402
from temporalio.api.enums.v1 import IndexedValueType  # noqa: E402
from temporalio.api.operatorservice.v1 import (  # noqa: E402
    AddSearchAttributesRequest,
)
from temporalio.client import WorkflowFailureError, WorkflowHistory  # noqa: E402
from temporalio.common import (  # noqa: E402
    SearchAttributePair,
    TypedSearchAttributes,
)
from temporalio.contrib.pydantic import pydantic_data_converter  # noqa: E402
from temporalio.exceptions import (  # noqa: E402
    ActivityError,
    ApplicationError,
    CancelledError,
)
from temporalio.testing import WorkflowEnvironment  # noqa: E402
from temporalio.worker import Replayer, Worker  # noqa: E402

from syncai_backend.tasks.config import (  # noqa: E402
    PATROL_PROGRESS_SIGNAL,
    STEP_STARTED_SIGNAL,
    reports_start,
)
from syncai_backend.tasks.schema import (  # noqa: E402
    MoveParams,
    PatrolParams,
    SpeakParams,
    Step,
    StepStatus,
    StepType,
    TaskKind,
    TaskRecord,
    WaitParams,
    WorkflowTask,
    WorkflowTaskDefinition,
    continuation_steps,
)
from syncai_backend.tasks.search_attributes import (  # noqa: E402
    TASK_KIND_KEY,
    TASK_MAP_KEY,
    TASK_NAME_KEY,
)
from syncai_backend.helpers.task_guard import (  # noqa: E402
    ERROR_LATCHED,
    NOT_INITIALIZED,
    ROBOT_BUSY,
    ROBOT_STATE_STALE,
    ROBOT_STATE_UNREACHABLE,
    UNREACHABLE_MESSAGE,
    goal_refusal,
)
from syncai_backend.tasks.task_admission import (  # noqa: E402
    TaskAdmission,
    TaskAdmissionRequest,
    TaskEnd,
    TaskOutcome,
)
from syncai_backend.tasks.template_resolution import ResolvedTemplate  # noqa: E402
from syncai_backend.temporal.activities import ActivityResult  # noqa: E402
from syncai_backend.temporal.worker import HEARTBEAT_THROTTLE_MAX  # noqa: E402
from syncai_backend.temporal.workflows import (  # noqa: E402
    BEGIN_TASK_ATTEMPTS,
    END_TASK_REASON_MAX,
    RELEASE_TASK_ATTEMPTS,
    RobotWorkflow,
)

# Same shape the production worker derives from robot_id; the value itself is
# arbitrary here since client and worker share it within one test.
TASK_QUEUE = "robot01.ROBOT_TASK_QUEUE"
IO_TASK_QUEUE = "robot01.ROBOT_IO_QUEUE"

# Ten minutes: long enough that a test finishing in real time proves the
# time-skipping server skipped it, and that a pause lands well inside it.
WAIT_SECONDS = 600.0


def _task(task_id: str, *step_types: StepType) -> WorkflowTask:
    steps = [
        Step(
            id=f"step-{i}",
            type=step_type,
            # Schema contract: MOVE, CHARGE, PATROL and WAIT carry params, the posture
            # steps none (Step validation enforces it, the workflow relies on it
            # for its empty-args special case).
            params={
                StepType.MOVE: MoveParams(x=1.5, y=-2.5, theta=90.0),
                StepType.CHARGE: MoveParams(x=4.0, y=1.0, theta=180.0),
                StepType.WAIT: WaitParams(seconds=WAIT_SECONDS),
                StepType.PATROL: PatrolParams(
                    waypoints=[
                        MoveParams(x=float(n), y=0.0, theta=0.0) for n in range(4)
                    ]
                ),
            }.get(step_type),
        )
        for i, step_type in enumerate(step_types)
    ]
    return WorkflowTask(id=task_id, definition=WorkflowTaskDefinition(steps=steps))


def _instant_activities(calls: list):
    """The three activity names the workflow dispatches to, succeeding at once.

    Each records ``(name, arg)`` so a test can pin ordering and the params
    hand-off without ever leaving the happy path.
    """

    @activity.defn(name="execute_move")
    async def execute_move(params: MoveParams) -> ActivityResult:
        calls.append(("execute_move", params))
        return ActivityResult(success=True, goal_id="goal-1", state="succeeded")

    @activity.defn(name="execute_stand")
    async def execute_stand() -> ActivityResult:
        calls.append(("execute_stand", None))
        return ActivityResult(success=True, state="succeeded")

    @activity.defn(name="execute_lie_down")
    async def execute_lie_down() -> ActivityResult:
        calls.append(("execute_lie_down", None))
        return ActivityResult(success=True, state="succeeded")

    return [execute_move, execute_stand, execute_lie_down]


async def _report_started() -> None:
    """What RobotActivities._report_started does, from an async stand-in.

    A group member's run starts the next member once this arrives (or once
    the member ends), so a stand-in that parks without it parks the group's
    later starts too. A lone step's id carries no mark and reports nothing.
    """
    info = activity.info()
    if not reports_start(info.activity_id):
        return
    await activity.client().get_workflow_handle(
        info.workflow_id, run_id=info.workflow_run_id
    ).signal(STEP_STARTED_SIGNAL, info.activity_id)


def _gated_move(started: asyncio.Event, release: asyncio.Event):
    """A MOVE activity that parks until released, heartbeating while it waits.

    The heartbeats are load-bearing twice over: the workflow schedules every
    activity with ``heartbeat_timeout=3s`` (real time — the test server does
    not skip while an activity is outstanding), and a cancel is only ever
    delivered to a running activity in a heartbeat response.
    """

    @activity.defn(name="execute_move")
    async def execute_move(params: MoveParams) -> ActivityResult:
        await _report_started()
        started.set()
        while not release.is_set():
            activity.heartbeat()
            try:
                await asyncio.wait_for(release.wait(), timeout=0.2)
            except asyncio.TimeoutError:
                pass
        return ActivityResult(success=True, state="succeeded")

    return execute_move


def _recorder(records: list):
    """A ``record_task`` that keeps what every run hands it."""

    @activity.defn(name="record_task")
    async def record_task(record: TaskRecord) -> None:
        records.append(record)

    return record_task


def _admitter(answer=None, asked: Optional[list] = None, events: Optional[list] = None):
    """A ``begin_task`` answering ``answer``: accepted when None, raised when an
    exception, and a list is one answer per call (the last one repeated), so a
    test can drop the first call and answer the second.

    ``asked`` keeps every request; ``events`` gets a ``"begin_task"`` per call,
    for a test that pins the order across activities.
    """
    answers = list(answer) if isinstance(answer, list) else [answer]

    @activity.defn(name="begin_task")
    async def begin_task(request: TaskAdmissionRequest) -> TaskAdmission:
        if asked is not None:
            asked.append(request)
        if events is not None:
            events.append("begin_task")
        current = answers.pop(0) if len(answers) > 1 else answers[0]
        if isinstance(current, BaseException):
            raise current
        return current if current is not None else TaskAdmission(accepted=True)

    return begin_task


def _closer(ended: list, events: Optional[list] = None):
    """An ``end_task`` that keeps what every run hands it."""

    @activity.defn(name="end_task")
    async def end_task(report: TaskEnd) -> None:
        ended.append(report)
        if events is not None:
            events.append("end_task")

    return end_task


@asynccontextmanager
async def _worker(
    activities, io_activities=None, record_task=None, begin_task=None, end_task=None
):
    """A time-skipping environment with a worker polling TASK_QUEUE.

    The client is built with ``pydantic_data_converter`` to mirror
    ``run_worker`` — the default converter cannot serialize WorkflowTask.
    ``io_activities`` (SPEAK) get a second worker on IO_TASK_QUEUE, the way
    run_worker splits them, always beside a ``record_task`` (a discarding
    one unless given): every run ends with it. And beside a ``begin_task``
    that accepts and an ``end_task`` that discards, unless given: every run
    starts by asking the robot, and with nothing registered to answer it
    would fail closed before its first step. Both workers cap the heartbeat
    throttle as run_worker does: a cancel rides a heartbeat's reply, and left
    at the SDK default (0.8 x heartbeat_timeout) SPEAK's 75 s window would
    hold one for a minute.
    """
    env = await WorkflowEnvironment.start_time_skipping(
        data_converter=pydantic_data_converter
    )
    try:
        async with Worker(
            env.client,
            task_queue=TASK_QUEUE,
            workflows=[RobotWorkflow],
            activities=activities,
            max_heartbeat_throttle_interval=HEARTBEAT_THROTTLE_MAX,
        ):
            async with Worker(
                env.client,
                task_queue=IO_TASK_QUEUE,
                activities=list(io_activities or [])
                + [
                    record_task or _recorder([]),
                    begin_task or _admitter(),
                    end_task or _closer([]),
                ],
                max_heartbeat_throttle_interval=HEARTBEAT_THROTTLE_MAX,
            ):
                yield env.client
    finally:
        await env.shutdown()


async def _start(client, task: WorkflowTask):
    return await client.start_workflow(
        RobotWorkflow.run, task, id=task.id, task_queue=TASK_QUEUE
    )


def test_steps_run_in_order_and_all_complete():
    async def scenario():
        calls: list = []
        task = _task("task-happy", StepType.MOVE, StepType.STANDUP, StepType.LIEDOWN)

        async with _worker(_instant_activities(calls)) as client:
            handle = await _start(client, task)
            await handle.result()
            return calls, await handle.query(RobotWorkflow.get_step_states)

    calls, steps = asyncio.run(scenario())

    # One activity per step, in definition order — the whole point of the
    # sequential loop over task.definition.steps.
    assert [name for name, _ in calls] == [
        "execute_move",
        "execute_stand",
        "execute_lie_down",
    ]
    # The MOVE params must survive the pydantic converter round trip intact
    # (still degrees here; the degrees→radians conversion is the activity's
    # job, pinned in test_activities.py).
    move_params = calls[0][1]
    assert (move_params.x, move_params.y, move_params.theta) == (1.5, -2.5, 90.0)
    assert [step.status for step in steps] == [StepStatus.COMPLETED] * 3


def test_a_runs_map_is_handed_to_every_move():
    """The run's TaskMap is the second MOVE argument, and only MOVE's: the
    posture activities take no argument, and execute_move checks the map
    against the loaded one before driving (pinned in test_activities.py)."""

    async def scenario():
        calls: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams, expected_map=None) -> ActivityResult:
            calls.append(("execute_move", expected_map))
            return ActivityResult(success=True, state="succeeded")

        activities = [execute_move] + _instant_activities(calls)[1:]
        task = _task("task-map", StepType.MOVE, StepType.STANDUP, StepType.MOVE)

        async with _worker(activities) as client:
            await client.operator_service.add_search_attributes(
                AddSearchAttributesRequest(
                    namespace=client.namespace,
                    search_attributes={
                        TASK_MAP_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD
                    },
                )
            )
            handle = await client.start_workflow(
                RobotWorkflow.run,
                task,
                id=task.id,
                task_queue=TASK_QUEUE,
                search_attributes=TypedSearchAttributes(
                    [SearchAttributePair(TASK_MAP_KEY, "lab")]
                ),
            )
            await handle.result()
        return calls

    assert asyncio.run(scenario()) == [
        ("execute_move", "lab"),
        ("execute_stand", None),
        ("execute_move", "lab"),
    ]


def test_a_runs_map_is_handed_to_a_charge():
    """CHARGE coordinates are a pose on the map too, so they get MOVE's
    second argument and the same check."""

    async def scenario():
        calls: list = []

        @activity.defn(name="execute_charge")
        async def execute_charge(params: MoveParams, expected_map=None) -> ActivityResult:
            calls.append(("execute_charge", params.x, expected_map))
            return ActivityResult(success=True, state="succeeded")

        task = _task("task-charge-map", StepType.CHARGE)

        async with _worker([execute_charge]) as client:
            await client.operator_service.add_search_attributes(
                AddSearchAttributesRequest(
                    namespace=client.namespace,
                    search_attributes={
                        TASK_MAP_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD
                    },
                )
            )
            handle = await client.start_workflow(
                RobotWorkflow.run,
                task,
                id=task.id,
                task_queue=TASK_QUEUE,
                search_attributes=TypedSearchAttributes(
                    [SearchAttributePair(TASK_MAP_KEY, "lab")]
                ),
            )
            await handle.result()
        return calls

    assert asyncio.run(scenario()) == [("execute_charge", 4.0, "lab")]


def test_a_run_without_a_map_hands_move_one_argument():
    # A schedule older than TaskMap: the one-argument call an older worker
    # made, which _instant_activities' one-parameter stand-in would reject
    # if a None were appended.
    async def scenario():
        calls: list = []
        async with _worker(_instant_activities(calls)) as client:
            handle = await _start(client, _task("task-nomap", StepType.MOVE))
            await handle.result()
        return calls

    assert [name for name, _ in asyncio.run(scenario())] == ["execute_move"]


def test_query_reports_per_step_state_mid_run():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        calls: list = []
        # Reuse the instant stand/lie_down mocks; only MOVE is gated.
        activities = [_gated_move(started, release)] + _instant_activities(calls)[1:]
        task = _task("task-midrun", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            mid = await handle.query(RobotWorkflow.get_step_states)
            release.set()
            await handle.result()
            done = await handle.query(RobotWorkflow.get_step_states)
            return mid, done

    mid, done = asyncio.run(scenario())

    # The workflow flips a step to IN_PROGRESS before scheduling its
    # activity, so once the activity has started the query must say so —
    # this is what the operator console polls while a task runs.
    assert [step.status for step in mid] == [
        StepStatus.IN_PROGRESS,
        StepStatus.PENDING,
    ]
    assert [step.status for step in done] == [StepStatus.COMPLETED] * 2


def test_activity_failure_fails_the_step_and_halts_the_run():
    async def scenario():
        calls: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            # non_retryable so the run fails on the first attempt; the
            # retryable shape has its own test below.
            raise ApplicationError("nav server says no", non_retryable=True)

        activities = [execute_move] + _instant_activities(calls)[1:]
        task = _task("task-failing", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return calls, exc_info.value, await handle.query(RobotWorkflow.get_step_states)

    calls, err, steps = asyncio.run(scenario())

    # The workflow re-raises the ActivityError itself, so the failure chain
    # keeps the activity's own error as the innermost cause.
    assert isinstance(err.cause, ActivityError)
    assert isinstance(err.cause.cause, ApplicationError)
    assert steps[0].status is StepStatus.FAILED
    # error_msg carries the *cause* (the activity's message), not the
    # ActivityError wrapper's generic text.
    assert "nav server says no" in steps[0].error_msg
    # Failure halts the run: the next step is untouched and its activity
    # never executed.
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


def test_unsuccessful_result_fails_the_workflow():
    async def scenario():
        @activity.defn(name="execute_stand")
        async def execute_stand() -> ActivityResult:
            # The activity completes normally but reports failure — the
            # other failure shape, distinct from raising. Today's activities
            # never actually return success=False (they raise instead), but
            # the workflow guards it, so pin what the guard does.
            return ActivityResult(success=False)

        task = _task("task-unsuccessful", StepType.STANDUP)

        async with _worker([execute_stand]) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return exc_info.value, await handle.query(RobotWorkflow.get_step_states)

    err, steps = asyncio.run(scenario())

    assert isinstance(err.cause, ApplicationError)
    assert err.cause.non_retryable is True
    assert steps[0].status is StepStatus.FAILED
    assert steps[0].error_msg == "activity failed"


def test_a_retryable_failure_gets_three_attempts_of_the_same_step():
    async def scenario():
        attempts: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            attempts.append(activity.info().attempt)
            raise ApplicationError("move ended in aborted", non_retryable=False)

        task = _task("task-retries", StepType.MOVE)

        async with _worker([execute_move]) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError):
                # Time skipping fast-forwards the retry backoff (5 s initial
                # interval) — this await is where the skipping happens.
                await handle.result()
            return attempts, await handle.query(RobotWorkflow.get_step_states)

    attempts, steps = asyncio.run(scenario())

    # maximum_attempts=3 in the workflow's RetryPolicy: exactly three tries
    # of the one step (Temporal-side retry, invisible to the step list), then
    # a single FAILED step — not an eternal IN_PROGRESS, which is the bug the
    # policy exists to prevent.
    assert attempts == [1, 2, 3]
    assert steps[0].status is StepStatus.FAILED


def test_cancellation_mid_move_ends_the_run_and_the_step_canceled():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        calls: list = []
        # release is never set: the MOVE parks until the cancel arrives.
        activities = [_gated_move(started, release)] + _instant_activities(calls)[1:]
        task = _task("task-canceled", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return calls, exc_info.value, await handle.query(RobotWorkflow.get_step_states)

    calls, err, steps = asyncio.run(scenario())

    # The run itself closes as canceled: WorkflowFailureError with a bare
    # CancelledError cause is the canceled-workflow shape, not the failed one.
    assert isinstance(err.cause, CancelledError)
    # So does the in-flight step. _run_move waits on a condition rather than
    # awaiting the activity, so the run's cancel arrives as
    # asyncio.CancelledError (not swallowed into the activity's
    # ActivityError) and the step reads CANCELED / "Task canceled" — not the
    # FAILED / "Cancelled" it read before the hold, which the console could
    # not tell apart from a real failure.
    assert steps[0].status is StepStatus.CANCELED
    assert steps[0].error_msg == "Task canceled"
    # The step that never started stays PENDING and its activity never ran —
    # "interrupted here" and "never got there" must stay distinguishable.
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


def test_cancellation_mid_posture_ends_the_step_canceled_too():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        # release is never set: the STANDUP parks until the cancel arrives.
        task = _task("task-canceled-stand", StepType.STANDUP)

        async with _worker([_gated_stand(started, release)]) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return exc_info.value, await handle.query(RobotWorkflow.get_step_states)

    err, steps = asyncio.run(scenario())

    assert isinstance(err.cause, CancelledError)
    # execute_activity hands the run's cancel back as the activity's
    # ActivityError(cause=CancelledError); the workflow folds that shape to
    # the same CANCELED a MOVE ends in, rather than FAILED.
    assert steps[0].status is StepStatus.CANCELED
    assert steps[0].error_msg == "Task canceled"


# --- The hold -----------------------------------------------------------------


def _recording_move(
    started: asyncio.Event,
    release: asyncio.Event,
    dispatches: list,
    cancels: list,
    name: str = "execute_move",
):
    """``_gated_move`` that also records each dispatch and each cancel it receives.

    ``dispatches`` collects ``activity.info().attempt`` so a test can tell a
    *re-sent* MOVE (two activities, each attempt 1) from a *retried* one.
    ``name`` stands it in for a CHARGE, which the hold treats the same.
    """

    @activity.defn(name=name)
    async def execute_move(params: MoveParams) -> ActivityResult:
        dispatches.append(activity.info().attempt)
        await _report_started()
        started.set()
        try:
            while not release.is_set():
                activity.heartbeat()
                try:
                    await asyncio.wait_for(release.wait(), timeout=0.2)
                except asyncio.TimeoutError:
                    pass
        except asyncio.CancelledError:
            cancels.append(True)
            raise
        return ActivityResult(success=True, state="succeeded")

    return execute_move


def _gated_stand(started: asyncio.Event, release: asyncio.Event):
    """A STANDUP activity that parks until released — the uninterruptible kind."""

    @activity.defn(name="execute_stand")
    async def execute_stand() -> ActivityResult:
        started.set()
        while not release.is_set():
            activity.heartbeat()
            try:
                await asyncio.wait_for(release.wait(), timeout=0.2)
            except asyncio.TimeoutError:
                pass
        return ActivityResult(success=True, state="succeeded")

    return execute_stand


async def _statuses_until(handle, predicate, timeout: float = 10.0):
    """Poll the step-state query until ``predicate(statuses)`` holds."""
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        steps = await handle.query(RobotWorkflow.get_step_states)
        statuses = [step.status for step in steps]
        if predicate(statuses):
            return statuses
        if asyncio.get_running_loop().time() > deadline:
            raise AssertionError(f"step states never reached the expected shape: {statuses}")
        await asyncio.sleep(0.05)


def test_pause_interrupts_a_move_and_resume_re_sends_it():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        dispatches: list = []
        cancels: list = []
        calls: list = []
        activities = [_recording_move(started, release, dispatches, cancels)]
        activities += _instant_activities(calls)[1:]
        task = _task("task-pause-move", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            started.clear()

            await handle.signal(RobotWorkflow.pause)
            held = await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)
            # The hold is a hold: nothing is dispatched while it lasts.
            await asyncio.sleep(0.5)
            dispatches_while_held = list(dispatches)

            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(started.wait(), timeout=10)
            resumed = await handle.query(RobotWorkflow.get_step_states)
            release.set()
            await handle.result()
            return dispatches, dispatches_while_held, cancels, held, resumed, calls

    dispatches, dispatches_while_held, cancels, held, resumed, calls = asyncio.run(scenario())

    # The in-flight MOVE received the cancel — in production that is the
    # ``except CancelledError`` that cancels the nav2 goal — and the step
    # read PAUSED while the run waited; the next step was never touched.
    assert cancels == [True]
    assert held == [StepStatus.PAUSED, StepStatus.PENDING]
    assert dispatches_while_held == [1]
    # Resume re-sent the MOVE as a *new* activity (attempt 1 again, not a
    # retry's attempt 2), and the step went back to IN_PROGRESS for it.
    assert dispatches == [1, 1]
    assert [step.status for step in resumed] == [StepStatus.IN_PROGRESS, StepStatus.PENDING]
    # Then the run went on to the end.
    assert calls == [("execute_stand", None)]


def test_pause_interrupts_a_charge_and_resume_re_sends_it():
    # A charge is a NavigateToGoal like a MOVE: the pause cancels it and
    # resume starts the tree over.
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        dispatches: list = []
        cancels: list = []
        activities = [
            _recording_move(started, release, dispatches, cancels, "execute_charge")
        ]
        task = _task("task-pause-charge", StepType.CHARGE)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            started.clear()

            await handle.signal(RobotWorkflow.pause)
            held = await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)

            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(started.wait(), timeout=10)
            release.set()
            await handle.result()
            return dispatches, cancels, held

    dispatches, cancels, held = asyncio.run(scenario())

    assert cancels == [True]
    assert held == [StepStatus.PAUSED]
    assert dispatches == [1, 1]


def test_pause_during_an_uninterruptible_step_holds_before_the_next():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        calls: list = []
        # STANDUP is gated; MOVE is the instant mock, which must NOT run while
        # the run is held.
        activities = [_gated_stand(started, release)] + [_instant_activities(calls)[0]]
        task = _task("task-pause-stand", StepType.STANDUP, StepType.MOVE)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)

            await handle.signal(RobotWorkflow.pause)
            # The posture finishes regardless of the pause...
            await asyncio.sleep(0.3)
            during = await handle.query(RobotWorkflow.get_step_states)
            release.set()
            # ...and the run holds at the boundary, with the next step PAUSED.
            held = await _statuses_until(handle, lambda s: s[1] is StepStatus.PAUSED)
            await asyncio.sleep(0.3)
            calls_while_held = list(calls)

            await handle.signal(RobotWorkflow.resume)
            await handle.result()
            return during, held, calls_while_held, calls

    during, held, calls_while_held, calls = asyncio.run(scenario())

    # A pause does not interrupt a posture step: it stayed IN_PROGRESS.
    assert [step.status for step in during] == [StepStatus.IN_PROGRESS, StepStatus.PENDING]
    # The hold lands before the next step starts — PAUSED, not PENDING, so
    # the console can see where the run is holding — and that step's
    # activity is not dispatched until the resume.
    assert held == [StepStatus.COMPLETED, StepStatus.PAUSED]
    assert calls_while_held == []
    assert [name for name, _ in calls] == ["execute_move"]


def test_cancel_while_held_ends_the_run_canceled_and_the_held_step_canceled():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        dispatches: list = []
        cancels: list = []
        calls: list = []
        activities = [_recording_move(started, release, dispatches, cancels)]
        activities += _instant_activities(calls)[1:]
        task = _task("task-cancel-held", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.signal(RobotWorkflow.pause)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)

            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return exc_info.value, await handle.query(RobotWorkflow.get_step_states), calls

    err, steps, calls = asyncio.run(scenario())

    assert isinstance(err.cause, CancelledError)
    # Nothing was in flight, so the cancel lands in the hold's
    # wait_condition and the held step is CANCELED / "Task canceled" — the
    # same as a cancel that lands mid-MOVE (pinned above).
    assert steps[0].status is StepStatus.CANCELED
    assert steps[0].error_msg == "Task canceled"
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


def test_a_pause_with_nothing_left_to_hold_lets_the_run_complete():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        task = _task("task-pause-last", StepType.STANDUP)

        async with _worker([_gated_stand(started, release)]) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            # A resume on a running task is a no-op; a pause during the last
            # step has no boundary left to hold at.
            await handle.signal(RobotWorkflow.resume)
            await handle.signal(RobotWorkflow.pause)
            release.set()
            await asyncio.wait_for(handle.result(), timeout=10)
            return await handle.query(RobotWorkflow.get_step_states)

    steps = asyncio.run(scenario())

    assert [step.status for step in steps] == [StepStatus.COMPLETED]


def test_a_pause_and_a_cancel_together_end_the_run_canceled_not_held():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        dispatches: list = []
        cancels: list = []
        calls: list = []
        activities = [_recording_move(started, release, dispatches, cancels)]
        activities += _instant_activities(calls)[1:]
        task = _task("task-pause-and-cancel", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            # Back to back, so both are very likely delivered in one
            # workflow task: the pause's interruption and the run's cancel
            # race for the same in-flight MOVE.
            await handle.signal(RobotWorkflow.pause)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await asyncio.wait_for(handle.result(), timeout=10)
            return exc_info.value, await handle.query(RobotWorkflow.get_step_states), calls

    err, steps, calls = asyncio.run(scenario())

    # The run closed — it did not park PAUSED waiting for a resume that the
    # operator, having cancelled, will never send.
    assert isinstance(err.cause, CancelledError)
    assert steps[0].status is StepStatus.CANCELED
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


# --- WAIT ---------------------------------------------------------------------


async def _timer_durations(handle) -> list:
    """The ``start_to_fire_timeout`` of every timer the run started, in seconds."""
    history = await handle.fetch_history()
    return [
        event.timer_started_event_attributes.start_to_fire_timeout.ToTimedelta().total_seconds()
        for event in history.events
        if event.HasField("timer_started_event_attributes")
    ]


def test_a_wait_is_a_timer_not_an_activity_and_the_run_goes_on():
    async def scenario():
        calls: list = []
        task = _task("task-wait", StepType.STANDUP, StepType.WAIT, StepType.LIEDOWN)

        async with _worker(_instant_activities(calls)) as client:
            handle = await _start(client, task)
            # Time skipping: ten minutes of timer, well under ten real seconds.
            await asyncio.wait_for(handle.result(), timeout=10)
            steps = await handle.query(RobotWorkflow.get_step_states)
            return calls, steps, await _timer_durations(handle)

    calls, steps, timers = asyncio.run(scenario())

    # No activity for the WAIT, and the steps either side of it ran in order.
    assert [name for name, _ in calls] == ["execute_stand", "execute_lie_down"]
    assert [step.status for step in steps] == [StepStatus.COMPLETED] * 3
    assert timers == [WAIT_SECONDS]


def test_pause_freezes_a_wait_and_resume_waits_out_only_the_rest():
    async def scenario():
        calls: list = []
        task = _task("task-pause-wait", StepType.WAIT, StepType.STANDUP)

        async with _worker(_instant_activities(calls)) as client:
            handle = await _start(client, task)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.IN_PROGRESS)
            # Real time passes here (nothing is awaiting the result, so the
            # server does not skip); the remainder must account for it.
            await asyncio.sleep(0.5)

            await handle.signal(RobotWorkflow.pause)
            held = await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)
            await asyncio.sleep(0.3)
            calls_while_held = list(calls)

            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(handle.result(), timeout=10)
            return held, calls_while_held, calls, await _timer_durations(handle)

    held, calls_while_held, calls, timers = asyncio.run(scenario())

    assert held == [StepStatus.PAUSED, StepStatus.PENDING]
    assert calls_while_held == []
    assert [name for name, _ in calls] == ["execute_stand"]
    # Two timers: the full wait, then — after the hold — only what was left
    # of it, not a fresh ten minutes.
    assert len(timers) == 2
    assert timers[0] == WAIT_SECONDS
    assert 0 < timers[1] < WAIT_SECONDS


def test_cancel_during_a_wait_ends_the_run_canceled_and_the_step_canceled():
    async def scenario():
        calls: list = []
        task = _task("task-cancel-wait", StepType.WAIT, StepType.STANDUP)

        async with _worker(_instant_activities(calls)) as client:
            handle = await _start(client, task)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.IN_PROGRESS)

            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return exc_info.value, await handle.query(RobotWorkflow.get_step_states), calls

    err, steps, calls = asyncio.run(scenario())

    assert isinstance(err.cause, CancelledError)
    # A timer, not an activity: the cancel lands in its wait_condition and
    # the step is CANCELED / "Task canceled", like every other cancel.
    assert steps[0].status is StepStatus.CANCELED
    assert steps[0].error_msg == "Task canceled"
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


# --- Groups ----------------------------------------------------------------


def _grouped_task(task_id: str, *steps: tuple) -> WorkflowTask:
    """``steps`` as ``(type, group)`` pairs; params as _task gives them."""
    params = {
        StepType.MOVE: MoveParams(x=1.5, y=-2.5, theta=90.0),
        StepType.WAIT: WaitParams(seconds=WAIT_SECONDS),
        StepType.SPEAK: SpeakParams(text="hello"),
    }
    return WorkflowTask(
        id=task_id,
        definition=WorkflowTaskDefinition(
            steps=[
                Step(id=f"step-{i}", type=step_type, params=params.get(step_type), group=group)
                for i, (step_type, group) in enumerate(steps)
            ]
        ),
    )


def _gated_speak(started: asyncio.Event, release: asyncio.Event, calls: list):
    """A SPEAK that parks until released. It does not heartbeat, so a cancel
    never reaches it -- the uninterruptible kind, for the tests where nothing
    should cancel it; SPEAK's heartbeat window outlasts every hold here.
    ``_cancellable_speak`` is the one that heartbeats."""

    @activity.defn(name="execute_speak")
    async def execute_speak(params: SpeakParams) -> ActivityResult:
        calls.append(("execute_speak", activity.info().task_queue))
        await _report_started()
        started.set()
        await release.wait()
        return ActivityResult(success=True, state="succeeded")

    return execute_speak


def _cancellable_speak(started: asyncio.Event, cancels: list):
    """A SPEAK that plays until cancelled, heartbeating as execute_speak does
    -- which is the only way a cancel reaches a running activity."""

    @activity.defn(name="execute_speak")
    async def execute_speak(params: SpeakParams) -> ActivityResult:
        await _report_started()
        started.set()
        try:
            while True:
                activity.heartbeat("playing")
                await asyncio.sleep(0.2)
        except asyncio.CancelledError:
            cancels.append(True)
            raise

    return execute_speak


def test_a_lone_speak_runs_on_the_io_queue():
    async def scenario():
        calls: list = []
        release = asyncio.Event()
        release.set()
        speak = _gated_speak(asyncio.Event(), release, calls)
        task = _grouped_task("task-speak-io", (StepType.SPEAK, None))

        # SPEAK is registered on the IO worker only: if the workflow sent it
        # to the body queue the run would never finish.
        async with _worker(_instant_activities([]), io_activities=[speak]) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(handle.result(), timeout=10)
        return calls

    assert asyncio.run(scenario()) == [("execute_speak", IO_TASK_QUEUE)]


def test_a_repeated_speak_gets_five_minutes_per_utterance():
    # One attempt says the text `repeat` times, so a timeout sized for one
    # utterance would cut a long repeat off partway.
    async def scenario():
        seen: list = []

        @activity.defn(name="execute_speak")
        async def execute_speak(params: SpeakParams) -> ActivityResult:
            seen.append(activity.info().start_to_close_timeout)
            return ActivityResult(success=True, state="succeeded")

        task = WorkflowTask(
            id="task-speak-repeat",
            definition=WorkflowTaskDefinition(
                steps=[
                    Step(id="s", type=StepType.SPEAK,
                         params=SpeakParams(text="hello", repeat=3)),
                ]
            ),
        )
        async with _worker(_instant_activities([]), io_activities=[execute_speak]) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(handle.result(), timeout=10)
        return seen

    assert asyncio.run(scenario()) == [timedelta(minutes=15)]


def test_a_group_runs_its_members_side_by_side_before_the_next_step():
    async def scenario():
        move_started, move_release = asyncio.Event(), asyncio.Event()
        speak_started, speak_release = asyncio.Event(), asyncio.Event()
        calls: list = []
        activities = [_gated_move(move_started, move_release)]
        activities += _instant_activities(calls)[1:]
        task = _grouped_task(
            "task-group",
            (StepType.MOVE, "a"),
            (StepType.SPEAK, "a"),
            (StepType.STANDUP, None),
        )

        async with _worker(
            activities, io_activities=[_gated_speak(speak_started, speak_release, calls)]
        ) as client:
            handle = await _start(client, task)
            # Neither is released, so both running at once is the group.
            await asyncio.wait_for(move_started.wait(), timeout=10)
            await asyncio.wait_for(speak_started.wait(), timeout=10)
            together = await _statuses_until(
                handle, lambda s: s[:2] == [StepStatus.IN_PROGRESS] * 2
            )

            # The SPEAK finishing is not enough: the next step waits for the
            # whole group.
            speak_release.set()
            spoke = await _statuses_until(handle, lambda s: s[1] is StepStatus.COMPLETED)
            await asyncio.sleep(0.3)
            stood_early = ("execute_stand", None) in calls

            move_release.set()
            await handle.result()
            done = await handle.query(RobotWorkflow.get_step_states)
            return together, spoke, stood_early, done, calls

    together, spoke, stood_early, done, calls = asyncio.run(scenario())

    assert together == [StepStatus.IN_PROGRESS, StepStatus.IN_PROGRESS, StepStatus.PENDING]
    assert spoke == [StepStatus.IN_PROGRESS, StepStatus.COMPLETED, StepStatus.PENDING]
    assert not stood_early
    assert [step.status for step in done] == [StepStatus.COMPLETED] * 3
    assert calls[-1] == ("execute_stand", None)


def test_a_failed_member_cancels_the_rest_and_fails_the_run():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        dispatches: list = []
        cancels: list = []
        calls: list = []
        activities = [_recording_move(started, release, dispatches, cancels)]
        activities += _instant_activities(calls)[1:]

        @activity.defn(name="execute_speak")
        async def execute_speak(params: SpeakParams) -> ActivityResult:
            await started.wait()
            raise ApplicationError("Speak failed: no speaker", non_retryable=True)

        task = _grouped_task(
            "task-group-fail",
            (StepType.MOVE, "a"),
            (StepType.SPEAK, "a"),
            (StepType.STANDUP, None),
        )

        async with _worker(activities, io_activities=[execute_speak]) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError) as exc_info:
                await asyncio.wait_for(handle.result(), timeout=15)
            steps = await handle.query(RobotWorkflow.get_step_states)
            return exc_info.value, steps, cancels, calls

    err, steps, cancels, calls = asyncio.run(scenario())

    # A failure, not a cancel: the run fails with the SPEAK's cause.
    assert isinstance(err.cause, ActivityError)
    assert steps[1].status is StepStatus.FAILED
    assert "no speaker" in steps[1].error_msg
    # The MOVE was stopped the way a cancel stops it, and says why.
    assert cancels == [True]
    assert steps[0].status is StepStatus.CANCELED
    assert steps[0].error_msg == "Canceled: step step-1 failed"
    assert steps[2].status is StepStatus.PENDING
    assert calls == []


def test_cancel_mid_group_cancels_every_member():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        dispatches: list = []
        cancels: list = []
        activities = [_recording_move(started, release, dispatches, cancels)]
        activities += _instant_activities([])[1:]
        task = _grouped_task(
            "task-group-cancel",
            (StepType.MOVE, "a"),
            (StepType.WAIT, "a"),
            (StepType.STANDUP, None),
        )

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await _statuses_until(handle, lambda s: s[1] is StepStatus.IN_PROGRESS)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return exc_info.value, await handle.query(RobotWorkflow.get_step_states), cancels

    err, steps, cancels = asyncio.run(scenario())

    assert isinstance(err.cause, CancelledError)
    assert cancels == [True]
    assert [(step.status, step.error_msg) for step in steps[:2]] == [
        (StepStatus.CANCELED, "Task canceled"),
        (StepStatus.CANCELED, "Task canceled"),
    ]
    assert steps[2].status is StepStatus.PENDING


def test_a_pause_interrupts_a_groups_move_while_its_speak_plays_on():
    async def scenario():
        move_started, move_release = asyncio.Event(), asyncio.Event()
        speak_started, speak_release = asyncio.Event(), asyncio.Event()
        dispatches: list = []
        cancels: list = []
        calls: list = []
        activities = [_recording_move(move_started, move_release, dispatches, cancels)]
        activities += _instant_activities(calls)[1:]
        task = _grouped_task(
            "task-group-pause", (StepType.MOVE, "a"), (StepType.SPEAK, "a")
        )

        async with _worker(
            activities, io_activities=[_gated_speak(speak_started, speak_release, calls)]
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(move_started.wait(), timeout=10)
            await asyncio.wait_for(speak_started.wait(), timeout=10)
            move_started.clear()

            await handle.signal(RobotWorkflow.pause)
            held = await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)
            speak_release.set()
            spoke = await _statuses_until(handle, lambda s: s[1] is StepStatus.COMPLETED)

            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(move_started.wait(), timeout=10)
            move_release.set()
            await handle.result()
            done = await handle.query(RobotWorkflow.get_step_states)
            return held, spoke, dispatches, cancels, done

    held, spoke, dispatches, cancels, done = asyncio.run(scenario())

    assert held == [StepStatus.PAUSED, StepStatus.IN_PROGRESS]
    assert spoke == [StepStatus.PAUSED, StepStatus.COMPLETED]
    assert cancels == [True]
    assert dispatches == [1, 1]
    assert [step.status for step in done] == [StepStatus.COMPLETED] * 2


def test_a_failed_member_stops_a_speak_mid_utterance():
    async def scenario():
        speak_started = asyncio.Event()
        speak_cancels: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            # Accepted, then aborted once the SPEAK behind it is playing.
            await _report_started()
            await speak_started.wait()
            raise ApplicationError("Move rejected: blocked", non_retryable=True)

        task = _grouped_task(
            "task-group-fail-speak", (StepType.MOVE, "a"), (StepType.SPEAK, "a")
        )

        async with _worker(
            [execute_move] + _instant_activities([])[1:],
            io_activities=[_cancellable_speak(speak_started, speak_cancels)],
        ) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError):
                await asyncio.wait_for(handle.result(), timeout=15)
            return await handle.query(RobotWorkflow.get_step_states), speak_cancels

    steps, speak_cancels = asyncio.run(scenario())

    assert steps[0].status is StepStatus.FAILED
    # The SPEAK was cancelled while it played, not left to finish its sentence.
    assert speak_cancels == [True]
    assert steps[1].status is StepStatus.CANCELED
    assert steps[1].error_msg == "Canceled: step step-0 failed"


def test_a_task_cancel_stops_a_speak_mid_utterance():
    async def scenario():
        move_started, move_release = asyncio.Event(), asyncio.Event()
        speak_started = asyncio.Event()
        dispatches: list = []
        move_cancels: list = []
        speak_cancels: list = []
        activities = [_recording_move(move_started, move_release, dispatches, move_cancels)]
        activities += _instant_activities([])[1:]
        task = _grouped_task(
            "task-group-cancel-speak", (StepType.MOVE, "a"), (StepType.SPEAK, "a")
        )

        async with _worker(
            activities, io_activities=[_cancellable_speak(speak_started, speak_cancels)]
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(move_started.wait(), timeout=10)
            await asyncio.wait_for(speak_started.wait(), timeout=10)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await asyncio.wait_for(handle.result(), timeout=15)
            steps = await handle.query(RobotWorkflow.get_step_states)
            return exc_info.value, steps, move_cancels, speak_cancels

    err, steps, move_cancels, speak_cancels = asyncio.run(scenario())

    assert isinstance(err.cause, CancelledError)
    assert move_cancels == [True]
    assert speak_cancels == [True]
    assert [(step.status, step.error_msg) for step in steps] == [
        (StepStatus.CANCELED, "Task canceled"),
        (StepStatus.CANCELED, "Task canceled"),
    ]


# --- A group's start order -------------------------------------------------


def test_a_groups_speaks_are_queued_in_list_order():
    # Synthesis time decides when a SPEAK is queued, and the speech service
    # plays in that order. Here the first takes longest: started together,
    # they would be queued last-first.
    async def scenario():
        events: list = []
        synthesis = {"one": 0.6, "two": 0.3, "three": 0.0}

        @activity.defn(name="execute_speak")
        async def execute_speak(params: SpeakParams) -> ActivityResult:
            events.append(("post", params.text))
            await asyncio.sleep(synthesis[params.text])
            events.append(("queued", params.text))
            await _report_started()
            return ActivityResult(success=True, state="succeeded")

        task = WorkflowTask(
            id="task-group-speak-order",
            definition=WorkflowTaskDefinition(
                steps=[
                    Step(id=f"s-{text}", type=StepType.SPEAK,
                         params=SpeakParams(text=text), group="a")
                    for text in synthesis
                ]
            ),
        )
        async with _worker(_instant_activities([]), io_activities=[execute_speak]) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(handle.result(), timeout=15)
            return events, await handle.query(RobotWorkflow.get_step_states)

    events, steps = asyncio.run(scenario())

    assert events == [
        ("post", "one"), ("queued", "one"),
        ("post", "two"), ("queued", "two"),
        ("post", "three"), ("queued", "three"),
    ]
    assert [step.status for step in steps] == [StepStatus.COMPLETED] * 3


def test_a_groups_speak_waits_for_the_standup_listed_ahead_of_it():
    # The posture steps never report: their activity ending is their start.
    async def scenario():
        events: list = []

        @activity.defn(name="execute_stand")
        async def execute_stand() -> ActivityResult:
            await asyncio.sleep(0.3)
            events.append("stand")
            return ActivityResult(success=True, state="succeeded")

        @activity.defn(name="execute_speak")
        async def execute_speak(params: SpeakParams) -> ActivityResult:
            events.append("speak")
            return ActivityResult(success=True, state="succeeded")

        activities = _instant_activities([])
        activities[1] = execute_stand
        task = _grouped_task(
            "task-group-stand-speak", (StepType.STANDUP, "a"), (StepType.SPEAK, "a")
        )
        async with _worker(activities, io_activities=[execute_speak]) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(handle.result(), timeout=15)
        return events

    assert asyncio.run(scenario()) == ["stand", "speak"]


def test_a_member_that_fails_before_it_starts_never_starts_the_next():
    async def scenario():
        calls: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            raise ApplicationError("Move rejected: no path", non_retryable=True)

        task = _grouped_task(
            "task-group-fail-first", (StepType.MOVE, "a"), (StepType.SPEAK, "a")
        )
        async with _worker(
            [execute_move] + _instant_activities([])[1:],
            io_activities=[_gated_speak(asyncio.Event(), asyncio.Event(), calls)],
        ) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError):
                await asyncio.wait_for(handle.result(), timeout=15)
            return await handle.query(RobotWorkflow.get_step_states), calls

    steps, calls = asyncio.run(scenario())

    assert steps[0].status is StepStatus.FAILED
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


def test_a_cancel_before_the_first_member_starts_leaves_the_rest_pending():
    async def scenario():
        started = asyncio.Event()
        cancels: list = []
        calls: list = []

        # Sent but never accepted: no report, so the SPEAK is never dispatched.
        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            started.set()
            try:
                while True:
                    activity.heartbeat("sending")
                    await asyncio.sleep(0.2)
            except asyncio.CancelledError:
                cancels.append(True)
                raise

        task = _grouped_task(
            "task-group-cancel-first", (StepType.MOVE, "a"), (StepType.SPEAK, "a")
        )
        async with _worker(
            [execute_move] + _instant_activities([])[1:],
            io_activities=[_gated_speak(asyncio.Event(), asyncio.Event(), calls)],
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await asyncio.sleep(0.3)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await asyncio.wait_for(handle.result(), timeout=15)
            steps = await handle.query(RobotWorkflow.get_step_states)
            return exc_info.value, steps, cancels, calls

    err, steps, cancels, calls = asyncio.run(scenario())

    assert isinstance(err.cause, CancelledError)
    assert cancels == [True]
    assert (steps[0].status, steps[0].error_msg) == (StepStatus.CANCELED, "Task canceled")
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


# --- Timing and the record --------------------------------------------------


def _seconds(start, end) -> float:
    return (end - start).total_seconds()


def test_a_run_records_every_step_with_its_times():
    async def scenario():
        records: list = []
        task = _task("task-record", StepType.STANDUP, StepType.WAIT, StepType.LIEDOWN)

        async with _worker(_instant_activities([]), record_task=_recorder(records)) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(handle.result(), timeout=10)
            return records, await handle.query(RobotWorkflow.get_step_states), handle

    records, steps, handle = asyncio.run(scenario())

    assert len(records) == 1
    record = records[0]
    assert record.task_id == "task-record"
    assert record.run_id == handle.result_run_id
    assert record.status == "COMPLETED"
    # Unlabelled dispatch: no kind, name, map or schedule.
    assert (record.kind, record.name, record.map_name, record.schedule_id) == (
        None,
        None,
        None,
        None,
    )
    assert record.started_at <= record.closed_at
    # The record carries the same steps the query does, timing included.
    assert [step.model_dump() for step in record.steps] == [
        step.model_dump() for step in steps
    ]
    for step in steps:
        assert step.started_at is not None and step.ended_at is not None
        assert step.started_at <= step.ended_at
        assert step.paused_seconds == 0.0
    # The WAIT's wall time is its timer (skipped, but on the workflow's clock),
    # and the steps follow one another.
    assert _seconds(steps[1].started_at, steps[1].ended_at) == pytest.approx(
        WAIT_SECONDS, abs=1.0
    )
    assert steps[0].ended_at <= steps[1].started_at <= steps[1].ended_at <= steps[2].started_at


def test_a_records_kind_comes_off_the_runs_own_attribute():
    async def scenario():
        records: list = []
        task = _task("task-record-kind", StepType.STANDUP)

        async with _worker(_instant_activities([]), record_task=_recorder(records)) as client:
            await client.operator_service.add_search_attributes(
                AddSearchAttributesRequest(
                    namespace=client.namespace,
                    search_attributes={
                        TASK_KIND_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD
                    },
                )
            )
            handle = await client.start_workflow(
                RobotWorkflow.run,
                task,
                id=task.id,
                task_queue=TASK_QUEUE,
                search_attributes=TypedSearchAttributes(
                    [SearchAttributePair(TASK_KIND_KEY, TaskKind.STANDUP.value)]
                ),
            )
            await handle.result()
            return records

    records = asyncio.run(scenario())

    assert records[0].kind is TaskKind.STANDUP


def test_a_hold_counts_on_the_step_under_way_and_not_on_one_not_started():
    async def scenario():
        records: list = []
        calls: list = []
        task = _task("task-record-pause", StepType.WAIT, StepType.STANDUP)

        async with _worker(_instant_activities(calls), record_task=_recorder(records)) as client:
            handle = await _start(client, task)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.IN_PROGRESS)

            await handle.signal(RobotWorkflow.pause)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)
            # Real time, while nothing awaits the result: the hold's length.
            await asyncio.sleep(1.0)
            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(handle.result(), timeout=10)
            return records

    records = asyncio.run(scenario())

    wait, stand = records[0].steps
    assert wait.paused_seconds >= 0.9
    # What the step itself took is its wall time less the hold: the timer.
    active = _seconds(wait.started_at, wait.ended_at) - wait.paused_seconds
    assert active == pytest.approx(WAIT_SECONDS, abs=2.0)
    assert stand.paused_seconds == 0.0


def test_a_hold_before_a_step_starts_is_not_that_steps():
    async def scenario():
        records: list = []
        started, release = asyncio.Event(), asyncio.Event()
        activities = [_gated_stand(started, release)] + [_instant_activities([])[0]]
        task = _task("task-record-hold-before", StepType.STANDUP, StepType.MOVE)

        async with _worker(activities, record_task=_recorder(records)) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.signal(RobotWorkflow.pause)
            release.set()
            # The run holds before the MOVE, which has not started.
            await _statuses_until(handle, lambda s: s[1] is StepStatus.PAUSED)
            await asyncio.sleep(0.5)
            await handle.signal(RobotWorkflow.resume)
            await handle.result()
            return records

    records = asyncio.run(scenario())

    stand, move = records[0].steps
    assert move.paused_seconds == 0.0
    # The MOVE's clock starts at the resume, so the hold is in neither step.
    assert _seconds(stand.ended_at, move.started_at) >= 0.4


def test_a_failed_run_is_recorded_failed():
    async def scenario():
        records: list = []

        @activity.defn(name="execute_stand")
        async def execute_stand() -> ActivityResult:
            raise ApplicationError("motor fault", non_retryable=True)

        task = _task("task-record-failed", StepType.STANDUP, StepType.LIEDOWN)
        async with _worker([execute_stand], record_task=_recorder(records)) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError):
                await handle.result()
            return records

    records = asyncio.run(scenario())

    record = records[0]
    assert record.status == "FAILED"
    failed, never = record.steps
    assert failed.status is StepStatus.FAILED
    assert failed.started_at is not None and failed.ended_at is not None
    assert "motor fault" in failed.error_msg
    assert never.status is StepStatus.PENDING
    assert never.started_at is None and never.ended_at is None


@pytest.mark.parametrize("step_type", [StepType.MOVE, StepType.STANDUP])
def test_a_canceled_run_is_recorded_canceled(step_type):
    """Both shapes a cancel reaches the run in: asyncio.CancelledError from a
    MOVE, and the activity's ActivityError(cause=CancelledError) from a posture."""

    async def scenario():
        records: list = []
        started, release = asyncio.Event(), asyncio.Event()
        gated = (
            _gated_move(started, release)
            if step_type is StepType.MOVE
            else _gated_stand(started, release)
        )
        task = _task(f"task-record-canceled-{step_type.value}", step_type)

        async with _worker([gated], record_task=_recorder(records)) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return records, exc_info.value

    records, err = asyncio.run(scenario())

    # Recording does not change how the run ends.
    assert isinstance(err.cause, CancelledError)
    assert records[0].status == "CANCELED"
    assert records[0].steps[0].status is StepStatus.CANCELED
    assert records[0].steps[0].ended_at is not None


def test_a_failing_record_does_not_fail_the_run():
    async def scenario():
        attempts: list = []

        @activity.defn(name="record_task")
        async def record_task(record: TaskRecord) -> None:
            attempts.append(record)
            raise ApplicationError("database down", non_retryable=True)

        task = _task("task-record-broken", StepType.STANDUP)
        async with _worker(_instant_activities([]), record_task=record_task) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(handle.result(), timeout=10)
            return attempts, await handle.describe()

    attempts, description = asyncio.run(scenario())

    assert len(attempts) == 1
    assert description.status.name == "COMPLETED"


def test_a_recorded_run_replays_deterministically():
    """The timing reads workflow.now() and the record is behind a patch: a
    replay of the run's own history must issue the same commands."""

    async def scenario():
        task = _task("task-record-replay", StepType.WAIT, StepType.STANDUP)
        async with _worker(_instant_activities([])) as client:
            handle = await _start(client, task)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.IN_PROGRESS)
            await handle.signal(RobotWorkflow.pause)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)
            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(handle.result(), timeout=10)
            history = await handle.fetch_history()

        await Replayer(
            workflows=[RobotWorkflow], data_converter=pydantic_data_converter
        ).replay_workflow(history)

    asyncio.run(scenario())


# --- PATROL --------------------------------------------------------------------


def _patrol(dispatches: list, started: asyncio.Event, release: asyncio.Event,
            report=None, cancels: Optional[list] = None):
    """A PATROL stand-in: records each dispatch's (step_id, start), reports
    ``report`` waypoints passed (and, if given as a pair, a stale lower one
    after it), then parks heartbeating until released or cancelled."""

    @activity.defn(name="execute_patrol")
    async def execute_patrol(
        params: PatrolParams, expected_map=None, step_id: str = "", start: int = 0
    ) -> ActivityResult:
        dispatches.append((step_id, start, len(params.waypoints), expected_map))
        info = activity.info()
        handle = activity.client().get_workflow_handle(
            info.workflow_id, run_id=info.workflow_run_id
        )
        for passed in report or ():
            await handle.signal(PATROL_PROGRESS_SIGNAL, args=[step_id, passed])
        started.set()
        try:
            while not release.is_set():
                activity.heartbeat("executing", start)
                try:
                    await asyncio.wait_for(release.wait(), timeout=0.2)
                except asyncio.TimeoutError:
                    pass
        except asyncio.CancelledError:
            if cancels is not None:
                cancels.append(True)
            raise
        return ActivityResult(success=True, state="succeeded")

    return execute_patrol


def test_a_patrol_is_dispatched_with_its_step_id_and_nothing_skipped():
    async def scenario():
        dispatches: list = []
        started, release = asyncio.Event(), asyncio.Event()
        release.set()
        task = _task("task-patrol", StepType.PATROL)

        async with _worker([_patrol(dispatches, started, release)]) as client:
            handle = await _start(client, task)
            await handle.result()
            return dispatches, await handle.query(RobotWorkflow.get_step_states)

    dispatches, steps = asyncio.run(scenario())

    # Every waypoint (the step's params are never trimmed), no map stamped.
    assert dispatches == [("step-0", 0, 4, None)]
    assert steps[0].status is StepStatus.COMPLETED


def test_pause_interrupts_a_patrol_and_resume_sends_only_the_rest():
    async def scenario():
        dispatches: list = []
        cancels: list = []
        started, release = asyncio.Event(), asyncio.Event()
        # Two passed, then a late lower report: the count only goes forward.
        activity_fn = _patrol(dispatches, started, release, report=(2, 1), cancels=cancels)
        task = _task("task-pause-patrol", StepType.PATROL, StepType.STANDUP)
        calls: list = []

        async with _worker([activity_fn] + _instant_activities(calls)[1:]) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            started.clear()

            await handle.signal(RobotWorkflow.pause)
            held = await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)
            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(started.wait(), timeout=10)
            release.set()
            await handle.result()
            steps = await handle.query(RobotWorkflow.get_step_states)
            return dispatches, cancels, held, steps, calls

    dispatches, cancels, held, steps, calls = asyncio.run(scenario())

    assert cancels == [True]
    assert held == [StepStatus.PAUSED, StepStatus.PENDING]
    # Re-sent past the two waypoints behind it, not from the first.
    assert [(step_id, start) for step_id, start, _, _ in dispatches] == [
        ("step-0", 0), ("step-0", 2)
    ]
    # The step still carries every waypoint: the activity skips, the run does not trim.
    assert len(steps[0].params.waypoints) == 4
    assert calls == [("execute_stand", None)]


def test_a_retried_patrol_reads_its_count_from_the_heartbeat():
    """The retry is the same dispatch (same args), so what it starts past has
    to come through Temporal's heartbeat details, beaten just before failing."""

    async def scenario():
        seen: list = []

        @activity.defn(name="execute_patrol")
        async def execute_patrol(
            params: PatrolParams, expected_map=None, step_id: str = "", start: int = 0
        ) -> ActivityResult:
            seen.append((activity.info().attempt, list(activity.info().heartbeat_details)))
            if activity.info().attempt == 1:
                activity.heartbeat("executing", 2)
                raise ApplicationError(
                    "patrol ended in aborted: Planning failed: the way to the goal is closed"
                )
            return ActivityResult(success=True, state="succeeded")

        async with _worker([execute_patrol]) as client:
            handle = await _start(client, _task("task-retry-patrol", StepType.PATROL))
            await handle.result()
        return seen

    assert asyncio.run(scenario()) == [(1, []), (2, ["executing", 2])]


def test_a_patrol_that_aborts_three_times_fails_with_the_robots_reason():
    async def scenario():
        @activity.defn(name="execute_patrol")
        async def execute_patrol(
            params: PatrolParams, expected_map=None, step_id: str = "", start: int = 0
        ) -> ActivityResult:
            raise ApplicationError(
                "patrol ended in aborted: Planning failed: the way to the goal is closed"
            )

        async with _worker([execute_patrol]) as client:
            handle = await _start(client, _task("task-fail-patrol", StepType.PATROL))
            with pytest.raises(WorkflowFailureError):
                await handle.result()
            return await handle.query(RobotWorkflow.get_step_states)

    steps = asyncio.run(scenario())

    assert steps[0].status is StepStatus.FAILED
    assert steps[0].error_msg == (
        "patrol ended in aborted: Planning failed: the way to the goal is closed"
    )


def test_cancellation_mid_patrol_ends_the_run_and_the_step_canceled():
    async def scenario():
        dispatches: list = []
        cancels: list = []
        started, release = asyncio.Event(), asyncio.Event()
        activity_fn = _patrol(dispatches, started, release, cancels=cancels)

        async with _worker([activity_fn]) as client:
            handle = await _start(client, _task("task-cancel-patrol", StepType.PATROL))
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError):
                await handle.result()
            return cancels, await handle.query(RobotWorkflow.get_step_states)

    cancels, steps = asyncio.run(scenario())

    assert cancels == [True]
    assert steps[0].status is StepStatus.CANCELED
    assert steps[0].error_msg == "Task canceled"


def test_a_runs_map_is_handed_to_a_patrol():
    async def scenario():
        dispatches: list = []
        started, release = asyncio.Event(), asyncio.Event()
        release.set()
        task = _task("task-patrol-map", StepType.PATROL)

        async with _worker([_patrol(dispatches, started, release)]) as client:
            await client.operator_service.add_search_attributes(
                AddSearchAttributesRequest(
                    namespace=client.namespace,
                    search_attributes={
                        TASK_MAP_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD
                    },
                )
            )
            handle = await client.start_workflow(
                RobotWorkflow.run,
                task,
                id=task.id,
                task_queue=TASK_QUEUE,
                search_attributes=TypedSearchAttributes(
                    [SearchAttributePair(TASK_MAP_KEY, "lab")]
                ),
            )
            await handle.result()
        return dispatches

    assert asyncio.run(scenario()) == [("step-0", 0, 4, "lab")]


# --- continuing a failed run -------------------------------------------------


def test_skipped_steps_are_never_run_and_stay_skipped():
    async def scenario():
        calls: list = []
        task = _task("task-continued", StepType.STANDUP, StepType.MOVE, StepType.LIEDOWN)
        task.definition.steps[0].status = StepStatus.SKIPPED

        async with _worker(_instant_activities(calls)) as client:
            handle = await _start(client, task)
            await handle.result()
            return calls, await handle.query(RobotWorkflow.get_step_states)

    calls, steps = asyncio.run(scenario())

    assert [name for name, _ in calls] == ["execute_move", "execute_lie_down"]
    assert [step.status for step in steps] == [
        StepStatus.SKIPPED, StepStatus.COMPLETED, StepStatus.COMPLETED,
    ]
    assert (steps[0].started_at, steps[0].ended_at) == (None, None)


def test_a_patrols_progress_is_on_its_step():
    async def scenario():
        dispatches: list = []
        started, release = asyncio.Event(), asyncio.Event()
        release.set()
        task = _task("task-patrol-progress", StepType.PATROL)

        async with _worker([_patrol(dispatches, started, release, report=(2, 1))]) as client:
            handle = await _start(client, task)
            await handle.result()
            return await handle.query(RobotWorkflow.get_step_states)

    steps = asyncio.run(scenario())

    # Forward only, as the dispatch count: the late lower report is no news.
    assert steps[0].waypoints_passed == 2


def test_a_continued_patrol_starts_past_the_waypoints_it_had_passed():
    async def scenario():
        dispatches: list = []
        started, release = asyncio.Event(), asyncio.Event()
        release.set()
        task = _task("task-patrol-continued", StepType.PATROL)
        task.definition.steps[0].waypoints_passed = 2

        async with _worker([_patrol(dispatches, started, release)]) as client:
            handle = await _start(client, task)
            await handle.result()
            return dispatches

    dispatches = asyncio.run(scenario())

    assert dispatches == [("step-0", 2, 4, None)]


# --- A schedule's run reads its template ------------------------------------


def _resolver(answer, asked: Optional[list] = None):
    """A ``resolve_template`` answering ``answer`` (raised, if an exception)."""

    @activity.defn(name="resolve_template")
    async def resolve_template(task_template_id: str) -> Optional[ResolvedTemplate]:
        if asked is not None:
            asked.append(task_template_id)
        if isinstance(answer, BaseException):
            raise answer
        return answer

    return resolve_template


async def _register_name_and_map(client) -> None:
    await client.operator_service.add_search_attributes(
        AddSearchAttributesRequest(
            namespace=client.namespace,
            search_attributes={
                TASK_NAME_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
                TASK_MAP_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
            },
        )
    )


def _scheduled(task_id: str, *step_types: StepType) -> WorkflowTask:
    """A run a template-backed schedule starts: the snapshot plus the template id."""
    task = _task(task_id, *step_types)
    task.task_template_id = "0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55"
    return task


def _run_scheduled(task, resolver):
    """Run ``task`` stamped as a schedule stamps it (old name, old map)."""

    async def scenario():
        calls: list = []
        records: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams, expected_map=None) -> ActivityResult:
            calls.append(("execute_move", params, expected_map))
            return ActivityResult(success=True, state="succeeded")

        @activity.defn(name="execute_speak")
        async def execute_speak(params: SpeakParams) -> ActivityResult:
            calls.append(("execute_speak", params.text, None))
            return ActivityResult(success=True, state="succeeded")

        body = [execute_move] + _instant_activities(calls)[1:]
        async with _worker(
            body, io_activities=[execute_speak, resolver], record_task=_recorder(records)
        ) as client:
            await _register_name_and_map(client)
            handle = await client.start_workflow(
                RobotWorkflow.run,
                task,
                id=task.id,
                task_queue=TASK_QUEUE,
                search_attributes=TypedSearchAttributes(
                    [
                        SearchAttributePair(TASK_NAME_KEY, "Old name"),
                        SearchAttributePair(TASK_MAP_KEY, "lab"),
                    ]
                ),
            )
            failure = None
            try:
                await handle.result()
            except WorkflowFailureError as err:
                failure = err
            steps = await handle.query(RobotWorkflow.get_step_states)
            return calls, records, steps, failure

    return asyncio.run(scenario())


def test_a_scheduled_run_executes_its_template_as_it_is_now():
    """An edit after registration reaches the next run: the snapshot's MOVE is
    replaced by what the template says now, and the run is named after it."""
    asked: list = []
    edited = ResolvedTemplate(
        name="Morning round",
        map_name="lab",
        steps=[
            Step(id="1-speak", type=StepType.SPEAK, params=SpeakParams(text="Hello")),
            Step(id="2-move", type=StepType.MOVE, params=MoveParams(x=7.0, y=8.0, theta=0.0)),
        ],
    )

    calls, records, steps, failure = _run_scheduled(
        _scheduled("sched-edited", StepType.MOVE), _resolver(edited, asked)
    )

    assert failure is None
    assert asked == ["0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55"]
    assert [(name, arg) for name, arg, _ in calls] == [
        ("execute_speak", "Hello"),
        ("execute_move", MoveParams(x=7.0, y=8.0, theta=0.0)),
    ]
    assert [step.id for step in steps] == ["1-speak", "2-move"]
    assert records[0].name == "Morning round"
    assert records[0].map_name == "lab"


def test_a_template_moved_to_another_map_is_checked_against_that_map():
    """TaskMap follows the template, so every MOVE checks the map it is on now."""
    moved = ResolvedTemplate(
        name="Round",
        map_name="warehouse",
        steps=[Step(id="1-move", type=StepType.MOVE, params=MoveParams(x=1.0, y=1.0, theta=0.0))],
    )

    calls, records, _, _ = _run_scheduled(
        _scheduled("sched-moved", StepType.MOVE), _resolver(moved)
    )

    assert calls[0][2] == "warehouse"
    assert records[0].map_name == "warehouse"


def test_a_refused_template_fails_its_first_step_before_anything_runs():
    refused = ResolvedTemplate(
        name="Round",
        map_name="warehouse",
        steps=[
            Step(id="1-speak", type=StepType.SPEAK, params=SpeakParams(text="Hello")),
            Step(id="2-move", type=StepType.MOVE, params=MoveParams(x=1.0, y=1.0, theta=0.0)),
        ],
        refusal="This job's positions are on the map 'warehouse', but the robot is using 'lab'.",
    )

    calls, records, steps, failure = _run_scheduled(
        _scheduled("sched-refused", StepType.MOVE), _resolver(refused)
    )

    assert failure is not None
    assert calls == []
    assert steps[0].status is StepStatus.FAILED
    assert steps[0].error_msg == refused.refusal
    assert steps[1].status is StepStatus.PENDING
    assert records[0].status == "FAILED"
    assert records[0].name == "Round"
    assert records[0].map_name == "warehouse"


@pytest.mark.parametrize(
    "answer",
    [None, ApplicationError("database unreachable", non_retryable=True)],
    ids=["template-gone", "unreadable"],
)
def test_a_template_that_cannot_be_read_leaves_the_snapshot(answer):
    calls, records, steps, failure = _run_scheduled(
        _scheduled("sched-snapshot", StepType.MOVE), _resolver(answer)
    )

    assert failure is None
    assert [(name, arg) for name, arg, _ in calls] == [
        ("execute_move", MoveParams(x=1.5, y=-2.5, theta=90.0))
    ]
    assert steps[0].status is StepStatus.COMPLETED
    assert (records[0].name, records[0].map_name) == ("Old name", "lab")


def test_a_run_without_a_template_never_asks_for_one():
    asked: list = []

    calls, _, _, failure = _run_scheduled(
        _task("sched-composer", StepType.MOVE), _resolver(None, asked)
    )

    assert failure is None
    assert asked == []
    assert len(calls) == 1


# --- Admission: the robot is asked before a run starts, and told how it ended --


_PRE_ADMISSION_HISTORIES = os.path.join(
    os.path.dirname(__file__), "fixtures", "workflow_histories", "pre_task_admission"
)


def _order(events: list) -> list:
    """``events`` as activity names: the step stand-ins append ``(name, arg)``."""
    return [event if isinstance(event, str) else event[0] for event in events]


def _ordered_recorder(records: list, events: list):
    """``_recorder`` that also marks its place among the other activities."""

    @activity.defn(name="record_task")
    async def record_task(record: TaskRecord) -> None:
        records.append(record)
        events.append("record_task")

    return record_task


def _refused(code: str, message: str = "The robot refused.") -> TaskAdmission:
    return TaskAdmission(accepted=False, code=code, message=message)


def _run_admission(task, answer=None, body=None):
    """Run ``task`` to its end with every activity recording into one list.

    ``answer`` is begin_task's (see ``_admitter``); ``body(events)`` builds the
    body activities when the instant ones will not do. Returns the activity
    order, begin_task's requests, end_task's reports, the records, the final
    steps and the run's failure (None when it completed).
    """

    async def scenario():
        events: list = []
        asked: list = []
        ended: list = []
        records: list = []
        async with _worker(
            body(events) if body is not None else _instant_activities(events),
            record_task=_ordered_recorder(records, events),
            begin_task=_admitter(answer, asked, events),
            end_task=_closer(ended, events),
        ) as client:
            handle = await _start(client, task)
            failure = None
            try:
                await asyncio.wait_for(handle.result(), timeout=30)
            except WorkflowFailureError as err:
                failure = err
            steps = await handle.query(RobotWorkflow.get_step_states)
        return _order(events), asked, ended, records, steps, failure

    return asyncio.run(scenario())


async def _cancel_and_let_it_land(handle, timeout: float = 10.0) -> None:
    """Cancel the run and return once the workflow has been handed the cancel.

    ``handle.cancel()`` only records the request; the workflow meets it in its
    next workflow task. A test that then releases a parked activity would
    otherwise race the two, and pin whichever happened to win. The run's own
    history says when: a workflow task completed after the request.
    """
    await handle.cancel()
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        requested = False
        history = await handle.fetch_history()
        for event in history.events:
            if event.HasField("workflow_execution_cancel_requested_event_attributes"):
                requested = True
            elif requested and event.HasField("workflow_task_completed_event_attributes"):
                return
        if asyncio.get_running_loop().time() > deadline:
            raise AssertionError("the workflow never processed the cancel")
        await asyncio.sleep(0.05)


def test_a_run_asks_the_robot_before_its_first_step_and_reports_its_end_once():
    order, asked, ended, records, steps, failure = _run_admission(
        _task("task-admitted", StepType.MOVE, StepType.STANDUP)
    )

    assert failure is None
    # The robot before anything moves; the robot again before the report, so
    # it is released without waiting for an insert.
    assert order == [
        "begin_task", "execute_move", "execute_stand", "end_task", "record_task",
    ]
    # The id is the run's own workflow id: what end_task is matched on.
    assert asked == [
        TaskAdmissionRequest(task_id="task-admitted", needs_localization=True)
    ]
    assert ended == [
        TaskEnd(task_id="task-admitted", outcome=TaskOutcome.COMPLETED, reason="")
    ]
    assert [step.status for step in steps] == [StepStatus.COMPLETED] * 2


@pytest.mark.parametrize(
    "step_types,needs",
    [
        ((StepType.STANDUP, StepType.WAIT, StepType.LIEDOWN), False),
        ((StepType.MOVE,), True),
        ((StepType.CHARGE,), True),
        ((StepType.PATROL,), True),
        ((StepType.STANDUP, StepType.PATROL), True),
    ],
    ids=["postures-and-wait", "move", "charge", "patrol", "posture-then-patrol"],
)
def test_begin_task_says_whether_the_run_drives(step_types, needs):
    # Refused, so the run stops there: only the question is under test.
    _, asked, _, _, _, _ = _run_admission(
        _task("task-needs", *step_types), answer=_refused(ROBOT_BUSY)
    )

    assert [request.needs_localization for request in asked] == [needs]


def test_a_skipped_step_does_not_make_a_continuation_need_a_position():
    # A Retry whose failed MOVE is behind it: only the LIEDOWN is left, and an
    # unconfirmed robot must still take it.
    task = _task("task-continued-no-nav", StepType.MOVE, StepType.LIEDOWN)
    task.definition.steps[0].status = StepStatus.SKIPPED

    order, asked, _, _, _, failure = _run_admission(task)

    assert failure is None
    assert asked[0].needs_localization is False
    assert order == ["begin_task", "execute_lie_down", "end_task", "record_task"]


def _failing_move_body(message: str):
    def body(events: list):
        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            events.append(("execute_move", params))
            raise ApplicationError(message, non_retryable=True)

        return [execute_move] + _instant_activities(events)[1:]

    return body


def test_a_failed_run_reports_failed_once_with_the_failed_steps_error():
    order, _, ended, records, steps, failure = _run_admission(
        _task("task-end-failed", StepType.STANDUP, StepType.MOVE, StepType.LIEDOWN),
        body=_failing_move_body("move ended in aborted: no route"),
    )

    assert failure is not None
    assert order == [
        "begin_task", "execute_stand", "execute_move", "end_task", "record_task",
    ]
    # FAILED is what latches ERROR on the robot; the reason is what the
    # console shows while it is locked.
    assert [(report.task_id, report.outcome) for report in ended] == [
        ("task-end-failed", TaskOutcome.FAILED)
    ]
    assert ended[0].reason == steps[1].error_msg
    assert "no route" in ended[0].reason
    assert records[0].status == "FAILED"


def test_the_reported_reason_is_cut_to_one_sentences_worth():
    _, _, ended, _, steps, _ = _run_admission(
        _task("task-end-long", StepType.MOVE),
        body=_failing_move_body("x" * (END_TASK_REASON_MAX * 3)),
    )

    assert len(steps[0].error_msg) > END_TASK_REASON_MAX
    assert len(ended[0].reason) == END_TASK_REASON_MAX


@pytest.mark.parametrize("step_type", [StepType.MOVE, StepType.STANDUP])
def test_a_canceled_run_reports_canceled_once(step_type):
    """Both shapes a cancel reaches the run in (a MOVE's asyncio.CancelledError,
    a posture's ActivityError): the operator's cancel and the safety lock's are
    the same workflow cancel, and neither may latch ERROR."""

    async def scenario():
        ended: list = []
        records: list = []
        started, release = asyncio.Event(), asyncio.Event()
        gated = (
            _gated_move(started, release)
            if step_type is StepType.MOVE
            else _gated_stand(started, release)
        )
        task = _task(f"task-end-canceled-{step_type.value}", step_type)

        async with _worker(
            [gated], record_task=_recorder(records), end_task=_closer(ended)
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return ended, records, exc_info.value

    ended, records, err = asyncio.run(scenario())

    assert isinstance(err.cause, CancelledError)
    assert [report.outcome for report in ended] == [TaskOutcome.CANCELED]
    assert ended[0].reason == ""
    assert records[0].status == "CANCELED"


@pytest.mark.parametrize("code", [ERROR_LATCHED, NOT_INITIALIZED, ROBOT_BUSY])
def test_a_refused_run_fails_at_once_and_reports_nothing(code):
    message = f"The robot said no ({code})."

    order, asked, ended, records, steps, failure = _run_admission(
        _task("task-refused", StepType.MOVE, StepType.STANDUP),
        answer=_refused(code, message),
    )

    # Nothing ran, and above all no end_task: a refusal changed nothing on the
    # robot, and a FAILED reported for it would latch ERROR on a robot that
    # was merely unconfirmed, or busy with another run. Not the best-effort
    # CANCELED an unanswered begin gets either: the robot ANSWERED, and said no.
    assert order == ["begin_task", "record_task"]
    assert ended == []
    # Asked once: a refusal is an answer, never retried.
    assert len(asked) == 1
    # The run fails with the robot's own sentence and its code, non-retryably.
    assert isinstance(failure.cause, ApplicationError)
    assert failure.cause.type == code
    assert failure.cause.non_retryable is True
    assert failure.cause.message == message
    assert steps[0].status is StepStatus.FAILED
    assert steps[0].error_msg == message
    assert steps[1].status is StepStatus.PENDING
    assert records[0].status == "FAILED"


def test_a_refused_continuation_fails_its_first_step_that_would_run():
    task = _task(
        "task-refused-continued", StepType.STANDUP, StepType.MOVE, StepType.LIEDOWN
    )
    task.definition.steps[0].status = StepStatus.SKIPPED

    _, _, ended, _, steps, failure = _run_admission(
        task, answer=_refused(NOT_INITIALIZED, "Confirm the position first.")
    )

    assert failure is not None and ended == []
    # Not steps[0]: marked FAILED, the next Retry would start from it and run
    # the STANDUP the first run had already completed.
    assert [step.status for step in steps] == [
        StepStatus.SKIPPED, StepStatus.FAILED, StepStatus.PENDING,
    ]
    assert steps[0].error_msg == ""
    assert steps[1].error_msg == "Confirm the position first."
    _, from_step_id = continuation_steps(steps)
    assert from_step_id == "step-1"


def test_one_unanswered_begin_task_is_asked_again_and_the_run_goes_on():
    order, asked, ended, _, _, failure = _run_admission(
        _task("task-begin-retried", StepType.STANDUP),
        answer=[ApplicationError("The robot did not answer begin_task"), None],
    )

    assert failure is None
    # The same id both times: the robot accepts its own retry again instead of
    # refusing it robot_busy against itself.
    assert [request.task_id for request in asked] == ["task-begin-retried"] * 2
    assert order == [
        "begin_task", "begin_task", "execute_stand", "end_task", "record_task",
    ]
    assert [report.outcome for report in ended] == [TaskOutcome.COMPLETED]


def test_a_robot_that_never_answers_fails_the_run_closed():
    order, asked, ended, records, steps, failure = _run_admission(
        _task("task-begin-dead", StepType.MOVE, StepType.STANDUP),
        answer=ApplicationError("The robot did not answer begin_task"),
    )

    assert len(asked) == BEGIN_TASK_ATTEMPTS
    # No step ran. One end_task follows, before the record: see the next test.
    assert order == ["begin_task"] * BEGIN_TASK_ATTEMPTS + ["end_task", "record_task"]
    assert failure.cause.type == ROBOT_STATE_UNREACHABLE
    assert failure.cause.non_retryable is True
    # The operator's sentence, with no node or service name in it.
    assert steps[0].status is StepStatus.FAILED
    assert steps[0].error_msg == UNREACHABLE_MESSAGE
    assert "begin_task" not in steps[0].error_msg
    assert steps[1].status is StepStatus.PENDING
    assert records[0].status == "FAILED"


def test_an_unanswered_begin_task_is_released_with_one_canceled_report():
    """No answer is not "no": the request may have landed, been accepted, and
    lost only its answer. The run then fails closed believing itself
    unadmitted while the robot counts it as RUNNING, refusing every task after
    it. One end_task closes that, and is a no-op on a robot that never accepted
    (matched by task_id). CANCELED, never FAILED: this must latch nothing."""
    _, _, ended, _, _, failure = _run_admission(
        _task("task-begin-lost", StepType.MOVE),
        answer=ApplicationError("The robot did not answer begin_task"),
    )

    assert ended == [
        TaskEnd(task_id="task-begin-lost", outcome=TaskOutcome.CANCELED, reason="")
    ]
    # And the run still ends as it was going to.
    assert failure.cause.type == ROBOT_STATE_UNREACHABLE


def test_a_release_the_robot_never_takes_does_not_change_how_the_run_ends():
    # The likeliest unanswered begin: the state service is down, and the
    # release goes unanswered too. Tried on begin's short budget, then dropped.
    async def scenario():
        attempts: list = []
        records: list = []

        @activity.defn(name="end_task")
        async def end_task(report: TaskEnd) -> None:
            attempts.append(report)
            raise ApplicationError("The robot did not answer end_task")

        task = _task("task-all-dead", StepType.STANDUP)
        async with _worker(
            _instant_activities([]),
            record_task=_recorder(records),
            begin_task=_admitter(ApplicationError("The robot did not answer begin_task")),
            end_task=end_task,
        ) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError) as exc_info:
                await asyncio.wait_for(handle.result(), timeout=30)
            return attempts, records, exc_info.value

    attempts, records, failure = asyncio.run(scenario())

    assert len(attempts) == RELEASE_TASK_ATTEMPTS
    assert {report.outcome for report in attempts} == {TaskOutcome.CANCELED}
    assert failure.cause.type == ROBOT_STATE_UNREACHABLE
    assert records[0].status == "FAILED"


@pytest.mark.parametrize(
    "error_type", ["not_initialized", "map_mismatch"],
)
def test_the_reported_reason_is_the_sentence_without_the_error_type(error_type):
    """The robot quotes the reason in its own sentence ("A task failed:
    <reason>. Release the error ...") and the console prints that verbatim, so
    a code in front of it would be shown to the operator. The step keeps the
    string it always stored: the history and the console's own rewriting of a
    failure read that one."""
    sentence = "The robot's position is no longer confirmed, so it will not drive."

    def body(events: list):
        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            # What _refuse_unconfirmed_position / _refuse_wrong_map raise.
            raise ApplicationError(sentence, type=error_type, non_retryable=True)

        return [execute_move] + _instant_activities(events)[1:]

    _, _, ended, records, steps, _ = _run_admission(
        _task(f"task-reason-{error_type}", StepType.MOVE), body=body
    )

    assert [report.outcome for report in ended] == [TaskOutcome.FAILED]
    assert ended[0].reason == sentence
    # Unchanged, prefix included, on the step and in the record.
    assert steps[0].error_msg == f"{error_type}: {sentence}"
    assert records[0].steps[0].error_msg == f"{error_type}: {sentence}"


def test_a_failure_with_no_sentence_of_its_own_falls_back_to_the_steps_error():
    # A result that says success=False carries no cause; the step's error is
    # then all there is.
    def body(events: list):
        @activity.defn(name="execute_stand")
        async def execute_stand() -> ActivityResult:
            return ActivityResult(success=False, state="aborted")

        return [execute_stand]

    _, _, ended, _, steps, _ = _run_admission(
        _task("task-reason-fallback", StepType.STANDUP), body=body
    )

    assert steps[0].error_msg == "activity failed"
    assert ended[0].reason == "activity failed"


def _gated_admitter(started: asyncio.Event, release: asyncio.Event, answer=None):
    """A ``begin_task`` that parks until released: the question is with the
    robot and the answer has not come."""

    @activity.defn(name="begin_task")
    async def begin_task(request: TaskAdmissionRequest) -> TaskAdmission:
        started.set()
        await release.wait()
        if isinstance(answer, BaseException):
            # ...and never does: every attempt after the release fails too.
            raise answer
        return answer if answer is not None else TaskAdmission(accepted=True)

    return begin_task


def _cancel_during_begin(task_id: str, answer=None):
    async def scenario():
        calls: list = []
        ended: list = []
        started, release = asyncio.Event(), asyncio.Event()
        task = _task(task_id, StepType.STANDUP, StepType.LIEDOWN)

        async with _worker(
            _instant_activities(calls),
            begin_task=_gated_admitter(started, release, answer),
            end_task=_closer(ended),
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await _cancel_and_let_it_land(handle)
            # Only now does the robot answer: the run must still be listening.
            release.set()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await asyncio.wait_for(handle.result(), timeout=10)
            steps = await handle.query(RobotWorkflow.get_step_states)
            return calls, ended, steps, exc_info.value

    return asyncio.run(scenario())


def test_a_cancel_while_begin_task_is_out_waits_for_the_answer_and_closes_it():
    calls, ended, steps, err = _cancel_during_begin("task-cancel-in-begin")

    assert isinstance(err.cause, CancelledError)
    # The robot had accepted: it counts the run as RUNNING, so the cancelled
    # run still owes it an end. Dropped, every later task would be robot_busy.
    assert ended == [
        TaskEnd(task_id="task-cancel-in-begin", outcome=TaskOutcome.CANCELED)
    ]
    # And nothing ran: the cancel was the run's outcome, not the first step's.
    assert calls == []
    assert [step.status for step in steps] == [StepStatus.PENDING] * 2


def test_a_cancel_while_begin_task_is_out_owes_nothing_when_the_robot_refuses():
    calls, ended, steps, err = _cancel_during_begin(
        "task-cancel-in-refused-begin", answer=_refused(ERROR_LATCHED)
    )

    # The operator's cancel is the outcome; the refusal only means there is
    # nothing to close.
    assert isinstance(err.cause, CancelledError)
    assert ended == []
    assert calls == []
    assert [step.status for step in steps] == [StepStatus.PENDING] * 2


def test_a_cancel_while_an_unanswered_begin_task_is_out_releases_the_robot():
    calls, ended, steps, err = _cancel_during_begin(
        "task-cancel-in-lost-begin",
        answer=ApplicationError("The robot did not answer begin_task"),
    )

    # Still the operator's cancel, not a failure...
    assert isinstance(err.cause, CancelledError)
    # ...and since nobody knows whether the robot took the run, it is told.
    assert ended == [
        TaskEnd(task_id="task-cancel-in-lost-begin", outcome=TaskOutcome.CANCELED)
    ]
    assert calls == []
    assert [step.status for step in steps] == [StepStatus.PENDING] * 2


def test_a_cancel_while_end_task_is_out_does_not_drop_the_report():
    async def scenario():
        ended: list = []
        records: list = []
        started, release = asyncio.Event(), asyncio.Event()

        @activity.defn(name="end_task")
        async def end_task(report: TaskEnd) -> None:
            started.set()
            await release.wait()
            ended.append(report)

        task = _task("task-cancel-in-end", StepType.STANDUP)
        async with _worker(
            _instant_activities([]), record_task=_recorder(records), end_task=end_task
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await _cancel_and_let_it_land(handle)
            release.set()
            await asyncio.wait_for(handle.result(), timeout=10)
            return ended, records, await handle.describe()

    ended, records, description = asyncio.run(scenario())

    # The report was made, once, and the run closed as what it was: its steps
    # had all completed before the cancel arrived.
    assert [report.outcome for report in ended] == [TaskOutcome.COMPLETED]
    assert description.status.name == "COMPLETED"
    assert records[0].status == "COMPLETED"


def test_a_report_the_robot_never_takes_does_not_fail_the_run():
    async def scenario():
        attempts: list = []
        records: list = []

        @activity.defn(name="end_task")
        async def end_task(report: TaskEnd) -> None:
            attempts.append(report)
            raise ApplicationError("The robot did not answer end_task")

        task = _task("task-end-broken", StepType.STANDUP)
        async with _worker(
            _instant_activities([]), record_task=_recorder(records), end_task=end_task
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(handle.result(), timeout=30)
            return attempts, records, await handle.describe()

    attempts, records, description = asyncio.run(scenario())

    # Retried, since a report that never lands leaves the robot RUNNING; then
    # dropped, since the run's outcome is not the robot's to hold open.
    assert len(attempts) == 5
    assert description.status.name == "COMPLETED"
    assert records[0].status == "COMPLETED"


def test_a_paused_run_reports_nothing_until_it_ends():
    async def scenario():
        asked: list = []
        ended: list = []
        started, release = asyncio.Event(), asyncio.Event()
        task = _task("task-paused-admitted", StepType.MOVE)

        async with _worker(
            [_gated_move(started, release)],
            begin_task=_admitter(None, asked),
            end_task=_closer(ended),
        ) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            started.clear()
            await handle.signal(RobotWorkflow.pause)
            await _statuses_until(handle, lambda s: s[0] is StepStatus.PAUSED)
            while_held = (list(asked), list(ended))

            await handle.signal(RobotWorkflow.resume)
            await asyncio.wait_for(started.wait(), timeout=10)
            release.set()
            await asyncio.wait_for(handle.result(), timeout=10)
            return while_held, asked, ended

    (asked_held, ended_held), asked, ended = asyncio.run(scenario())

    # Held, the run is still the robot's running task: no end, and the resume
    # does not ask again either.
    assert len(asked_held) == 1 and ended_held == []
    assert len(asked) == 1
    assert [report.outcome for report in ended] == [TaskOutcome.COMPLETED]


def _run_scheduled_admission(task, template, answer=None):
    """A template-backed scheduled run, with every activity in one order."""

    async def scenario():
        events: list = []
        asked: list = []
        ended: list = []
        records: list = []

        @activity.defn(name="resolve_template")
        async def resolve_template(task_template_id: str) -> Optional[ResolvedTemplate]:
            events.append("resolve_template")
            return template

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams, expected_map=None) -> ActivityResult:
            events.append(("execute_move", params))
            return ActivityResult(success=True, state="succeeded")

        async with _worker(
            [execute_move] + _instant_activities(events)[1:],
            io_activities=[resolve_template],
            record_task=_ordered_recorder(records, events),
            begin_task=_admitter(answer, asked, events),
            end_task=_closer(ended, events),
        ) as client:
            await _register_name_and_map(client)
            handle = await client.start_workflow(
                RobotWorkflow.run,
                task,
                id=task.id,
                task_queue=TASK_QUEUE,
                search_attributes=TypedSearchAttributes(
                    [
                        SearchAttributePair(TASK_NAME_KEY, "Old name"),
                        SearchAttributePair(TASK_MAP_KEY, "lab"),
                    ]
                ),
            )
            failure = None
            try:
                await asyncio.wait_for(handle.result(), timeout=30)
            except WorkflowFailureError as err:
                failure = err
            return _order(events), asked, ended, failure

    return asyncio.run(scenario())


def test_a_scheduled_run_asks_the_robot_about_the_steps_it_reads_from_its_template():
    """The one gate a run a Schedule starts by itself ever meets. Asked after
    the template is read: the snapshot only stands up, the template now drives."""
    edited = ResolvedTemplate(
        name="Morning round",
        map_name="lab",
        steps=[
            Step(id="1-move", type=StepType.MOVE, params=MoveParams(x=7.0, y=8.0, theta=0.0)),
        ],
    )

    order, asked, ended, failure = _run_scheduled_admission(
        _scheduled("sched-admitted", StepType.STANDUP), edited
    )

    assert failure is None
    assert order == [
        "resolve_template", "begin_task", "execute_move", "end_task", "record_task",
    ]
    assert asked == [
        TaskAdmissionRequest(task_id="sched-admitted", needs_localization=True)
    ]
    assert [report.outcome for report in ended] == [TaskOutcome.COMPLETED]


def test_a_scheduled_run_whose_template_no_longer_drives_needs_no_position():
    edited = ResolvedTemplate(
        name="Stand", map_name=None, steps=[Step(id="1-stand", type=StepType.STANDUP)]
    )

    _, asked, _, failure = _run_scheduled_admission(
        _scheduled("sched-no-nav", StepType.MOVE), edited
    )

    assert failure is None
    assert asked[0].needs_localization is False


def test_a_schedule_firing_during_another_run_is_refused_and_reports_nothing():
    """What used to be admitted and interleave its steps with the running
    task's: the robot answers robot_busy, the run fails on its first step and
    never reaches end_task, so the task that IS running keeps the robot."""
    template = ResolvedTemplate(
        name="Round",
        map_name="lab",
        steps=[
            Step(id="1-move", type=StepType.MOVE, params=MoveParams(x=1.0, y=1.0, theta=0.0)),
        ],
    )

    order, _, ended, failure = _run_scheduled_admission(
        _scheduled("sched-busy", StepType.MOVE),
        template,
        answer=_refused(ROBOT_BUSY, "The robot is running another task."),
    )

    assert order == ["resolve_template", "begin_task", "record_task"]
    assert ended == []
    assert failure.cause.type == ROBOT_BUSY


def test_a_template_refused_for_its_map_never_asks_the_robot():
    refused = ResolvedTemplate(
        name="Round",
        map_name="warehouse",
        steps=[
            Step(id="1-move", type=StepType.MOVE, params=MoveParams(x=1.0, y=1.0, theta=0.0)),
        ],
        refusal="This job's positions are on the map 'warehouse'.",
    )

    order, asked, ended, failure = _run_scheduled_admission(
        _scheduled("sched-wrong-map", StepType.MOVE), refused
    )

    assert failure is not None
    assert order == ["resolve_template", "record_task"]
    assert asked == [] and ended == []


def _quiet_robot_body(stale_attempts: int):
    """A MOVE whose position check finds no fresh robot_state sample on its
    first ``stale_attempts`` attempts: what RobotActivities raises then, built
    from the same rule (task_guard.goal_refusal), so its type, sentence and
    retryability are the real ones."""

    def body(events: list):
        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            attempt = activity.info().attempt
            events.append(("execute_move", attempt))
            if attempt <= stale_attempts:
                refusal = goal_refusal(None, silent_for_s=3.0 + 5.0 * attempt)
                raise ApplicationError(
                    refusal.message,
                    type=refusal.code,
                    non_retryable=not refusal.retryable,
                )
            return ActivityResult(success=True, state="succeeded")

        return [execute_move] + _instant_activities(events)[1:]

    return body


def test_a_robot_that_goes_quiet_and_speaks_again_does_not_fail_the_move():
    """A few seconds without a robot_state frame is far more often a hiccup
    than a dead node: the refusal is retryable, so the step's own retry policy
    (three attempts) outlasts it and the run goes on."""
    order, _, ended, _, steps, failure = _run_admission(
        _task("task-quiet-then-back", StepType.MOVE, StepType.STANDUP),
        body=_quiet_robot_body(stale_attempts=2),
    )

    assert failure is None
    assert order == [
        "begin_task", "execute_move", "execute_move", "execute_move",
        "execute_stand", "end_task", "record_task",
    ]
    assert [step.status for step in steps] == [StepStatus.COMPLETED] * 2
    assert [report.outcome for report in ended] == [TaskOutcome.COMPLETED]


def test_a_robot_that_stays_quiet_fails_the_run_with_a_sentence():
    order, _, ended, records, steps, failure = _run_admission(
        _task("task-quiet-for-good", StepType.MOVE, StepType.STANDUP),
        body=_quiet_robot_body(stale_attempts=99),
    )

    # Three attempts of the MOVE, never the STANDUP after it.
    assert order == [
        "begin_task", "execute_move", "execute_move", "execute_move",
        "end_task", "record_task",
    ]
    assert failure is not None
    assert steps[0].status is StepStatus.FAILED
    # The step keeps the typed string, as every activity failure does...
    assert steps[0].error_msg.startswith(f"{ROBOT_STATE_STALE}: The robot has not")
    assert steps[1].status is StepStatus.PENDING
    assert records[0].status == "FAILED"
    # ...and the robot is told why in a sentence it can quote to the operator:
    # the last attempt's, with how long it had been by then, and no prefix.
    assert [report.outcome for report in ended] == [TaskOutcome.FAILED]
    assert ended[0].reason == (
        "The robot has not reported its state for 18 seconds, so it was not "
        "sent a navigation goal. Check that the robot's software is running "
        "and start the task again."
    )


def test_a_refused_run_replays_deterministically():
    async def scenario():
        task = _task("task-refused-replay", StepType.MOVE)
        async with _worker(
            _instant_activities([]), begin_task=_admitter(_refused(NOT_INITIALIZED))
        ) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError):
                await handle.result()
            history = await handle.fetch_history()

        await Replayer(
            workflows=[RobotWorkflow], data_converter=pydantic_data_converter
        ).replay_workflow(history)

    asyncio.run(scenario())


_PRE_ADMISSION_RUNS = ["canceled_mid_move", "completed", "continued", "failed", "scheduled"]


def test_the_pre_admission_fixtures_are_the_ones_replayed():
    # A fixture nobody replays pins nothing; one that already holds a
    # begin_task was not recorded before the gate.
    paths = sorted(glob.glob(os.path.join(_PRE_ADMISSION_HISTORIES, "*.json")))

    assert [os.path.basename(path)[: -len(".json")] for path in paths] == _PRE_ADMISSION_RUNS
    for path in paths:
        with open(path) as recorded:
            assert "begin_task" not in recorded.read()


@pytest.mark.parametrize("name", _PRE_ADMISSION_RUNS)
def test_a_run_from_before_the_gate_replays_without_it(name):
    """Histories recorded from the workflow as it was before begin_task /
    end_task (commit 721783c, temporalio 1.35.0): a run in flight across the
    deploy, and every closed run the console still queries for its steps,
    replays through the new code. TASK_ADMISSION_PATCH is what lets it: no
    marker in the history, so the replay schedules neither activity."""
    with open(os.path.join(_PRE_ADMISSION_HISTORIES, f"{name}.json")) as recorded:
        history = WorkflowHistory.from_json(f"pre-admission-{name}", recorded.read())

    asyncio.run(
        Replayer(
            workflows=[RobotWorkflow], data_converter=pydantic_data_converter
        ).replay_workflow(history)
    )
