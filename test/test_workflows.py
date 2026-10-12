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
  member; a pause interrupts the group's MOVE while its SPEAK plays on.
  SPEAK is dispatched to the IO queue, so a stand-in registered only there
  is what runs it.
* WAIT is a workflow timer, not an activity: a pause freezes it and the
  resume starts a second timer for only the remainder (read off the run's
  history), and a cancel during it is CANCELED / "Task canceled".

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
from contextlib import asynccontextmanager

import pytest

pytest.importorskip("temporalio")

from temporalio import activity  # noqa: E402
from temporalio.api.enums.v1 import IndexedValueType  # noqa: E402
from temporalio.api.operatorservice.v1 import (  # noqa: E402
    AddSearchAttributesRequest,
)
from temporalio.client import WorkflowFailureError  # noqa: E402
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
from temporalio.worker import Worker  # noqa: E402

from syncai_backend.gateways.workflow.schema import (  # noqa: E402
    MoveParams,
    SpeakParams,
    Step,
    StepStatus,
    StepType,
    WaitParams,
    WorkflowTask,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.search_attributes import (  # noqa: E402
    TASK_MAP_KEY,
)
from syncai_backend.temporal.activities import ActivityResult  # noqa: E402
from syncai_backend.temporal.workflows import RobotWorkflow  # noqa: E402

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
            # Schema contract: MOVE and WAIT carry params, the posture steps
            # carry none (Step validation enforces it, the workflow relies on
            # it for its empty-args special case).
            params={
                StepType.MOVE: MoveParams(x=1.5, y=-2.5, theta=90.0),
                StepType.WAIT: WaitParams(seconds=WAIT_SECONDS),
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


def _gated_move(started: asyncio.Event, release: asyncio.Event):
    """A MOVE activity that parks until released, heartbeating while it waits.

    The heartbeats are load-bearing twice over: the workflow schedules every
    activity with ``heartbeat_timeout=3s`` (real time — the test server does
    not skip while an activity is outstanding), and a cancel is only ever
    delivered to a running activity in a heartbeat response.
    """

    @activity.defn(name="execute_move")
    async def execute_move(params: MoveParams) -> ActivityResult:
        started.set()
        while not release.is_set():
            activity.heartbeat()
            try:
                await asyncio.wait_for(release.wait(), timeout=0.2)
            except asyncio.TimeoutError:
                pass
        return ActivityResult(success=True, state="succeeded")

    return execute_move


@asynccontextmanager
async def _worker(activities, io_activities=None):
    """A time-skipping environment with a worker polling TASK_QUEUE.

    The client is built with ``pydantic_data_converter`` to mirror
    ``run_worker`` — the default converter cannot serialize WorkflowTask.
    ``io_activities`` (SPEAK) get a second worker on IO_TASK_QUEUE, the way
    run_worker splits them.
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
        ):
            if io_activities:
                async with Worker(
                    env.client, task_queue=IO_TASK_QUEUE, activities=io_activities
                ):
                    yield env.client
            else:
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
    started: asyncio.Event, release: asyncio.Event, dispatches: list, cancels: list
):
    """``_gated_move`` that also records each dispatch and each cancel it receives.

    ``dispatches`` collects ``activity.info().attempt`` so a test can tell a
    *re-sent* MOVE (two activities, each attempt 1) from a *retried* one.
    """

    @activity.defn(name="execute_move")
    async def execute_move(params: MoveParams) -> ActivityResult:
        dispatches.append(activity.info().attempt)
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
    """A SPEAK that parks until released. No heartbeat: the workflow gives
    SPEAK no heartbeat_timeout, exactly as execute_speak runs."""

    @activity.defn(name="execute_speak")
    async def execute_speak(params: SpeakParams) -> ActivityResult:
        calls.append(("execute_speak", activity.info().task_queue))
        started.set()
        await release.wait()
        return ActivityResult(success=True, state="succeeded")

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
