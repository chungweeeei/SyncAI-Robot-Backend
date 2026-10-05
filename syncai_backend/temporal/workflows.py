import asyncio
from datetime import timedelta

from temporalio import workflow
from temporalio.common import RetryPolicy
from temporalio.exceptions import ActivityError, ApplicationError, CancelledError

with workflow.unsafe.imports_passed_through():
    from syncai_backend.gateways.workflow.schema import (
        Step,
        StepStatus,
        StepType,
        WaitParams,
        WorkflowTask,
    )
    from syncai_backend.gateways.workflow.search_attributes import TASK_MAP_KEY
    from syncai_backend.temporal.activities import ActivityResult, RobotActivities


# Heartbeat window for MOVE (and the posture steps). execute_move heartbeats
# once before the goal is sent and then once per poll, so the only stretch with
# no heartbeat in it is the send itself -- which the gateway bounds by
# NAV_GOAL_SEND_BUDGET_S, and test_activities.py checks that this stays above
# it. Tighten either side and the other has to follow.
MOVE_HEARTBEAT_TIMEOUT = timedelta(seconds=3)


def _own_map() -> str | None:
    """The map this run's coordinates are in, off its own ``TaskMap``.

    Read from the run's attributes rather than its argument because a
    scheduled run's argument was frozen when the schedule was registered and
    carries no map, while the schedule action's attributes reach every run.
    Never fatal: a run without the attribute (a schedule older than it) skips
    the map check and drives as it always did.
    """
    try:
        return workflow.info().typed_search_attributes.get(TASK_MAP_KEY) or None
    except Exception:
        return None


def _cancel_requested() -> bool:
    """Whether the server has asked this workflow to cancel.

    Needed in exactly one place: a MOVE that comes back cancelled while a
    pause is pending could owe that to the pause *or* to a task cancel that
    landed in the same breath, and only the former may hold -- a hold after a
    cancel would park the run for good, because the SDK delivers a workflow
    cancel once and ``wait_condition`` lets code that caught it keep going.
    ``workflow.cancellation_reason`` is the SDK's answer (1.33); an older
    worker without it falls back to "no", which keeps every ordinary pause
    working and leaves only that same-breath race to a second cancel.
    """
    reason = getattr(workflow, "cancellation_reason", None)
    return reason is not None and reason() is not None


@workflow.defn
class RobotWorkflow:
    def __init__(self) -> None:
        self._steps: list[Step] = []
        # The hold (POST /api/v1/tasks/{id}/pause). A flag rather than an
        # event because both signals are idempotent: a second pause while
        # held, or a resume while running, must change nothing.
        self._pause_requested: bool = False
        # The MOVE activity in flight, if any -- the one thing a pause
        # interrupts rather than waits out -- and whether the pause did so.
        self._move_handle: workflow.ActivityHandle | None = None
        self._move_interrupted: bool = False

    @workflow.query
    def get_step_states(self) -> list[Step]:
        return self._steps

    @workflow.signal
    def pause(self) -> None:
        self._pause_requested = True
        # A MOVE is interrupted at once: cancelling its handle lands in
        # execute_move's ``except CancelledError``, which cancels the nav2 goal
        # -- the same path a task cancel takes, shield and all. Nothing is in
        # flight here for any other step type; those finish and the run holds
        # before the next one (see run()).
        if self._move_handle is not None and not self._move_handle.done():
            self._move_interrupted = True
            self._move_handle.cancel()

    @workflow.signal
    def resume(self) -> None:
        self._pause_requested = False

    async def _hold(self, step: Step) -> None:
        """Park the run at ``step`` while a pause is requested.

        The step reads PAUSED for the duration -- whether it was interrupted
        (a MOVE) or simply has not started yet -- so the console's poll sees
        where the run is holding; the gateway derives the task-level PAUSED
        from exactly this. A workflow cancel during the hold surfaces here as
        asyncio.CancelledError and the caller's except branch marks the step
        CANCELED, the one path on which that branch is reachable.
        """
        if not self._pause_requested:
            return
        step.status = StepStatus.PAUSED
        await workflow.wait_condition(lambda: not self._pause_requested)
        step.status = StepStatus.IN_PROGRESS

    @workflow.run
    async def run(self, task: WorkflowTask):
        self._steps = task.definition.steps
        own_map = _own_map()

        activity_map = {
            StepType.MOVE: RobotActivities.execute_move,
            StepType.STANDUP: RobotActivities.execute_stand,
            StepType.LIEDOWN: RobotActivities.execute_lie_down,
            StepType.SPEAK: RobotActivities.execute_speak,
        }

        for step in self._steps:
            # WAIT is the one step that is not an activity: it is a timer in
            # the workflow itself (see _run_wait), so it has no entry here.
            activity_fn = activity_map.get(step.type)
            if activity_fn is None and step.type is not StepType.WAIT:
                step.status = StepStatus.FAILED
                step.error_msg = f"Unknown step type: {step.type}"
                raise ApplicationError(
                    f"Unknown step type: {step.type}", non_retryable=True
                )

            # The posture activities (STANDUP/LIEDOWN) take no argument, so
            # they must be invoked with an empty arg list -- handing them a
            # None would fail the worker's argument-count check. The schema
            # guarantees params is None for exactly those step types.
            args = [] if step.params is None else [step.params]
            # The map rides as a second argument to MOVE only, and only when
            # there is one: execute_move defaults it to None, so a run that
            # started under an older worker (one argument in its history) and
            # a run without the attribute look the same to it.
            if step.type is StepType.MOVE and own_map is not None:
                args.append(own_map)

            # SPEAK cannot heartbeat: execute_speak sits in a single blocking
            # gateway call -- one HTTP request to the speech service, held open
            # for the whole utterance by `wait=true` -- so the
            # MOVE_HEARTBEAT_TIMEOUT below would kill every attempt before its
            # first heartbeat could ever arrive. Dead-worker detection for
            # SPEAK therefore falls to start_to_close alone -- which is also
            # why it gets a much shorter one than the heartbeating activities:
            # a worst-case utterance (1000 chars at 0.5x speed, plus the
            # one-time model load) is minutes, and an hour of a dead worker
            # silently holding the step would defeat the point of the timeout.
            if step.type is StepType.SPEAK:
                start_to_close_timeout = timedelta(minutes=5)
                heartbeat_timeout = None
            else:
                start_to_close_timeout = timedelta(hours=1)
                heartbeat_timeout = MOVE_HEARTBEAT_TIMEOUT

            activity_options = dict(
                args=args,
                # Without an explicit policy Temporal retries forever
                # (maximum_attempts=0). The activities mark aborted and
                # rejected moves retryable because those are sometimes
                # transient -- but against unlimited attempts, a
                # permanently blocked MOVE re-dispatched every backoff
                # interval kept this run open forever: the step showed
                # IN_PROGRESS for good, and ScheduleOverlapPolicy.SKIP
                # silently dropped every later trigger of the schedule.
                # Three attempts keeps the self-healing for the transient
                # cases and turns the persistent ones into a visible
                # FAILED step. The 5s initial interval is so a MOVE's
                # second try isn't 1s after the first (the default) --
                # too soon for e.g. a restarting action server to be back.
                retry_policy=RetryPolicy(
                    initial_interval=timedelta(seconds=5),
                    maximum_attempts=3,
                ),
                # Per-attempt ceiling. This used to be minutes=3600 -- 60
                # hours, a units slip (the intent was one hour), so it
                # could never fire. A dead worker is caught by the 3s
                # heartbeat regardless; this bounds the live-but-stuck
                # case where an attempt heartbeats forever without ever
                # reaching a terminal state. (Both values are picked per
                # step type above -- SPEAK cannot heartbeat.)
                start_to_close_timeout=start_to_close_timeout,
                heartbeat_timeout=heartbeat_timeout,
                cancellation_type=(
                    workflow.ActivityCancellationType.WAIT_CANCELLATION_COMPLETED
                ),
            )

            # A pause that landed before this step (during the previous one,
            # or between the two) holds here, before anything is dispatched.
            # This is the whole of the hold for SPEAK and the posture steps:
            # they are never interrupted, they finish and the run stops at
            # the next boundary. A MOVE is cut short and a WAIT's countdown
            # frozen, below.
            try:
                await self._hold(step)
            except asyncio.CancelledError:
                step.status = StepStatus.CANCELED
                step.error_msg = "Task canceled"
                raise

            step.status = StepStatus.IN_PROGRESS
            try:
                if step.type is StepType.WAIT:
                    await self._run_wait(step, step.params)
                    result = None
                elif step.type is StepType.MOVE:
                    result = await self._run_move(step, activity_fn, activity_options)
                else:
                    result = await workflow.execute_activity(activity_fn, **activity_options)
            except asyncio.CancelledError:
                step.status = StepStatus.CANCELED
                step.error_msg = "Task canceled"
                raise
            except ActivityError as err:
                step.status = StepStatus.FAILED
                step.error_msg = str(err.cause or err)
                raise

            if result is not None and not result.success:
                step.status = StepStatus.FAILED
                step.error_msg = "activity failed"
                raise ApplicationError("activity failed", non_retryable=True)

            step.status = StepStatus.COMPLETED

        return

    async def _run_move(self, step: Step, activity_fn, activity_options: dict) -> ActivityResult:
        """Run a MOVE, letting a pause interrupt it and a resume re-send it.

        Awaiting the handle is exactly what ``execute_activity`` does, so a
        task cancel behaves as it always has: the activity is cancelled, the
        nav goal with it, and the attempt surfaces as
        ActivityError(cause=CancelledError) -> FAILED / "Cancelled". The pause
        signal cancels the same handle, so its interruption arrives in the
        same shape; ``_move_interrupted`` is what tells the two apart (with
        ``_cancel_requested`` as the tie-breaker for the pause-then-cancel
        race). On resume the step is dispatched again from scratch -- same
        target, fresh attempts, from wherever the robot is now.
        """
        while True:
            self._move_handle = workflow.start_activity(activity_fn, **activity_options)
            try:
                return await self._move_handle
            except ActivityError as err:
                paused = (
                    self._move_interrupted
                    and isinstance(err.cause, CancelledError)
                    and not _cancel_requested()
                )
                if not paused:
                    raise
            finally:
                self._move_handle = None
                self._move_interrupted = False

            # A resume that beat the cancellation's completion makes this a
            # no-op and the goal is simply re-sent.
            await self._hold(step)

    async def _run_wait(self, step: Step, params: WaitParams) -> None:
        """Wait ``params.seconds``, with the countdown frozen while held.

        A durable timer rather than a ``time.sleep`` activity, on purpose: it
        occupies no slot of the single-thread activity executor, needs no
        heartbeat, and survives a worker restart -- the server owns the timer,
        so a backend restarted mid-wait picks the step up with the time already
        served. A pause interrupts it at once (no activity to cancel, the
        wait_condition simply returns) and the resume waits out only what was
        left, measured on ``workflow.now()`` so replay sees the same numbers. A
        task cancel lands as asyncio.CancelledError and the caller marks the
        step CANCELED / "Task canceled", like a cancel during a hold.
        """
        remaining = timedelta(seconds=params.seconds)
        while remaining > timedelta(0):
            started = workflow.now()
            try:
                await workflow.wait_condition(
                    lambda: self._pause_requested, timeout=remaining
                )
            except asyncio.TimeoutError:
                return
            remaining -= workflow.now() - started
            await self._hold(step)
