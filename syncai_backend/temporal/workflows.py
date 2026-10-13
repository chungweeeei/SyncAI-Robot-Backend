import asyncio
from datetime import timedelta

from temporalio import workflow
from temporalio.common import RetryPolicy
from temporalio.exceptions import ActivityError, ApplicationError, CancelledError

with workflow.unsafe.imports_passed_through():
    from syncai_backend.gateways.workflow.config import (
        PAUSE_SIGNAL,
        RESUME_SIGNAL,
        io_task_queue_for,
    )
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

# Heartbeat window for SPEAK. execute_speak heartbeats once per read of the
# speech job, but not while the POST that queues it is out -- the service
# synthesises inline, and a cold one loads its model first -- so this has to
# outlast the gateway's SPEAK_START_TIMEOUT_S (test_activities.py checks it).
# A cancel is not slowed by a long window: it rides the next heartbeat's
# reply, and the worker's HEARTBEAT_THROTTLE_MAX is what paces those.
SPEAK_HEARTBEAT_TIMEOUT = timedelta(seconds=75)

# SPEAK moved from the body queue to the IO queue when steps could run side by
# side (Step.group). Behind a patch so a run that scheduled a SPEAK on the body
# queue before the upgrade replays the same command after it.
SPEAK_ON_IO_QUEUE_PATCH = "speak-on-io-queue"

ACTIVITY_MAP = {
    StepType.MOVE: RobotActivities.execute_move,
    StepType.STANDUP: RobotActivities.execute_stand,
    StepType.LIEDOWN: RobotActivities.execute_lie_down,
    StepType.SPEAK: RobotActivities.execute_speak,
}


def _blocks(steps: list[Step]) -> list[list[Step]]:
    """``steps`` cut into what runs together: a group's adjacent members, or
    one ungrouped step on its own."""
    blocks: list[list[Step]] = []
    for step in steps:
        if blocks and step.group is not None and blocks[-1][-1].group == step.group:
            blocks[-1].append(step)
        else:
            blocks.append([step])
    return blocks


def _ended_badly(task: asyncio.Future) -> bool:
    return task.done() and (task.cancelled() or task.exception() is not None)


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


@workflow.defn
class RobotWorkflow:
    def __init__(self) -> None:
        self._steps: list[Step] = []
        # The hold (POST /api/v1/tasks/{id}/pause). A flag rather than an
        # event because both signals are idempotent: a second pause while
        # held, or a resume while running, must change nothing.
        self._pause_requested: bool = False

    @workflow.query
    def get_step_states(self) -> list[Step]:
        return self._steps

    # The handlers only flip the flag. What a pause interrupts is decided by
    # the code waiting on it: _run_move cancels an in-flight MOVE, _run_wait
    # freezes its countdown, and every other step is waited out (see run()).
    @workflow.signal(name=PAUSE_SIGNAL)
    def pause(self) -> None:
        self._pause_requested = True

    @workflow.signal(name=RESUME_SIGNAL)
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

        # Block by block. A lone step is awaited directly -- exactly the
        # commands the loop issued before groups existed, so those histories
        # replay unchanged; a group's members start together (_run_group).
        for block in _blocks(self._steps):
            if len(block) == 1:
                await self._run_step(block[0], own_map)
            else:
                await self._run_group(block, own_map)

    async def _run_group(self, steps: list[Step], own_map: str | None) -> None:
        """Run ``steps`` side by side and return when all have finished.

        Fail-fast: the first member to fail cancels the rest -- each lands in
        _run_step's own CancelledError branch, so a MOVE stops the robot and a
        SPEAK stops talking the way a task cancel does -- and once they have
        finished the failure is raised as the run's, as a lone step's would
        be. The siblings read
        CANCELED with the failed step named, not "Task canceled", because
        nobody canceled the task.

        A pause reaches each member on its own terms (a MOVE is interrupted,
        a WAIT frozen, SPEAK and the postures finish) and the run holds before
        the next block as usual. A cancel of the run surfaces here, from the
        wait, and is passed on to every member before it is re-raised.
        """
        tasks = [asyncio.ensure_future(self._run_step(step, own_map)) for step in steps]
        try:
            await workflow.wait_condition(
                lambda: all(t.done() for t in tasks) or any(_ended_badly(t) for t in tasks)
            )
        except asyncio.CancelledError:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            raise

        failed = next((i for i, t in enumerate(tasks) if _ended_badly(t)), None)
        if failed is None:
            return

        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        for i, step in enumerate(steps):
            if i != failed and step.status is StepStatus.CANCELED:
                step.error_msg = f"Canceled: step {steps[failed].id} failed"
        raise tasks[failed].exception()

    async def _run_step(self, step: Step, own_map: str | None) -> None:
        # WAIT is the one step that is not an activity: it is a timer in
        # the workflow itself (see _run_wait), so it has no entry here.
        activity_fn = ACTIVITY_MAP.get(step.type)
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

        # SPEAK's heartbeat window is wider than MOVE's (see
        # SPEAK_HEARTBEAT_TIMEOUT: the POST that queues it cannot beat), and
        # its start_to_close much shorter than the hour the others get: a
        # worst-case utterance (1000 chars at 0.5x speed, plus the one-time
        # model load) is minutes, and the gateway gives up on a backed-up
        # speech queue at 240 s with a sentence -- before this fires.
        if step.type is StepType.SPEAK:
            # Per utterance: a SPEAK says its text `repeat` times in one
            # attempt. Older histories carry no repeat and read 1, so they
            # schedule the same five minutes they always did.
            start_to_close_timeout = timedelta(minutes=5) * step.params.repeat
            heartbeat_timeout = SPEAK_HEARTBEAT_TIMEOUT
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
            # could never fire. A dead worker is caught by the
            # heartbeat regardless; this bounds the live-but-stuck
            # case where an attempt heartbeats forever without ever
            # reaching a terminal state. (Both values are picked per
            # step type above.)
            start_to_close_timeout=start_to_close_timeout,
            heartbeat_timeout=heartbeat_timeout,
            cancellation_type=(
                workflow.ActivityCancellationType.WAIT_CANCELLATION_COMPLETED
            ),
        )
        # SPEAK only talks to the speech service, so it goes to the IO queue,
        # whose worker takes several at once; on the body queue a SPEAK grouped
        # with a MOVE would wait for the MOVE's one slot. Unset, an activity
        # goes to the run's own queue -- the body queue.
        if step.type is StepType.SPEAK and workflow.patched(SPEAK_ON_IO_QUEUE_PATCH):
            activity_options["task_queue"] = io_task_queue_for(workflow.info().task_queue)

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
            if isinstance(err.cause, CancelledError):
                # A cancel of the whole run that reached us through the
                # activity: execute_activity (SPEAK, the posture steps)
                # answers the run's cancel by cancelling the activity and
                # handing back its resolution, so the run's own
                # asyncio.CancelledError never surfaces. A cancel is the
                # only thing that resolves an activity with this cause
                # (timeouts are TimeoutError), so it is named for what it
                # is -- the same CANCELED a MOVE, a WAIT or a hold ends in.
                step.status = StepStatus.CANCELED
                step.error_msg = "Task canceled"
                raise
            step.status = StepStatus.FAILED
            step.error_msg = str(err.cause or err)
            raise

        if result is not None and not result.success:
            step.status = StepStatus.FAILED
            step.error_msg = "activity failed"
            raise ApplicationError("activity failed", non_retryable=True)

        step.status = StepStatus.COMPLETED

    async def _run_move(self, step: Step, activity_fn, activity_options: dict) -> ActivityResult:
        """Run a MOVE, letting a pause interrupt it and a resume re-send it.

        Never ``await handle`` directly here. Awaiting the activity task is what
        lets a cancel of the whole run be swallowed by the activity's own
        resolution: asyncio cancels the awaited task, the activity answers with
        ActivityError(cause=CancelledError), and the run's CancelledError is
        spent -- indistinguishable, from inside this method, from the
        cancellation a pause asked for. Waiting on a condition instead keeps
        the run's cancel addressed to the run: it always arrives as
        asyncio.CancelledError out of ``wait_condition``, so an
        ActivityError(cause=CancelledError) seen below can only be the
        pause's own, and no SDK help (``workflow.cancellation_reason``) is
        needed to tell the two apart. start_activity + wait_condition issues
        exactly the command execute_activity did, so histories recorded before
        the hold existed replay unchanged.

        On resume the step is dispatched again from scratch -- same target,
        fresh attempts, from wherever the robot is now.
        """
        while True:
            handle = workflow.start_activity(activity_fn, **activity_options)
            interrupted = False
            try:
                await workflow.wait_condition(
                    lambda: handle.done() or self._pause_requested
                )
                if not handle.done():
                    # The pause: cancelling the handle lands in
                    # execute_move's ``except CancelledError``, which cancels
                    # the nav2 goal -- the same path a task cancel takes.
                    # WAIT_CANCELLATION_COMPLETED, so wait for it to finish.
                    interrupted = True
                    handle.cancel()
                    await workflow.wait_condition(handle.done)
            except asyncio.CancelledError:
                # The whole run is being cancelled with a MOVE in flight. The
                # activity has to be cancelled too and allowed to finish its
                # cleanup (the robot stops) before the run closes, which is
                # what execute_activity guaranteed. The run's cancel has been
                # consumed by this except, so this await sees only the
                # activity's outcome; the caller marks the step CANCELED.
                handle.cancel()
                try:
                    await handle
                except ActivityError:
                    pass
                raise

            try:
                return handle.result()
            except ActivityError as err:
                # A real failure that raced the pause, or any failure with no
                # pause at all, is the step's to report.
                if not (interrupted and isinstance(err.cause, CancelledError)):
                    raise
            # Either it ran to the end in the gap between the pause and the
            # cancel landing (returned above: the result stands and the hold
            # happens before the next step), or it was interrupted and holds
            # here. A resume that beat the cancellation's completion makes this
            # a no-op and the goal is simply re-sent.
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
