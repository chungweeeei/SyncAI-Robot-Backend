import asyncio
from datetime import timedelta

from temporalio import workflow
from temporalio.common import RetryPolicy
from temporalio.exceptions import ActivityError, ApplicationError, CancelledError

with workflow.unsafe.imports_passed_through():
    from syncai_backend.tasks.config import (
        PATROL_PROGRESS_SIGNAL,
        PAUSE_SIGNAL,
        RESUME_SIGNAL,
        STEP_STARTED_SIGNAL,
        group_activity_id,
        io_task_queue_for,
    )
    from syncai_backend.tasks.schema import (
        NAV_STEP_TYPES,
        POSE_STEP_TYPES,
        Step,
        StepStatus,
        StepType,
        TaskKind,
        TaskRecord,
        WaitParams,
        WorkflowTask,
    )
    from syncai_backend.tasks.search_attributes import (
        SCHEDULED_BY_KEY,
        TASK_KIND_KEY,
        TASK_MAP_KEY,
        TASK_NAME_KEY,
    )
    from syncai_backend.temporal.activities import ActivityResult, RobotActivities


# Heartbeat window for MOVE and the postures. The only unbeaten stretch is the goal
# send, bounded by NAV_GOAL_SEND_BUDGET_S; test_activities.py keeps this above it.
MOVE_HEARTBEAT_TIMEOUT = timedelta(seconds=3)

# Heartbeat window for SPEAK. The POST that queues it (inline synthesis, maybe a cold
# model load) cannot beat, so this outlasts SPEAK_START_TIMEOUT_S (test_activities.py).
# A long window does not slow a cancel: that rides the next heartbeat's reply.
SPEAK_HEARTBEAT_TIMEOUT = timedelta(seconds=75)

# SPEAK moved to the IO queue with Step.group; patched so older runs replay.
SPEAK_ON_IO_QUEUE_PATCH = "speak-on-io-queue"

# Group members used to be scheduled all at once; now in list order (_run_group).
# Patched so older runs replay.
GROUP_START_IN_ORDER_PATCH = "group-start-in-order"

# Every run writes a task_record row as it closes (record_task); patched so runs from
# before it replay.
TASK_RECORD_PATCH = "task-record"

# record_task is one insert. Bounded from scheduling, not only from start: with no IO
# worker polling it would otherwise hold the run open, and the report is not worth that.
RECORD_TASK_TIMEOUT = timedelta(minutes=2)

ACTIVITY_MAP = {
    StepType.MOVE: RobotActivities.execute_move,
    StepType.STANDUP: RobotActivities.execute_stand,
    StepType.LIEDOWN: RobotActivities.execute_lie_down,
    StepType.SPEAK: RobotActivities.execute_speak,
    StepType.DOCK: RobotActivities.execute_dock,
    StepType.UNDOCK: RobotActivities.execute_undock,
    StepType.PATROL: RobotActivities.execute_patrol,
}


def _blocks(steps: list[Step]) -> list[list[Step]]:
    """``steps`` cut into blocks: a group's adjacent members, or one lone step."""
    blocks: list[list[Step]] = []
    for step in steps:
        if blocks and step.group is not None and blocks[-1][-1].group == step.group:
            blocks[-1].append(step)
        else:
            blocks.append([step])
    return blocks


def _ended_badly(task: asyncio.Future) -> bool:
    return task.done() and (task.cancelled() or task.exception() is not None)


def _is_cancel(err: BaseException) -> bool:
    """Whether ``err`` is the run's cancel, in either shape it reaches ``run`` in.

    A MOVE or WAIT re-raises asyncio.CancelledError; a posture or SPEAK hands back
    the activity's ActivityError(cause=CancelledError) (see _run_step).
    """
    return isinstance(err, asyncio.CancelledError) or (
        isinstance(err, ActivityError) and isinstance(err.cause, CancelledError)
    )


def _end(step: Step, status: StepStatus, error_msg: str | None = None) -> None:
    """Close ``step`` as ``status``, stamping when it ended."""
    step.status = status
    if error_msg is not None:
        step.error_msg = error_msg
    step.ended_at = workflow.now()


def _attribute(key):
    """One of this run's own search attributes, or None. Never fatal."""
    try:
        return workflow.info().typed_search_attributes.get(key) or None
    except Exception:
        return None


def _own_map() -> str | None:
    """The map this run's coordinates are in, off its own ``TaskMap``.

    Not from the argument: a scheduled run's argument was frozen at registration
    and carries no map. A run without the attribute skips the map check.
    """
    return _attribute(TASK_MAP_KEY)


def _own_kind() -> TaskKind | None:
    """How this run was started; a scheduled run without ``TaskKind`` is SCHEDULE,
    as the history reads it (WorkflowGateway's _provenance_of)."""
    raw_kind = _attribute(TASK_KIND_KEY)
    try:
        kind = TaskKind(raw_kind) if raw_kind else None
    except ValueError:
        kind = None
    if kind is None and _attribute(SCHEDULED_BY_KEY):
        kind = TaskKind.SCHEDULE
    return kind


@workflow.defn
class RobotWorkflow:
    def __init__(self) -> None:
        self._steps: list[Step] = []
        # The hold (POST /api/v1/tasks/{id}/pause): a flag, since both signals are
        # idempotent.
        self._pause_requested: bool = False
        # For a group's start order: each member's last dispatched activity id, and
        # the ids that reported STEP_STARTED_SIGNAL.
        self._dispatches: int = 0
        self._dispatch_ids: dict[str, str] = {}
        self._started: set[str] = set()
        # Per PATROL step, how many of its waypoints the robot has passed
        # (PATROL_PROGRESS_SIGNAL); a re-dispatch after a pause starts past them.
        self._patrol_passed: dict[str, int] = {}

    @workflow.query
    def get_step_states(self) -> list[Step]:
        return self._steps

    # The handlers only flip the flag; the waiting code decides what a pause
    # interrupts (_run_move cancels, _run_wait freezes, the rest finish).
    @workflow.signal(name=PAUSE_SIGNAL)
    def pause(self) -> None:
        self._pause_requested = True

    @workflow.signal(name=RESUME_SIGNAL)
    def resume(self) -> None:
        self._pause_requested = False

    @workflow.signal(name=STEP_STARTED_SIGNAL)
    def step_started(self, activity_id: str) -> None:
        self._started.add(activity_id)

    @workflow.signal(name=PATROL_PROGRESS_SIGNAL)
    def patrol_progress(self, step_id: str, passed: int) -> None:
        # Only ever forward: a late report from a cancelled dispatch is no news.
        self._patrol_passed[step_id] = max(self._patrol_passed.get(step_id, 0), passed)

    def _new_dispatch_id(self, step: Step) -> str:
        self._dispatches += 1
        activity_id = group_activity_id(step.id, self._dispatches)
        self._dispatch_ids[step.id] = activity_id
        return activity_id

    def _has_started(self, step: Step) -> bool:
        """Whether a group member is far enough along for the next to go.

        A WAIT once its timer runs; a MOVE or SPEAK once it reports
        STEP_STARTED_SIGNAL under its latest dispatch id (a MOVE re-sent after a
        pause reports again). Postures never report; _run_group checks for ended.
        """
        if step.type is StepType.WAIT:
            return step.status is StepStatus.IN_PROGRESS
        return self._dispatch_ids.get(step.id) in self._started

    async def _hold(self, step: Step) -> None:
        """Park the run at ``step`` while a pause is requested.

        The step reads PAUSED meanwhile, which is what the gateway derives the
        task-level PAUSED from. A workflow cancel here surfaces as
        asyncio.CancelledError; the caller marks the step CANCELED.
        """
        if not self._pause_requested:
            return
        step.status = StepStatus.PAUSED
        held_from = workflow.now()
        try:
            await workflow.wait_condition(lambda: not self._pause_requested)
        finally:
            # Only a hold the step was under way for is its own; one before it
            # started is the run's, not this step's.
            if step.started_at is not None:
                step.paused_seconds += (workflow.now() - held_from).total_seconds()
        step.status = StepStatus.IN_PROGRESS

    @workflow.run
    async def run(self, task: WorkflowTask):
        self._steps = task.definition.steps
        own_map = _own_map()

        # Not BaseException: the SDK evicts a cached run by raising one through it,
        # and that must not schedule anything.
        try:
            # A lone step is awaited directly, so pre-group histories replay unchanged.
            for block in _blocks(self._steps):
                if len(block) == 1:
                    await self._run_step(block[0], own_map)
                else:
                    await self._run_group(block, own_map)
        except (asyncio.CancelledError, Exception) as err:
            await self._record("CANCELED" if _is_cancel(err) else "FAILED", own_map)
            raise
        await self._record("COMPLETED", own_map)

    async def _record(self, status: str, own_map: str | None) -> None:
        """Leave the run's ``task_record`` row (the report export's source).

        On the IO queue: one insert, nothing to do with the body. Runs after a
        cancel too: the run's cancel has been consumed by the time it arrives here,
        so this activity is not cancelled with it. Never changes how the run ends:
        a failure is logged and dropped.
        """
        if not workflow.patched(TASK_RECORD_PATCH):
            return
        info = workflow.info()
        record = TaskRecord(
            task_id=info.workflow_id,
            run_id=info.run_id,
            status=status,
            started_at=info.start_time,
            closed_at=workflow.now(),
            kind=_own_kind(),
            name=_attribute(TASK_NAME_KEY),
            map_name=own_map,
            schedule_id=_attribute(SCHEDULED_BY_KEY),
            steps=self._steps,
        )
        try:
            await workflow.execute_activity(
                RobotActivities.record_task,
                record,
                task_queue=io_task_queue_for(info.task_queue),
                schedule_to_close_timeout=RECORD_TASK_TIMEOUT,
                retry_policy=RetryPolicy(
                    initial_interval=timedelta(seconds=5),
                    maximum_attempts=3,
                ),
            )
        except ActivityError as err:
            workflow.logger.warning(
                "Could not record the run; it is missing from the report export: %s",
                err.cause or err,
            )

    async def _run_group(self, steps: list[Step], own_map: str | None) -> None:
        """Run ``steps`` side by side and return when all have finished.

        They start in list order, each once the one before has started
        (_has_started) or ended; scheduled at once, two SPEAKs raced through
        synthesis and played in either order. A member held by a pause holds back
        those behind it. A member never started when the group ends stays PENDING.

        Fail-fast: the first failure cancels the rest (each through _run_step's
        CancelledError branch, so the robot stops and speech stops), then is
        raised as the run's. Siblings read CANCELED naming the failed step, not
        "Task canceled".

        A pause reaches each member on its own terms and the run holds before the
        next block. A run cancel is passed on to every member, then re-raised.
        """
        tasks: list[asyncio.Future] = []
        try:
            if workflow.patched(GROUP_START_IN_ORDER_PATCH):
                for step in steps:
                    task = asyncio.ensure_future(
                        self._run_step(step, own_map, report_start=True)
                    )
                    tasks.append(task)
                    await workflow.wait_condition(
                        lambda step=step, task=task: (
                            task.done()
                            or self._has_started(step)
                            or any(_ended_badly(t) for t in tasks)
                        )
                    )
                    if any(_ended_badly(t) for t in tasks):
                        break
            else:
                tasks = [
                    asyncio.ensure_future(self._run_step(step, own_map)) for step in steps
                ]
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

    async def _run_step(
        self, step: Step, own_map: str | None, report_start: bool = False
    ) -> None:
        # WAIT is a workflow timer (_run_wait), not an activity.
        activity_fn = ACTIVITY_MAP.get(step.type)
        if activity_fn is None and step.type is not StepType.WAIT:
            _end(step, StepStatus.FAILED, f"Unknown step type: {step.type}")
            raise ApplicationError(
                f"Unknown step type: {step.type}", non_retryable=True
            )

        # Postures take no argument; passing None would fail the arg-count check.
        args = [] if step.params is None else [step.params]
        # MOVE / DOCK get the map as a second argument only when there is one, so old
        # one-argument histories and runs without the attribute look the same. UNDOCK
        # has no pose to check against it and takes no argument at all.
        if step.type in POSE_STEP_TYPES and own_map is not None:
            args.append(own_map)
        # PATROL is new, so no old history constrains it: always the map (None
        # for a run without one), its step id, and the waypoints already passed,
        # which _run_move refreshes on every dispatch.
        if step.type is StepType.PATROL:
            args = [step.params, own_map, step.id, 0]

        # SPEAK: wider heartbeat window (SPEAK_HEARTBEAT_TIMEOUT) and a much shorter
        # start_to_close than the hour others get; the gateway gives up on a backed-up
        # speech queue at 240 s, before this fires.
        if step.type is StepType.SPEAK:
            # Per utterance; older histories read repeat=1.
            start_to_close_timeout = timedelta(minutes=5) * step.params.repeat
            heartbeat_timeout = SPEAK_HEARTBEAT_TIMEOUT
        else:
            start_to_close_timeout = timedelta(hours=1)
            heartbeat_timeout = MOVE_HEARTBEAT_TIMEOUT

        activity_options = dict(
            args=args,
            # Without a policy Temporal retries forever, and a permanently blocked
            # MOVE kept the run open (SKIP then dropped every later schedule
            # trigger). Three attempts, 5 s apart so a restarting server is back.
            retry_policy=RetryPolicy(
                initial_interval=timedelta(seconds=5),
                maximum_attempts=3,
            ),
            # Per-attempt ceiling for a live-but-stuck attempt that heartbeats
            # forever; a dead worker is caught by the heartbeat.
            start_to_close_timeout=start_to_close_timeout,
            heartbeat_timeout=heartbeat_timeout,
            cancellation_type=(
                workflow.ActivityCancellationType.WAIT_CANCELLATION_COMPLETED
            ),
        )
        # SPEAK goes to the IO queue so a grouped SPEAK need not wait for the MOVE's
        # body slot. Unset, an activity goes to the run's own (body) queue.
        if step.type is StepType.SPEAK and workflow.patched(SPEAK_ON_IO_QUEUE_PATCH):
            activity_options["task_queue"] = io_task_queue_for(workflow.info().task_queue)

        # Only a group member gets our own id (what makes it report and what
        # _has_started matches); a lone step keeps Temporal's, so its commands are
        # unchanged.
        new_activity_id = (lambda: self._new_dispatch_id(step)) if report_start else None

        # A pause that landed before this step holds here, before dispatch. That is
        # the whole hold for SPEAK and postures; MOVE / DOCK / UNDOCK and WAIT are
        # interrupted below.
        try:
            await self._hold(step)
        except asyncio.CancelledError:
            _end(step, StepStatus.CANCELED, "Task canceled")
            raise

        step.status = StepStatus.IN_PROGRESS
        step.started_at = workflow.now()
        try:
            if step.type is StepType.WAIT:
                await self._run_wait(step, step.params)
                result = None
            elif step.type in NAV_STEP_TYPES:
                result = await self._run_move(
                    step, activity_fn, activity_options, new_activity_id
                )
            else:
                if new_activity_id is not None:
                    activity_options["activity_id"] = new_activity_id()
                result = await workflow.execute_activity(activity_fn, **activity_options)
        except asyncio.CancelledError:
            _end(step, StepStatus.CANCELED, "Task canceled")
            raise
        except ActivityError as err:
            if isinstance(err.cause, CancelledError):
                # A run cancel arriving through execute_activity, which hands back
                # the activity's resolution instead of the run's CancelledError.
                # Only a cancel has this cause (timeouts are TimeoutError).
                _end(step, StepStatus.CANCELED, "Task canceled")
                raise
            _end(step, StepStatus.FAILED, str(err.cause or err))
            raise

        if result is not None and not result.success:
            _end(step, StepStatus.FAILED, "activity failed")
            raise ApplicationError("activity failed", non_retryable=True)

        _end(step, StepStatus.COMPLETED)

    async def _run_move(
        self, step: Step, activity_fn, activity_options: dict, new_activity_id=None
    ) -> ActivityResult:
        """Run a MOVE / DOCK / UNDOCK, letting a pause interrupt it and a resume
        re-send it (a dock from its staging leg; an undock backs out a further
        metre from wherever the pause stopped it, since its goal has no pose).

        Never ``await handle`` directly: that lets a run cancel be swallowed into
        ActivityError(cause=CancelledError), indistinguishable from the pause's own.
        Waiting on a condition keeps the run's cancel arriving as
        asyncio.CancelledError, so no ``workflow.cancellation_reason`` is needed.
        start_activity + wait_condition issues the same command execute_activity
        did, so older histories replay unchanged.

        On resume the step is re-sent from scratch, and in a group under a fresh
        activity id so the members behind wait for the new goal's acceptance --
        except a PATROL, which is re-sent past the waypoints it reported passed
        (patrol_progress), so a pause does not drive it round from the first.
        """
        while True:
            if new_activity_id is not None:
                activity_options["activity_id"] = new_activity_id()
            if step.type is StepType.PATROL:
                args = activity_options["args"]
                activity_options["args"] = [
                    *args[:3],
                    self._patrol_passed.get(step.id, 0),
                ]
            handle = workflow.start_activity(activity_fn, **activity_options)
            interrupted = False
            try:
                await workflow.wait_condition(
                    lambda: handle.done() or self._pause_requested
                )
                if not handle.done():
                    # The pause: lands in execute_move's except CancelledError, which
                    # cancels the nav2 goal. WAIT_CANCELLATION_COMPLETED: wait for it.
                    interrupted = True
                    handle.cancel()
                    await workflow.wait_condition(handle.done)
            except asyncio.CancelledError:
                # Run cancel with a MOVE in flight: cancel it and let its cleanup
                # finish (the robot stops) before the run closes. The run's cancel
                # is consumed here, so this await sees only the activity's outcome.
                handle.cancel()
                try:
                    await handle
                except ActivityError:
                    pass
                raise

            try:
                return handle.result()
            except ActivityError as err:
                # A real failure, even one racing the pause, is the step's.
                if not (interrupted and isinstance(err.cause, CancelledError)):
                    raise
            # Interrupted: hold here (a no-op if the resume already came), then re-send.
            await self._hold(step)

    async def _run_wait(self, step: Step, params: WaitParams) -> None:
        """Wait ``params.seconds``, with the countdown frozen while held.

        A durable timer, not a sleep activity: no executor slot, no heartbeat, and
        it survives a worker restart with the time served. A pause returns the
        wait_condition at once; the remainder is measured on ``workflow.now()`` so
        replay sees the same numbers. A task cancel lands as asyncio.CancelledError.
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
