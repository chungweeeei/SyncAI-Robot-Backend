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
        needs_localization,
        step_blocks,
    )
    from syncai_backend.tasks.search_attributes import (
        SCHEDULED_BY_KEY,
        TASK_KIND_KEY,
        TASK_MAP_KEY,
        TASK_NAME_KEY,
    )
    from syncai_backend.tasks.task_admission import (
        TaskAdmission,
        TaskAdmissionRequest,
        TaskEnd,
        TaskOutcome,
    )
    from syncai_backend.tasks.template_resolution import ResolvedTemplate
    from syncai_backend.helpers.task_guard import (
        ROBOT_STATE_REFUSED,
        ROBOT_STATE_UNREACHABLE,
        UNREACHABLE_MESSAGE,
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

# A schedule registered from a template reads it as each run starts
# (_resolve_template); patched so runs from before it replay on their snapshot.
SCHEDULE_RESOLVES_TEMPLATE_PATCH = "schedule-resolves-template"

# resolve_template is two reads. Bounded from scheduling like record_task: with no
# IO worker polling, the run goes on its snapshot rather than waiting.
RESOLVE_TEMPLATE_TIMEOUT = timedelta(seconds=30)

# Every run asks the robot whether it may start (begin_task) and tells it how
# it ended (end_task); syncai_robot_state is the judge, 2026-11. One patch for
# both: it is read once, where begin_task goes, and end_task follows the
# answer (RobotWorkflow._admitted) rather than a second patched() call, so a
# run from before it replays without either.
TASK_ADMISSION_PATCH = "task-admission"

# begin_task, per attempt: above the gateway's whole wait for the service and
# its answer (TASK_STATE_CALL_BUDGET_S, 5 s; test_activities.py pins the
# relation), so an attempt ends on the gateway's "no answer", which is
# retried, and not on a timeout that would leave the question out.
BEGIN_TASK_START_TO_CLOSE = timedelta(seconds=8)
# ...and from scheduling, for all of them: three attempts 2 s apart are 19 s
# of answering at worst, and with no IO worker polling the run fails closed
# here instead of waiting unstarted for as long as the outage lasts.
BEGIN_TASK_TIMEOUT = timedelta(seconds=30)
BEGIN_TASK_ATTEMPTS = 3

# end_task, from scheduling. Longer than begin's and with more attempts: a
# report that never lands leaves the robot counting a finished run as RUNNING,
# and every task after it refused robot_busy until an operator clears it. Five
# attempts 2 s apart fit (33 s at worst; test_activities.py pins it), and past
# this the run closes anyway: its outcome is not the robot's to hold open.
END_TASK_TIMEOUT = timedelta(seconds=60)
END_TASK_ATTEMPTS = 5

# The best-effort end_task(CANCELED) after a begin_task nobody answered
# (RobotWorkflow._release_unanswered_begin). begin's own budget, not
# end_task's: the likeliest reason for the silence is a robot whose state
# service is down, and a run that is already failing closed must not be held
# open a minute to tell a dead service about it. Three attempts 2 s apart are
# 19 s at worst (test_activities.py pins it).
RELEASE_TASK_TIMEOUT = timedelta(seconds=30)
RELEASE_TASK_ATTEMPTS = 3

# One retry interval for both, flat (backoff 1.0): the service is a lookup in
# another process on this host, so there is nothing a longer wait would let
# recover that 2 s would not, and the default doubling would spend begin's
# budget on waiting rather than asking.
TASK_STATE_RETRY_INTERVAL = timedelta(seconds=2)

# EndTask.reason is what the console shows while the robot is locked in ERROR:
# syncai_robot_state builds its state_message as "A task failed: <reason>. ..."
# and the console prints that verbatim. So it is one sentence (_failure_sentence)
# and, since a step's error can carry a planner's whole explanation, cut to
# this many characters.
END_TASK_REASON_MAX = 300

ACTIVITY_MAP = {
    StepType.MOVE: RobotActivities.execute_move,
    StepType.STANDUP: RobotActivities.execute_stand,
    StepType.LIEDOWN: RobotActivities.execute_lie_down,
    StepType.SPEAK: RobotActivities.execute_speak,
    StepType.CHARGE: RobotActivities.execute_charge,
    StepType.PATROL: RobotActivities.execute_patrol,
}


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


def _failure_sentence(err: BaseException | None, fallback: str = "") -> str:
    """Why the run failed, as a sentence an operator reads: EndTask.reason.

    The message of the ApplicationError behind ``err`` when there is one --
    the activity's own, under the ActivityError the workflow sees, or one the
    workflow raised itself. NOT ``str()`` of it, which is what a step's
    error_msg stores: that prefixes the error's type (``not_initialized: The
    robot's position is no longer confirmed...``, ``map_mismatch: ...``), a
    code where the robot's state_message quotes the reason mid-sentence.
    error_msg itself is left as it is: the history and the console's own
    rewriting of a failure read that string.

    ``fallback`` (the failed step's error_msg) when there is no such message:
    a timeout, a failure with no cause.
    """
    cause = err.cause if isinstance(err, ActivityError) and err.cause else err
    if isinstance(cause, ApplicationError) and cause.message:
        return cause.message
    return fallback or str(cause or "")


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
        # Whether the robot accepted this run (begin_task), and so counts it as
        # RUNNING until it is told otherwise: what end_task is owed on. False
        # for a refused run, one that got no answer, and one from before
        # TASK_ADMISSION_PATCH, none of which the robot is counting.
        self._admitted: bool = False

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
        # On the step too, for the query and the record: what a run continuing
        # this one, should it fail, starts the PATROL past.
        for step in self._steps:
            if step.id == step_id:
                step.waypoints_passed = self._patrol_passed[step_id]

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
        # A run continuing a failed one starts each PATROL past what it had passed;
        # every other run's steps read 0, so dispatches are unchanged.
        self._patrol_passed = {
            step.id: step.waypoints_passed
            for step in self._steps
            if step.waypoints_passed > 0
        }

        # Not BaseException: the SDK evicts a cached run by raising one through it,
        # and that must not schedule anything.
        try:
            if task.task_template_id and workflow.patched(
                SCHEDULE_RESOLVES_TEMPLATE_PATCH
            ):
                own_map, refusal = await self._resolve_template(
                    task.task_template_id, own_map
                )
                if refusal is not None:
                    _end(self._steps[0], StepStatus.FAILED, refusal)
                    raise ApplicationError(refusal, non_retryable=True)
            # After the template is read, never before: needs_localization is a
            # fact about the steps this run will execute, and a scheduled run
            # only has those once _resolve_template has replaced the snapshot.
            # Before the first step, always: this is the one gate a run a
            # Schedule started by itself ever meets.
            if workflow.patched(TASK_ADMISSION_PATCH):
                await self._begin_task()
            # A lone step is awaited directly, so pre-group histories replay unchanged.
            for block in step_blocks(self._steps):
                # What the failed run this one continues got past (continuation_steps
                # skips whole blocks). No run before SKIPPED carries one.
                if all(step.status is StepStatus.SKIPPED for step in block):
                    continue
                if len(block) == 1:
                    await self._run_step(block[0], own_map)
                else:
                    await self._run_group(block, own_map)
        except (asyncio.CancelledError, Exception) as err:
            canceled = _is_cancel(err)
            # The robot first, the report second: until end_task lands the
            # robot refuses every other run as robot_busy, and that must not
            # wait behind an insert's retries.
            await self._end_task(
                TaskOutcome.CANCELED if canceled else TaskOutcome.FAILED, err
            )
            await self._record("CANCELED" if canceled else "FAILED", own_map)
            raise
        await self._end_task(TaskOutcome.COMPLETED)
        await self._record("COMPLETED", own_map)

    async def _begin_task(self) -> None:
        """Ask the robot whether this run may start; return only if it may.

        syncai_robot_state decides and applies the decision in one place: an
        accepted run IS running there from that call on (so two runs cannot
        both be let through, and a schedule firing during another run hears
        robot_busy instead of interleaving its steps with it), and it must then
        be closed with end_task exactly once -- ``_admitted`` is that debt. A
        refused run owes nothing: a refusal changes nothing on the robot, and
        in particular latches no ERROR.

        On the IO queue: the body worker has one slot, and behind a running
        MOVE this question would wait out the MOVE instead of being answered.

        A refusal fails the run at once, non-retryably, with the robot's own
        sentence on the first step that would have run. So does no answer
        after the retries (fail closed): a run the robot does not know of is
        one nothing would refuse, count or stop. No answer is not "no": the
        request may have landed and only the answer been lost, so that case
        alone is followed by a best-effort end_task (_release_unanswered_begin)
        before the run ends as it was going to.

        Never ``await`` the handle, and never cancel it: by the time a run
        cancel arrives the question is already with the robot, which may have
        accepted it. The answer is waited for through the cancel, so the run
        knows whether it owes an end_task, and the cancel is then re-raised.
        """
        info = workflow.info()
        handle = workflow.start_activity(
            RobotActivities.begin_task,
            TaskAdmissionRequest(
                task_id=info.workflow_id,
                needs_localization=needs_localization(self._steps),
            ),
            task_queue=io_task_queue_for(info.task_queue),
            start_to_close_timeout=BEGIN_TASK_START_TO_CLOSE,
            schedule_to_close_timeout=BEGIN_TASK_TIMEOUT,
            retry_policy=RetryPolicy(
                initial_interval=TASK_STATE_RETRY_INTERVAL,
                backoff_coefficient=1.0,
                maximum_attempts=BEGIN_TASK_ATTEMPTS,
            ),
        )
        cancel = await self._outlast_cancel(handle)

        refusal: tuple[str, str] | None = None
        try:
            admission: TaskAdmission = handle.result()
        except ActivityError as err:
            workflow.logger.warning(
                "The robot did not answer begin_task in %d attempts; failing the "
                "run closed: %s",
                BEGIN_TASK_ATTEMPTS,
                err.cause or err,
            )
            refusal = (ROBOT_STATE_UNREACHABLE, UNREACHABLE_MESSAGE)
            # Before either way out below, the cancel's included: whether the
            # robot counts this run is unknown on both.
            await self._release_unanswered_begin()
        else:
            if admission.accepted:
                self._admitted = True
            else:
                refusal = (
                    admission.code or ROBOT_STATE_REFUSED,
                    admission.message or "The robot refused the task.",
                )

        if cancel is not None:
            # The operator's cancel is the run's outcome whatever the robot
            # answered; run()'s handler reports CANCELED if it had accepted,
            # and an unanswered begin was released just above.
            raise cancel
        if refusal is None:
            return

        code, message = refusal
        # Not steps[0]: in a run continuing a failed one it may be SKIPPED, and
        # a SKIPPED step marked FAILED would have the next Retry start from it
        # and run again what the first run had completed.
        first = next(
            (step for step in self._steps if step.status is not StepStatus.SKIPPED),
            None,
        )
        if first is not None:
            _end(first, StepStatus.FAILED, message)
        raise ApplicationError(message, type=code, non_retryable=True)

    async def _end_task(
        self, outcome: TaskOutcome, err: BaseException | None = None
    ) -> None:
        """Tell the robot how this run ended, if it had accepted it.

        Exactly once per accepted run, on every way out of ``run``: COMPLETED,
        FAILED (which latches ERROR on the robot until an operator releases it,
        with why the run failed as the reason shown meanwhile) or CANCELED
        (the operator's cancel and the safety lock's alike: neither is a
        fault). A paused run is still running and sends nothing.

        Its own activity on the IO queue, before record_task: see
        RobotActivities.end_task. Never changes how the run ends: a report the
        robot could not be given is logged and dropped, as _record's is, and
        the robot is then cleared by an operator (POST /api/v1/robot/state).

        A cancel that arrives while the report is out is swallowed, not obeyed:
        the run is already closing, and dropping the report would leave the
        robot counting it as RUNNING for good.
        """
        if not self._admitted:
            return
        reason = ""
        if outcome is TaskOutcome.FAILED:
            failed = next(
                (step for step in self._steps if step.status is StepStatus.FAILED),
                None,
            )
            # The sentence, not the step's stored error: see _failure_sentence.
            reason = _failure_sentence(
                err, fallback=(failed.error_msg or "") if failed is not None else ""
            )[:END_TASK_REASON_MAX]
        failure = await self._report_end(
            outcome, reason, timeout=END_TASK_TIMEOUT, attempts=END_TASK_ATTEMPTS
        )
        if failure is not None:
            workflow.logger.warning(
                "Could not tell the robot the run ended (%s); it still counts "
                "the run as running until an operator clears it: %s",
                outcome.value,
                failure.cause or failure,
            )

    async def _release_unanswered_begin(self) -> None:
        """Best-effort end_task(CANCELED) for a begin_task nobody answered.

        Three attempts without an answer do not mean the robot did not hear:
        a request can land, be accepted, and lose only its answer (the state
        node busy past the gateway's wait, say). The run then fails closed
        believing itself unadmitted while the robot counts it as RUNNING, and
        every task after it is refused robot_busy until an operator notices
        and clears it. One report closes that, and it is safe whichever way it
        was, by EndTask's own contract: matched by task_id, it is a no-op (no
        state changed, nothing latched) on a robot that is not running this
        id, and CANCELED never latches ERROR on one that is.

        Only for an UNKNOWN outcome. An answered refusal gets none -- the
        robot said no, so it is not running the task -- and ``_admitted``
        stays False: nothing here says the robot accepted.

        On begin's budget, not end_task's (RELEASE_TASK_TIMEOUT), and never
        changes how the run ends: a failure is logged and dropped, and a
        cancel arriving meanwhile is swallowed as in _end_task -- the run is
        already on its way out, as FAILED robot_state_unreachable or as the
        cancel that came during begin.
        """
        failure = await self._report_end(
            TaskOutcome.CANCELED,
            "",
            timeout=RELEASE_TASK_TIMEOUT,
            attempts=RELEASE_TASK_ATTEMPTS,
        )
        if failure is not None:
            workflow.logger.warning(
                "Could not release the robot after the unanswered begin_task "
                "either; if it had accepted the run it counts it as running "
                "until an operator clears it: %s",
                failure.cause or failure,
            )

    async def _report_end(
        self, outcome: TaskOutcome, reason: str, *, timeout: timedelta, attempts: int
    ) -> ActivityError | None:
        """Run the end_task activity for this run to its end, through a cancel;
        the failure it ended in, or None once the robot answered.

        Shared by _end_task and _release_unanswered_begin, which differ only in
        when they may send and how long they try. Neither awaits the handle:
        see _outlast_cancel.
        """
        info = workflow.info()
        handle = workflow.start_activity(
            RobotActivities.end_task,
            TaskEnd(task_id=info.workflow_id, outcome=outcome, reason=reason),
            task_queue=io_task_queue_for(info.task_queue),
            schedule_to_close_timeout=timeout,
            retry_policy=RetryPolicy(
                initial_interval=TASK_STATE_RETRY_INTERVAL,
                backoff_coefficient=1.0,
                maximum_attempts=attempts,
            ),
        )
        await self._outlast_cancel(handle)
        try:
            handle.result()
        except ActivityError as failure:
            return failure
        return None

    @staticmethod
    async def _outlast_cancel(handle) -> asyncio.CancelledError | None:
        """Wait for ``handle`` to finish, through a run cancel; the cancel that
        arrived meanwhile, or None.

        For the activities whose answer the run needs whatever happens to it
        (begin_task, and end_task in both its uses). On a condition, never ``await handle``:
        awaiting the handle would pass the cancel on to the activity. Looped,
        because the wait is what a cancel interrupts and the answer is still
        wanted after it. Only asyncio.CancelledError is caught: the SDK ends an
        evicted run by cancelling its tasks too, and the next wait_condition
        then raises its own BaseException, which goes through untouched.
        """
        cancel: asyncio.CancelledError | None = None
        while not handle.done():
            try:
                await workflow.wait_condition(handle.done)
            except asyncio.CancelledError as err:
                cancel = err
        return cancel

    async def _resolve_template(
        self, task_template_id: str, own_map: str | None
    ) -> tuple[str | None, str | None]:
        """Run the template as it is now instead of the snapshot.

        Answers the run's map and why it must not go ahead (None: it may).

        The run is renamed and re-mapped after it (TaskName / TaskMap), so the
        history names it as the operator does today and every MOVE checks the
        map the template is on now. Another map loaded fails the run on its
        first step, before anything moves or speaks. A template that is gone or
        cannot be read leaves the snapshot: an unattended run keeps its last
        known route rather than not running at all.
        """
        info = workflow.info()
        try:
            resolved: ResolvedTemplate | None = await workflow.execute_activity(
                RobotActivities.resolve_template,
                task_template_id,
                task_queue=io_task_queue_for(info.task_queue),
                schedule_to_close_timeout=RESOLVE_TEMPLATE_TIMEOUT,
                retry_policy=RetryPolicy(
                    initial_interval=timedelta(seconds=2),
                    maximum_attempts=3,
                ),
            )
        except ActivityError as err:
            workflow.logger.warning(
                "Could not read the schedule's task; running its snapshot: %s",
                err.cause or err,
            )
            return own_map, None
        if resolved is None:
            return own_map, None

        workflow.upsert_search_attributes(
            [
                TASK_NAME_KEY.value_set(resolved.name),
                TASK_MAP_KEY.value_set(resolved.map_name)
                if resolved.map_name
                else TASK_MAP_KEY.value_unset(),
            ]
        )
        self._steps = resolved.steps
        return resolved.map_name, resolved.refusal

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
        # MOVE / CHARGE get the map as a second argument only when there is one, so old
        # one-argument histories and runs without the attribute look the same.
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
        # the whole hold for SPEAK and postures; MOVE / PATROL and WAIT are
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
        """Run a MOVE / PATROL, letting a pause interrupt it and a resume re-send it.

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
