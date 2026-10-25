import asyncio
import structlog
import time
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Awaitable, Callable, Iterator, List, Optional, Tuple

from temporalio.contrib.pydantic import pydantic_data_converter
from temporalio.client import (
    Client,
    WorkflowExecutionStatus,
    Schedule,
    ScheduleActionStartWorkflow,
    ScheduleAlreadyRunningError,
    ScheduleDescription,
    ScheduleHandle,
    ScheduleIntervalSpec,
    ScheduleOverlapPolicy,
    SchedulePolicy,
    ScheduleSpec,
    ScheduleUpdate,
    ScheduleUpdateInput,
)
from temporalio.api.common.v1 import Payload
from temporalio.common import (
    SearchAttributeKey,
    SearchAttributePair,
    TypedSearchAttributes,
)
from temporalio.exceptions import WorkflowAlreadyStartedError
from temporalio.service import RPCError, RPCStatusCode

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    UpstreamError,
    NotFoundError,
)

from syncai_backend.tasks.schema import (
    ActiveTask,
    ScheduleTask,
    ScheduleTrigger,
    ScheduleView,
    Step,
    StepStatus,
    TaskHistoryEntry,
    TaskHistoryStats,
    TaskKind,
    TaskProvenance,
    TaskSource,
    TaskState,
    WorkflowTask,
)
from syncai_backend.tasks.search_attributes import (
    ROBOT_ID_KEY,
    TASK_KIND_KEY,
    TASK_MAP_KEY,
    TASK_NAME_KEY,
)
from syncai_backend.tasks.config import (
    ACTIVE_TASK_CACHE_TTL_S,
    ACTIVE_TASK_LIST_LIMIT,
    ACTIVE_TASK_RPC_TIMEOUT_S,
    PAUSE_SIGNAL,
    RESUME_SIGNAL,
    TASK_HISTORY_RPC_TIMEOUT_S,
    TASK_STATE_RPC_TIMEOUT_S,
    TASK_STATS_RPC_TIMEOUT_S,
    WORKFLOW_TYPE_NAME,
    task_queue,
    temporal_server_url,
)


# Temporal's execution statuses, folded to the REST task statuses. PAUSED has no
# Temporal counterpart: a held run is RUNNING, so get_task_state derives PAUSED from
# the held step in the workflow query.
_WORKFLOW_STATUS_MAP = {
    WorkflowExecutionStatus.RUNNING: "IN_PROGRESS",
    WorkflowExecutionStatus.CONTINUED_AS_NEW: "IN_PROGRESS",
    WorkflowExecutionStatus.COMPLETED: "COMPLETED",
    WorkflowExecutionStatus.FAILED: "FAILED",
    WorkflowExecutionStatus.TIMED_OUT: "FAILED",
    WorkflowExecutionStatus.CANCELED: "CANCELED",
    WorkflowExecutionStatus.TERMINATED: "CANCELED",
}


# The schedule that started a run: a predefined attribute Temporal's schedule machinery
# writes on every triggered run. The only way to recover that provenance (a scheduled
# run's id embeds a nominal time the backend cannot reconstruct, and the memo is the
# schedule's), and the authority for "a schedule started it", since schedules
# registered before TaskKind/TaskName fire without them.
_SCHEDULED_BY_KEY = SearchAttributeKey.for_keyword("TemporalScheduledById")


_TASK_STATE_RPC = timedelta(seconds=TASK_STATE_RPC_TIMEOUT_S)


def _active_query(task_queue: str) -> str:
    """The visibility List Filter behind GET /api/v1/active_tasks.

    `TaskQueue` is the per-robot scope; `WorkflowType` keeps a future second
    workflow type out. Needs SQL advanced visibility (server >= 1.20 on Postgres,
    as compose pins); see _fetch_active_tasks for the INVALID_ARGUMENT otherwise.
    """
    return (
        f"WorkflowType = '{WORKFLOW_TYPE_NAME}' "
        f"AND TaskQueue = '{task_queue}' "
        "AND ExecutionStatus = 'Running'"
    )


# REST status -> the closed Temporal statuses it folds, as the List Filter spells
# them (GET /api/v1/task_history's filter). ContinuedAsNew maps to IN_PROGRESS, so it
# is left out.
_CLOSED_STATUS_FILTER = {
    "COMPLETED": ("Completed",),
    "FAILED": ("Failed", "TimedOut"),
    "CANCELED": ("Canceled", "Terminated"),
}

# And back, for reading a `GROUP BY ExecutionStatus` count into REST statuses.
_REST_STATUS_OF = {
    temporal: rest
    for rest, temporals in _CLOSED_STATUS_FILTER.items()
    for temporal in temporals
}


def _quote(value: str) -> str:
    r"""A List Filter string literal.

    For a template name with an apostrophe; the parser rejects anything but
    ``column op literal`` anyway, so this is not the only injection guard.
    """
    return "'" + value.replace("\\", "\\\\").replace("'", "\\'") + "'"


def _schedule_list_query(robot_id: str) -> str:
    """The List Filter behind GET /api/v1/schedules: this robot's, server-side.

    The namespace is the fleet's, so an unfiltered list pulled every robot's
    schedules to keep this one's. ``RobotId`` is stamped on each schedule at
    create; ``IS NULL`` keeps the ones from before it, which the memo filter
    after the list still scopes. Checked on the pinned server (1.29.7, Postgres
    visibility): mine and legacy listed, another robot's new one not.
    """
    return f"RobotId = {_quote(robot_id)} OR RobotId IS NULL"


def _kind_predicate(kind: TaskKind) -> str:
    """The List Filter clause for one kind.

    SCHEDULE keys on TemporalScheduledById, not TaskKind, so runs of schedules
    that predate the custom attributes still count.
    """
    if kind is TaskKind.SCHEDULE:
        return "TemporalScheduledById IS NOT NULL"
    return f"TaskKind = {_quote(kind.value)}"


def _history_query(
    task_queue: str,
    status: Optional[str] = None,
    since: Optional[datetime] = None,
    until: Optional[datetime] = None,
    kind: Optional[TaskKind] = None,
    name: Optional[str] = None,
) -> str:
    """The visibility List Filter behind GET /api/v1/task_history and /stats.

    Same scope as _active_query. No ORDER BY: SQL visibility rejects a custom one,
    and its default (newest CloseTime first) is what a history wants.
    """
    if status is None:
        statuses = tuple(s for group in _CLOSED_STATUS_FILTER.values() for s in group)
    else:
        statuses = _CLOSED_STATUS_FILTER[status]

    query = (
        f"WorkflowType = '{WORKFLOW_TYPE_NAME}' "
        f"AND TaskQueue = '{task_queue}' "
        "AND ExecutionStatus IN ({})".format(", ".join(f"'{s}'" for s in statuses))
    )
    if since is not None:
        query += " AND CloseTime >= '{}'".format(
            since.astimezone(timezone.utc).isoformat()
        )
    if until is not None:
        query += " AND CloseTime <= '{}'".format(
            until.astimezone(timezone.utc).isoformat()
        )
    if kind is not None:
        query += " AND " + _kind_predicate(kind)
    if name is not None:
        query += f" AND TaskName = {_quote(name)}"
    return query


def _schedule_id_of(execution) -> Optional[str]:
    """The schedule that started this run, or None for a direct dispatch."""
    try:
        return execution.typed_search_attributes.get(_SCHEDULED_BY_KEY)
    except Exception:
        # Never fatal: provenance is decoration, presence is the safety fact.
        return None


def _provenance_of(
    execution, schedule_id: Optional[str]
) -> Tuple[Optional[TaskKind], Optional[str]]:
    """The run's kind and template name off its search attributes.

    Never fatal. A scheduled run without the attributes falls back to SCHEDULE
    (off TemporalScheduledById); an unknown kind (a newer backend's) reads None.
    """
    try:
        attributes = execution.typed_search_attributes
        raw_kind = attributes.get(TASK_KIND_KEY)
        name = attributes.get(TASK_NAME_KEY)
    except Exception:
        raw_kind, name = None, None
    try:
        kind = TaskKind(raw_kind) if raw_kind is not None else None
    except ValueError:
        kind = None
    if kind is None and schedule_id:
        kind = TaskKind.SCHEDULE
    return kind, (name or None)


def _map_of(execution) -> Optional[str]:
    """The run's ``TaskMap``, or None. Never fatal.

    None is ambiguous on purpose (no MOVE, or older than the attribute);
    ActiveTask.map_in_use tells them apart, as only it knows the loaded map.
    """
    try:
        return execution.typed_search_attributes.get(TASK_MAP_KEY) or None
    except Exception:
        return None


def _direct_search_attributes(provenance: TaskProvenance) -> TypedSearchAttributes:
    """What a direct dispatch stamps on its run: only the fields it gave."""
    pairs = []
    if provenance.kind is not None:
        pairs.append(SearchAttributePair(TASK_KIND_KEY, provenance.kind.value))
    if provenance.name:
        pairs.append(SearchAttributePair(TASK_NAME_KEY, provenance.name))
    if provenance.map_name:
        pairs.append(SearchAttributePair(TASK_MAP_KEY, provenance.map_name))
    return TypedSearchAttributes(pairs)


def _schedule_search_attributes(schedule: ScheduleTask) -> TypedSearchAttributes:
    """What every run a schedule starts inherits from its action.

    SCHEDULE, plus the template name and map _schedule_to_memo also writes:
    repeated here because only the action's attributes reach the runs. The map
    lets a run firing after a map switch refuse to drive (execute_move).
    """
    pairs = [SearchAttributePair(TASK_KIND_KEY, TaskKind.SCHEDULE.value)]
    if schedule.task_template_name:
        pairs.append(SearchAttributePair(TASK_NAME_KEY, schedule.task_template_name))
    if schedule.map_name:
        pairs.append(SearchAttributePair(TASK_MAP_KEY, schedule.map_name))
    return TypedSearchAttributes(pairs)


@dataclass
class _ActiveSnapshot:
    """One cached answer (success or failure) and when it was taken.

    `fetched_at` is monotonic for the TTL; `as_of` is wall clock for the client.
    """

    fetched_at: float
    as_of: datetime
    tasks: Optional[List[ActiveTask]] = None
    error: Optional[Exception] = None


def _build_schedule_spec(trigger: ScheduleTrigger) -> ScheduleSpec:
    """Map a ScheduleTrigger (cron or interval) to a Temporal ScheduleSpec.

    A cron goes out as ``"<cron> # <cron>"``: Temporal forgets the cron string but
    keeps the text after ``#`` as the calendar's ``comment``, echoed by describe,
    list and update. That is how ``_read_trigger`` reports the cron, since an
    update cannot rewrite the memo (server 1.29.7 ignores ``memo``).

    Refused because they break that echo: a ``#`` of the caller's, a
    ``CRON_TZ=`` / ``TZ=`` prefix (use ``timezone``), and ``@every`` (compiled to
    an IntervalSpec with no comment; use ``intervalSeconds``).
    """
    if trigger.cron:
        cron = trigger.cron.strip()
        if "#" in cron:
            raise BadRequestError("Cron expression must not contain '#'")
        if cron.startswith(("CRON_TZ=", "TZ=")):
            raise BadRequestError(
                "Cron expression must not carry a CRON_TZ=/TZ= prefix; "
                "use the timezone field"
            )
        if cron.startswith("@every"):
            raise BadRequestError(
                "'@every' is not a cron expression here; use intervalSeconds"
            )
        return ScheduleSpec(
            cron_expressions=[f"{cron} # {cron}"],
            time_zone_name=trigger.timezone or "",
        )
    if trigger.interval_seconds:
        return ScheduleSpec(
            intervals=[
                ScheduleIntervalSpec(every=timedelta(seconds=trigger.interval_seconds))
            ]
        )
    raise BadRequestError("Schedule trigger must set either cron or intervalSeconds")


def _spec_to_trigger(spec: ScheduleSpec) -> ScheduleTrigger:
    """Reconstruct a ScheduleTrigger from a Temporal ScheduleSpec.

    The cron is the first calendar's ``comment``, or, for a spec not yet sent
    (tests), ``cron_expressions`` with the comment stripped. A calendar without a
    comment (older or foreign schedule) yields an empty trigger and
    ``_read_trigger`` falls through to the memo.
    """
    if spec.calendars:
        comment = spec.calendars[0].comment
        if comment:
            return ScheduleTrigger(
                cron=comment,
                timezone=spec.time_zone_name or None,
            )
    if spec.cron_expressions:
        return ScheduleTrigger(
            cron=spec.cron_expressions[0].split("#", 1)[0].strip(),
            timezone=spec.time_zone_name or None,
        )
    if spec.intervals:
        return ScheduleTrigger(
            interval_seconds=int(spec.intervals[0].every.total_seconds())
        )
    return ScheduleTrigger()


def _schedule_to_memo(schedule: ScheduleTask, robot_id: str) -> dict:
    """Serialise a schedule's provenance into memo fields.

    The memo is readable from ``list_schedules()``, the start-workflow args are
    not. ``robot_id`` is always written because the namespace is shared and the
    list element's action carries no task queue; it answers "mine?" without a
    describe per row (older schedules fall back to describe, in list_schedules).

    The trigger is deliberately NOT here: the server ignores memo on update, so
    a copy would go stale (see ``_build_schedule_spec``). ``_read_trigger``
    still reads the old keys off schedules from before.
    """
    memo: dict = {"robot_id": robot_id}
    if schedule.map_name:
        memo["map_name"] = schedule.map_name
    if schedule.task_template_id:
        memo["task_template_id"] = schedule.task_template_id
    if schedule.task_template_name:
        memo["task_template_name"] = schedule.task_template_name
    return memo


async def _read_memo(described) -> dict:
    """One memo read per description (decode once per row), tolerating none."""
    try:
        return await described.memo() or {}
    except Exception:
        return {}


def _read_trigger(memo: dict, spec: ScheduleSpec) -> ScheduleTrigger:
    """The schedule's *current* trigger: the spec first, the memo as legacy.

    The spec is what an update rewrites. The memo's ``cron`` /
    ``interval_seconds`` / ``timezone`` keys are honoured only when the spec has
    nothing to say (schedules from before the comment), so an edited trigger wins
    over the stale copy.
    """
    trigger = _spec_to_trigger(spec)
    if trigger.cron or trigger.interval_seconds:
        return trigger

    cron = memo.get("cron")
    interval_seconds = memo.get("interval_seconds")
    if cron or interval_seconds:
        return ScheduleTrigger(
            cron=cron,
            interval_seconds=interval_seconds,
            timezone=memo.get("timezone"),
        )

    return trigger


def _schedule_task_queue(desc) -> Optional[str]:
    """The task queue a described schedule dispatches onto, or None.

    The ownership fact (the same predicate as ``_active_query``). Only the describe
    path has it; the list path's action carries no task queue, hence the memo's
    ``robot_id``.

    None for a non-start-workflow action, which callers treat as "not owned":
    unlike ``_read_steps``' never-fatal decoration, ownership decides whether
    another robot's schedule can be paused or deleted here, so it degrades closed.
    Module-level so tests can hand it a built description.
    """
    action = getattr(desc.schedule, "action", None)
    if not isinstance(action, ScheduleActionStartWorkflow):
        return None
    return action.task_queue


async def _read_steps(logger: structlog.stdlib.BoundLogger, desc) -> List[Step]:
    """Decode the frozen step list out of a *described* schedule's action.

    ``args`` are raw Payload protos, decoded with the description's
    data_converter. Module-level, taking the logger, for unit tests.

    Never fatal: a foreign or undecodable schedule still reports its trigger and
    next run times.
    """
    action = getattr(desc.schedule, "action", None)
    if not isinstance(action, ScheduleActionStartWorkflow) or not action.args:
        return []

    try:
        args = list(action.args)
        # Payloads only on the describe path; a just-built ScheduleTask or a test
        # stub holds python objects.
        if isinstance(args[0], Payload):
            args = await desc.data_converter.decode(args, [WorkflowTask])
        task = args[0]
        return list(task.definition.steps) if isinstance(task, WorkflowTask) else []
    except Exception as err:
        logger.warn(
            "[WorkflowGateway] Failed to decode schedule steps",
            schedule_id=getattr(desc, "id", None),
            error=str(err),
        )
        return []


class WorkflowGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, robot_id: str):
        self._logger = logger
        # This robot's own queue (matches temporal/worker.py), so only its worker
        # executes them.
        self._robot_id = robot_id
        self._task_queue = task_queue(robot_id)
        self._client: Client | None = None

        # Ownership verdicts for schedules whose memo predates robot_id, so
        # list_schedules' describe fallback is paid once per schedule, not per poll.
        # Never invalidated, soundly: only memo-less rows consult it, and every
        # schedule created now writes robot_id. Uvicorn loop only, so no lock.
        self._schedule_owner_cache: dict[str, bool] = {}

        # The active-task cache, touched ONLY from the uvicorn loop (the worker
        # thread has its own Client). The lock is made lazily: this is constructed
        # on the main thread, before that loop exists.
        self._active_snapshot: Optional[_ActiveSnapshot] = None
        self._active_lock: Optional[asyncio.Lock] = None
        # Single-flight for the lazy connect, made lazily for the same reason.
        self._connect_lock: Optional[asyncio.Lock] = None

        # The last task start_task started, for _require_idle's first check:
        # visibility is eventually consistent, describe-by-id is not, so this
        # closes the back-to-back-dispatch window. Lost on restart, when the
        # visibility sweep covers it. Uvicorn loop only.
        self._last_started_task_id: Optional[str] = None

    async def _get_client(self) -> Client:
        """The one Temporal client this gateway uses, connected on first use.

        Under a lock: the console opens a page with four requests at once
        (active tasks, schedules, history, stats), and every one of them
        found ``_client`` None after a backend or Temporal restart and opened
        a connection of its own -- the last assigned won, the rest leaked.
        """
        if self._client is not None:
            return self._client

        if self._connect_lock is None:
            self._connect_lock = asyncio.Lock()
        async with self._connect_lock:
            if self._client is None:
                self._client = await Client.connect(
                    target_host=temporal_server_url(),
                    data_converter=pydantic_data_converter,
                )
            return self._client

    async def _connect(self) -> Client:
        """``_get_client`` for a verb: a failed connect is logged and is the
        caller's 502. Every public method starts here."""
        try:
            return await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise UpstreamError("Failed to connect to Temporal server")

    @contextmanager
    def _rpc_refusals(
        self, *, not_found: str, failure: str, log: str, **context
    ) -> Iterator[None]:
        """Translate an RPCError from the block into this module's two answers.

        NOT_FOUND is ``NotFoundError(not_found)`` (404: the id is gone, or
        never was); anything else is logged as ``log`` and is
        ``UpstreamError(failure)`` (502). Other exceptions pass through.
        """
        try:
            yield
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(not_found)
            self._logger.error(f"[WorkflowGateway] {log}", error=str(err), **context)
            raise UpstreamError(failure)

    async def _require_idle(self, client: Client) -> None:
        """Refuse to dispatch while anything is already running on this robot.

        SKIP only constrains a schedule against itself, and the one-slot worker
        serialises activities, not tasks: two runs' steps would interleave. Every
        direct dispatch comes through here; scheduled runs cannot be gated
        (Temporal starts them), so a dispatch is refused during a scheduled run
        but not the reverse.

        Two reads, cheapest-and-strongest first:

        1. Describe the task this process last started (strongly consistent;
           covers a start not yet in the visibility index).
        2. A fresh visibility sweep of the queue (scheduled runs, starts before a
           restart). Not the TTL cache: stale is fine to display, not to dispatch.

        Check-then-start is not atomic; with one console and the MCP server as
        writers that window is accepted (the alternative is a per-robot mutex
        workflow).
        """
        if self._last_started_task_id is not None:
            handle = client.get_workflow_handle(self._last_started_task_id)
            try:
                description = await handle.describe(rpc_timeout=_TASK_STATE_RPC)
            except RPCError as err:
                if err.status == RPCStatusCode.NOT_FOUND:
                    # Retention already dropped it; certainly not running.
                    description = None
                else:
                    # Not fatal: the sweep below still runs and fails closed.
                    self._logger.warn(
                        "[WorkflowGateway] Failed to describe last started task",
                        task_id=self._last_started_task_id,
                        error=str(err),
                    )
                    description = None

            if (
                description is not None
                and _WORKFLOW_STATUS_MAP.get(description.status) == "IN_PROGRESS"
            ):
                raise ConflictError(
                    f"Robot is busy: task {self._last_started_task_id} is running"
                )
            self._last_started_task_id = None

        snapshot = await self._fetch_active_tasks()
        # Store it: keeps the active_tasks cache warm.
        self._active_snapshot = snapshot
        if snapshot.error is not None:
            # Fail closed: "could not tell" must not become "assume idle".
            raise snapshot.error
        if snapshot.tasks:
            raise ConflictError(
                f"Robot is busy: task {snapshot.tasks[0].id} is running"
            )

    async def start_task(
        self, request: WorkflowTask, provenance: TaskProvenance = TaskProvenance()
    ):
        """Dispatch a run. `provenance` is stamped as search attributes, not passed
        as the argument: history filters on them, and the map must also reach
        scheduled runs, which only attributes do."""
        client = await self._connect()

        await self._require_idle(client)

        try:
            await client.start_workflow(
                workflow=WORKFLOW_TYPE_NAME,
                id=request.id,
                args=[request],
                task_queue=self._task_queue,
                search_attributes=_direct_search_attributes(provenance),
            )
        except WorkflowAlreadyStartedError:
            # Ids are namespace-global, so this is a re-post or another robot's id;
            # Temporal does not say which. A 400 naming the id beats the generic 502.
            raise BadRequestError(f"Task {request.id} already exists")
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to start workflow", error=str(err)
            )
            raise UpstreamError("Start workflow failed")

        self._last_started_task_id = request.id

    async def _describe_owned_task(self, task_id: str, failure_message: str):
        """Connect, describe ``task_id`` and refuse it unless it is this robot's.

        Every single-task verb starts here. Ids are namespace-global; the task
        queue scopes an execution to this robot, so robot01's console cannot read
        or cancel robot02's runs. 404, not 403 (as the map router's
        _require_vertex): confirming the id exists elsewhere would leak it.

        ``failure_message`` names the verb in the UpstreamError a failed describe
        raises.
        """
        client = await self._connect()

        handle = client.get_workflow_handle(task_id)

        with self._rpc_refusals(
            not_found=f"Task {task_id} not found",
            failure=failure_message,
            log="Failed to describe workflow",
        ):
            description = await handle.describe(rpc_timeout=_TASK_STATE_RPC)

        if description.task_queue != self._task_queue:
            raise NotFoundError(f"Task {task_id} not found")

        return handle, description

    async def get_task_state(self, task_id: str) -> TaskState:
        handle, description = await self._describe_owned_task(
            task_id, "Describe workflow failed"
        )

        status = _WORKFLOW_STATUS_MAP.get(description.status)
        if status is None:
            self._logger.error(
                "[WorkflowGateway] Unmapped workflow status",
                task_id=task_id,
                status=str(description.status),
            )
            raise UpstreamError("Unknown workflow status")

        # Per-step state is a workflow query. It can fail before the first workflow
        # task runs or with no worker polling; degrade to no steps. Bounded: with no
        # worker the server held it ~30 s (TASK_STATE_RPC_TIMEOUT_S), every poll.
        try:
            steps = await handle.query(
                "get_step_states", result_type=List[Step], rpc_timeout=_TASK_STATE_RPC
            )
        except Exception as err:
            self._logger.warn(
                "[WorkflowGateway] Failed to query step states",
                task_id=task_id,
                error=str(err),
            )
            steps = []

        # A held run is RUNNING to Temporal; the held step reads PAUSED
        # (RobotWorkflow._hold). If the query failed, IN_PROGRESS is the honest answer.
        if status == "IN_PROGRESS" and any(
            step.status is StepStatus.PAUSED for step in steps
        ):
            status = "PAUSED"

        return TaskState(id=task_id, status=status, steps=steps)

    async def list_active_tasks(self) -> Tuple[List[ActiveTask], datetime]:
        """What is running on this robot right now, and when that was read.

        The only source for it: Temporal visibility sees every execution on this
        robot's queue, whoever started it (console, MCP, a schedule).

        Cached in one slot for ACTIVE_TASK_CACHE_TTL_S, since every open tab polls.
        """
        # Lazily, on the owning loop; no race, as everything here is on that loop.
        if self._active_lock is None:
            self._active_lock = asyncio.Lock()

        cached = self._read_active_cache()
        if cached is not None:
            return cached

        async with self._active_lock:
            # Re-check: otherwise N callers arriving together each make their own
            # serialised RPC, worse than no lock.
            cached = self._read_active_cache()
            if cached is not None:
                return cached

            snapshot = await self._fetch_active_tasks()
            self._active_snapshot = snapshot

        if snapshot.error is not None:
            raise snapshot.error
        return snapshot.tasks or [], snapshot.as_of

    def _read_active_cache(self) -> Optional[Tuple[List[ActiveTask], datetime]]:
        """Replay the snapshot while it is fresh, re-raising a cached failure.

        Failures are cached too: a connect to a dead Temporal can block for
        seconds, and the lock serialises those, so uncached failures would pile
        waiters on the loop. The cost: a recovery shows up to one TTL late.
        """
        snapshot = self._active_snapshot
        if snapshot is None:
            return None
        if time.monotonic() - snapshot.fetched_at >= ACTIVE_TASK_CACHE_TTL_S:
            return None
        if snapshot.error is not None:
            raise snapshot.error
        return snapshot.tasks or [], snapshot.as_of

    async def _fetch_active_tasks(self) -> _ActiveSnapshot:
        """One visibility query, packaged as a snapshot. Never raises."""
        fetched_at = time.monotonic()
        as_of = datetime.now(timezone.utc)

        def failed(err: Exception) -> _ActiveSnapshot:
            return _ActiveSnapshot(fetched_at=fetched_at, as_of=as_of, error=err)

        try:
            client = await self._connect()
        except UpstreamError as err:
            return failed(err)

        tasks: List[ActiveTask] = []
        try:
            # No `await` on list_workflows: it returns the iterator (unlike
            # list_schedules); awaiting it is a TypeError.
            async for execution in client.list_workflows(
                _active_query(self._task_queue),
                limit=ACTIVE_TASK_LIST_LIMIT,
                page_size=ACTIVE_TASK_LIST_LIMIT,
                rpc_timeout=timedelta(seconds=ACTIVE_TASK_RPC_TIMEOUT_S),
            ):
                status = _WORKFLOW_STATUS_MAP.get(execution.status)
                if status is None:
                    # Skipped, not raised: one surprise must not cost the whole answer.
                    self._logger.warn(
                        "[WorkflowGateway] Unmapped status in active tasks",
                        task_id=execution.id,
                        status=str(execution.status),
                    )
                    continue

                schedule_id = _schedule_id_of(execution)
                kind, name = _provenance_of(execution, schedule_id)
                tasks.append(
                    ActiveTask(
                        id=execution.id,
                        run_id=execution.run_id,
                        status=status,
                        started_at=execution.start_time.astimezone(timezone.utc),
                        source=(
                            TaskSource.SCHEDULE if schedule_id else TaskSource.DIRECT
                        ),
                        schedule_id=schedule_id,
                        kind=kind,
                        name=name,
                        map_name=_map_of(execution),
                    )
                )
        except RPCError as err:
            if err.status == RPCStatusCode.INVALID_ARGUMENT:
                # Almost certainly *standard* visibility. The fix would be to drop
                # the TaskQueue / WorkflowType predicates and filter in Python; not
                # pre-built, since the pinned server has advanced visibility.
                self._logger.error(
                    "[WorkflowGateway] Visibility rejected the active-task query; "
                    "is this server on standard visibility?",
                    query=_active_query(self._task_queue),
                    error=str(err),
                )
            else:
                self._logger.error(
                    "[WorkflowGateway] Failed to list active tasks", error=str(err)
                )
            return failed(UpstreamError("List active tasks failed"))
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to list active tasks", error=str(err)
            )
            return failed(UpstreamError("List active tasks failed"))

        # Debug: the cache-miss line, the only way to see how often Temporal is asked.
        self._logger.debug(
            "[WorkflowGateway] Active task snapshot refreshed", count=len(tasks)
        )
        return _ActiveSnapshot(fetched_at=fetched_at, as_of=as_of, tasks=tasks)

    def _history_filter(
        self,
        status: Optional[str],
        since: Optional[datetime],
        until: Optional[datetime],
        kind: Optional[TaskKind],
        name: Optional[str],
    ) -> str:
        """The List Filter the history page and its stats share, validated.

        ``status`` comes from a REST enum: an unknown one is a bug here, not a
        400. Checked before connecting, as it always was.
        """
        if status is not None and status not in _CLOSED_STATUS_FILTER:
            raise ValueError(f"Not a closed task status: {status}")
        return _history_query(
            self._task_queue,
            status=status,
            since=since,
            until=until,
            kind=kind,
            name=name,
        )

    async def list_task_history(
        self,
        page_size: int,
        next_page_token: Optional[bytes] = None,
        status: Optional[str] = None,
        since: Optional[datetime] = None,
        until: Optional[datetime] = None,
        kind: Optional[TaskKind] = None,
        name: Optional[str] = None,
    ) -> Tuple[List[TaskHistoryEntry], Optional[bytes]]:
        """One page of this robot's finished executions, newest first.

        Answers the rows and the next page's token (None on the last), which is
        valid only for the same filters. Reach is the namespace retention: the
        backend stores no runs.
        """
        query = self._history_filter(status, since, until, kind, name)
        client = await self._connect()

        # One page only: `async for` would follow the token to the end of retention.
        pages = client.list_workflows(
            query,
            page_size=page_size,
            next_page_token=next_page_token,
            rpc_timeout=timedelta(seconds=TASK_HISTORY_RPC_TIMEOUT_S),
        )
        try:
            await pages.fetch_next_page()
        except RPCError as err:
            if err.status == RPCStatusCode.INVALID_ARGUMENT:
                if next_page_token is not None:
                    # A token from another query, or a made-up one.
                    raise BadRequestError("Invalid page token")
                self._logger.error(
                    "[WorkflowGateway] Visibility rejected the task-history query; "
                    "is this server on standard visibility, or are the TaskKind / "
                    "TaskName search attributes not registered (see "
                    "ensure_search_attributes)?",
                    query=query,
                    error=str(err),
                )
            else:
                self._logger.error(
                    "[WorkflowGateway] Failed to list task history", error=str(err)
                )
            raise UpstreamError("List task history failed")
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to list task history", error=str(err)
            )
            raise UpstreamError("List task history failed")

        entries: List[TaskHistoryEntry] = []
        for execution in pages.current_page or []:
            mapped = _WORKFLOW_STATUS_MAP.get(execution.status)
            if mapped is None:
                # Skipped, not raised, as in _fetch_active_tasks.
                self._logger.warn(
                    "[WorkflowGateway] Unmapped status in task history",
                    task_id=execution.id,
                    status=str(execution.status),
                )
                continue

            schedule_id = _schedule_id_of(execution)
            run_kind, run_name = _provenance_of(execution, schedule_id)
            entries.append(
                TaskHistoryEntry(
                    id=execution.id,
                    run_id=execution.run_id,
                    status=mapped,
                    started_at=execution.start_time.astimezone(timezone.utc),
                    closed_at=(
                        execution.close_time.astimezone(timezone.utc)
                        if execution.close_time is not None
                        else None
                    ),
                    source=TaskSource.SCHEDULE if schedule_id else TaskSource.DIRECT,
                    schedule_id=schedule_id,
                    kind=run_kind,
                    name=run_name,
                    map_name=_map_of(execution),
                )
            )

        return entries, pages.next_page_token

    async def task_history_stats(
        self,
        status: Optional[str] = None,
        since: Optional[datetime] = None,
        until: Optional[datetime] = None,
        kind: Optional[TaskKind] = None,
        name: Optional[str] = None,
    ) -> TaskHistoryStats:
        """How this robot's finished executions under a filter ended, counted.

        list_task_history's filter, answered by one `count_workflows ... GROUP BY
        ExecutionStatus` (see TASK_STATS_RPC_TIMEOUT_S).
        """
        query = self._history_filter(status, since, until, kind, name)
        client = await self._connect()

        try:
            result = await client.count_workflows(
                query + " GROUP BY ExecutionStatus",
                rpc_timeout=timedelta(seconds=TASK_STATS_RPC_TIMEOUT_S),
            )
        except RPCError as err:
            if err.status == RPCStatusCode.INVALID_ARGUMENT:
                self._logger.error(
                    "[WorkflowGateway] Visibility rejected the task-history count; "
                    "is this server on standard visibility, or are the TaskKind / "
                    "TaskName search attributes not registered (see "
                    "ensure_search_attributes)?",
                    error=str(err),
                )
            else:
                self._logger.error(
                    "[WorkflowGateway] Failed to count task history", error=str(err)
                )
            raise UpstreamError("Task history stats failed")
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to count task history", error=str(err)
            )
            raise UpstreamError("Task history stats failed")

        counts = {rest: 0 for rest in _CLOSED_STATUS_FILTER}
        for group in result.groups:
            # A value not in the table (e.g. Running) is not a finished run.
            rest = _REST_STATUS_OF.get(str(group.group_values[0]))
            if rest is not None:
                counts[rest] += group.count

        return TaskHistoryStats(
            as_of=datetime.now(timezone.utc),
            total=sum(counts.values()),
            completed=counts["COMPLETED"],
            failed=counts["FAILED"],
            canceled=counts["CANCELED"],
        )

    async def cancel_task(self, task_id: str):
        # Describe first, for the ownership check: an id alone must not stop another
        # robot mid-run. The describe->cancel race is benign (a closed run makes
        # cancel a no-op; a deleted one answers NOT_FOUND).
        handle, _ = await self._describe_owned_task(task_id, "Cancel workflow failed")

        with self._rpc_refusals(
            not_found=f"Task {task_id} not found",
            failure="Cancel workflow failed",
            log="Failed to cancel workflow",
        ):
            await handle.cancel()

    async def cancel_active_tasks(self) -> List[str]:
        """Cancel every run on this robot's task queue; the ids that were asked.

        For services/safety_lock.py. Same two reads as _require_idle: a fresh sweep
        (a stale "nothing running" leaves a run driving into a locked robot) plus
        the last task this process started.

        Raises only when the sweep fails. A run that cannot be cancelled is logged
        and the rest still are; one already closed (NotFoundError) is fine.
        """
        snapshot = await self._fetch_active_tasks()
        if snapshot.error is not None:
            raise snapshot.error

        task_ids = [task.id for task in snapshot.tasks or []]
        if (
            self._last_started_task_id is not None
            and self._last_started_task_id not in task_ids
        ):
            task_ids.append(self._last_started_task_id)

        cancelled: List[str] = []
        for task_id in task_ids:
            try:
                await self.cancel_task(task_id)
            except NotFoundError:
                continue
            except Exception as err:
                self._logger.error(
                    "[WorkflowGateway] Failed to cancel task",
                    task_id=task_id,
                    error=str(err),
                )
                continue
            cancelled.append(task_id)
        return cancelled

    async def pause_task(self, task_id: str):
        """Ask the running workflow to hold (POST /api/v1/tasks/{id}/pause)."""
        await self._signal_running_task(task_id, PAUSE_SIGNAL, "Pause workflow failed")

    async def resume_task(self, task_id: str):
        """Release a held workflow (POST /api/v1/tasks/{id}/resume)."""
        await self._signal_running_task(task_id, RESUME_SIGNAL, "Resume workflow failed")

    async def _signal_running_task(self, task_id: str, signal: str, failure_message: str):
        """Deliver ``signal`` to ``task_id`` if it is this robot's and still open.

        Addressed by name, so this module does not import the workflow. Signals are
        fire-and-forget and idempotent, so the REST ack only covers the request; the
        hold is read back from GET /api/v1/tasks/{id}.

        A closed run answers 409 ``task_not_running``, not the server's 502: a
        console's stale poll sends a pause just after the run finished. The
        describe->signal race is as benign as cancel_task's.
        """
        handle, description = await self._describe_owned_task(task_id, failure_message)

        if description.status is not WorkflowExecutionStatus.RUNNING:
            raise ConflictError(f"Task {task_id} is not running", code="task_not_running")

        with self._rpc_refusals(
            not_found=f"Task {task_id} not found",
            failure=failure_message,
            log="Failed to signal workflow",
            task_id=task_id,
            signal=signal,
        ):
            await handle.signal(signal)

    async def create_schedule(self, schedule: ScheduleTask):

        client = await self._connect()

        spec = _build_schedule_spec(schedule.trigger)

        # Temporal appends the nominal trigger time to this id, so each run is a
        # distinct execution, readable via GET /api/v1/tasks/{id}.
        workflow_task = WorkflowTask(id=schedule.id, definition=schedule.definition)

        # Provenance + robot_id into the memo; the trigger rides in the spec.
        memo = _schedule_to_memo(schedule, self._robot_id)

        try:
            await client.create_schedule(
                schedule.id,
                Schedule(
                    action=ScheduleActionStartWorkflow(
                        WORKFLOW_TYPE_NAME,
                        args=[workflow_task],
                        id=schedule.id,
                        task_queue=self._task_queue,
                        # Inherited by every run, beside the server's
                        # TemporalScheduledById.
                        typed_search_attributes=_schedule_search_attributes(
                            schedule
                        ),
                    ),
                    spec=spec,
                    # Never start a run while the previous one is still executing.
                    policy=SchedulePolicy(overlap=ScheduleOverlapPolicy.SKIP),
                ),
                # Never empty: robot_id is always present, so no `or None`.
                memo=memo,
                # On the schedule itself (not its runs): what list_schedules
                # asks the server to filter on.
                search_attributes=TypedSearchAttributes(
                    [SearchAttributePair(ROBOT_ID_KEY, self._robot_id)]
                ),
            )
        except ScheduleAlreadyRunningError:
            raise BadRequestError(f"Schedule {schedule.id} already exists")
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to create schedule", error=str(err)
            )
            raise UpstreamError("Create schedule failed")

    async def _require_owned_schedule(
        self, client: Client, schedule_id: str
    ) -> ScheduleDescription:
        """Describe a schedule, insisting it dispatches onto this robot's queue.

        Schedule ids are namespace-global, so without this the single-schedule verbs
        would act on any robot's schedule. Reads the action's task queue (which
        covers memo-less legacy schedules too). 404 on a foreign one, as
        _describe_owned_task.
        """
        handle = client.get_schedule_handle(schedule_id)

        with self._rpc_refusals(
            not_found=f"Schedule {schedule_id} not found",
            failure="Describe schedule failed",
            log="Failed to describe schedule",
        ):
            desc = await handle.describe()

        if _schedule_task_queue(desc) != self._task_queue:
            raise NotFoundError(f"Schedule {schedule_id} not found")

        return desc

    async def get_schedule(self, schedule_id: str) -> ScheduleView:

        client = await self._connect()

        desc = await self._require_owned_schedule(client, schedule_id)

        memo = await _read_memo(desc)

        return ScheduleView(
            id=desc.id,
            trigger=_read_trigger(memo, desc.schedule.spec),
            paused=desc.schedule.state.paused,
            next_run_times=[
                t.astimezone(timezone.utc) for t in desc.info.next_action_times
            ],
            map_name=memo.get("map_name"),
            # Pre-TaskTemplate-rename memo keys; memos are immutable, so read both.
            task_template_id=memo.get("task_template_id", memo.get("saved_task_id")),
            task_template_name=memo.get(
                "task_template_name", memo.get("saved_task_name")
            ),
            # Only the describe path reaches the frozen steps.
            steps=await _read_steps(self._logger, desc),
        )

    async def _owns_listed_schedule(
        self, client: Client, schedule_id: str, memo: dict
    ) -> bool:
        """Whether a schedule met on the *list* path belongs to this robot.

        The memo's robot_id when present (the only per-row fact without an RPC).
        A legacy memo-less schedule falls back to one describe, cached in
        ``_schedule_owner_cache``.

        A failed describe answers False (hidden this round) without caching: one
        bad row must not cost the listing, and ownership degrades closed.
        """
        owner = memo.get("robot_id")
        if owner is not None:
            return owner == self._robot_id

        cached = self._schedule_owner_cache.get(schedule_id)
        if cached is not None:
            return cached

        try:
            desc = await client.get_schedule_handle(schedule_id).describe()
        except Exception as err:
            self._logger.warn(
                "[WorkflowGateway] Failed to resolve legacy schedule owner; "
                "hiding it from this listing",
                schedule_id=schedule_id,
                error=str(err),
            )
            return False

        owned = _schedule_task_queue(desc) == self._task_queue
        self._schedule_owner_cache[schedule_id] = owned
        return owned

    async def _list_schedule_items(self, client: Client) -> list:
        """The schedule list rows, filtered server-side when the server can.

        Read to the end here, not handed back as the iterator: the SDK sends
        the first page's RPC on the first ``__anext__``, not on the ``await``,
        so a refusal only surfaces while iterating, and has to be caught where
        the iterating happens.

        INVALID_ARGUMENT is the namespace without ``RobotId`` registered yet
        (the worker registers it at connect, and may not have yet): fall back
        to the whole namespace, which the memo filter in the caller scopes as
        it always did. Probed on 1.29.7: an unregistered attribute in the
        filter is exactly that status. Anything else is the listing failing.
        """
        try:
            # `list_schedules` is a coroutine returning the iterator, hence the
            # await (`list_workflows` is not; do not copy this shape there).
            query = _schedule_list_query(self._robot_id)
            return [item async for item in await client.list_schedules(query)]
        except RPCError as err:
            if err.status != RPCStatusCode.INVALID_ARGUMENT:
                raise
            self._logger.warn(
                "[WorkflowGateway] RobotId not queryable; listing every schedule",
                error=str(err),
            )
            return [item async for item in await client.list_schedules()]

    async def list_schedules(self) -> List[ScheduleView]:

        client = await self._connect()

        schedules: List[ScheduleView] = []
        try:
            for item in await self._list_schedule_items(client):
                memo = await _read_memo(item)
                # The namespace is shared; keep only this robot's schedules.
                if not await self._owns_listed_schedule(client, item.id, memo):
                    continue
                schedules.append(
                    ScheduleView(
                        id=item.id,
                        trigger=_read_trigger(memo, item.schedule.spec),
                        paused=item.schedule.state.paused,
                        next_run_times=[
                            t.astimezone(timezone.utc)
                            for t in item.info.next_action_times
                        ],
                        map_name=memo.get("map_name"),
                        # Pre-rename memo keys, as in get_schedule.
                        task_template_id=memo.get(
                            "task_template_id", memo.get("saved_task_id")
                        ),
                        task_template_name=memo.get(
                            "task_template_name", memo.get("saved_task_name")
                        ),
                        # `steps` deliberately empty: the list element's action
                        # carries only the workflow type name, and a describe per
                        # row would make one RPC 1+N. GET /{id} carries steps.
                    )
                )
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to list schedules", error=str(err)
            )
            raise UpstreamError("List schedules failed")

        return schedules

    async def _change_owned_schedule(
        self,
        schedule_id: str,
        verb: str,
        change: Callable[[ScheduleHandle], Awaitable[None]],
    ) -> None:
        """Apply ``change`` to one of this robot's schedules.

        The ownership gate first (a foreign id is a 404); the verify->act race
        is benign, since a schedule deleted in between answers NOT_FOUND, which
        is the same 404. ``verb`` names it in the log and the 502.
        """
        client = await self._connect()
        await self._require_owned_schedule(client, schedule_id)

        with self._rpc_refusals(
            not_found=f"Schedule {schedule_id} not found",
            failure=f"{verb.capitalize()} schedule failed",
            log=f"Failed to {verb} schedule",
        ):
            await change(client.get_schedule_handle(schedule_id))

    async def delete_schedule(self, schedule_id: str):
        await self._change_owned_schedule(schedule_id, "delete", lambda h: h.delete())

    async def pause_schedule(self, schedule_id: str):
        await self._change_owned_schedule(schedule_id, "pause", lambda h: h.pause())

    async def resume_schedule(self, schedule_id: str):
        await self._change_owned_schedule(schedule_id, "resume", lambda h: h.unpause())

    async def update_schedule_trigger(self, schedule_id: str, trigger: ScheduleTrigger):
        """Replace a schedule's firing rule in place, touching nothing else.

        ``ScheduleHandle.update`` is describe-then-UpdateSchedule; only ``spec`` is
        swapped, action, policy and state go back as described. Updating rather
        than recreating keeps the server's record of a running run, so SKIP holds
        across the edit.

        The ownership check runs inside the callback (the SDK already describes,
        so no extra RPC; raising aborts the update). 404 for a foreign schedule.
        The memo is left alone: it carries no trigger, and the server ignores it.
        """
        client = await self._connect()

        # Validate before any RPC: a malformed trigger is the caller's 400.
        spec = _build_schedule_spec(trigger)

        def _retrigger(inp: ScheduleUpdateInput) -> ScheduleUpdate:
            if _schedule_task_queue(inp.description) != self._task_queue:
                raise NotFoundError(f"Schedule {schedule_id} not found")
            schedule = inp.description.schedule
            schedule.spec = spec
            return ScheduleUpdate(schedule=schedule)

        handle = client.get_schedule_handle(schedule_id)

        try:
            await handle.update(_retrigger)
        except NotFoundError:
            raise
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Schedule {schedule_id} not found")
            self._logger.error(
                "[WorkflowGateway] Failed to update schedule", error=str(err)
            )
            raise UpstreamError("Update schedule failed")
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to update schedule", error=str(err)
            )
            raise UpstreamError("Update schedule failed")


def init_workflow_gateway(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
) -> WorkflowGateway:
    workflow_gw = WorkflowGateway(logger=logger, robot_id=robot_id)
    return workflow_gw
