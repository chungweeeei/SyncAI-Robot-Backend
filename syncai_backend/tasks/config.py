import os

# Temporal frontend gRPC endpoint; defaults to the local docker-compose service.
DEFAULT_TEMPORAL_ADDRESS = "127.0.0.1:7233"


def temporal_server_url() -> str:
    """The Temporal address, read from the environment at call time.

    A function, not a module constant: main.py imports this before its own
    ``dotenv.load_dotenv()``, so an import-time read never saw a TEMPORAL_ADDRESS
    that lived only in ``.env``. Every caller runs after load_dotenv.
    """
    return os.getenv("TEMPORAL_ADDRESS", DEFAULT_TEMPORAL_ADDRESS)


# The task queue is per-robot: its name is the robot_id.
WORKFLOW_TYPE_NAME = "RobotWorkflow"

# Two queues per robot, split by what an activity occupies. The body queue carries
# the workflow and every activity that moves the robot; its worker takes one activity
# at a time, so two runs never drive at once and nothing ticks its timeouts queued
# (see run_worker). The IO queue carries activities that only talk to another service
# (SPEAK), several at a time, so a run can speak while it drives (Step.group).
TASK_QUEUE_SUFFIX = ".ROBOT_TASK_QUEUE"
IO_TASK_QUEUE_SUFFIX = ".ROBOT_IO_QUEUE"


def task_queue(robot_id: str) -> str:
    return f"{robot_id}{TASK_QUEUE_SUFFIX}"


def io_task_queue(robot_id: str) -> str:
    return f"{robot_id}{IO_TASK_QUEUE_SUFFIX}"


def io_task_queue_for(body_queue: str) -> str:
    """The IO queue beside ``body_queue``, the queue a run itself is on."""
    return io_task_queue(body_queue.removesuffix(TASK_QUEUE_SUFFIX))


# RobotWorkflow's signals, by their wire names; shared by workflow and gateway so a
# rename lands in both. Idempotent flags: a second pause, or a resume while running,
# changes nothing.
PAUSE_SIGNAL = "pause"
RESUME_SIGNAL = "resume"

# Sent by an activity: "my step has started" (MOVE goal accepted, SPEAK utterance
# queued). A group starts its members in list order, each once the one before has
# started (RobotWorkflow._run_group); a workflow cannot see heartbeats, so this is how
# it learns. The payload is the activity id minted by group_activity_id; only ids of
# that shape report, so a lone step adds no history event.
STEP_STARTED_SIGNAL = "step_started"
_GROUP_ACTIVITY_ID_MARK = "#"


def group_activity_id(step_id: str, dispatch: int) -> str:
    """A group member's activity id: step id plus a per-run dispatch count, so a MOVE
    re-sent after a pause gets a fresh one."""
    return f"{step_id}{_GROUP_ACTIVITY_ID_MARK}{dispatch}"


def reports_start(activity_id: str) -> bool:
    """Whether ``activity_id`` is a group member whose run waits on its
    STEP_STARTED_SIGNAL. Temporal's own ids are sequence numbers, never marked."""
    return _GROUP_ACTIVITY_ID_MARK in activity_id


# --- "What is running right now" (GET /api/v1/active_tasks) -----------------
#
# One visibility query (it already carries status, start time, queue and search
# attributes, so no describe/query), cached in one slot on the gateway: at most
# 1/TTL = 0.67 RPC/s per robot however many clients poll. The frontend polls at 2 s,
# deliberately longer than the TTL; a TTL >= the poll interval would only age the
# snapshot. Worst-case staleness is TTL + poll ~ 3.5 s.
ACTIVE_TASK_CACHE_TTL_S = 1.5

# A ceiling, not a page size: stops a pathological state from paging forever inside
# a request polled every 2 s.
ACTIVE_TASK_LIST_LIMIT = 10

# Explicit: the single-flight lock serialises callers, and a wedged frontend must not
# hold it for the gRPC default timeout.
ACTIVE_TASK_RPC_TIMEOUT_S = 3.0


# --- One task (GET /api/v1/tasks/{id}, and cancel / pause / resume) ---------
#
# The describe every single-task verb starts with, and the step-state query the
# detail page polls. The query is the one that hangs: it is answered by a worker,
# and with none polling the server holds it until its own deadline. Measured on
# the pinned server (auto-setup 1.29.7, 2026-10-25): with no rpc_timeout the query
# took 29-30 s to fail (worker gone; or never run), with 3 s it failed in 2-3 s and
# the page degraded to "no steps" as designed. Describe answered at once either way;
# bounded anyway, since it is in front of every verb.
TASK_STATE_RPC_TIMEOUT_S = 3.0


# --- "What has run" (GET /api/v1/task_history) ------------------------------
#
# One visibility page per request, uncached (paged by hand, not polled). How far back
# it reaches is the namespace's retention.
TASK_HISTORY_PAGE_SIZE_DEFAULT = 20
TASK_HISTORY_PAGE_SIZE_MAX = 100

# No lock held; this only bounds the spinner. Longer than the active-task one: a
# closed-execution page is a bigger scan.
TASK_HISTORY_RPC_TIMEOUT_S = 5.0


# --- "How did they end" (GET /api/v1/task_history/stats) --------------------
#
# One `count_workflows ... GROUP BY ExecutionStatus` (the one GROUP BY the server
# allows) per call, uncached like the history page.
TASK_STATS_RPC_TIMEOUT_S = 5.0
