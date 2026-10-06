import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import timedelta
from typing import Optional

import structlog

from temporalio.client import Client
from temporalio.contrib.pydantic import pydantic_data_converter
from temporalio.worker import Worker

from syncai_backend.temporal.shared import temporal_server_url
from syncai_backend.temporal.workflows import RobotWorkflow
from syncai_backend.temporal.activities import RobotActivities
from syncai_backend.gateways.workflow.search_attributes import (
    ensure_search_attributes,
)

from syncai_backend.gateways.robot.robot import RobotGateway
from syncai_backend.gateways.tts.tts import TtsGateway

from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.services.gridmap_conversion import GridmapConversionService

# Mirrors database/postgres.py: same bounded-retry shape for the same reason —
# on a robot boot the shared docker-compose services (postgres, temporal) may
# come up after the backend does. The difference is what happens when the
# budget runs out: postgres raises and takes the process down (a backend
# without its DB is useless), while the worker only marks itself dead. The
# backend runs in a byobu pane with no supervisor to restart it, and with
# Temporal gone the rest of the process (telemetry, maps, manual control) is
# still worth keeping alive — so the failure is surfaced through /health
# instead of a crash.
MAX_RETRIES = 20
RETRY_INTERVAL = 5

# The longest the worker sits on an activity heartbeat before sending it. The
# server never pushes an activity cancel to the worker: it answers the next
# heartbeat with "cancel requested", so this interval is most of how long a
# paused or cancelled MOVE keeps driving. Left at the SDK default (60 s cap,
# so 0.8 x the 3 s MOVE_HEARTBEAT_TIMEOUT = 2.4 s in effect) a pause took up
# to ~3.5 s to reach nav2. Only the sending is throttled -- the heartbeat
# timeout the server enforces is unchanged -- and two small RPCs a second
# from one robot are nothing to the server.
HEARTBEAT_THROTTLE_MAX = timedelta(milliseconds=500)


class TemporalWorkerHandle:
    """Cross-thread view of the worker's lifecycle for /health.

    The worker lives in a daemon thread; before this handle existed, a failed
    ``Client.connect`` killed that thread silently and the only symptom was
    tasks that queued forever. Every exit path of the thread now lands in one
    of these states, so "is the task server actually polling?" is answerable
    from the REST side.
    """

    STATUS_CONNECTING = "connecting"
    STATUS_RUNNING = "running"
    STATUS_DEAD = "dead"

    def __init__(self):
        self._lock = threading.Lock()
        self._status = self.STATUS_CONNECTING
        self._last_error: Optional[str] = None
        self.thread: Optional[threading.Thread] = None

    def mark_running(self) -> None:
        with self._lock:
            self._status = self.STATUS_RUNNING
            self._last_error = None

    def mark_dead(self, error: str) -> None:
        with self._lock:
            self._status = self.STATUS_DEAD
            self._last_error = error

    def snapshot(self) -> tuple[str, Optional[str]]:
        with self._lock:
            return self._status, self._last_error


async def run_worker(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    activities: RobotActivities,
    handle: TemporalWorkerHandle,
    ready: Optional[threading.Event] = None,
) -> None:
    """Connect to Temporal (with bounded retries), register the
    workflow/activities, and run forever.

    `ready` is set right before the worker starts polling, so a caller running
    this in a background thread can block until the worker is up.
    """
    server = temporal_server_url()
    client = None
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            client = await Client.connect(
                server, data_converter=pydantic_data_converter
            )
            break
        except Exception as err:
            logger.warning(
                "Connection attempt failed",
                component="Temporal",
                attempt=attempt,
                max_retries=MAX_RETRIES,
                error=str(err),
            )
            if attempt == MAX_RETRIES:
                handle.mark_dead(str(err))
                logger.error(
                    "Giving up on Temporal; task server is dead until restart",
                    server=server,
                )
                return
            await asyncio.sleep(RETRY_INTERVAL)

    # Here rather than in the gateway's lazy connect: this is the one place
    # that already waits for Temporal to be up, and the attributes have to
    # exist before the first run is stamped with them. Non-fatal -- see the
    # module; a worker without them still runs every task.
    await ensure_search_attributes(client, logger)

    worker = Worker(
        client,
        task_queue=f"{robot_id}.ROBOT_TASK_QUEUE",
        workflows=[RobotWorkflow],
        activities=[
            activities.execute_move,
            activities.execute_stand,
            activities.execute_lie_down,
            activities.execute_speak,
        ],
        # One activity at a time, stated twice because the two settings mean
        # different things. The executor is the thread that runs activities;
        # max_concurrent_activities is how many the worker *accepts* from the
        # server. Left at its default (100) the worker would take a second
        # activity while the first held the only thread, and that activity's
        # heartbeat_timeout and start_to_close would tick while it queued
        # behind the thread -- so the one realistic overlap, a schedule firing
        # during a direct task (see _require_idle's docstring), failed its
        # MOVE by heartbeat timeout instead of waiting. Capped to 1, Temporal
        # holds the second task server-side until this worker asks for it.
        activity_executor=ThreadPoolExecutor(max_workers=1),
        max_concurrent_activities=1,
        max_heartbeat_throttle_interval=HEARTBEAT_THROTTLE_MAX,
    )

    logger.info(
        "Temporal worker started",
        server=server,
        task_queue=f"{robot_id}.ROBOT_TASK_QUEUE",
    )

    handle.mark_running()
    if ready is not None:
        ready.set()

    await worker.run()


def start_temporal_worker(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    robot_gw: RobotGateway,
    tts_gw: TtsGateway,
    map_catalog_repo: MapCatalogRepo,
    conversion_svc: GridmapConversionService,
) -> TemporalWorkerHandle:

    ready = threading.Event()
    handle = TemporalWorkerHandle()

    def _thread_target() -> None:
        # Anything that escapes run_worker — including worker.run() dying
        # mid-flight — must land in the handle, or we are back to the silent
        # dead thread this exists to prevent.
        try:
            activities = RobotActivities(
                logger=logger,
                robot_gw=robot_gw,
                tts_gw=tts_gw,
                map_catalog_repo=map_catalog_repo,
                conversion_svc=conversion_svc,
            )
            asyncio.run(
                run_worker(
                    logger,
                    robot_id=robot_id,
                    activities=activities,
                    handle=handle,
                    ready=ready,
                )
            )
        except Exception as err:
            handle.mark_dead(str(err))
            logger.error("Temporal worker thread died", exc_info=True)

    thread = threading.Thread(target=_thread_target, daemon=True)
    thread.start()
    handle.thread = thread
    # Not becoming ready in 10s is expected when Temporal is still booting —
    # the retry budget above runs ~100s — so this is a heads-up, not a failure.
    if not ready.wait(timeout=10.0):
        logger.warning(
            "Temporal worker not ready yet; still connecting in the background",
            server=temporal_server_url(),
        )

    return handle
