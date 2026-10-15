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
from syncai_backend.gateways.workflow.config import io_task_queue, task_queue
from syncai_backend.gateways.workflow.search_attributes import (
    ensure_search_attributes,
)

from syncai_backend.gateways.robot.robot import RobotGateway
from syncai_backend.gateways.tts.tts import TtsGateway

from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.services.gridmap_conversion import GridmapConversionService

# Same bounded retry as database/postgres.py (compose services may come up after the
# backend), but running out only marks the worker dead: there is no supervisor, and
# the rest of the process is worth keeping, so /health reports it instead.
MAX_RETRIES = 20
RETRY_INTERVAL = 5

# The longest the worker holds a heartbeat before sending it. The server never pushes
# a cancel; it answers the next heartbeat, so this is most of how long a paused or
# cancelled MOVE keeps driving (the SDK default made it ~3.5 s). Only sending is
# throttled; the heartbeat timeout is unchanged. Must stay under 0.8 x
# MOVE_HEARTBEAT_TIMEOUT (pinned in test_activities.py).
HEARTBEAT_THROTTLE_MAX = timedelta(milliseconds=500)

# Concurrent IO activities (SPEAK): more than one so a second run's SPEAK does not tick
# its start_to_close behind the first; few, as each holds an HTTP request open.
IO_ACTIVITY_SLOTS = 4


class TemporalWorkerHandle:
    """Cross-thread view of the worker's lifecycle for /health.

    Every exit path of the worker thread lands in one of these states, so a failed
    connect is visible instead of tasks silently queueing forever.
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
    """Connect to Temporal (bounded retries), register workflow/activities, run forever.

    `ready` is set right before polling starts.
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

    # Here, not in the gateway: this already waits for Temporal, and the attributes
    # must exist before the first run is stamped. Non-fatal.
    await ensure_search_attributes(client, logger)

    # For STEP_STARTED_SIGNAL: activities run in executor threads, the client on this loop.
    activities.bind_client(client, asyncio.get_running_loop())

    worker = Worker(
        client,
        task_queue=task_queue(robot_id),
        workflows=[RobotWorkflow],
        activities=[
            activities.execute_move,
            activities.execute_dock,
            activities.execute_undock,
            activities.execute_stand,
            activities.execute_lie_down,
            # For runs from before SPEAK_ON_IO_QUEUE_PATCH.
            activities.execute_speak,
        ],
        # One body activity at a time, set twice: the executor is the thread that runs
        # activities, max_concurrent_activities is how many the worker accepts. With
        # the default (100) a second activity (e.g. a schedule firing during a direct
        # task) ticked its heartbeat/start_to_close timeouts queued behind the thread;
        # capped, Temporal holds it server-side. Also keeps two runs from driving.
        activity_executor=ThreadPoolExecutor(max_workers=1),
        max_concurrent_activities=1,
        max_heartbeat_throttle_interval=HEARTBEAT_THROTTLE_MAX,
    )
    # SPEAK on its own queue so a run can speak while its MOVE holds the body slot.
    # Executor and cap equal for the same reason as above.
    io_worker = Worker(
        client,
        task_queue=io_task_queue(robot_id),
        activities=[activities.execute_speak],
        activity_executor=ThreadPoolExecutor(max_workers=IO_ACTIVITY_SLOTS),
        max_concurrent_activities=IO_ACTIVITY_SLOTS,
        max_heartbeat_throttle_interval=HEARTBEAT_THROTTLE_MAX,
    )

    logger.info(
        "Temporal worker started",
        server=server,
        task_queue=task_queue(robot_id),
        io_task_queue=io_task_queue(robot_id),
    )

    handle.mark_running()
    if ready is not None:
        ready.set()

    await asyncio.gather(worker.run(), io_worker.run())


def start_temporal_worker(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    robot_gw: RobotGateway,
    tts_gw: TtsGateway,
    map_catalog_repo: MapCatalogRepo,
    conversion_svc: GridmapConversionService,
    robot_repo: RobotRepo,
) -> TemporalWorkerHandle:

    ready = threading.Event()
    handle = TemporalWorkerHandle()

    def _thread_target() -> None:
        # Anything escaping run_worker, including worker.run() dying, must land in
        # the handle.
        try:
            activities = RobotActivities(
                logger=logger,
                robot_gw=robot_gw,
                tts_gw=tts_gw,
                map_catalog_repo=map_catalog_repo,
                conversion_svc=conversion_svc,
                robot_repo=robot_repo,
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
    # Expected while Temporal boots (the retry budget is ~100 s): a heads-up only.
    if not ready.wait(timeout=10.0):
        logger.warning(
            "Temporal worker not ready yet; still connecting in the background",
            server=temporal_server_url(),
        )

    return handle
