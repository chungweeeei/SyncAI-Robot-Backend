import os
import signal
import sys
import threading
import dotenv
import structlog

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from syncai_backend.logger import setup_log_handler
from syncai_backend.database.postgres import connect_to_postgres
from syncai_backend.temporal.worker import (
    TemporalWorkerHandle,
    WORKER_SHUTDOWN_TIMEOUT_S,
    start_temporal_worker,
)

from syncai_backend.interfaces.rest.server import RestDeps, start_rest_server

from syncai_backend.repositories.robot.robot import init_robot_repo
from syncai_backend.repositories.map.map import init_map_repo
from syncai_backend.repositories.task.task_template import init_task_template_repo
from syncai_backend.repositories.map.catalog import init_map_catalog_repo
from syncai_backend.repositories.pointcloud.pointcloud import init_pointcloud_repo
from syncai_backend.repositories.mapping.mapping import init_mapping_status_repo
from syncai_backend.repositories.telemetry.telemetry import init_telemetry_repo
from syncai_backend.repositories.recording.catalog import init_recording_catalog_repo

from syncai_backend.gateways.robot.robot import init_robot_gateway
from syncai_backend.gateways.map.map import init_map_gateway
from syncai_backend.gateways.workflow.workflow import init_workflow_gateway
from syncai_backend.gateways.tts.tts import init_tts_gateway
from syncai_backend.gateways.webrtc.webrtc import init_webrtc_gateway
from syncai_backend.gateways.recording.recording import init_recording_gateway

from syncai_backend.services.dock_state import init_dock_state_service
from syncai_backend.services.map_import import init_map_import_service
from syncai_backend.services.map_switch import init_map_switch_service
from syncai_backend.services.map_usage import init_job_gate, init_map_usage_policy
from syncai_backend.services.mode_restart import init_mode_restart_service
from syncai_backend.services.safety_lock import init_safety_lock_service

from syncai_backend.subscribers.robot_state_subscriber import (
    init_robot_state_subscriber,
)
from syncai_backend.subscribers.pointcloud_subscriber import (
    init_pointcloud_subscriber,
)
from syncai_backend.subscribers.map_cloud_subscriber import (
    init_map_cloud_subscriber,
)
from syncai_backend.subscribers.mapping_status_subscriber import (
    init_mapping_status_subscriber,
)
from syncai_backend.subscribers.telemetry_subscriber import (
    init_telemetry_subscriber,
)
from syncai_backend.subscribers.path_subscriber import (
    init_docking_plan_subscriber,
    init_path_subscriber,
)
from syncai_backend.subscribers.local_nav_subscriber import (
    init_local_nav_subscriber,
)
from syncai_backend.subscribers.tf import init_tf_listener


dotenv.load_dotenv()
setup_log_handler()
logger = structlog.get_logger()


class SyncAIBackend(Node):
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        super().__init__("syncai_backend_node")

        robot_id = self.get_namespace().strip("/") or "default_robot"

        # Connect to the PostgreSQL database
        try:
            engine = connect_to_postgres(logger=logger, robot_id=robot_id)
        except Exception as e:
            logger.error("Failed to connect to PostgreSQL", error=str(e))
            raise

        robot_repo = init_robot_repo(logger=logger)
        map_repo = init_map_repo(logger=logger, engine=engine)

        # The operator's library of re-dispatchable step lists.
        task_template_repo = init_task_template_repo(logger=logger, engine=engine)
        # The maps on disk, as opposed to the one that is loaded. Filesystem only.
        map_catalog_repo = init_map_catalog_repo(logger=logger)
        # Single-slot cache for the live body_cloud, drained by the WS stream.
        pointcloud_repo = init_pointcloud_repo(logger=logger)
        # Same slot shape for pgo's merged "map so far" cloud (mapping mode only).
        map_cloud_repo = init_pointcloud_repo(logger=logger)
        # pgo's latched run state, so GET /api/v1/mapping survives a console
        # reload and run routes can refuse with a code. Ages its sample out,
        # because this container outlives the mapping session.
        mapping_status_repo = init_mapping_status_repo(logger=logger)
        # Pose/joints cache feeding the internal telemetry WS (the 3D viewer's
        # high-rate channel).
        telemetry_repo = init_telemetry_repo(logger=logger)
        # The bags on disk, as opposed to the one being written. Filesystem only.
        recording_catalog_repo = init_recording_catalog_repo(logger=logger)

        robot_gw = init_robot_gateway(logger=logger, node=self)
        # LoadMap client. Its own gateway so the map router holds no handle that
        # can command the robot to move.
        map_gw = init_map_gateway(logger=logger, node=self)
        workflow_gw = init_workflow_gateway(logger=logger, robot_id=robot_id)
        # Speech out, over HTTP to the syncai_tts container. The speaker lock
        # lives in that service now, so this is just a pooled httpx.Client.
        tts_gw = init_tts_gateway(logger=logger)
        # WHEP signalling for the camera stream. One instance: the worker's
        # sync.Once InitWorker and the single-viewer slot need a single owner.
        # The dlopen (Go runtime + signal handlers, never unloaded) happens on
        # the first WHEP request, not here.
        webrtc_gw = init_webrtc_gateway(logger=logger)
        # `ros2 bag record` as a supervised child, out of process, so a bag of the
        # lidar cloud never shares this executor. robot_id expands a relative
        # topic (`livox/lidar`) into the namespaced one; records_dir comes off the
        # repo so both halves point at the same directory.
        recording_gw = init_recording_gateway(
            logger=logger,
            robot_id=robot_id,
            records_dir=recording_catalog_repo.records_dir,
        )

        # Whether a map may be edited, destroyed, switched to or driven on. The
        # policy reads disk and the template table only, so the worker holds it too;
        # the gate asks Temporal and stays on the REST loop (WorkflowGateway's).
        map_usage = init_map_usage_policy(
            map_catalog_repo=map_catalog_repo,
            task_template_repo=task_template_repo,
        )
        job_gate = init_job_gate(
            logger=logger, map_catalog_repo=map_catalog_repo, workflow_gw=workflow_gw
        )

        # The live map switch: localizer, Octo3D and the INI moved together,
        # each later failure undoing the earlier ones.
        map_switch_svc = init_map_switch_service(
            logger=logger,
            map_catalog_repo=map_catalog_repo,
            map_gw=map_gw,
        )

        # POST /api/v1/maps/import: the directory and its vertex rows published
        # together, swapped back if the rows cannot be written.
        map_import_svc = init_map_import_service(
            logger=logger,
            map_repo=map_repo,
            map_catalog_repo=map_catalog_repo,
            map_usage=map_usage,
        )

        # The latest restart of the live mode and how it ended; outlives the POST
        # because sys_manager replies only once the rebuild is over.
        restart_svc = init_mode_restart_service(logger=logger, robot_gw=robot_gw)

        # Cancels every task when the driver's safety lock engages. Fed from the
        # ROS thread, run on the REST loop (the only one WorkflowGateway may use).
        safety_lock_svc = init_safety_lock_service(
            logger=logger, robot_gw=robot_gw, workflow_gw=workflow_gw
        )

        # Where the last DOCK ended, so a MOVE off the dock undocks first. In
        # memory on purpose (see services/dock_state.py); only the activities
        # use it today.
        dock_state_svc = init_dock_state_service()

        # One /tf + /tf_static subscription for the whole process (see
        # subscribers/tf.py).
        self._tf_listener = init_tf_listener(logger=logger, node=self)

        init_robot_state_subscriber(
            logger=logger,
            node=self,
            robot_repo=robot_repo,
            safety_lock_svc=safety_lock_svc,
        )
        init_pointcloud_subscriber(
            logger=logger,
            node=self,
            pointcloud_repo=pointcloud_repo,
            tf_buffer=self._tf_listener.buffer,
        )
        # pgo's merged cloud is already in the map frame: no tf_buffer.
        init_map_cloud_subscriber(logger=logger, node=self, map_cloud_repo=map_cloud_repo)
        init_mapping_status_subscriber(
            logger=logger, node=self, mapping_status_repo=mapping_status_repo
        )
        init_telemetry_subscriber(
            logger=logger,
            node=self,
            telemetry_repo=telemetry_repo,
            tf_buffer=self._tf_listener.buffer,
        )
        # The planner's route, onto the telemetry socket; already in the map frame.
        init_path_subscriber(logger=logger, node=self, telemetry_repo=telemetry_repo)
        # The dock's last metre, which no planner draws; also map frame.
        init_docking_plan_subscriber(
            logger=logger, node=self, telemetry_repo=telemetry_repo
        )
        # The controller's local costmap and local plan, onto the same socket
        # while a navigation runs; from odom into the map frame by TF.
        init_local_nav_subscriber(
            logger=logger,
            node=self,
            telemetry_repo=telemetry_repo,
            tf_buffer=self._tf_listener.buffer,
        )

        worker_handle = start_temporal_worker(
            logger=logger,
            robot_id=robot_id,
            robot_gw=robot_gw,
            tts_gw=tts_gw,
            map_catalog_repo=map_catalog_repo,
            map_usage=map_usage,
            robot_repo=robot_repo,
            dock_state=dock_state_svc,
        )
        # Kept for main()'s shutdown: the worker is stopped before the ROS
        # context goes, so a running MOVE can still cancel its nav2 goal.
        self.worker_handle: TemporalWorkerHandle = worker_handle
        start_rest_server(
            logger=logger,
            deps=RestDeps(
                workflow_gw=workflow_gw,
                robot_repo=robot_repo,
                robot_gw=robot_gw,
                map_repo=map_repo,
                map_catalog_repo=map_catalog_repo,
                map_gw=map_gw,
                pointcloud_repo=pointcloud_repo,
                map_cloud_repo=map_cloud_repo,
                telemetry_repo=telemetry_repo,
                task_template_repo=task_template_repo,
                worker_handle=worker_handle,
                tts_gw=tts_gw,
                webrtc_gw=webrtc_gw,
                recording_gw=recording_gw,
                recording_catalog_repo=recording_catalog_repo,
                restart_svc=restart_svc,
                mapping_status_repo=mapping_status_repo,
                safety_lock_svc=safety_lock_svc,
                map_switch_svc=map_switch_svc,
                map_usage=map_usage,
                job_gate=job_gate,
                map_import_svc=map_import_svc,
            ),
        )


# How often the spin loop comes up for air to run a pending Python signal
# handler. rcl_wait is not interrupted by a signal, and the handler only runs
# between bytecodes on the main thread, so without a timeout an idle graph
# could hold a SIGTERM until the next ROS event.
_SPIN_TIMEOUT_S = 0.5

# SIGTERM: docker stop / compose down. SIGINT: ros2 launch's shutdown, Ctrl-C.
# SIGHUP: the byobu session this process is a pane of being killed
# (switch_mode). Without a handler for the last one, a mode switch mid-MOVE
# left the nav2 goal driving with nobody supervising it.
_STOP_SIGNALS = (signal.SIGTERM, signal.SIGINT, signal.SIGHUP)


class _Shutdown:
    """The orderly stop: the Temporal worker first, the ROS context last.

    The order is the point. The worker's shutdown cancels the in-flight MOVE,
    whose cleanup (cancel the nav2 goal, restore locomotion) is ROS service and
    action traffic that needs the executor still spinning. rclpy's own signal
    handler shut the context down first, so the daemon worker thread died with
    the process and that cleanup never ran: nav2 kept the goal, and the next
    worker re-sent it from wherever the robot had got to.

    The signal handler only starts a thread: the stop itself blocks for up to
    WORKER_SHUTDOWN_TIMEOUT_S, and a handler that blocks the main thread would
    stall the very executor the cleanup is waiting on. A second signal while
    the first is being honoured ends the process at once.
    """

    def __init__(self):
        # Bound once the node is built; a stop before that has nothing to wait
        # for (the signal may land during the Postgres retry).
        self._worker_handle: TemporalWorkerHandle | None = None
        self.requested = threading.Event()
        self.done = threading.Event()

    def install(self) -> None:
        for signum in _STOP_SIGNALS:
            signal.signal(signum, self._on_signal)

    def bind(self, worker_handle: TemporalWorkerHandle) -> None:
        self._worker_handle = worker_handle

    def _on_signal(self, signum, _frame) -> None:
        if self.requested.is_set():
            logger.warning("Second stop signal; exiting now", signal=signum)
            os._exit(1)
        self.requested.set()
        logger.info("Stop signal received; stopping the Temporal worker", signal=signum)
        threading.Thread(target=self._run, name="shutdown", daemon=True).start()

    def _run(self) -> None:
        try:
            handle = self._worker_handle
            if handle is None:
                logger.info("Stopped before start-up finished")
            elif handle.request_shutdown(WORKER_SHUTDOWN_TIMEOUT_S):
                logger.info("Temporal worker stopped; shutting down")
            else:
                logger.error(
                    "Temporal worker did not stop in time; shutting down anyway",
                    timeout_s=WORKER_SHUTDOWN_TIMEOUT_S,
                )
        finally:
            self.done.set()


def main():
    # No rclpy signal handlers: they shut the context down on SIGINT/SIGTERM,
    # before anything else gets a say. _Shutdown installs its own.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    shutdown = _Shutdown()
    shutdown.install()

    try:
        backend_node = SyncAIBackend(logger=logger)
        shutdown.bind(backend_node.worker_handle)
        # Lets the point-cloud callback groups run beside robot_state/TF, so a
        # busy scan frame can't starve them.
        executor = MultiThreadedExecutor()
        executor.add_node(backend_node)
        # spin() with a timeout per turn, so a signal handler runs promptly;
        # the loop keeps serving ROS until the worker's cleanup is through.
        while rclpy.ok() and not shutdown.done.is_set():
            executor.spin_once(timeout_sec=_SPIN_TIMEOUT_S)
    except Exception:
        logger.error("Failed to execute SyncAIBackend node", exc_info=True)
    finally:
        rclpy.shutdown()
        # A requested stop exits clean; anything else is a failure, so a
        # restart policy keyed on it (restart: on-failure) acts.
        sys.exit(0 if shutdown.requested.is_set() else 1)


if __name__ == "__main__":
    main()
