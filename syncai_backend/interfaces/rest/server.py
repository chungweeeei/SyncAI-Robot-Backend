import asyncio
import contextlib
import threading
from dataclasses import dataclass
import uvicorn
import structlog
from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    UpstreamError,
    NotFoundError,
    UnauthorizedError,
)

from syncai_backend.interfaces.rest.routers.task import init_task_router
from syncai_backend.interfaces.rest.routers.schedule import init_schedule_router
from syncai_backend.interfaces.rest.routers.task_template import init_task_template_router
from syncai_backend.interfaces.rest.routers.robot import init_robot_router
from syncai_backend.interfaces.rest.routers.network import init_network_router
from syncai_backend.interfaces.rest.routers.map import init_map_router
from syncai_backend.interfaces.rest.routers.pointcloud import init_pointcloud_router
from syncai_backend.interfaces.rest.routers.telemetry import init_telemetry_router
from syncai_backend.interfaces.rest.routers.teleop import init_teleop_router
from syncai_backend.interfaces.rest.routers.tts import init_tts_router
from syncai_backend.interfaces.rest.routers.webrtc import init_webrtc_router
from syncai_backend.interfaces.rest.routers.recording import init_recording_router

from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.repositories.mapping.mapping import MappingStatusRepo
from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo
from syncai_backend.repositories.task.task_template import TaskTemplateRepo
from syncai_backend.repositories.recording.catalog import RecordingCatalogRepo

from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.gateways.robot.robot import RobotGateway
from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.gateways.tts.tts import TtsGateway
from syncai_backend.gateways.webrtc.webrtc import WebRtcGateway
from syncai_backend.gateways.recording.recording import RecordingGateway

from syncai_backend.services.gridmap_conversion import GridmapConversionService
from syncai_backend.services.map_import import MapImportService
from syncai_backend.services.map_switch import MapSwitchService
from syncai_backend.services.map_usage import JobGate, MapUsagePolicy
from syncai_backend.services.mode_restart import ModeRestartService
from syncai_backend.services.safety_lock import SafetyLockService

from syncai_backend.temporal.worker import TemporalWorkerHandle


def register_exception_handlers(app: FastAPI) -> None:
    """Map domain exceptions to HTTP responses so routers don't have to."""

    def _json(status_code: int, detail: str) -> JSONResponse:
        return JSONResponse(status_code=status_code, content={"detail": detail})

    @app.exception_handler(NotFoundError)
    async def _not_found(_: Request, exc: NotFoundError) -> JSONResponse:
        return _json(status.HTTP_404_NOT_FOUND, str(exc))

    @app.exception_handler(BadRequestError)
    async def _bad_request(_: Request, exc: BadRequestError) -> JSONResponse:
        # Both keys only when set, so every pre-existing 400 is the body it was.
        content: dict = {"detail": str(exc)}
        if getattr(exc, "code", None):
            content["code"] = exc.code
        if getattr(exc, "step_id", None):
            content["step_id"] = exc.step_id
        return JSONResponse(status_code=status.HTTP_400_BAD_REQUEST, content=content)

    @app.exception_handler(ConflictError)
    async def _conflict(_: Request, exc: ConflictError) -> JSONResponse:
        # ``code`` only when set, beside ``detail``.
        content: dict = {"detail": str(exc)}
        if getattr(exc, "code", None):
            content["code"] = exc.code
        return JSONResponse(status_code=status.HTTP_409_CONFLICT, content=content)

    @app.exception_handler(UnauthorizedError)
    async def _unauthorized(_: Request, exc: UnauthorizedError) -> JSONResponse:
        return _json(status.HTTP_401_UNAUTHORIZED, str(exc))

    @app.exception_handler(UpstreamError)
    async def _upstream(_: Request, exc: UpstreamError) -> JSONResponse:
        return _json(status.HTTP_502_BAD_GATEWAY, str(exc))


@dataclass(frozen=True)
class RestDeps:
    """Everything the REST surface is handed, built once in ``main.py``.

    Still explicit constructor injection, not a container: each router's
    ``init_<x>_router(...)`` names the fields it takes, so what a router can
    reach is read off its call below. This only stops the list from being
    written out three times (main.py, start_rest_server, init_rest_server).
    """

    workflow_gw: WorkflowGateway
    robot_repo: RobotRepo
    robot_gw: RobotGateway
    map_repo: MapRepo
    map_catalog_repo: MapCatalogRepo
    map_gw: MapGateway
    pointcloud_repo: PointCloudRepo
    map_cloud_repo: PointCloudRepo
    telemetry_repo: TelemetryRepo
    task_template_repo: TaskTemplateRepo
    worker_handle: TemporalWorkerHandle
    tts_gw: TtsGateway
    webrtc_gw: WebRtcGateway
    recording_gw: RecordingGateway
    recording_catalog_repo: RecordingCatalogRepo
    conversion_svc: GridmapConversionService
    restart_svc: ModeRestartService
    mapping_status_repo: MappingStatusRepo
    safety_lock_svc: SafetyLockService
    map_switch_svc: MapSwitchService
    map_usage: MapUsagePolicy
    job_gate: JobGate
    map_import_svc: MapImportService


def init_rest_server(
    logger: structlog.stdlib.BoundLogger, deps: RestDeps
) -> FastAPI:

    description = """
    This is the backend for the SyncAI Robotic System. It provides APIs for
    controlling and monitoring the robot, as well as managing data and workflows.
    """

    # The safety lock's cancel (started from a ROS callback) must run on this
    # loop, which owns WorkflowGateway. See services/safety_lock.py.
    @contextlib.asynccontextmanager
    async def lifespan(_: FastAPI):
        deps.safety_lock_svc.bind_loop(asyncio.get_running_loop())
        yield

    app = FastAPI(
        title="SyncAI Robot backend Server",
        description=description,
        version="1.0.0",
        lifespan=lifespan,
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"],
        allow_headers=["Content-Type", "Content-Length", "Authorization"],
        # Location is not CORS-safelisted; without this JS cannot read it, a
        # WHEP client never learns its session URL to DELETE, and every page
        # load leaks a camera session while video still plays fine.
        expose_headers=["Location"],
    )

    register_exception_handlers(app=app)

    @app.get("/health")
    async def health() -> dict:
        """Liveness probe, plus the Temporal worker's state as ``task_server``.

        Always HTTP 200: a non-200 in a container healthcheck would
        restart-loop the whole backend while Temporal is down, though the rest
        of the process is healthy. Degradation is reported in the body.
        """
        worker_status, worker_error = deps.worker_handle.snapshot()
        return {
            "status": (
                "ok"
                if worker_status == TemporalWorkerHandle.STATUS_RUNNING
                else "degraded"
            ),
            "task_server": worker_status,
            "task_server_error": worker_error,
        }

    # The catalogue answers which map a dispatched job is stamped with, and
    # map_usage whether it may drive on it at all.
    app.include_router(
        init_task_router(
            logger=logger,
            workflow_gw=deps.workflow_gw,
            map_catalog_repo=deps.map_catalog_repo,
            map_usage=deps.map_usage,
            robot_gw=deps.robot_gw,
        )
    )
    app.include_router(init_schedule_router(logger=logger, workflow_gw=deps.workflow_gw))
    app.include_router(
        init_robot_router(
            logger=logger,
            robot_repo=deps.robot_repo,
            robot_gw=deps.robot_gw,
            restart_svc=deps.restart_svc,
            safety_lock_svc=deps.safety_lock_svc,
        )
    )
    app.include_router(init_network_router(logger=logger, robot_gw=deps.robot_gw))
    # /api/v1/maps: catalogue on disk plus the vertex table. task_template_repo
    # is for rename (task_templates.map_name keys on the directory name);
    # map_usage / job_gate answer whether a map may be changed (job_gate is
    # the one that asks Temporal); mapping_status_repo is pgo's latched run
    # state; map_switch_svc is the live switch's saga.
    app.include_router(
        init_map_router(
            logger=logger,
            map_repo=deps.map_repo,
            map_catalog_repo=deps.map_catalog_repo,
            map_gw=deps.map_gw,
            task_template_repo=deps.task_template_repo,
            conversion_svc=deps.conversion_svc,
            mapping_status_repo=deps.mapping_status_repo,
            map_switch_svc=deps.map_switch_svc,
            map_usage=deps.map_usage,
            job_gate=deps.job_gate,
            map_import_svc=deps.map_import_svc,
        )
    )
    # /api/v1/task_templates: map_repo resolves a MOVE step's vertex to its
    # current pose; workflow_gw is for freezing a template into a schedule. The
    # prefix (not /api/v1/tasks/templates) avoids colliding with
    # /api/v1/tasks/{id}, so include order does not matter.
    app.include_router(
        init_task_template_router(
            logger=logger,
            task_template_repo=deps.task_template_repo,
            map_repo=deps.map_repo,
            map_catalog_repo=deps.map_catalog_repo,
            workflow_gw=deps.workflow_gw,
        )
    )
    # Two streams, one router: the live body_cloud and pgo's merged "map so
    # far" cloud (mapping mode only) share the wire format and the pump.
    app.include_router(
        init_pointcloud_router(
            logger=logger,
            pointcloud_repo=deps.pointcloud_repo,
            map_cloud_repo=deps.map_cloud_repo,
        )
    )
    app.include_router(
        init_telemetry_router(
            logger=logger,
            telemetry_repo=deps.telemetry_repo,
            # The gate for the local costmap and plan frames: a navigation this
            # process sent is running.
            navigation_active=deps.robot_gw.navigation_active,
        )
    )
    # Inbound WS: the console's manual-control velocity frames.
    app.include_router(init_teleop_router(logger=logger, robot_gw=deps.robot_gw))
    # Speech out, via the syncai_tts container over HTTP (it owns the speaker).
    app.include_router(init_tts_router(logger=logger, tts_gw=deps.tts_gw))
    # Video out, as WHEP. The backend owns signalling only; media runs in a Go
    # c-shared library dlopened on first request. See gateways/webrtc.
    app.include_router(init_webrtc_router(logger=logger, webrtc_gw=deps.webrtc_gw))
    # Bag recording needs both halves: the gateway owns the live `ros2 bag
    # record` child, the repo owns record/ on disk. Only holding both can say
    # "no metadata and no process behind it" means interrupted.
    app.include_router(
        init_recording_router(
            logger=logger,
            recording_gw=deps.recording_gw,
            recording_catalog_repo=deps.recording_catalog_repo,
        )
    )

    return app


def start_rest_server(logger: structlog.stdlib.BoundLogger, deps: RestDeps):

    app = init_rest_server(logger=logger, deps=deps)

    def _run():
        uvicorn.run(app, host="0.0.0.0", port=3000)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
