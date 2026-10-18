import asyncio
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# 20 Hz poll, the lio_bridge odom rate; a type is skipped when nothing is new.
_STREAM_INTERVAL_S = 0.05


def init_telemetry_router(
    logger: structlog.stdlib.BoundLogger, telemetry_repo: TelemetryRepo
) -> APIRouter:
    telemetry_router = APIRouter(prefix="", tags=["Telemetry"])

    @telemetry_router.websocket("/api/v1/robot/telemetry/stream")
    async def stream_telemetry(ws: WebSocket):
        """Stream high-rate robot telemetry for the frontend 3D viewer.

        Internal, so unlike the frozen GET /api/v1/robot/state it may change
        shape freely; it shares no models with routers/robot.py.

        Wire format: JSON text frames, multiplexed by ``type``:

            {"type": "pose",   "x": .., "y": .., "z": .., "yaw_deg": ..,
             "stamp": ..}
            {"type": "joints", "joints": {"FL_HipX_joint": .., ...},
             "stamp": ..}
            {"type": "path",   "points": [[x, y, z], ...], "stamp": ..}
            {"type": "dock_path", "points": [[x, y, z], ...], "stamp": ..}

        Pose is map-frame planar (yaw in degrees); joints are radians by URDF
        joint name. Each type is sent only when newer, at its own rate.

        ``path`` is the planner's remaining global route in map-frame metres,
        thinned, each time the planner re-sends it (1 Hz). z is the ground
        each point stands on, as a waypoint's z is. An **empty** ``points``
        means "no route": the frame that tells a client to erase what it
        draws -- sent when a navigation is cancelled (the planner publishes an
        empty plan) or when the repo's TTL expires a route that stopped
        arriving.

        ``dock_path`` is the last metre of a dock: the approach the docking
        controller predicts from the robot to the dock, map-frame metres,
        thinned like ``path`` (z included), at 2 Hz while it drives. The
        staging leg before it is an ordinary ``path``. Empty ``points`` clears
        it, sent when the dock goal ends however it ends, or by its own (3 s)
        TTL.
        """
        await ws.accept()
        last_pose_seq = 0
        last_joints_seq = 0
        last_path_seq = 0
        last_dock_path_seq = 0
        try:
            while True:
                pose = telemetry_repo.get_pose(after_seq=last_pose_seq)
                if pose is not None:
                    last_pose_seq = pose.seq
                    await ws.send_json(
                        {
                            "type": "pose",
                            "x": pose.x,
                            "y": pose.y,
                            "z": pose.z,
                            "yaw_deg": pose.yaw_deg,
                            "stamp": pose.stamp,
                        }
                    )

                joints = telemetry_repo.get_joints(after_seq=last_joints_seq)
                if joints is not None:
                    last_joints_seq = joints.seq
                    await ws.send_json(
                        {
                            "type": "joints",
                            "joints": joints.joints,
                            "stamp": joints.stamp,
                        }
                    )

                # Polling also runs the repo's TTL check, so a route the
                # planner stopped re-sending clears with no further ROS traffic.
                path = telemetry_repo.get_path(after_seq=last_path_seq)
                if path is not None:
                    last_path_seq = path.seq
                    await ws.send_json(
                        {
                            "type": "path",
                            "points": path.points,
                            "stamp": path.stamp,
                        }
                    )

                dock_path = telemetry_repo.get_docking_plan(
                    after_seq=last_dock_path_seq
                )
                if dock_path is not None:
                    last_dock_path_seq = dock_path.seq
                    await ws.send_json(
                        {
                            "type": "dock_path",
                            "points": dock_path.points,
                            "stamp": dock_path.stamp,
                        }
                    )

                await asyncio.sleep(_STREAM_INTERVAL_S)
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("telemetry stream failed", exc_info=True)
            return

    return telemetry_router
