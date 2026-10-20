import asyncio
import base64
import time
import structlog

from typing import Callable

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.repositories.telemetry.telemetry import (
    LocalCostmapSample,
    TelemetryRepo,
)


# 20 Hz poll, the lio_bridge odom rate; a type is skipped when nothing is new.
_STREAM_INTERVAL_S = 0.05


def _local_costmap_frame(sample: LocalCostmapSample) -> dict:
    return {
        "type": "local_costmap",
        "resolution": sample.resolution,
        "width": sample.width,
        "height": sample.height,
        "origin": {
            "x": sample.origin[0],
            "y": sample.origin[1],
            "yaw": sample.origin[2],
        },
        # 6400 cells at the robot's 4 x 4 m and 0.05 m: ~8.5 kB as base64,
        # where a JSON list of numbers would be ~20 kB.
        "data": base64.b64encode(sample.data).decode("ascii"),
        "stamp": sample.stamp,
    }


def _local_costmap_clear(stamp: float) -> dict:
    return {
        "type": "local_costmap",
        "resolution": 0.0,
        "width": 0,
        "height": 0,
        "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "data": "",
        "stamp": stamp,
    }


def init_telemetry_router(
    logger: structlog.stdlib.BoundLogger,
    telemetry_repo: TelemetryRepo,
    navigation_active: Callable[[], bool],
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

        Two more, **only while a navigation this backend sent is running** (a
        MOVE, DOCK or UNDOCK goal; ``navigation_active``), because they mean
        something only while the robot drives:

            {"type": "local_costmap", "resolution": .., "width": .., "height": ..,
             "origin": {"x": .., "y": .., "yaw": ..}, "data": "<base64>",
             "stamp": ..}
            {"type": "local_plan", "points": [[x, y], ...], "stamp": ..}

        ``local_costmap`` is the controller's rolling costmap around the robot
        (2 Hz): ``data`` is one byte per cell, row-major from the ``origin``
        corner, x along a row -- 0 free, 1-252 inflation, 253 inscribed, 254
        lethal, 255 unknown -- and the grid is turned by ``origin.yaw`` in the
        map frame. ``local_plan`` is the trajectory the controller is following
        for the next 2 s (5 Hz), map-frame (x, y). Both are planar: draw them
        on the ground the robot stands on. When the navigation ends, however
        it ends, each is sent once empty (``width`` 0, ``points`` []), which is
        the clear; either also clears by its own TTL when its source stops.
        """
        await ws.accept()
        last_pose_seq = 0
        last_joints_seq = 0
        last_path_seq = 0
        last_dock_path_seq = 0
        last_local_costmap_seq = 0
        last_local_plan_seq = 0
        # Whether this client was last sent the local picture: the edge that
        # owes it the two clears.
        navigating = False
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

                if navigation_active():
                    navigating = True
                    costmap = telemetry_repo.get_local_costmap(
                        after_seq=last_local_costmap_seq
                    )
                    if costmap is not None:
                        last_local_costmap_seq = costmap.seq
                        await ws.send_json(_local_costmap_frame(costmap))

                    local_plan = telemetry_repo.get_local_plan(
                        after_seq=last_local_plan_seq
                    )
                    if local_plan is not None:
                        last_local_plan_seq = local_plan.seq
                        await ws.send_json(
                            {
                                "type": "local_plan",
                                "points": local_plan.points,
                                "stamp": local_plan.stamp,
                            }
                        )
                elif navigating:
                    # The navigation ended: the clears are sent, not left to a
                    # TTL, since the robot's costmap goes on publishing.
                    navigating = False
                    now = time.time()
                    await ws.send_json(_local_costmap_clear(now))
                    await ws.send_json({"type": "local_plan", "points": [], "stamp": now})

                await asyncio.sleep(_STREAM_INTERVAL_S)
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("telemetry stream failed", exc_info=True)
            return

    return telemetry_router
