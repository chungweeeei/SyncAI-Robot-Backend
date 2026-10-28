import math
import time
from typing import Optional, Tuple

import structlog

import rclpy
from rclpy.node import Node

from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from syncai_backend.subscribers import qos
from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# The frame everything on the telemetry socket is in.
_MAP_FRAME = "map"

# The local plan arrives at the controller's 20 Hz; a line the console redraws
# five times a second looks as live and costs a quarter of the frames.
_LOCAL_PLAN_MIN_INTERVAL_S = 0.2

# Consecutive points closer than this (metres) are one point, as in
# PathSubscriber: a zero-length segment blanks the frontend's ribbon.
_MIN_POINT_SPACING_M = 1e-3


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (radians) of a quaternion: the ZYX yaw component."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LocalNavSubscriber:
    """The robot's local navigation picture onto the telemetry WebSocket: the
    controller's rolling local costmap and the local plan MPPI follows.

    Both are in the local costmap's frame (``<robot_id>/odom``) and are put into
    the map frame here with TF's map->odom, as planar math: lio_bridge projects
    that transform onto the plane (x, y, yaw), so the composition is exact.
    Until the localizer has provided map->odom both are dropped, with an
    edge-triggered log, as TelemetrySubscriber drops the pose.

    * ``local_costmap/costmap/raw`` (``nav2_msgs/Costmap``), not the
      OccupancyGrid on ``local_costmap/costmap``: the publisher sends a whole
      OccupancyGrid only when the rolling window's origin moves, and only
      ``costmap_updates`` while the robot stands, so a reader that joins with
      the robot stopped would get nothing. The raw message is the whole map on
      every publish (2 Hz), with the costmap's own 0-255 values: the terrain
      layer's, inflated.
    * ``local_plan`` (``nav_msgs/Path``), MPPI's optimal trajectory for the
      next 2 s, published only while something subscribes and only on the
      cycles its optimizer runs (not on a stair, nor in the final rotation).
      Kept at most every _LOCAL_PLAN_MIN_INTERVAL_S.

    Both are stored whether or not a navigation runs; the telemetry router
    decides what to send (see ``RobotGateway.navigation_active``).
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        telemetry_repo: TelemetryRepo,
        tf_buffer: Buffer,
    ):
        self._logger = logger
        self._repo = telemetry_repo
        self._tf_buffer = tf_buffer

        # Edge-triggered TF logging, tri-state as in TelemetrySubscriber.
        self._tf_available: Optional[bool] = None
        # Monotonic time the last local plan was kept.
        self._last_plan_s = float("-inf")

    def register(self, node: Node):
        # RELIABLE + TRANSIENT_LOCAL, the costmap publisher's own QoS: the
        # latest costmap arrives on subscribe instead of on the next publish.
        node.create_subscription(
            msg_type=Costmap,
            topic="local_costmap/costmap/raw",  # relative: the robot_id namespace
            callback=self._costmap_cb,
            qos_profile=qos.latched(),
        )
        # BEST_EFFORT against MPPI's RELIABLE depth-1 publisher: only the
        # newest trajectory is worth drawing.
        node.create_subscription(
            msg_type=Path,
            topic="local_plan",
            callback=self._local_plan_cb,
            qos_profile=qos.best_effort(depth=1),
        )

    def _map_from(self, frame: str) -> Optional[Tuple[float, float, float]]:
        """map_T_frame as planar (x, y, yaw), or None while TF lacks it."""
        try:
            # Latest, not the stamp: map->odom moves slowly, and the stamp
            # would extrapolate into the future.
            tf = self._tf_buffer.lookup_transform(_MAP_FRAME, frame, rclpy.time.Time())
        except TransformException as exc:
            if self._tf_available is not False:
                self._logger.warning(
                    "local costmap / plan dropping: TF unavailable (relocalized "
                    "yet? map->odom comes from lio_bridge only after the "
                    "localizer is up)",
                    frame=frame,
                    error=str(exc),
                )
                self._tf_available = False
            return None
        if self._tf_available is not True:
            self._logger.info("local costmap / plan streaming", frame=frame)
            self._tf_available = True
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, _yaw_from_quat(q.x, q.y, q.z, q.w))

    @staticmethod
    def _apply(
        tf: Tuple[float, float, float], x: float, y: float
    ) -> Tuple[float, float]:
        tx, ty, tyaw = tf
        c, s = math.cos(tyaw), math.sin(tyaw)
        return (tx + c * x - s * y, ty + s * x + c * y)

    def _costmap_cb(self, msg: Costmap):
        meta = msg.metadata
        width, height = int(meta.size_x), int(meta.size_y)
        data = bytes(msg.data)
        if width * height != len(data):
            # A malformed message would be drawn as a smeared grid.
            self._logger.warning(
                "local costmap dropped: size and data disagree",
                width=width,
                height=height,
                cells=len(data),
            )
            return

        tf = self._map_from(msg.header.frame_id)
        if tf is None:
            return
        o = meta.origin
        x, y = self._apply(tf, o.position.x, o.position.y)
        q = o.orientation
        yaw = tf[2] + _yaw_from_quat(q.x, q.y, q.z, q.w)

        self._repo.update_local_costmap(
            resolution=float(meta.resolution),
            width=width,
            height=height,
            origin=(round(x, 4), round(y, 4), round(yaw, 6)),
            data=data,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        )

    def _local_plan_cb(self, msg: Path):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not msg.poses:
            # A clear, honoured at once and in any frame (PathSubscriber's rule).
            self._repo.update_local_plan(points=(), stamp=stamp)
            return

        now = time.monotonic()
        if now - self._last_plan_s < _LOCAL_PLAN_MIN_INTERVAL_S:
            return

        tf = self._map_from(msg.header.frame_id)
        if tf is None:
            return
        self._last_plan_s = now

        points = []
        for pose in msg.poses:
            p = pose.pose.position
            x, y = self._apply(tf, p.x, p.y)
            xy = (round(x, 3), round(y, 3))
            if points and all(
                abs(a - b) < _MIN_POINT_SPACING_M for a, b in zip(points[-1], xy)
            ):
                continue
            points.append(xy)
        self._repo.update_local_plan(points=tuple(points), stamp=stamp)


def init_local_nav_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    telemetry_repo: TelemetryRepo,
    tf_buffer: Buffer,
) -> LocalNavSubscriber:
    subscriber = LocalNavSubscriber(
        logger=logger, telemetry_repo=telemetry_repo, tf_buffer=tf_buffer
    )
    subscriber.register(node=node)
    return subscriber
