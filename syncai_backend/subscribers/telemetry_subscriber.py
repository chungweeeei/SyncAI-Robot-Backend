import math
import time

import numpy as np
import rclpy
import structlog

from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from syncai_common.msg import MotorStates

from syncai_backend.helpers.pointcloud import quat_to_rotation_matrix
from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# How long one `odom` sample keeps ownership of the pose: well above a 20 Hz
# period, so the mapping fallback stays inert in AUTO through a hiccup.
PRIMARY_POSE_HOLD_S = 1.0
# How long one `lio_odom` height stands in for the odom feed's flat z: well
# above its ~10 Hz (one per scan), so a late scan does not drop the robot a
# storey for a frame.
LIO_HEIGHT_HOLD_S = 1.0

# The URDF frames the mapping fallback composes through (fixed by
# description/G23.urdf), under the LIO body frame's namespace prefix.
BASE_FRAME = "base_link"
LIDAR_FRAME = "lidar_top"


def _rigid(translation, rotation) -> tuple[np.ndarray, np.ndarray]:
    """(3x3 rotation, 3-vector translation) from a Transform's or Pose's pair."""
    return (
        quat_to_rotation_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
        np.array([translation.x, translation.y, translation.z]),
    )


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (radians) of a quaternion: the ZYX yaw component."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TelemetrySubscriber:
    """Feed the internal telemetry WebSocket from the raw high-rate topics.

    Bypasses ``robot_state`` (whose REST payload is a frozen contract with
    whole-second timestamps): the 3D viewer needs pose at gait rate.

    1. Pose, from whichever feed the running mode provides:

       * AUTO: ``odom`` (lio_bridge, 20 Hz) composed with TF's map->odom as
         planar math (both inputs are 2D-projected). Dropped, with an
         edge-triggered log, until the localizer provides map->odom.
       * MANUAL: ``pointlio/lio_odom`` composed the long way round (see
         ``_lio_odom_cb``); the mapping session has no ``odom`` or map->odom.

       ``odom`` wins whenever it is live (``PRIMARY_POSE_HOLD_S``): it is what
       the stack navigates by. Except for z: the projection makes the planar
       chain's z 0 on every storey, so in AUTO the height is the LIO chain's
       (``LIO_HEIGHT_HOLD_S``), the same number MANUAL reports.

    2. Joints: ``motor_states`` reduced to {URDF joint name: position}, the
       names the frontend looks GLB nodes up by.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        telemetry_repo: TelemetryRepo,
        tf_buffer: Buffer,
    ):
        self._logger = logger
        self._repo = telemetry_repo
        # The shared TF buffer (subscribers/tf.py); tf2's Buffer is thread-safe.
        self._tf_buffer = tf_buffer

        # Edge-triggered TF logging, as PointCloudSubscriber._cloud_tf_available.
        self._tf_available = None
        # The same, for the mapping fallback's own (different) lookups.
        self._lio_tf_available = None

        # When the primary feed last delivered (monotonic). -inf so the
        # fallback is live at once in a session with no primary feed.
        self._primary_seen_s = float("-inf")
        # base_link's map-frame z from the LIO chain and when (monotonic), for
        # the primary feed. One tuple, so a reader never pairs a z with
        # another sample's time.
        self._lio_height = (0.0, float("-inf"))

    def register(self, node: Node):
        # Best-effort keep-last: only the newest sample matters. Matches odom
        # (reliable) and motor_states (SensorDataQoS, best-effort, must match).
        qos = QoSProfile(
            depth=5,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )

        # Relative topic names, so both inherit the robot_id namespace.
        node.create_subscription(
            msg_type=Odometry, topic="odom", callback=self._odom_cb, qos_profile=qos
        )
        # The mapping-mode pose feed, subscribed unconditionally: an unpublished
        # topic costs nothing.
        node.create_subscription(
            msg_type=Odometry,
            topic="pointlio/lio_odom",
            callback=self._lio_odom_cb,
            qos_profile=qos,
        )
        node.create_subscription(
            msg_type=MotorStates,
            topic="motor_states",
            callback=self._motor_states_cb,
            qos_profile=qos,
        )

    def _odom_cb(self, msg: Odometry):
        # Before the TF gate: it means "lio_bridge is running", true of a
        # dropped sample too (and the fallback would be blind then as well).
        self._primary_seen_s = time.monotonic()

        odom_frame = msg.header.frame_id
        try:
            # Latest (Time()), not the stamp: map->odom varies slowly, and the
            # stamp would extrapolate into the future.
            tf = self._tf_buffer.lookup_transform("map", odom_frame, rclpy.time.Time())
        except TransformException as exc:
            if self._tf_available is not False:
                self._logger.warning(
                    "telemetry pose dropping: TF unavailable (relocalized yet? "
                    "map->odom comes from lio_bridge only after the localizer "
                    "is up)",
                    odom_frame=odom_frame,
                    error=str(exc),
                )
                self._tf_available = False
            return

        if self._tf_available is not True:
            self._logger.info("telemetry pose streaming", odom_frame=odom_frame)
            self._tf_available = True

        # Planar compose: map_T_base = map_T_odom * odom_T_base (exact, since
        # lio_bridge projects everything to x, y, yaw).
        t = tf.transform.translation
        q = tf.transform.rotation
        tf_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        p = msg.pose.pose.position
        oq = msg.pose.pose.orientation
        odom_yaw = _yaw_from_quat(oq.x, oq.y, oq.z, oq.w)

        # lio_bridge projects both onto the plane, so t.z + p.z is 0 upstairs
        # too; the LIO chain's height stands in while it is fresh.
        lio_z, lio_z_s = self._lio_height
        if time.monotonic() - lio_z_s < LIO_HEIGHT_HOLD_S:
            z = lio_z
        else:
            z = t.z + p.z

        cos_y, sin_y = math.cos(tf_yaw), math.sin(tf_yaw)
        self._repo.update_pose(
            x=t.x + cos_y * p.x - sin_y * p.y,
            y=t.y + sin_y * p.x + cos_y * p.y,
            z=z,
            yaw_deg=math.degrees(tf_yaw + odom_yaw),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        )

    def _lio_odom_cb(self, msg: Odometry):
        """Map-frame base_link pose during a mapping run.

        The mapping TF tree is two disconnected halves (``map -> pointlio_odom
        -> pointlio_body`` and ``base_link -> lidar_top``; lio_bridge joins them
        only in AUTO). The LIO body frame IS the lidar, so

            map_T_base = map_T_pointlio_odom . odom_T_body . (base_T_lidar)^-1

        closes the tree. Full 3x3 algebra, not planar: the mount is tilted and
        the LIO solution is 6-DOF, so composing yaws would bake the tilt into
        the heading. z is kept, or the robot would float above its map.

        In AUTO the same chain exists (the localizer broadcasts
        map->pointlio_odom), but ``odom`` owns the pose; this then only hands
        it the height, which is the one thing the planar chain cannot carry.
        """
        body_frame = msg.child_frame_id
        # The namespace prefix is taken from the frame id, so an upstream rename
        # needs no edit here.
        ns = body_frame.rsplit("/", 1)[0] + "/" if "/" in body_frame else ""
        try:
            map_tf = self._tf_buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time()
            )
            # Static, but looked up per sample (cheap) rather than cached.
            base_lidar = self._tf_buffer.lookup_transform(
                f"{ns}{BASE_FRAME}", f"{ns}{LIDAR_FRAME}", rclpy.time.Time()
            )
        except TransformException as exc:
            if self._lio_tf_available is not False:
                self._logger.warning(
                    "telemetry LIO pose dropping: TF unavailable (map->"
                    "pointlio_odom comes from pgo while mapping, from the "
                    "localizer in AUTO; is bringup up for the URDF?)",
                    body_frame=body_frame,
                    error=str(exc),
                )
                self._lio_tf_available = False
            return

        if self._lio_tf_available is not True:
            self._logger.info("telemetry pose streaming from LIO", body_frame=body_frame)
            self._lio_tf_available = True

        map_r, map_t = _rigid(map_tf.transform.translation, map_tf.transform.rotation)
        odom_r, odom_t = _rigid(msg.pose.pose.position, msg.pose.pose.orientation)
        mount_r, mount_t = _rigid(
            base_lidar.transform.translation, base_lidar.transform.rotation
        )

        lidar_base_r = mount_r.T
        lidar_base_t = -mount_r.T @ mount_t

        rot = map_r @ odom_r @ lidar_base_r
        pos = map_t + map_r @ (odom_t + odom_r @ lidar_base_t)

        self._lio_height = (float(pos[2]), time.monotonic())
        if time.monotonic() - self._primary_seen_s < PRIMARY_POSE_HOLD_S:
            return

        self._repo.update_pose(
            x=float(pos[0]),
            y=float(pos[1]),
            z=float(pos[2]),
            # Heading with the mount tilt already removed.
            yaw_deg=math.degrees(math.atan2(rot[1, 0], rot[0, 0])),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        )

    def _motor_states_cb(self, msg: MotorStates):
        self._repo.update_joints(
            joints={m.name: float(m.q) for m in msg.states},
            # MotorStates.timestamp is nanoseconds ON THIS TOPIC. Do not "fix"
            # it against RobotState.motor_status.timestamp, which is seconds
            # (syncai_robot_state rescales it).
            stamp=msg.timestamp * 1e-9,
        )


def init_telemetry_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    telemetry_repo: TelemetryRepo,
    tf_buffer: Buffer,
) -> TelemetrySubscriber:
    subscriber = TelemetrySubscriber(
        logger=logger, telemetry_repo=telemetry_repo, tf_buffer=tf_buffer
    )
    subscriber.register(node=node)
    return subscriber
