import numpy as np
import rclpy
import structlog
import yaml

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import LookupException, TransformException
from tf2_ros.buffer import Buffer

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.helpers.pointcloud import (
    cap_points,
    pack_xyz_f32,
    transform_points,
    voxel_downsample,
)


class PointCloudSubscriber:
    """Feed the frontend 3D viewer two clouds from ROS.

    1. Live body_cloud: pointlio's per-scan cloud in
       ``<robot_id>/pointlio_body``, transformed into ``map`` through the
       localizer's ``map -> pointlio_odom`` and pointlio's
       ``pointlio_odom -> pointlio_body``. Source frame and its TF parent come
       from the cloud header and the live tree, so renames need no change here
       as long as the cloud's frame has exactly ONE TF parent. Packed into the
       single-slot repo the WS router drains.

    The stored map cloud is read from ``map/<name>/map.pcd`` on request, not
    cached here.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        pointcloud_repo: PointCloudRepo,
        tf_buffer: Buffer,
    ):
        self._logger = logger
        self._pc_repo = pointcloud_repo
        # The shared TF buffer (subscribers/tf.py); tf2's Buffer is thread-safe.
        self._tf_buffer = tf_buffer
        self._voxel_size = 0.0
        self._max_points = 30000
        # Same frame as the stored map cloud, so the two overlay.
        self._target_frame = "map"

        # Edge-triggered TF logging: None until the first frame, then
        # True/False; log once on drop (no localizer) and once on recovery.
        self._cloud_tf_available = None

        # {cloud source frame: its TF parent}, so a rename resolves afresh.
        self._fixed_frames: dict[str, str] = {}

    def _resolve_fixed_frame(self, source_frame: str) -> str:
        """The TF parent of ``source_frame`` — where _cloud_cb splits the chain.

        Read from the live TF tree, not configured, so any odom frame name
        works (``pointlio_odom``, ``lio_odom`` in Isaac). Resolved once per
        source frame. Raises LookupException until the parent is in the buffer.
        """
        cached = self._fixed_frames.get(source_frame)
        if cached is not None:
            return cached

        # The Python Buffer has no per-frame parent accessor; the yaml dump does.
        frames = yaml.safe_load(self._tf_buffer.all_frames_as_yaml()) or {}
        entry = frames.get(source_frame) if isinstance(frames, dict) else None
        parent = entry.get("parent") if isinstance(entry, dict) else None
        if not parent:
            raise LookupException(
                f'cloud frame "{source_frame}" has no TF parent yet '
                "(is pointlio broadcasting?)"
            )

        self._fixed_frames[source_frame] = parent
        self._logger.info(
            "resolved body_cloud TF split point",
            source_frame=source_frame,
            fixed_frame=parent,
        )
        return parent

    def register(self, node: Node):
        # Own MutuallyExclusive group, so a busy scan can't starve other
        # callbacks, and so _cloud_cb's blocking lookup is safe: /tf is in the
        # default group, serviced by another executor thread.
        node.create_subscription(
            msg_type=PointCloud2,
            topic="pointlio/body_cloud",
            callback=self._cloud_cb,
            qos_profile=QoSProfile(
                depth=5,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _cloud_cb(self, msg: PointCloud2):

        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )

        if points.shape[0] == 0:
            return

        # Thin BEFORE transforming, so the transform touches the fewest points.
        points = voxel_downsample(points=points, voxel_size=self._voxel_size)
        points = cap_points(points=points, max_points=self._max_points)

        source_frame = msg.header.frame_id
        if source_frame != self._target_frame:
            try:
                # lookup_transform_full, NOT a single-time lookup: the two
                # halves of map->body are valid at different times.
                #  - All at the cloud stamp always extrapolates: the localizer's
                #    map->pointlio_odom lags ~100 ms, so every frame drops.
                #  - All at Time() places each scan at the CURRENT pose and
                #    smears a moving robot's cloud.
                # So split at the odom frame: map <- odom @ Time() (the slow ICP
                # correction), odom <- body @ msg.header.stamp (this scan's pose).
                #
                # The timeout covers the cloud arriving before /tf has filed its
                # odom->body (~20% of frames otherwise). Safe to block: see
                # register().
                fixed_frame = self._resolve_fixed_frame(source_frame)
                tf = self._tf_buffer.lookup_transform_full(
                    target_frame=self._target_frame,
                    target_time=rclpy.time.Time(),
                    source_frame=source_frame,
                    source_time=rclpy.time.Time.from_msg(msg.header.stamp),
                    fixed_frame=fixed_frame,
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
            except TransformException as exc:
                if self._cloud_tf_available is not False:
                    self._logger.warning(
                        "body_cloud frames dropping: TF unavailable "
                        "(localizer up? map->pointlio_odom comes from "
                        "syncai_localizer once it has pointlio's cloud + odom)",
                        target_frame=self._target_frame,
                        source_frame=source_frame,
                        error=str(exc),
                    )
                    self._cloud_tf_available = False
                return

            t = tf.transform.translation
            q = tf.transform.rotation
            points = transform_points(
                points=points,
                translation=np.array([t.x, t.y, t.z]),
                quat_xyzw=np.array([q.x, q.y, q.z, q.w]),
            )

        if self._cloud_tf_available is not True:
            self._logger.info(
                "body_cloud streaming to frontend",
                target_frame=self._target_frame,
                source_frame=source_frame,
            )
            self._cloud_tf_available = True

        self._pc_repo.update_frame(
            num_points=points.shape[0], data=pack_xyz_f32(points)
        )


def init_pointcloud_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    pointcloud_repo: PointCloudRepo,
    tf_buffer: Buffer,
) -> PointCloudSubscriber:
    subscriber = PointCloudSubscriber(
        logger=logger,
        pointcloud_repo=pointcloud_repo,
        tf_buffer=tf_buffer,
    )
    subscriber.register(node=node)
    return subscriber
