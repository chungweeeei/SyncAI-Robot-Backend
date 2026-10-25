import json

from dataclasses import dataclass
from pathlib import PurePosixPath

import structlog

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from std_msgs.msg import String

from syncai_backend.subscribers import qos
from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.helpers.pointcloud import cap_points, pack_xyz_f32, read_pcd_xyz


@dataclass(frozen=True)
class MapCloudNotice:
    """One ``pgo/map_cloud_file`` message, parsed.

    ``seq`` is informational: it restarts with every pgo process. ``path`` is
    ``""`` and ``points`` is ``0`` when the map is empty.
    """

    seq: int
    path: str
    points: int
    frame_id: str


def parse_map_cloud_notice(data: str) -> MapCloudNotice:
    """Parse the JSON pgo publishes on ``pgo/map_cloud_file``.

    Raises ``ValueError`` for anything but the object pgo writes (see
    ``mapCloudNoticeJson`` in pgo_node.cpp). ROS-free, for unit tests.
    """
    try:
        obj = json.loads(data)
    except json.JSONDecodeError as exc:
        raise ValueError(f"map cloud notice is not JSON: {exc}") from exc
    if not isinstance(obj, dict):
        raise ValueError("map cloud notice is not a JSON object")
    try:
        seq, path, points, frame_id = (
            obj["seq"],
            obj["path"],
            obj["points"],
            obj["frame_id"],
        )
    except KeyError as exc:
        raise ValueError(f"map cloud notice lacks {exc.args[0]!r}") from exc
    # bool is an int subclass; a `true` here would be a bug upstream, not a count.
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise ValueError("map cloud notice seq must be a non-negative integer")
    if not isinstance(points, int) or isinstance(points, bool) or points < 0:
        raise ValueError("map cloud notice points must be a non-negative integer")
    if not isinstance(path, str) or not isinstance(frame_id, str):
        raise ValueError("map cloud notice path and frame_id must be strings")
    return MapCloudNotice(seq=seq, path=path, points=points, frame_id=frame_id)


class MapCloudSubscriber:
    """Feed the console's "map so far" layer from pgo's merged keyframe cloud.

    The merge arrives as a **file**, not a topic payload: pgo writes a binary
    PCD to ``/dev/shm/syncai_pgo/<robot_id>/map_cloud_<seq>.pcd`` (tmp +
    rename, newest two kept) and publishes a ~200 B JSON notice naming it on
    ``pgo/map_cloud_file``. This subscription is what un-gates pgo's merge.

    Do not go back to the ``PointCloud2`` on ``pgo/map_cloud`` (kept for rviz):
    16-45 MB samples overflow the kernel's default UDP receive buffer over
    CycloneDDS, and raising ``net.core.rmem_max`` only moves the cliff. Both
    containers need ``ipc: host`` to share ``/dev/shm``; without it notices
    arrive and every read is ENOENT.

    The notice is RELIABLE + **TRANSIENT_LOCAL** depth 1 so a (re)started
    backend draws the current map at once; durability must match pgo's
    publisher exactly, or nothing is replayed.

    Unlike PointCloudSubscriber: no TF (already in ``map``) and no voxel
    downsample (pgo voxelised); ``cap_points`` is the one wire-size guard.
    Each notice replaces the whole map (a loop closure moves all of it).

    An **empty** notice (``points: 0``, ``path: ""``) is pgo's reset signal and
    clears the slot.

    Every failure in the callback is a warning that leaves the slot untouched,
    never an exception: one out of a callback ends ``spin()`` and the process.
    A missing file is the normal "stale" case (pgo pruned or reset).
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        map_cloud_repo: PointCloudRepo,
        allowed_root: str = "/dev/shm",
    ):
        self._logger = logger
        self._map_cloud_repo = map_cloud_repo
        # A file named by a topic must sit under this root. A foot-gun guard,
        # not a security boundary. Tests point it at a tmp dir.
        self._allowed_root = PurePosixPath(allowed_root)
        # The browser's wire/GPU budget (~6 MB per frame). Real sites exceed it
        # (2.79 M points on a large floor), so expect the stride to apply.
        self._max_points = 500000

        # Edge-triggered: log once when the first merge lands, not per frame.
        self._streaming = False

    def register(self, node: Node):
        # Own MutuallyExclusive group: parsing a multi-MB PCD must not starve
        # the other callbacks. Depth 1: an older merge is never worth delivering.
        node.create_subscription(
            msg_type=String,
            topic="pgo/map_cloud_file",
            callback=self._notice_cb,
            qos_profile=qos.latched(),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _clear(self, notice: MapCloudNotice, reason: str):
        self._streaming = False
        self._map_cloud_repo.update_frame(num_points=0, data=b"")
        self._logger.info(
            "map cloud cleared", reason=reason, seq=notice.seq, frame=notice.frame_id
        )

    def _notice_cb(self, msg: String):
        try:
            notice = parse_map_cloud_notice(msg.data)
        except ValueError as exc:
            self._logger.warning("map cloud notice rejected", error=str(exc))
            return

        if notice.points == 0 or not notice.path:
            self._clear(notice, reason="empty notice")
            return

        path = PurePosixPath(notice.path)
        if self._allowed_root not in path.parents:
            self._logger.warning(
                "map cloud notice names a file outside the shared tmpfs; ignored",
                path=notice.path,
                allowed_root=str(self._allowed_root),
                seq=notice.seq,
            )
            return

        try:
            points = read_pcd_xyz(notice.path)
        except OSError as exc:
            # ENOENT is ordinary (pgo pruned or reset); the next notice corrects.
            self._logger.warning(
                "map cloud file unreadable; keeping the current map",
                path=notice.path,
                seq=notice.seq,
                error=str(exc),
            )
            return
        except (ValueError, KeyError) as exc:
            # ValueError: truncated body, unsupported DATA format, no x/y/z;
            # KeyError: a TYPE/SIZE pair the reader has no dtype for.
            self._logger.warning(
                "map cloud file is not a PCD this reader accepts; keeping the current map",
                path=notice.path,
                seq=notice.seq,
                error=str(exc),
            )
            return

        if points.shape[0] == 0:
            # read_pcd_xyz drops non-finite rows, so a NaN-only file lands here.
            self._clear(notice, reason="no finite points")
            return

        points = cap_points(points=points, max_points=self._max_points)

        if not self._streaming:
            self._streaming = True
            self._logger.info(
                "map cloud streaming",
                num_points=int(points.shape[0]),
                announced_points=notice.points,
                frame=notice.frame_id,
                path=notice.path,
            )

        self._map_cloud_repo.update_frame(
            num_points=points.shape[0], data=pack_xyz_f32(points)
        )


def init_map_cloud_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    map_cloud_repo: PointCloudRepo,
    allowed_root: str = "/dev/shm",
) -> MapCloudSubscriber:
    map_cloud_subscriber = MapCloudSubscriber(
        logger=logger, map_cloud_repo=map_cloud_repo, allowed_root=allowed_root
    )
    map_cloud_subscriber.register(node=node)
    return map_cloud_subscriber
