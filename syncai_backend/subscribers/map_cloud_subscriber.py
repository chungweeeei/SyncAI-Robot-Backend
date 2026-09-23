import json

from dataclasses import dataclass
from pathlib import PurePosixPath

import rclpy
import structlog

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.helpers.pointcloud import cap_points, pack_xyz_f32, read_pcd_xyz


@dataclass(frozen=True)
class MapCloudNotice:
    """One ``pgo/map_cloud_file`` message, parsed.

    ``seq`` is pgo's own counter and is informational only: it restarts at 1
    with every pgo process, so a lower seq is not stale, it is a new run.
    ``path`` is ``""`` and ``points`` is ``0`` when the map is empty.
    """

    seq: int
    path: str
    points: int
    frame_id: str


def parse_map_cloud_notice(data: str) -> MapCloudNotice:
    """Parse the JSON pgo publishes on ``pgo/map_cloud_file``.

    Raises ``ValueError`` for anything that is not the five-field object pgo
    writes (see ``mapCloudNoticeJson`` in pgo_node.cpp) -- bad JSON, a missing
    key, or a wrong type. Kept free of ROS so it is unit-testable as-is.
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

    pgo re-merges every keyframe with its *current* loop-closure-corrected pose
    while a mapping (MANUAL) session is up, at most every few seconds and
    subscriber-gated -- this subscription is what un-gates it. The merge itself
    arrives as a **file**, not as a topic payload:

    * pgo writes the voxelised merge as a binary PCD to
      ``/dev/shm/syncai_pgo/<robot_id>/map_cloud_<seq>.pcd`` (tmp + rename, so
      the file is whole or absent, never partial; the newest two are kept), and
    * publishes a ~200 B JSON notice naming it on ``pgo/map_cloud_file``
      (relative, so ``/<robot_id>/pgo/map_cloud_file``).

    Why not the ``PointCloud2`` pgo still publishes on ``pgo/map_cloud`` (it is
    kept for rviz): a large floor at pgo's 0.2 m voxel is ~16 MB per merge and
    44.7 MB at full resolution (2.79 M points, 2026-09). CycloneDDS sends that
    over UDP on ``lo`` as tens of thousands of datagrams in one burst, into a
    receive buffer capped by the kernel's default ``net.core.rmem_max`` of
    208 KB. Fragments drop, and a BEST_EFFORT reader loses the whole sample, so
    the preview simply stopped once the map grew past a size cliff. Raising the
    sysctl is host state on every robot and only moves the cliff; a tmpfs write
    has no cliff at all. The directory is the host's ``/dev/shm``, which both
    containers see because both compose services run with ``ipc: host`` --
    without that the notices still arrive and every read here is ENOENT.

    Why the notice still rides a topic rather than the backend polling the
    directory: it keeps pgo's subscriber gating (no merge when nobody is
    looking), the namespace scoping, and the reset semantics below, all for
    free. And it is RELIABLE + **TRANSIENT_LOCAL** depth 1: latching a notice
    costs nothing, and it means a backend (re)started mid-mapping draws the
    current map at once instead of waiting for the next keyframe. The
    durability has to match pgo's publisher exactly, or nothing is replayed.

    Deliberately NOT a copy of PointCloudSubscriber's pipeline:

    * **No TF.** The cloud is already in the ``map`` frame -- every point was
      placed with the keyframes' corrected global poses at merge time.
    * **No voxel_downsample.** pgo already voxelised at its publish resolution
      (``map_cloud_resolution``). ``cap_points`` (a stride) stays as the one
      wire-size guard.

    The single-slot repo semantics fit exactly: each notice is a complete
    replacement -- a loop closure moves the *whole* map, so deltas are
    impossible.

    An **empty** notice (``points: 0``, ``path: ""``) is a message, not a
    non-event: pgo publishes one from ``reset_mapping`` to say the map has been
    discarded, and it is the only thing that tells a browser to stop drawing a
    map that no longer exists, so it clears the slot rather than being skipped.
    Being latched, it also replaces the notice that named the files the reset
    deleted, so a late joiner never sees a path that is gone.

    Every failure in the callback is a warning that leaves the slot untouched,
    never an exception: this runs on the node's executor, and an exception out
    of a callback ends ``spin()`` and with it the process (``main.py``). A
    missing file is the expected shape of "stale": the notice we are acting on
    can be one behind pgo's newest, pgo keeps two, and a reset or a pgo restart
    in between simply means there is nothing to draw yet.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        map_cloud_repo: PointCloudRepo,
        allowed_root: str = "/dev/shm",
    ):
        self._logger = logger
        self._map_cloud_repo = map_cloud_repo
        # The only place a file named by a topic may come from. A foot-gun
        # guard, not a security boundary -- anyone who can publish on this
        # lo-only DDS domain can drive the robot -- so it is one check: the
        # path must sit under the shared tmpfs (which implies absolute). Tests
        # point it at a tmp dir.
        self._allowed_root = PurePosixPath(allowed_root)
        # ~6 MB per frame at the cap, and real sites reach it: a large floor
        # at pgo's 0.2 m voxel came out at 2.79 M points (44.7 MB on the
        # topic, 2026-09), so the stride keeps every ~6th point. This is the
        # browser's wire/GPU budget, not a guard against a misconfigured
        # resolution -- lower the ceiling here if the viewer struggles, and
        # do not expect the cap to be a no-op on anything bigger than a room.
        self._max_points = 500000

        # Edge-triggered: log once when the first merge lands (the "mapping is
        # visibly producing a map" moment), not per multi-MB frame.
        self._streaming = False

    def register(self, node: Node):
        # Own MutuallyExclusive group for the same reason as the live-cloud
        # subscriber: reading and parsing a multi-MB PCD must not starve the
        # 10 Hz body_cloud callback or the state/TF callbacks on the executor.
        #
        # Depth 1 where the live cloud uses 5: each notice replaces the map
        # wholesale, so a queued older merge is never worth delivering.
        node.create_subscription(
            msg_type=String,
            topic="pgo/map_cloud_file",
            callback=self._notice_cb,
            qos_profile=QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
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
            # ENOENT is the ordinary case: pgo pruned or reset between publishing
            # and our open, or pgo is gone. Keep whatever the slot holds -- the
            # next notice (or the reset's empty one) is the correction.
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
            # read_pcd_xyz drops non-finite rows, so a NaN-only file lands here
            # too. Same call as the live-topic days: both mean "nothing to
            # draw", and a second signal pgo does not send would be needed to
            # tell them apart.
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
