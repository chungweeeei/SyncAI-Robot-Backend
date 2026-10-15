import threading
import time
import structlog

from dataclasses import dataclass
from typing import Dict, Optional, Tuple


# How long a planned route stays live without being republished. A fallback,
# not the clear: since 2026-10-14 the planner re-sends its last plan on `plan`
# at plan_republish_rate (1 Hz), and a cancelled navigation empties it with an
# explicit empty Path (task_runner -> planner's clear_plan, sent three times),
# which PathSubscriber turns into the empty sample itself. Arrival needs no
# clear either: the re-send is trimmed to a stub inside xy_goal_tolerance. A
# failure deliberately keeps the route up, so the operator can see where it
# failed. What expiry still catches is a planner that stopped re-sending
# (restarted, plan_republish_rate 0, clear_plan unreachable at cancel).
# 6 s is several republish periods, so a dropped BEST_EFFORT sample or two
# never blinks a live route out.
_PATH_TTL_S = 6.0


@dataclass(frozen=True)
class PoseSample:
    """Map-frame robot pose ready to ship over the telemetry WebSocket.

    ``seq`` lets the WebSocket loop skip samples already sent. ``yaw_deg``
    matches the frontend's RobotPose.theta. ``stamp`` is epoch seconds from
    the odom header.
    """

    seq: int
    x: float
    y: float
    z: float
    yaw_deg: float
    stamp: float


@dataclass(frozen=True)
class JointsSample:
    """Joint positions keyed by URDF joint name (MotorState.name), radians."""

    seq: int
    joints: Dict[str, float]
    stamp: float


@dataclass(frozen=True)
class PathSample:
    """The planner's global route, map-frame metres, downsampled for the wire.

    ``points`` is (x, y) pairs; heading is dropped (the route is drawn as a
    band). An empty ``points`` is the explicit "no route" sample (see
    ``TelemetryRepo.get_path``).
    """

    seq: int
    points: Tuple[Tuple[float, float], ...]
    stamp: float


class TelemetryRepo:
    """In-process single-slot caches for the internal telemetry stream.

    Pose, joints and path are sequenced independently because they arrive at
    very different rates. Newest sample only, under a lock (ROS thread writes,
    asyncio reads), as in PointCloudRepo.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger

        self._lock = threading.Lock()
        self._pose_seq = 0
        self._pose: Optional[PoseSample] = None
        self._joints_seq = 0
        self._joints: Optional[JointsSample] = None
        self._path_seq = 0
        self._path: Optional[PathSample] = None
        # Monotonic, not the message stamp: the TTL is about when we heard it.
        self._path_recv_at = 0.0

    def update_pose(self, x: float, y: float, z: float, yaw_deg: float, stamp: float):
        with self._lock:
            self._pose_seq += 1
            self._pose = PoseSample(
                seq=self._pose_seq, x=x, y=y, z=z, yaw_deg=yaw_deg, stamp=stamp
            )

    def update_joints(self, joints: Dict[str, float], stamp: float):
        with self._lock:
            self._joints_seq += 1
            self._joints = JointsSample(
                seq=self._joints_seq, joints=joints, stamp=stamp
            )

    def update_path(self, points: Tuple[Tuple[float, float], ...], stamp: float):
        with self._lock:
            self._path_seq += 1
            self._path = PathSample(seq=self._path_seq, points=points, stamp=stamp)
            self._path_recv_at = time.monotonic()

    def get_pose(self, after_seq: int = 0) -> Optional[PoseSample]:
        """Return the cached pose if newer than ``after_seq``, else None."""
        with self._lock:
            if self._pose is None:
                return None
            if self._pose.seq <= after_seq:
                return None
            return self._pose

    def get_joints(self, after_seq: int = 0) -> Optional[JointsSample]:
        """Return the cached joints if newer than ``after_seq``, else None."""
        with self._lock:
            if self._joints is None:
                return None
            if self._joints.seq <= after_seq:
                return None
            return self._joints

    def get_path(self, after_seq: int = 0) -> Optional[PathSample]:
        """Return the cached path if newer than ``after_seq``, else None.

        Expires a route not republished within _PATH_TTL_S, in place, on read,
        so the rule lives here rather than in every WebSocket client.
        """
        with self._lock:
            if self._path is None:
                return None
            if (
                self._path.points
                and time.monotonic() - self._path_recv_at > _PATH_TTL_S
            ):
                # Bump seq so each client gets exactly one empty sample.
                self._path_seq += 1
                self._path = PathSample(
                    seq=self._path_seq, points=(), stamp=self._path.stamp
                )
            if self._path.seq <= after_seq:
                return None
            return self._path


def init_telemetry_repo(logger: structlog.stdlib.BoundLogger) -> TelemetryRepo:
    return TelemetryRepo(logger)
