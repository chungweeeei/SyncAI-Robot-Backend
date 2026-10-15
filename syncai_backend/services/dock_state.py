"""Where the robot last docked, so a MOVE after a DOCK can undock first.

A MOVE straight off a dock fails: the robot's footprint sits on the dock's
lethal cells in the local costmap, so the controller starts in collision. The
robot side deliberately keeps no "docked" state (SyncAI-Robot-Workspace,
2026-10-15: the task runner cannot see a LIEDOWN sent straight to
`set_motion_key`, and a flag there would hide a blind 1 m back-out inside a
MOVE), so the backend, which sequences DOCK / LIEDOWN / MOVE, keeps it.

What is kept is the last successful DOCK's target pose and map, not a bare
flag: a dock may be any vertex (a CHARGE one or a MOVE one), so "on a dock"
cannot be derived from the map catalogue, and the pose is what lets the check
notice that the robot is no longer there. Every read compares it with the
robot's current pose; a robot that teleop, a cancelled undock or anything else
moved off it drops the record, because the alternative -- trusting a stale
flag -- is a blind 1 m back-out from wherever the robot now stands.

In memory on purpose: a backend restart forgets it, and the next MOVE behaves
as it did before auto-undock existed (it fails on the dock's lethal cells);
POST /api/v1/tasks with an UNDOCK step still gets the robot off by hand. One
record per process is enough, because one backend process serves one robot.
"""

import math
import threading
from dataclasses import dataclass
from typing import Optional

# How far the robot may be from the recorded dock pose and still count as on
# it. The dock tree's goal checker ends the approach within 5 cm / 0.1 rad, and
# the localizer's pose wanders a few centimetres on top of that, so the window
# is a few times wider than the arrival tolerance; a robot that left the dock
# under its own power is 1 m or more away, far outside it. Unmeasured guesses.
ON_DOCK_XY_TOLERANCE_M = 0.2
ON_DOCK_YAW_TOLERANCE_RAD = 0.3


@dataclass(frozen=True)
class DockRecord:
    """The last successful DOCK's target, in its map's frame."""

    map: Optional[str]
    x: float
    y: float
    theta_deg: float


def _yaw_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class DockStateService:
    """The last DOCK target, and whether the robot is still standing on it.

    Thread-safe: activities run on the worker's executor threads.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._record: Optional[DockRecord] = None

    def record(self, map_name: Optional[str], x: float, y: float, theta_deg: float) -> None:
        with self._lock:
            self._record = DockRecord(map=map_name, x=x, y=y, theta_deg=theta_deg)

    def clear(self) -> None:
        with self._lock:
            self._record = None

    def current(self) -> Optional[DockRecord]:
        with self._lock:
            return self._record

    def is_on_dock(self, robot_state, active_map: Optional[str]) -> bool:
        """Whether ``robot_state`` (a syncai_common RobotState, or None) places
        the robot on the recorded dock, on ``active_map``.

        No record: False. A record the robot is provably not at -- another
        map loaded, or a localized pose outside the tolerances -- is dropped
        and reads False. No state yet, or a pose that is the zeroed
        placeholder (``localization_valid`` False), proves nothing either way:
        False, but the record is kept, so a MOVE sent before localization comes
        back does not forget a dock it may still be standing on.
        """
        with self._lock:
            record = self._record
            if record is None:
                return False
            # An unknown map on either side is not a mismatch (move_guard's rule).
            if record.map is not None and active_map is not None and record.map != active_map:
                self._record = None
                return False
            if robot_state is None or not robot_state.localization_valid:
                return False
            pose = robot_state.localization_status.position
            near = (
                math.hypot(pose.x - record.x, pose.y - record.y) <= ON_DOCK_XY_TOLERANCE_M
                and _yaw_error(pose.yaw, math.radians(record.theta_deg))
                <= ON_DOCK_YAW_TOLERANCE_RAD
            )
            if not near:
                self._record = None
            return near
