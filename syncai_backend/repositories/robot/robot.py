"""Single-slot cache of the robot's aggregated state (``robot_state``).

``GET /api/v1/robot/state`` answers from it, the task guard reads the robot's
state machine off it (the dispatch's advisory 409, the check before every
navigation goal), and the landmark route reads which storey the robot is on.

Like ``MappingStatusRepo`` and unlike the other caches, a read ages the sample
out: syncai_robot_state can die (or be torn down by a mode switch) while this
process lives on, and the last thing it said would otherwise go on answering
for it -- a state chip showing IDLE for a robot nobody is reporting on, a
navigation goal sent on a confirmation nobody is vouching for any more.
Expiry is applied on read, on this process's monotonic clock.
"""

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import structlog

from syncai_common.msg import RobotState as RobotStateMsg, RobotStatus


# How long a sample is believed. Tied to syncai_robot_state's shipped publish
# rate, 1 Hz (params publish_rate: 1.0; it also publishes at once on every
# transition of its state machine): three periods, so that two best-effort
# frames dropped in a row -- the subscription is BEST_EFFORT, depth 3 -- are
# not read as a dead robot, and a dead one is noticed within a few seconds
# rather than never. Change the publish rate and this moves with it.
ROBOT_STATE_MAX_AGE_S = 3.0


# Reverse lookup: RobotStatus uint8 constant -> name. Here, beside the slot the
# code is read from, because three readers share it: GET /api/v1/robot/state,
# POST /api/v1/robot/state's answer, and the task guard (helpers/task_guard.py,
# which reads the names so that it stays free of syncai_common). All six are
# emitted since syncai_robot_state took the state machine (2026-11; RUNNING and
# ERROR were declared and never derived before). A code outside the message
# reads UNKNOWN, never a KeyError: an interface that grows a state must not
# 500 the state endpoint or fail a task on a lookup.
ROBOT_STATUS_NAMES = {
    RobotStatus.UNINITIALIZED: "UNINITIALIZED",
    RobotStatus.IDLE: "IDLE",
    RobotStatus.RUNNING: "RUNNING",
    RobotStatus.WARNING: "WARNING",
    RobotStatus.ERROR: "ERROR",
    RobotStatus.CHARGING: "CHARGING",
}


def status_name(status: int) -> str:
    return ROBOT_STATUS_NAMES.get(status, "UNKNOWN")


@dataclass(frozen=True)
class RobotStateSample:
    """One read of the slot: the sample if it is still believed, and how long
    ago the last one arrived whether or not it is.

    Both from one look under the lock, for a reader that must tell "never
    heard from" (``age_s`` None) from "went silent" (``state`` None, ``age_s``
    set) and say for how long -- two separate reads could straddle a sample.
    """

    state: Optional[RobotStateMsg]
    age_s: Optional[float]


class RobotRepo:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        now: Callable[[], float] = time.monotonic,
    ):
        self.logger = logger
        # Injected, as MappingStatusRepo's is, so the ageing is tested by
        # moving time rather than waiting.
        self._now = now

        # In-Process memory cache for robot state
        self._robot_state_lock = threading.Lock()
        self._robot_state: Optional[RobotStateMsg] = None
        # When the sample in the slot arrived, on this process's clock: the
        # message's own `timestamp` is the publisher's wall clock in whole
        # seconds, which can neither order samples nor be compared to ours.
        self._received_at: Optional[float] = None

    def update_robot_state(self, state: RobotStateMsg):
        with self._robot_state_lock:
            self._robot_state = state
            self._received_at = self._now()

    def sample(self) -> RobotStateSample:
        """The slot as it reads now: the sample while it is fresh, and its age.

        At exactly ROBOT_STATE_MAX_AGE_S a sample is still believed; past it,
        ``state`` is None and only ``age_s`` says there ever was one.
        """
        with self._robot_state_lock:
            state, received_at = self._robot_state, self._received_at
        if state is None or received_at is None:
            return RobotStateSample(state=None, age_s=None)
        age_s = self._now() - received_at
        if age_s > ROBOT_STATE_MAX_AGE_S:
            return RobotStateSample(state=None, age_s=age_s)
        return RobotStateSample(state=state, age_s=age_s)

    def get_robot_state(self) -> Optional[RobotStateMsg]:
        """The latest sample, or None when there is none or it has aged out.

        The default read, and deliberately the strict one: every reader gets
        "unknown" for a robot that has stopped reporting, and none gets the
        last sample whatever its age -- there is no accessor for that, because
        no reader has a use for a state nobody is reporting any more.
        """
        return self.sample().state

    def sample_age(self) -> Optional[float]:
        """Seconds since the last sample arrived, fresh or not; None before
        any. For the sentence that says how long the robot has been silent."""
        return self.sample().age_s

    def status(self) -> Optional[str]:
        """The fresh sample's ``state`` by name, or None when there is none
        (never received, or aged out).

        What the task guard reads. None is "nobody is saying", and what that
        means is the caller's: the dispatch's advisory check lets it through,
        because the robot's own begin_task answers for the run (and fails
        closed when the state service is gone); the check before a navigation
        goal holds the goal back, because nothing answers after it.
        """
        state = self.get_robot_state()
        return None if state is None else status_name(state.state)

    def state_message(self) -> str:
        """The robot's own sentence for its state (``state_message``), or ""
        without a fresh sample.

        For ERROR it says what failed, which is what a refused dispatch tells
        the operator.
        """
        state = self.get_robot_state()
        return "" if state is None else state.state_message


def init_robot_repo(
    logger: structlog.stdlib.BoundLogger, now: Callable[[], float] = time.monotonic
) -> RobotRepo:
    robot_repo = RobotRepo(logger, now=now)
    return robot_repo
