"""Single-slot cache of pgo's run state (``pgo/mapping_status``).

The third of pgo's surfaces this process mirrors, next to the live merge
(``map_cloud_repo``) and the services the map gateway calls, and the one REST
reads back: ``GET /api/v1/mapping`` answers from it, and the start / save /
reset routes pre-empt the refusal pgo would give from it (a 409 with a code
rather than a 502 with pgo's sentence). pgo comes up IDLE in a mapping session
since 2026-10 and banks nothing until ``start_mapping``; a successful
``save_maps`` returns it to IDLE. The console needs that answer after a page
reload, which is why the topic is latched and why this slot exists.

``get`` ages the sample out. The other single-slot caches here never do
(``main.py`` notes the process used to restart with every mapping session), but
this backend runs as its own container now and outlives a MANUAL -> AUTO
switch, and a latched MAPPING sample from a pgo that no longer exists would
otherwise be reported forever -- there is no pgo in a nav session to say
otherwise. pgo republishes at 1 Hz; five periods is enough slack for a busy
executor and short enough that a dead pgo reads as ``unknown`` before an
operator acts on it. Read-time expiry rather than a timer, the same way
``ModeRestartService`` applies its deadline: nothing to schedule, nothing to
cancel, and the injected clock makes it testable.
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import structlog


# How long a latched sample is believed. pgo publishes on every transition and
# at 1 Hz; past this the likelier story is that the mapping session is gone.
MAPPING_STATUS_TTL_S = 5.0


class MappingState(str, Enum):
    """pgo's run state, in REST vocabulary. Mirrors the constants of
    ``syncai_common/msg/MappingStatus``; the subscriber does the mapping so
    nothing above it imports a ROS type."""

    IDLE = "idle"
    MAPPING = "mapping"
    RESETTING = "resetting"


@dataclass(frozen=True)
class MappingStatus:
    state: MappingState
    key_poses: int
    loop_closures: int
    # pgo's clock when it published, seconds. For display and log correlation
    # only -- the TTL is measured on this process's monotonic clock, because
    # the two clocks need not agree.
    stamp: float
    received_at: float


class MappingStatusRepo:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        now: Callable[[], float] = time.monotonic,
    ):
        self._logger = logger
        self._now = now
        self._lock = threading.Lock()
        self._status: Optional[MappingStatus] = None

    def update(self, state: MappingState, key_poses: int, loop_closures: int, stamp: float):
        with self._lock:
            self._status = MappingStatus(
                state=state,
                key_poses=key_poses,
                loop_closures=loop_closures,
                stamp=stamp,
                received_at=self._now(),
            )

    def get(self) -> Optional[MappingStatus]:
        """The latest sample, or None when there is none or it has expired.

        None is the one representation of "unknown": no pgo has published since
        this process started (the normal state of a navigating robot), or the
        last one to publish has been silent for longer than the TTL.
        """
        with self._lock:
            status = self._status
        if status is None:
            return None
        if self._now() - status.received_at > MAPPING_STATUS_TTL_S:
            return None
        return status


def init_mapping_status_repo(
    logger: structlog.stdlib.BoundLogger, now: Callable[[], float] = time.monotonic
) -> MappingStatusRepo:
    return MappingStatusRepo(logger=logger, now=now)
