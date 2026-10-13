"""Single-slot cache of pgo's run state (``pgo/mapping_status``).

``GET /api/v1/mapping`` answers from it, and the start / save / reset routes
use it to pre-empt pgo's refusal (409 with a code, not 502 with pgo's sentence).

Unlike the other caches, ``get`` ages the sample out: this process outlives a
switch to a nav session, where no pgo would ever replace a latched MAPPING.
Expiry is applied on read, like ``ModeRestartService``'s deadline.
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import structlog


# How long a latched sample is believed. pgo republishes at 1 Hz.
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
    # pgo's clock, for display only; the TTL uses this process's monotonic clock.
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

        None means "unknown": no pgo has published, or it went silent.
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
