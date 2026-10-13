"""The latest restart_mode dispatch and how it ended.

sys_manager answers only once the rebuild is over (tens of seconds), so this
record is the only statement that the rebuild worked. GET /robot/state cannot
stand in: it keeps serving the last robot_state frame through the outage.

Kept in memory on purpose: the outcome belongs to the rebuild this process
watched, so a backend restarted meanwhile honestly reports `idle`.
"""

import threading
from dataclasses import dataclass, replace
from datetime import datetime, timedelta, timezone
from enum import Enum
from typing import Callable, Optional, Tuple

import structlog

from syncai_backend.exceptions import ConflictError


# How long a restart may go unanswered before it is reported failed. The
# rebuild takes under a minute; past three, sys_manager likely died mid-rebuild.
# Without a ceiling the record would say `restarting` forever and block the next.
RESTART_DEADLINE = timedelta(minutes=3)

RESTART_RUNNING = "restart_running"


class RestartStatus(str, Enum):
    IDLE = "idle"
    RESTARTING = "restarting"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


@dataclass(frozen=True)
class RestartRecord:
    status: RestartStatus
    message: str
    started_at: Optional[datetime]
    finished_at: Optional[datetime]


IDLE = RestartRecord(RestartStatus.IDLE, "", None, None)


class ModeRestartService:
    """One restart at a time, and how the latest one ended.

    `robot_gw` is anything with the gateway's ``restart_mode(on_done=...)``.
    `now` is injected so the deadline can be tested without waiting for it.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_gw,
        now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
    ):
        self._logger = logger
        self._robot_gw = robot_gw
        self._now = now
        self._lock = threading.Lock()
        self._record = IDLE
        # The attempt a late answer belongs to: an expired restart's answer
        # must not overwrite a newer attempt's record.
        self._attempt: Optional[object] = None

    def start(self) -> Tuple[Optional[bool], str]:
        """Dispatch a restart; return the gateway's three-valued answer.

        ``(False, msg)``: refused or unreachable, the previous record stands.
        ``(True, msg)``: finished inside the ack window. ``(None, msg)``: under
        way, GET reports the rest. Raises ConflictError(code="restart_running")
        while one is.
        """
        attempt = object()
        with self._lock:
            current = self._expire(self._record)
            if current.status is RestartStatus.RESTARTING:
                raise ConflictError(
                    "The robot is already restarting. Wait for it to finish.",
                    code=RESTART_RUNNING,
                )
            previous = current
            # Recorded before the blocking call so a second press in the ack
            # window sees this one as running.
            self._record = RestartRecord(
                RestartStatus.RESTARTING, "", self._now(), None
            )
            self._attempt = attempt

        success, message = self._robot_gw.restart_mode(
            on_done=lambda ok, text: self._finish(attempt, ok, text)
        )

        if success is False:
            with self._lock:
                if self._attempt is attempt:
                    self._record = previous
                    self._attempt = None
        elif success is True:
            self._finish(attempt, True, message)
        return success, message

    def snapshot(self) -> RestartRecord:
        with self._lock:
            self._record = self._expire(self._record)
            return self._record

    def _finish(self, attempt: object, success: bool, message: str) -> None:
        with self._lock:
            if self._attempt is not attempt:
                return
            self._record = replace(
                self._record,
                status=RestartStatus.SUCCEEDED if success else RestartStatus.FAILED,
                message=message,
                finished_at=self._now(),
            )
            self._attempt = None
        self._logger.info(
            "Mode restart finished", success=success, message=message
        )

    def _expire(self, record: RestartRecord) -> RestartRecord:
        # Callers hold _lock. Applied on read rather than by a timer.
        if (
            record.status is RestartStatus.RESTARTING
            and record.started_at is not None
            and self._now() - record.started_at > RESTART_DEADLINE
        ):
            self._attempt = None
            return replace(
                record,
                status=RestartStatus.FAILED,
                message=(
                    "The robot did not report back from the restart. Check "
                    "that it is running, then try again."
                ),
                finished_at=self._now(),
            )
        return record


def init_mode_restart_service(
    logger: structlog.stdlib.BoundLogger, robot_gw
) -> ModeRestartService:
    return ModeRestartService(logger=logger, robot_gw=robot_gw)
