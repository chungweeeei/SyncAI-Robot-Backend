"""The live mode's restart, as something that can be asked about afterwards.

POST /api/v1/robot/restart used to answer "dispatched" and forget: the gateway
waits two seconds for sys_manager, and sys_manager only answers once the
rebuild is over, tens of seconds later. While the backend was a byobu pane that
was all it could do -- the process died with the session it had asked to
rebuild. Run as its own container it survives, so the answer does arrive, and
it is the only statement anywhere that the rebuild worked. GET /robot/state
cannot stand in for it: the backend keeps serving the last frame robot_state
published, so the poll stays green through the whole outage.

So this object holds one record -- the latest restart and how it ended -- and
the router reads it. In memory, and that is not a caveat to work around: the
outcome belongs to the rebuild this process watched. A backend that was itself
restarted meanwhile reports `idle`, which is the honest answer; it did not see
how the last one went.
"""

import threading
from dataclasses import dataclass, replace
from datetime import datetime, timedelta, timezone
from enum import Enum
from typing import Callable, Optional, Tuple

import structlog

from syncai_backend.exceptions import ConflictError


# How long a dispatched restart may go unanswered before it is reported as
# failed. sys_manager's rebuild is ~40 byobu commands with sleep offsets, well
# under a minute; three is room for a slow Jetson, and past it the likelier
# story is a sys_manager that died mid-rebuild and will never answer. Without
# a ceiling that record would say `restarting` forever and block the next one.
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
        # Identifies the attempt a late answer belongs to. A restart that hit
        # the deadline and was followed by a new one must not have its own
        # answer, arriving at last, overwrite the new one's record.
        self._attempt: Optional[object] = None

    def start(self) -> Tuple[Optional[bool], str]:
        """Dispatch a restart. The gateway's three-valued answer, passed on.

        ``(False, msg)`` is a refusal or an unreachable sys_manager: nothing
        was touched, so the previous record stands. ``(True, msg)`` finished
        inside the ack window; ``(None, msg)`` is under way and GET reports
        the rest. Raises ConflictError(code="restart_running") while one is.
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
            # Recorded before the call, which blocks for up to the ack window:
            # a second press in that window must see this one as running.
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
        # Callers hold _lock. Applied on read rather than by a timer: the
        # record only matters when someone asks for it.
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
