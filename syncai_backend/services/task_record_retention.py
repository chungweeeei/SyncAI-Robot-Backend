"""Keep ``task_record`` to the last TASK_RECORD_RETENTION_DAYS days.

Every TASK_RECORD_MAINTENANCE_INTERVAL_S (and once at start-up) a daemon thread
creates the partitions for today and the next TASK_RECORD_PARTITIONS_AHEAD days, then
detaches and drops every partition older than the retention, so the table cannot
grow until the disk is full.

A thread in this process rather than a Temporal schedule: PostgreSQL is a hard
dependency of the backend and Temporal a soft one, and the clean-up must not stop
because Temporal is down. A daemon, so it dies with the process; ``stop`` is for
tests.
"""

import threading
from datetime import date, datetime, timedelta, timezone
from typing import Callable, Optional

import structlog

from syncai_backend.repositories.task.task_record import TaskRecordRepo
from syncai_backend.tasks.config import (
    TASK_RECORD_MAINTENANCE_INTERVAL_S,
    TASK_RECORD_PARTITIONS_AHEAD,
    TASK_RECORD_RETENTION_DAYS,
)


def _utc_today() -> date:
    return datetime.now(timezone.utc).date()


class TaskRecordRetention:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        repo: TaskRecordRepo,
        interval_s: float = TASK_RECORD_MAINTENANCE_INTERVAL_S,
        retention_days: int = TASK_RECORD_RETENTION_DAYS,
        today: Callable[[], date] = _utc_today,
    ):
        self._logger = logger
        self._repo = repo
        self._interval_s = interval_s
        self._retention_days = retention_days
        self._today = today
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def tick(self) -> None:
        """One maintenance pass. Never raises: a failure is retried next tick."""
        today = self._today()
        try:
            for ahead in range(TASK_RECORD_PARTITIONS_AHEAD + 1):
                self._repo.ensure_partition(today + timedelta(days=ahead))
        except Exception:
            self._logger.error(
                "[TaskRecordRetention] could not create partitions", exc_info=True
            )

        # Day ``today - retention_days`` is the 31st day back: it goes, today and
        # the 29 before it stay.
        cutoff = today - timedelta(days=self._retention_days - 1)
        try:
            dropped = self._repo.drop_before(cutoff)
            if dropped:
                self._logger.info(
                    "[TaskRecordRetention] dropped expired task records",
                    days=[day.isoformat() for day in dropped],
                )
        except Exception:
            self._logger.error(
                "[TaskRecordRetention] could not drop expired records", exc_info=True
            )

    def _run(self) -> None:
        while True:
            self.tick()
            if self._stop.wait(self._interval_s):
                return

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="task-record-retention", daemon=True
        )
        self._thread.start()

    def stop(self, timeout_s: float = 5.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)


def init_task_record_retention(
    logger: structlog.stdlib.BoundLogger, repo: TaskRecordRepo
) -> TaskRecordRetention:
    retention = TaskRecordRetention(logger=logger, repo=repo)
    retention.start()
    return retention
