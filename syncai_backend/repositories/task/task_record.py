import re
import structlog

from collections.abc import Iterator
from datetime import date, datetime, time, timedelta, timezone
from typing import Optional

from sqlalchemy import Engine, delete, select, text
from sqlalchemy.dialects import postgresql, sqlite
from sqlalchemy.exc import DBAPIError

from syncai_backend.database.models import TaskRecordRow
from syncai_backend.database.session import logged_session, session_factory
from syncai_backend.tasks.schema import TaskKind, TaskRecord

TABLE = TaskRecordRow.__tablename__

# One partition per UTC day, named for it: task_record_p20261101.
_PARTITION_PREFIX = f"{TABLE}_p"
_PARTITION_NAME = re.compile(rf"^{_PARTITION_PREFIX}(\d{{8}})$")


def partition_name(day: date) -> str:
    return f"{_PARTITION_PREFIX}{day:%Y%m%d}"


def utc_day(moment: datetime) -> date:
    """The UTC day ``moment`` falls in, i.e. the partition that holds it."""
    return moment.astimezone(timezone.utc).date()


def _day_start(day: date) -> str:
    # An explicit offset: a bare date bound on a timestamptz column is read in the
    # session's TimeZone, which would shift every partition off the UTC day.
    return f"{day.isoformat()} 00:00:00+00"


def as_utc(moment: Optional[datetime]) -> Optional[datetime]:
    """A stored timestamp as aware UTC; SQLite hands them back naive."""
    if moment is not None and moment.tzinfo is None:
        return moment.replace(tzinfo=timezone.utc)
    return moment


class TaskRecordRepo:
    """The ``task_record`` table: one row per finished run (database/models.py).

    On PostgreSQL the table is partitioned by day: a row's partition is created
    on demand before its insert, and TaskRecordRetention creates the next few days'
    and drops the expired ones. On SQLite (tests) there are no partitions and the
    retention is a ``DELETE``, so the same calls stay testable.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        engine: Engine,
    ):
        self.logger = logger
        self.engine = engine
        self.session_maker = session_factory(engine)
        self.partitioned = engine.dialect.name == "postgresql"

    def _session(self, op: str):
        """A session for one repo operation, naming ``op`` if it fails."""
        return logged_session(self.session_maker, self.logger, f"[TaskRecordRepo][{op}]")

    def insert(self, record: TaskRecord) -> bool:
        """Write ``record``; False when its row was already there (a retry)."""
        if self.partitioned:
            self.ensure_partition(utc_day(record.closed_at))

        dialect = postgresql if self.partitioned else sqlite
        statement = (
            dialect.insert(TaskRecordRow)
            .values(
                run_id=record.run_id,
                closed_at=record.closed_at,
                task_id=record.task_id,
                status=record.status,
                started_at=record.started_at,
                kind=record.kind.value if record.kind is not None else None,
                name=record.name,
                map_name=record.map_name,
                schedule_id=record.schedule_id,
                # By field name, not the camelCase alias: the export reads these keys.
                steps=[step.model_dump(mode="json") for step in record.steps],
            )
            .on_conflict_do_nothing(index_elements=["run_id", "closed_at"])
        )
        with self._session("insert") as session:
            result = session.execute(statement)
            session.commit()
            return result.rowcount == 1

    def ensure_partition(self, day: date) -> None:
        """Create ``day``'s partition if it is missing; a no-op off PostgreSQL.

        Two callers racing (the retention tick and an insert) can both pass
        ``IF NOT EXISTS`` and the loser fails on the catalog; the retry then
        finds the table.
        """
        if not self.partitioned:
            return
        ddl = text(
            f"CREATE TABLE IF NOT EXISTS {partition_name(day)} PARTITION OF {TABLE} "
            f"FOR VALUES FROM ('{_day_start(day)}') TO ('{_day_start(day + timedelta(days=1))}')"
        )
        for attempt in range(2):
            try:
                with self.engine.begin() as conn:
                    conn.execute(ddl)
                return
            except DBAPIError:
                if attempt:
                    self.logger.error(
                        "[TaskRecordRepo][ensure_partition] database operation failed",
                        day=day.isoformat(),
                        exc_info=True,
                    )
                    raise

    def partition_days(self) -> list[date]:
        """The days that have a partition, oldest first; [] off PostgreSQL."""
        if not self.partitioned:
            return []
        with self.engine.connect() as conn:
            names = conn.execute(
                text(
                    "SELECT c.relname FROM pg_inherits i "
                    "JOIN pg_class c ON c.oid = i.inhrelid "
                    f"WHERE i.inhparent = '{TABLE}'::regclass"
                )
            ).scalars()
            days = []
            for name in names:
                match = _PARTITION_NAME.match(name)
                if match:
                    days.append(datetime.strptime(match.group(1), "%Y%m%d").date())
        return sorted(days)

    def drop_before(self, cutoff: date) -> list[date]:
        """Drop every record that closed before ``cutoff`` (a UTC day); the days dropped.

        PostgreSQL: each older partition is detached, then dropped, in its own
        transaction, so one that fails (a lock held by a long export) is retried at
        the next tick while the others still go. SQLite: a ``DELETE``, returning [].
        """
        if not self.partitioned:
            with self._session("drop_before") as session:
                session.execute(
                    delete(TaskRecordRow).where(
                        TaskRecordRow.closed_at
                        < datetime.combine(cutoff, time(), tzinfo=timezone.utc)
                    )
                )
                session.commit()
            return []

        dropped = []
        for day in self.partition_days():
            if day >= cutoff:
                continue
            name = partition_name(day)
            try:
                with self.engine.begin() as conn:
                    conn.execute(text(f"ALTER TABLE {TABLE} DETACH PARTITION {name}"))
                    conn.execute(text(f"DROP TABLE {name}"))
                dropped.append(day)
            except DBAPIError:
                self.logger.error(
                    "[TaskRecordRepo][drop_before] could not drop a partition",
                    partition=name,
                    exc_info=True,
                )
        return dropped

    def iter_records(
        self,
        since: Optional[datetime] = None,
        until: Optional[datetime] = None,
        status: Optional[str] = None,
        kind: Optional[TaskKind] = None,
        name: Optional[str] = None,
        batch: int = 500,
    ) -> Iterator[TaskRecordRow]:
        """Records matching the history's filters, oldest close first, fetched in
        batches so an export never holds the whole window in memory.

        A generator: the session stays open until it is exhausted or closed.
        """
        statement = select(TaskRecordRow)
        if since is not None:
            statement = statement.where(TaskRecordRow.closed_at >= since)
        if until is not None:
            statement = statement.where(TaskRecordRow.closed_at <= until)
        if status is not None:
            statement = statement.where(TaskRecordRow.status == status)
        if kind is not None:
            statement = statement.where(TaskRecordRow.kind == kind.value)
        if name is not None:
            statement = statement.where(TaskRecordRow.name == name)
        statement = statement.order_by(TaskRecordRow.closed_at, TaskRecordRow.run_id)

        with self._session("iter_records") as session:
            yield from session.execute(
                statement.execution_options(yield_per=batch)
            ).scalars()


def init_task_record_repo(
    logger: structlog.stdlib.BoundLogger,
    engine: Engine,
) -> TaskRecordRepo:
    # The partitioned parent on PostgreSQL (see TaskRecordRow); create_all leaves an
    # existing table alone.
    TaskRecordRow.metadata.create_all(engine)
    return TaskRecordRepo(logger=logger, engine=engine)
