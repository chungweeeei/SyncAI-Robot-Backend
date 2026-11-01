"""Tests for TaskRecordRepo.

Against in-memory SQLite for what both dialects share (the insert, its
idempotence, the export's filters, the retention's cut-off). SQLite has no
partitions, so the PostgreSQL DDL -- the partition's name and bounds, and
DETACH before DROP -- is pinned on a recording stand-in engine instead.
"""

from contextlib import contextmanager
from datetime import date, datetime, timezone
from unittest.mock import MagicMock

import pytest

from syncai_backend.repositories.task.task_record import (
    TaskRecordRepo,
    as_utc,
    partition_name,
)
from syncai_backend.tasks.schema import Step, StepStatus, StepType, TaskKind, TaskRecord


def _record(run_id="run-1", closed=None, status="COMPLETED", **extra) -> TaskRecord:
    closed = closed or datetime(2026, 11, 1, 8, 5, tzinfo=timezone.utc)
    return TaskRecord(
        task_id=f"robot01-task-{run_id}",
        run_id=run_id,
        status=status,
        started_at=closed.replace(minute=0),
        closed_at=closed,
        steps=[
            Step(
                id="s1",
                type=StepType.STANDUP,
                status=StepStatus.COMPLETED,
                started_at=closed.replace(minute=1),
                ended_at=closed.replace(minute=2),
                paused_seconds=1.5,
            )
        ],
        **extra,
    )


def test_insert_round_trips_the_record_and_its_step_times(task_record_repo):
    assert task_record_repo.insert(_record(kind=TaskKind.SCHEDULE, schedule_id="sched-1"))

    (row,) = task_record_repo.iter_records()
    assert row.task_id == "robot01-task-run-1"
    assert row.status == "COMPLETED"
    assert row.kind == "schedule"
    assert row.schedule_id == "sched-1"
    assert as_utc(row.closed_at) == datetime(2026, 11, 1, 8, 5, tzinfo=timezone.utc)
    # Snake-case keys (the export reads them), ISO times, the hold.
    step = row.steps[0]
    assert step["id"] == "s1"
    assert step["status"] == "COMPLETED"
    assert step["started_at"].startswith("2026-11-01T08:01:00")
    assert step["paused_seconds"] == 1.5


def test_a_retried_insert_is_a_no_op(task_record_repo):
    assert task_record_repo.insert(_record()) is True
    assert task_record_repo.insert(_record()) is False

    assert len(list(task_record_repo.iter_records())) == 1


def test_records_come_oldest_close_first_and_filtered(task_record_repo):
    for minute, run_id, status, name in [
        (30, "c", "COMPLETED", "patrol"),
        (10, "a", "FAILED", "patrol"),
        (20, "b", "COMPLETED", "dock"),
    ]:
        task_record_repo.insert(
            _record(
                run_id=run_id,
                status=status,
                name=name,
                closed=datetime(2026, 11, 1, 8, minute, tzinfo=timezone.utc),
            )
        )

    def runs(**filters):
        return [row.run_id for row in task_record_repo.iter_records(**filters)]

    assert runs() == ["a", "b", "c"]
    assert runs(status="COMPLETED") == ["b", "c"]
    assert runs(name="patrol") == ["a", "c"]
    assert runs(
        since=datetime(2026, 11, 1, 8, 15, tzinfo=timezone.utc),
        until=datetime(2026, 11, 1, 8, 25, tzinfo=timezone.utc),
    ) == ["b"]


def test_drop_before_removes_only_what_closed_before_the_cutoff_day(task_record_repo):
    task_record_repo.insert(
        _record(run_id="old", closed=datetime(2026, 10, 2, 23, 59, tzinfo=timezone.utc))
    )
    task_record_repo.insert(
        _record(run_id="kept", closed=datetime(2026, 10, 3, 0, 0, tzinfo=timezone.utc))
    )

    task_record_repo.drop_before(date(2026, 10, 3))

    assert [row.run_id for row in task_record_repo.iter_records()] == ["kept"]


# --- PostgreSQL DDL, on a stand-in engine ------------------------------------


class _RecordingEngine:
    """Enough of an Engine for the partition DDL: records each statement."""

    def __init__(self, partitions=(), fail_on=()):
        self.dialect = MagicMock()
        self.dialect.name = "postgresql"
        self.statements: list[str] = []
        self._partitions = list(partitions)
        self._fail_on = fail_on

    def _conn(self):
        conn = MagicMock()

        def execute(statement):
            sql = str(statement)
            self.statements.append(sql)
            if any(marker in sql for marker in self._fail_on):
                from sqlalchemy.exc import DBAPIError

                raise DBAPIError(sql, None, Exception("locked"))
            result = MagicMock()
            result.scalars.return_value = iter(self._partitions)
            return result

        conn.execute.side_effect = execute
        return conn

    @contextmanager
    def begin(self):
        yield self._conn()

    @contextmanager
    def connect(self):
        yield self._conn()


@pytest.fixture
def pg_repo(logger):
    def make(**engine_kwargs):
        engine = _RecordingEngine(**engine_kwargs)
        repo = TaskRecordRepo.__new__(TaskRecordRepo)
        repo.logger = logger
        repo.engine = engine
        repo.partitioned = True
        return repo, engine

    return make


def test_a_partition_covers_one_utc_day(pg_repo):
    repo, engine = pg_repo()

    repo.ensure_partition(date(2026, 12, 31))

    assert engine.statements == [
        "CREATE TABLE IF NOT EXISTS task_record_p20261231 PARTITION OF task_record "
        "FOR VALUES FROM ('2026-12-31 00:00:00+00') TO ('2027-01-01 00:00:00+00')"
    ]


def test_expired_partitions_are_detached_then_dropped(pg_repo):
    repo, engine = pg_repo(
        partitions=[
            "task_record_p20261003",
            "task_record_p20261001",
            "task_record_p20261002",
            # Not one of ours: left alone.
            "task_record_default",
        ]
    )

    dropped = repo.drop_before(date(2026, 10, 3))

    assert dropped == [date(2026, 10, 1), date(2026, 10, 2)]
    ddl = [sql for sql in engine.statements if not sql.startswith("SELECT")]
    assert ddl == [
        "ALTER TABLE task_record DETACH PARTITION task_record_p20261001",
        "DROP TABLE task_record_p20261001",
        "ALTER TABLE task_record DETACH PARTITION task_record_p20261002",
        "DROP TABLE task_record_p20261002",
    ]


def test_one_partition_failing_does_not_stop_the_others(pg_repo):
    repo, engine = pg_repo(
        partitions=["task_record_p20261001", "task_record_p20261002"],
        fail_on=("DETACH PARTITION task_record_p20261001",),
    )

    assert repo.drop_before(date(2026, 10, 3)) == [date(2026, 10, 2)]


def test_partition_name_is_the_day():
    assert partition_name(date(2026, 1, 9)) == "task_record_p20260109"
