"""Tests for TaskRecordRetention: what one tick asks the repo for, and that a
failing repo never ends the thread."""

import threading
from datetime import date
from unittest.mock import MagicMock

from syncai_backend.services.task_record_retention import TaskRecordRetention
from syncai_backend.tasks.config import (
    TASK_RECORD_PARTITIONS_AHEAD,
    TASK_RECORD_RETENTION_DAYS,
)


def _retention(logger, repo, today=date(2026, 11, 30), **kwargs):
    return TaskRecordRetention(logger=logger, repo=repo, today=lambda: today, **kwargs)


def test_a_tick_readies_the_coming_days_and_keeps_thirty(logger):
    repo = MagicMock()
    repo.drop_before.return_value = []

    _retention(logger, repo).tick()

    # Today and the days ahead, across the month's end.
    assert TASK_RECORD_PARTITIONS_AHEAD == 3
    assert [call.args[0] for call in repo.ensure_partition.call_args_list] == [
        date(2026, 11, 30),
        date(2026, 12, 1),
        date(2026, 12, 2),
        date(2026, 12, 3),
    ]
    # Today and the 29 days before it stay: day 31 back (Oct 31) is the first to go.
    assert TASK_RECORD_RETENTION_DAYS == 30
    repo.drop_before.assert_called_once_with(date(2026, 11, 1))


def test_a_failing_repo_is_logged_not_raised(logger):
    repo = MagicMock()
    repo.ensure_partition.side_effect = RuntimeError("db down")
    repo.drop_before.side_effect = RuntimeError("db down")

    _retention(logger, repo).tick()

    # The drop is still tried when the creation fails.
    repo.drop_before.assert_called_once()


def test_the_thread_ticks_at_start_and_on_its_interval_until_stopped(logger):
    ticked = threading.Event()
    repo = MagicMock()
    repo.drop_before.return_value = []
    calls = []

    def drop_before(cutoff):
        calls.append(cutoff)
        if len(calls) >= 3:
            ticked.set()
        return []

    repo.drop_before.side_effect = drop_before
    retention = _retention(logger, repo, interval_s=0.01)

    retention.start()
    assert ticked.wait(timeout=5)
    retention.stop()

    assert not retention._thread.is_alive()
