"""Unit tests for ModeRestartService: the record GET /api/v1/robot/restart reads.

No ROS: the gateway is a stub that hands back its late-answer callback, and the
clock is injected, so the deadline is tested by moving time rather than waiting.
"""

from datetime import datetime, timedelta, timezone

import pytest

from syncai_backend.exceptions import ConflictError
from syncai_backend.services.mode_restart import (
    RESTART_DEADLINE,
    ModeRestartService,
    RestartStatus,
)


class _Gateway:
    def __init__(self):
        self.result = (None, "dispatched")
        self.callbacks = []

    def restart_mode(self, on_done=None):
        self.callbacks.append(on_done)
        return self.result


class _Clock:
    def __init__(self):
        self.now = datetime(2026, 9, 29, 8, 0, tzinfo=timezone.utc)

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return _Clock()


@pytest.fixture
def gateway():
    return _Gateway()


@pytest.fixture
def service(logger, gateway, clock):
    return ModeRestartService(logger=logger, robot_gw=gateway, now=clock)


def test_an_unanswered_restart_fails_at_the_deadline(service, clock):
    """Otherwise a sys_manager that died mid-rebuild says `restarting` forever."""
    service.start()

    clock.now += RESTART_DEADLINE
    assert service.snapshot().status is RestartStatus.RESTARTING

    clock.now += timedelta(seconds=1)
    record = service.snapshot()
    assert record.status is RestartStatus.FAILED
    assert record.finished_at == clock.now


def test_a_restart_past_its_deadline_does_not_block_the_next(service, clock):
    service.start()
    clock.now += RESTART_DEADLINE + timedelta(seconds=1)

    service.start()

    assert service.snapshot().status is RestartStatus.RESTARTING


def test_a_late_answer_does_not_overwrite_the_restart_after_it(
    service, gateway, clock
):
    """The first attempt's answer, arriving at last, belongs to that attempt."""
    service.start()
    clock.now += RESTART_DEADLINE + timedelta(seconds=1)
    service.start()

    first, second = gateway.callbacks
    first(True, "Restarted AUTO")
    assert service.snapshot().status is RestartStatus.RESTARTING

    second(False, "ended up in MAINTENANCE")
    assert service.snapshot().status is RestartStatus.FAILED


def test_a_refusal_keeps_the_previous_outcome(service, gateway):
    service.start()
    gateway.callbacks[0](True, "Restarted AUTO")

    gateway.result = (False, "MANUAL")
    service.start()

    record = service.snapshot()
    assert record.status is RestartStatus.SUCCEEDED
    assert record.message == "Restarted AUTO"


def test_a_second_start_while_restarting_is_refused(service, gateway):
    service.start()

    with pytest.raises(ConflictError) as refused:
        service.start()

    assert refused.value.code == "restart_running"
    assert len(gateway.callbacks) == 1
