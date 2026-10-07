"""Tests for SafetyLockService: the lock's rising edge cancels every task.

The edge is the whole contract. robot_state repeats the lock's value at 1 Hz,
so a cancel per sample would cancel every task dispatched while the robot is
locked, and a cancel on release would stop a run the operator started after
clearing it. Only false -> true (and a first sample that is already true) acts,
and the estop request and the samples share that edge.

The service is driven the way production drives it: ``observe`` from a thread
that is not the loop's, the cancel landing on a loop running in another thread.
No ROS, no Temporal -- both collaborators are stubs.
"""

import asyncio
import threading

import pytest

from syncai_backend.services.safety_lock import init_safety_lock_service


class _StubRobotGateway:
    def __init__(self):
        self.calls = 0
        self.raises = None

    def cancel_active_moves(self):
        self.calls += 1
        if self.raises is not None:
            raise self.raises
        return True, "no goal was executing"


class _StubWorkflowGateway:
    def __init__(self):
        self.calls = 0
        self.raises = None
        self.done = threading.Event()

    async def cancel_active_tasks(self):
        self.calls += 1
        self.done.set()
        if self.raises is not None:
            raise self.raises
        return ["robot01-task-001"]


@pytest.fixture
def loop():
    """An event loop running in its own thread, as uvicorn's does."""
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=5)
    loop.close()


@pytest.fixture
def robot_gw():
    return _StubRobotGateway()


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


@pytest.fixture
def svc(logger, robot_gw, workflow_gw):
    return init_safety_lock_service(
        logger=logger, robot_gw=robot_gw, workflow_gw=workflow_gw
    )


def _drain(loop):
    """Wait until everything already scheduled on ``loop`` has run."""
    async def _noop():
        # Two turns: the cancel's own executor hop resolves on the second.
        for _ in range(10):
            await asyncio.sleep(0.01)

    asyncio.run_coroutine_threadsafe(_noop(), loop).result(timeout=5)


def test_a_rising_edge_cancels_once(svc, loop, robot_gw, workflow_gw):
    svc.bind_loop(loop)

    svc.observe(engaged=False)
    svc.observe(engaged=True)
    assert workflow_gw.done.wait(timeout=5)
    svc.observe(engaged=True)
    _drain(loop)

    # Nav goals first and directly, then the runs; and only once for a lock
    # that robot_state keeps repeating.
    assert (robot_gw.calls, workflow_gw.calls) == (1, 1)


@pytest.mark.parametrize(
    "samples",
    [
        [False, False],
        [True, False],  # a first sample counts; its release does not
        [False],
    ],
)
def test_a_release_or_steady_state_cancels_nothing_more(
    svc, loop, workflow_gw, samples
):
    svc.bind_loop(loop)
    expected = 1 if samples[0] else 0

    for engaged in samples:
        svc.observe(engaged=engaged)
    _drain(loop)

    assert workflow_gw.calls == expected


def test_each_engagement_cancels_again(svc, loop, workflow_gw):
    svc.bind_loop(loop)

    for engaged in (False, True, False, True):
        svc.observe(engaged=engaged)
    _drain(loop)

    assert workflow_gw.calls == 2


def test_a_lock_already_engaged_at_startup_cancels(svc, loop, workflow_gw):
    # Runs outlive a backend restart; a lock that engaged while this process
    # was down must still stop them.
    svc.bind_loop(loop)

    svc.observe(engaged=True)
    _drain(loop)

    assert workflow_gw.calls == 1


def test_an_edge_before_the_loop_is_bound_runs_on_bind(svc, loop, workflow_gw):
    svc.observe(engaged=True)
    assert workflow_gw.calls == 0

    svc.bind_loop(loop)
    _drain(loop)

    assert workflow_gw.calls == 1


def test_a_failed_nav_cancel_still_cancels_the_tasks(svc, loop, robot_gw, workflow_gw):
    robot_gw.raises = RuntimeError("nav2 gone")
    svc.bind_loop(loop)

    svc.observe(engaged=True)
    _drain(loop)

    assert workflow_gw.calls == 1


def test_a_failed_task_cancel_does_not_escape(svc, loop, workflow_gw):
    workflow_gw.raises = RuntimeError("Temporal down")
    svc.bind_loop(loop)

    svc.observe(engaged=True)
    _drain(loop)

    # Nothing awaits the cancel's future; the loop must still be serving.
    assert workflow_gw.calls == 1
    _drain(loop)


def test_an_estop_request_cancels_and_its_echo_does_not(svc, loop, workflow_gw):
    svc.bind_loop(loop)
    svc.observe(engaged=False)

    assert svc.engage_requested() is True
    assert workflow_gw.done.wait(timeout=5)
    # robot_state then reports the lock the request engaged: same edge.
    svc.observe(engaged=True)
    _drain(loop)

    assert workflow_gw.calls == 1


def test_an_estop_request_on_an_engaged_lock_cancels_nothing(svc, loop, workflow_gw):
    svc.bind_loop(loop)
    svc.observe(engaged=True)
    _drain(loop)

    assert svc.engage_requested() is False
    _drain(loop)

    assert workflow_gw.calls == 1
