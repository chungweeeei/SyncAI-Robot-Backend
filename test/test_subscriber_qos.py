"""subscribers/qos.py: the two shapes, pinned to what each subscriber had
inline before they were named -- a drift here is a silent "no messages"."""

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)

from syncai_backend.subscribers import qos  # noqa: E402


def test_latched_is_reliable_transient_local_depth_one():
    profile = qos.latched()

    assert profile.reliability == ReliabilityPolicy.RELIABLE
    assert profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert profile.history == HistoryPolicy.KEEP_LAST
    assert profile.depth == 1


@pytest.mark.parametrize("depth", [1, 3, 5])
def test_best_effort_is_volatile_and_keeps_the_depth_asked(depth):
    profile = qos.best_effort(depth=depth)

    assert profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert profile.durability == DurabilityPolicy.VOLATILE
    assert profile.history == HistoryPolicy.KEEP_LAST
    assert profile.depth == depth


def test_each_call_is_a_fresh_profile():
    """QoSProfile is mutable; one subscriber adjusting its copy must not
    change another's."""
    assert qos.latched() is not qos.latched()
    assert qos.best_effort(depth=1) is not qos.best_effort(depth=1)
