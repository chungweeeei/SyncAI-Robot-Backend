"""The two subscription QoS shapes this process uses, named.

Each must still match its publisher (the README's ROS interfaces table gives
the pairing): a durability or reliability mismatch is a silent "no messages",
not an error. A fresh profile per call, since ``QoSProfile`` is mutable.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def latched() -> QoSProfile:
    """The last message, delivered to a late joiner: a state topic published
    rarely (pgo's run state and map-cloud notice, the costmap). RELIABLE +
    TRANSIENT_LOCAL, depth 1, as the publisher must be."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


def best_effort(depth: int) -> QoSProfile:
    """A stream where only the newest matters (state, odometry, clouds,
    paths): BEST_EFFORT + VOLATILE, keeping ``depth``."""
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
