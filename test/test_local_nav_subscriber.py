"""Tests for LocalNavSubscriber: the controller's local costmap and local plan
-> TelemetryRepo, in the map frame.

What is pinned:

* The topics and their QoS. The costmap is read off ``costmap/raw`` because
  the OccupancyGrid topic carries only incremental updates while the robot
  stands; and its RELIABLE + TRANSIENT_LOCAL is the publisher's own, so the
  latest map arrives on subscribe. Both names are relative (the robot_id
  namespace).

* The planar composition into the map frame, against a map->odom that both
  moves and turns: a sign slip in the rotation would draw the costmap
  mirrored round the robot while every number looked plausible. The cells
  themselves are passed through byte for byte.

* No TF, no sample -- dropped and not raised, since an exception here escapes
  into the executor at the topic's rate.

* The local plan's 5 Hz thinning, and an empty plan clearing at once.
"""

import math
from types import SimpleNamespace
from unittest.mock import patch

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs")
pytest.importorskip("nav2_msgs")
pytest.importorskip("tf2_ros")

import rclpy.qos  # noqa: E402

from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from nav2_msgs.msg import Costmap  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from tf2_ros import TransformException  # noqa: E402

from syncai_backend.repositories.telemetry.telemetry import (  # noqa: E402
    init_telemetry_repo,
)
from syncai_backend.subscribers.local_nav_subscriber import (  # noqa: E402
    _LOCAL_PLAN_MIN_INTERVAL_S,
    init_local_nav_subscriber,
)

MONOTONIC = "syncai_backend.subscribers.local_nav_subscriber.time.monotonic"


class _FakeNode:
    """Records create_subscription calls instead of touching the DDS graph."""

    def __init__(self):
        self.subscriptions = {}

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        sub = SimpleNamespace(
            msg_type=msg_type, topic=topic, callback=callback, qos_profile=qos_profile
        )
        self.subscriptions[topic] = sub
        return sub


class _FakeTfBuffer:
    """lookup_transform with one answer, or TransformException when None."""

    def __init__(self, transform=None):
        self.transform = transform
        self.lookups = []

    def lookup_transform(self, target_frame, source_frame, time):
        self.lookups.append((target_frame, source_frame))
        if self.transform is None:
            raise TransformException("map->odom not in the buffer")
        return self.transform


def _map_odom(x, y, yaw):
    tf = TransformStamped()
    tf.transform.translation.x = float(x)
    tf.transform.translation.y = float(y)
    tf.transform.rotation.z = math.sin(yaw / 2.0)
    tf.transform.rotation.w = math.cos(yaw / 2.0)
    return tf


def _costmap(width=2, height=2, data=(0, 100, 254, 255), ox=1.0, oy=0.0):
    msg = Costmap()
    msg.header.frame_id = "robot01/odom"
    msg.header.stamp.sec = 50
    msg.header.stamp.nanosec = 500_000_000
    msg.metadata.resolution = 0.05
    msg.metadata.size_x = width
    msg.metadata.size_y = height
    msg.metadata.origin.position.x = ox
    msg.metadata.origin.position.y = oy
    msg.metadata.origin.orientation.w = 1.0
    msg.data = list(data)
    return msg


def _plan(points, sec=60):
    msg = Path()
    msg.header.frame_id = "robot01/odom"
    msg.header.stamp.sec = sec
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        msg.poses.append(pose)
    return msg


@pytest.fixture
def repo(logger):
    return init_telemetry_repo(logger=logger)


@pytest.fixture
def tf_buffer():
    # map->odom: 10 m east, 5 m north, a quarter turn left.
    return _FakeTfBuffer(_map_odom(10.0, 5.0, math.pi / 2))


@pytest.fixture
def subs(logger, repo, tf_buffer):
    node = _FakeNode()
    init_local_nav_subscriber(
        logger=logger, node=node, telemetry_repo=repo, tf_buffer=tf_buffer
    )
    return node.subscriptions


def test_subscribes_to_the_raw_costmap_and_the_local_plan(subs):
    assert set(subs) == {"local_costmap/costmap/raw", "local_plan"}
    assert subs["local_costmap/costmap/raw"].msg_type is Costmap
    assert subs["local_plan"].msg_type is Path


def test_the_costmap_qos_is_the_publishers_own(subs):
    qos = subs["local_costmap/costmap/raw"].qos_profile
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.RELIABLE
    assert qos.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.depth == 1


def test_the_costmap_lands_in_the_map_frame_cells_untouched(subs, repo):
    # The odom-frame corner (1, 0) is, after a quarter turn left and the
    # (10, 5) shift, (10, 6); and the grid itself turns with map->odom.
    subs["local_costmap/costmap/raw"].callback(_costmap())

    sample = repo.get_local_costmap()
    x, y, yaw = sample.origin
    assert (x, y) == pytest.approx((10.0, 6.0))
    assert yaw == pytest.approx(math.pi / 2)
    assert (sample.width, sample.height, sample.resolution) == (2, 2, 0.05)
    assert sample.data == bytes([0, 100, 254, 255])
    assert sample.stamp == pytest.approx(50.5)


def test_a_costmap_whose_size_and_data_disagree_is_dropped(subs, repo):
    subs["local_costmap/costmap/raw"].callback(_costmap(width=3, height=3))
    assert repo.get_local_costmap() is None


def test_no_tf_drops_the_costmap_without_raising(subs, repo, tf_buffer):
    tf_buffer.transform = None
    subs["local_costmap/costmap/raw"].callback(_costmap())
    assert repo.get_local_costmap() is None

    # Not latched: the next one with TF goes through.
    tf_buffer.transform = _map_odom(0.0, 0.0, 0.0)
    subs["local_costmap/costmap/raw"].callback(_costmap())
    assert repo.get_local_costmap().origin[:2] == pytest.approx((1.0, 0.0))


def test_the_local_plan_is_map_frame_pairs(subs, repo):
    # Reads inside the patch too: it patches the one shared `time` module, so
    # the repo's TTL runs on the same fake clock.
    with patch(MONOTONIC, return_value=100.0):
        subs["local_plan"].callback(_plan([(0.0, 0.0), (1.0, 0.0)]))

        # (1, 0) in odom is (10, 6) in the map; pairs, no height.
        assert repo.get_local_plan().points == ((10.0, 5.0), (10.0, 6.0))


def test_the_local_plan_is_thinned_to_five_hertz(subs, repo):
    with patch(MONOTONIC, return_value=100.0):
        subs["local_plan"].callback(_plan([(0.0, 0.0)]))
    first = repo.get_local_plan().seq

    with patch(MONOTONIC, return_value=100.0 + _LOCAL_PLAN_MIN_INTERVAL_S / 2):
        subs["local_plan"].callback(_plan([(1.0, 0.0)]))
        assert repo.get_local_plan(after_seq=first) is None

    with patch(MONOTONIC, return_value=100.0 + _LOCAL_PLAN_MIN_INTERVAL_S + 0.01):
        subs["local_plan"].callback(_plan([(1.0, 0.0)]))
        assert repo.get_local_plan(after_seq=first).points == ((10.0, 6.0),)


def test_an_empty_local_plan_clears_at_once(subs, repo):
    with patch(MONOTONIC, return_value=100.0):
        subs["local_plan"].callback(_plan([(0.0, 0.0)]))
        seq = repo.get_local_plan().seq
        # Inside the thinning interval, and still not held back.
        subs["local_plan"].callback(_plan([]))

        assert repo.get_local_plan(after_seq=seq).points == ()
