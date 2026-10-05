"""Tests for the pgo mapping-status subscriber (run state -> repo slot).

Same harness as test_map_cloud_subscriber.py: a fake node records the
subscription and the recorded callback is fed real syncai_common/MappingStatus
messages. Pinned here is the contract shared with pgo_node.cpp:

* the RELATIVE ``pgo/mapping_status`` topic, RELIABLE + TRANSIENT_LOCAL depth 1
  (a late joiner -- a backend restarted mid-session -- gets the latched state);
* the message's uint8 constants map onto the repo's enum, and a code this
  backend does not know is ignored rather than raised (an exception out of a
  callback would end the executor and the process);
* each sample REPLACES the slot, which is how a save's IDLE supersedes the
  MAPPING before it.
"""

import pytest

pytest.importorskip("rclpy")
# syncai_common -- only present once the workspace is built and sourced.
pytest.importorskip("syncai_common")

import rclpy.qos  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from syncai_common.msg import MappingStatus as MappingStatusMsg  # noqa: E402

from syncai_backend.repositories.mapping.mapping import (  # noqa: E402
    MappingState,
    init_mapping_status_repo,
)
from syncai_backend.subscribers.mapping_status_subscriber import (  # noqa: E402
    init_mapping_status_subscriber,
)


class FakeNode:
    """Records create_subscription calls; the source only ever calls that."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        self.subscriptions.append(
            {
                "msg_type": msg_type,
                "topic": topic,
                "callback": callback,
                "qos": qos_profile,
            }
        )
        return object()


def make_status(state, key_poses=0, loop_closures=0, sec=1758600000, nanosec=500000000):
    return MappingStatusMsg(
        state=state,
        key_poses=key_poses,
        loop_closures=loop_closures,
        stamp=Time(sec=sec, nanosec=nanosec),
    )


@pytest.fixture
def repo(logger):
    return init_mapping_status_repo(logger=logger)


@pytest.fixture
def wire(logger, repo):
    """(fake node, recorded callback) with the subscriber wired up."""
    node = FakeNode()
    init_mapping_status_subscriber(logger=logger, node=node, mapping_status_repo=repo)
    assert len(node.subscriptions) == 1
    return node, node.subscriptions[0]["callback"]


def test_subscribes_to_the_relative_latched_status_topic(wire):
    node, _ = wire
    sub = node.subscriptions[0]

    # Relative on purpose: the node's namespace (robot_id) scopes it.
    assert sub["topic"] == "pgo/mapping_status"
    assert sub["msg_type"] is MappingStatusMsg

    qos = sub["qos"]
    # Matching pgo's publisher exactly: RELIABLE + TRANSIENT_LOCAL depth 1. A
    # VOLATILE reader would connect and never receive the replay, and the
    # replay is the whole point -- a backend (re)started mid-session must know
    # whether a Start is needed without waiting for the next transition.
    assert qos.depth == 1
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.RELIABLE
    assert qos.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.history == rclpy.qos.HistoryPolicy.KEEP_LAST


def test_the_slot_is_empty_until_pgo_has_published(wire, repo):
    assert repo.get() is None


def test_a_mapping_sample_lands_with_its_counts_and_stamp(wire, repo):
    _, callback = wire

    callback(make_status(MappingStatusMsg.MAPPING, key_poses=42, loop_closures=3))

    status = repo.get()
    assert status.state is MappingState.MAPPING
    assert (status.key_poses, status.loop_closures) == (42, 3)
    assert status.stamp == pytest.approx(1758600000.5)


@pytest.mark.parametrize(
    "code, state",
    [
        (MappingStatusMsg.IDLE, MappingState.IDLE),
        (MappingStatusMsg.MAPPING, MappingState.MAPPING),
        (MappingStatusMsg.RESETTING, MappingState.RESETTING),
    ],
)
def test_every_state_code_maps_onto_the_repo_enum(wire, repo, code, state):
    _, callback = wire

    callback(make_status(code))

    assert repo.get().state is state


def test_an_idle_sample_replaces_a_mapping_one(wire, repo):
    # The save path: pgo goes idle and its next sample must win.
    _, callback = wire
    callback(make_status(MappingStatusMsg.MAPPING, key_poses=40))

    callback(make_status(MappingStatusMsg.IDLE))

    status = repo.get()
    assert status.state is MappingState.IDLE
    assert status.key_poses == 0


def test_an_unknown_state_code_is_ignored_and_raises_nothing(wire, repo):
    _, callback = wire
    callback(make_status(MappingStatusMsg.MAPPING, key_poses=5))

    callback(make_status(250))

    # Slot untouched: a pgo newer than this backend must not blank the console.
    status = repo.get()
    assert status.state is MappingState.MAPPING
    assert status.key_poses == 5
