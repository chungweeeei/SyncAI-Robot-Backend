"""Tests for the pgo map-cloud subscriber (merged keyframe cloud -> repo slot).

Same harness as test_pointcloud_subscriber.py: a fake node records the
subscription, and the recorded callback is fed real std_msgs/String notices
that name real binary PCD files written into tmp_path. Pinned here is what
makes this subscriber deliberately NOT a copy of the live-cloud one, and the
file hand-off contract it shares with pgo_node.cpp:

* the merge is a FILE named by a ~200 B JSON notice on the RELATIVE
  ``pgo/map_cloud_file`` topic, RELIABLE + TRANSIENT_LOCAL depth 1 (a queued
  older merge is never worth delivering, a late joiner gets the latched one);
* the file must sit under the shared tmpfs root (injected here as tmp_path);
* no TF anywhere -- the constructor does not even take a buffer, because pgo
  writes the merge already in the map frame;
* each merge REPLACES the single slot (a loop closure moves the whole map, so
  deltas are impossible) with num_points/bytes agreeing;
* an EMPTY notice is a message, not a non-event: pgo publishes one from
  reset_mapping to say the map has been discarded, so it clears the slot
  rather than being skipped. A NaN-only file lands on that same clearing path;
* every failure -- missing file, bad JSON, bad PCD, foreign path -- leaves the
  slot exactly as it was and raises nothing (an exception out of a callback
  would end the executor and the process).

pack_xyz_f32 / cap_points / read_pcd_xyz have their own tests in
test_pointcloud.py; this file only asserts the slot's resulting floats.
"""

import json
import struct

import pytest

pytest.importorskip("numpy")
pytest.importorskip("rclpy")
pytest.importorskip("std_msgs")

import numpy as np  # noqa: E402
import rclpy.qos  # noqa: E402

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from syncai_backend.repositories.pointcloud.pointcloud import (  # noqa: E402
    init_pointcloud_repo,
)
from syncai_backend.subscribers.map_cloud_subscriber import (  # noqa: E402
    MapCloudNotice,
    init_map_cloud_subscriber,
    parse_map_cloud_notice,
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
                "callback_group": kwargs.get("callback_group"),
            }
        )
        return object()


def write_pcd(path, points, intensity=None):
    """A binary PCD in the layout pgo writes: FIELDS x y z intensity, 16 B/pt.

    pcl::io::savePCDFileBinary of a PointXYZI cloud strips PCL's padding, so
    the four float32 columns are packed -- which is what the saved map.pcd on
    every robot looks like too.
    """
    arr = np.asarray(points, dtype="<f4").reshape(-1, 3)
    n = arr.shape[0]
    if intensity is None:
        intensity = np.zeros(n, dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    body = b"".join(struct.pack("<ffff", *arr[i], intensity[i]) for i in range(n))
    path.write_bytes(header.encode("ascii") + body)
    return str(path)


def notice(path="", points=0, seq=1, frame_id="map", **extra):
    """A String carrying the JSON pgo_node.cpp's mapCloudNoticeJson emits."""
    payload = {
        "seq": seq,
        "path": path,
        "points": points,
        "frame_id": frame_id,
        "stamp": {"sec": 1758600000, "nanosec": 0},
    }
    payload.update(extra)
    return String(data=json.dumps(payload))


def slot_points(repo):
    frame = repo.get_latest()
    assert frame is not None
    pts = np.frombuffer(frame.data, dtype="<f4").reshape(-1, 3)
    # num_points and the payload must agree -- the WS pump prefixes the count
    # the frontend uses to size its buffers.
    assert frame.num_points == pts.shape[0]
    return frame, pts


@pytest.fixture
def repo(logger):
    return init_pointcloud_repo(logger=logger)


@pytest.fixture
def shm(tmp_path):
    """Stands in for /dev/shm/syncai_pgo/<robot_id>: the only place a notice
    may point at."""
    d = tmp_path / "shm" / "syncai_pgo" / "robot01"
    d.mkdir(parents=True)
    return d


@pytest.fixture
def wire(logger, repo, tmp_path):
    """(fake node, recorded callback) with the subscriber wired up, its
    allowed root being tmp_path/shm."""
    node = FakeNode()
    init_map_cloud_subscriber(
        logger=logger,
        node=node,
        map_cloud_repo=repo,
        allowed_root=str(tmp_path / "shm"),
    )
    assert len(node.subscriptions) == 1
    return node, node.subscriptions[0]["callback"]


# ---- the subscription -----------------------------------------------------


def test_subscribes_to_the_relative_notice_topic_latched_with_a_depth_one_queue(wire):
    node, _ = wire
    sub = node.subscriptions[0]

    # Relative on purpose: the node's namespace (robot_id) scopes it, and this
    # subscription existing is what un-gates pgo's subscriber-gated merge.
    assert sub["topic"] == "pgo/map_cloud_file"
    assert sub["msg_type"] is String

    qos = sub["qos"]
    # Depth 1 where the live cloud uses 5: each notice replaces the map
    # wholesale, so delivering a queued older merge is pure waste.
    assert qos.depth == 1
    # RELIABLE + TRANSIENT_LOCAL, matching pgo's publisher exactly: a 200 B
    # notice is free to latch, and it is what lets a backend (re)started
    # mid-mapping draw the current map at once. A VOLATILE reader would match
    # but never receive the replay.
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.RELIABLE
    assert qos.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL

    # Own group: reading and parsing a multi-MB PCD must not starve the 10 Hz
    # body_cloud / state / TF callbacks on the shared executor.
    assert isinstance(sub["callback_group"], MutuallyExclusiveCallbackGroup)


# ---- the happy path -------------------------------------------------------


def test_a_notice_loads_the_file_into_the_slot_as_packed_f32_xyz(wire, repo, shm):
    _, callback = wire
    path = write_pcd(
        shm / "map_cloud_1.pcd",
        [(1.0, 2.0, 3.0), (-4.5, 0.0, 9.25)],
        intensity=np.array([7.0, 8.0], dtype="<f4"),
    )

    callback(notice(path=path, points=2, seq=1))

    frame, pts = slot_points(repo)
    assert frame.seq == 1
    # intensity is read past, not packed: the viewer colours by height.
    assert pts.tolist() == [[1.0, 2.0, 3.0], [-4.5, 0.0, 9.25]]
    assert frame.data == np.asarray(
        [(1.0, 2.0, 3.0), (-4.5, 0.0, 9.25)], dtype="<f4"
    ).tobytes()


def test_the_frame_id_is_taken_as_is_without_a_map_frame_check(wire, repo, shm):
    # No TF path exists in this subscriber (the constructor takes no buffer),
    # so the frame is trusted, not verified: pgo places every point with the
    # keyframes' corrected global poses at merge time.
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 0.0, 0.0)])

    callback(notice(path=path, points=1, frame_id="robot01/anything"))

    _, pts = slot_points(repo)
    assert pts.tolist() == [[1.0, 0.0, 0.0]]


def test_each_notice_replaces_the_slot_wholesale(wire, repo, shm):
    _, callback = wire
    p1 = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])
    p2 = write_pcd(shm / "map_cloud_2.pcd", [(2.0, 2.0, 2.0), (3.0, 3.0, 3.0)])

    callback(notice(path=p1, points=1, seq=1))
    callback(notice(path=p2, points=2, seq=2))

    frame, pts = slot_points(repo)
    assert frame.seq == 2  # seq advanced: the WS pump's dedup cursor moves on
    assert pts.tolist() == [[2.0, 2.0, 2.0], [3.0, 3.0, 3.0]]
    # Single-slot: after a loop closure the OLD map must be unreachable, not
    # queued behind the new one.
    assert repo.get_latest(after_seq=frame.seq) is None


def test_a_lower_seq_is_still_applied_because_pgo_restarts_at_one(wire, repo, shm):
    # pgo's seq is per process. After a mode switch and back the counter is 1
    # again while this process may still hold seq 40; that is a new run, not
    # a stale notice, and the repo keeps its own counter anyway.
    _, callback = wire
    p40 = write_pcd(shm / "map_cloud_40.pcd", [(4.0, 4.0, 4.0)])
    p1 = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])

    callback(notice(path=p40, points=1, seq=40))
    callback(notice(path=p1, points=1, seq=1))

    frame, pts = slot_points(repo)
    assert frame.seq == 2
    assert pts.tolist() == [[1.0, 1.0, 1.0]]


def test_the_announced_point_count_is_not_trusted_over_the_file(wire, repo, shm):
    # `points` in the notice is what pgo counted before writing; the file is
    # the truth. A mismatch is not an error -- the slot holds what was read.
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0), (2.0, 2.0, 2.0)])

    callback(notice(path=path, points=999, seq=1))

    _, pts = slot_points(repo)
    assert pts.shape[0] == 2


# ---- clearing -------------------------------------------------------------


def test_an_empty_notice_clears_the_slot(wire, repo, shm):
    # pgo's reset publishes exactly this, and it is the only thing that tells a
    # browser to stop drawing a map that no longer exists.
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])

    callback(notice(path=path, points=1, seq=1))
    callback(notice(path="", points=0, seq=2))

    frame = repo.get_latest()
    assert frame.num_points == 0
    assert frame.data == b""
    # The seq MUST advance: the WS pump is seq-driven, so a same-seq write is
    # one no connected client would ever be handed.
    assert frame.seq == 2


def test_an_empty_notice_before_any_map_is_still_a_frame(wire, repo):
    # No special case for "nothing was there anyway" -- a late subscriber that
    # joins after the reset still needs to be told the map is empty.
    _, callback = wire

    callback(notice(path="", points=0, seq=1))

    frame = repo.get_latest()
    assert (frame.num_points, frame.data) == (0, b"")


def test_a_zero_point_count_clears_even_if_a_path_is_given(wire, repo, shm):
    # Either half of the empty contract is enough; pgo sends both, but the
    # count is what "there is nothing to draw" means.
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])

    callback(notice(path=path, points=1, seq=1))
    callback(notice(path=path, points=0, seq=2))

    frame = repo.get_latest()
    assert (frame.num_points, frame.data) == (0, b"")


def test_a_nan_only_file_clears_the_slot_too(wire, repo, shm):
    # read_pcd_xyz drops non-finite rows before the count check, so this
    # reaches the same path. Right call: both mean "there is nothing to draw",
    # and splitting them would need a second signal pgo does not send.
    _, callback = wire
    p1 = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])
    p2 = write_pcd(shm / "map_cloud_2.pcd", [(float("nan"), 0.0, 0.0)])

    callback(notice(path=p1, points=1, seq=1))
    callback(notice(path=p2, points=1, seq=2))

    frame = repo.get_latest()
    assert (frame.num_points, frame.data) == (0, b"")


def test_nan_points_are_dropped_before_packing(wire, repo, shm):
    _, callback = wire
    path = write_pcd(
        shm / "map_cloud_1.pcd", [(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0)]
    )

    callback(notice(path=path, points=2, seq=1))

    _, pts = slot_points(repo)
    assert pts.tolist() == [[1.0, 2.0, 3.0]]


# ---- failures leave the slot alone ----------------------------------------


def _unchanged_after(repo, callback, msg):
    before = repo.get_latest()
    callback(msg)  # must not raise: an exception here ends the executor
    after = repo.get_latest()
    assert (after.seq, after.num_points, after.data) == (
        before.seq,
        before.num_points,
        before.data,
    )


def test_a_missing_file_keeps_the_current_map(wire, repo, shm):
    # The ordinary "stale" shape: the notice we act on can be one behind pgo's
    # newest, pgo keeps two, and a reset or restart in between removed the
    # file. The next notice is the correction, not this one.
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])
    callback(notice(path=path, points=1, seq=1))

    _unchanged_after(
        repo, callback, notice(path=str(shm / "map_cloud_2.pcd"), points=5, seq=2)
    )


def test_a_path_outside_the_shared_root_is_ignored(wire, repo, shm, tmp_path):
    # A topic told us to read a file; only the tmpfs pgo writes into counts.
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])
    callback(notice(path=path, points=1, seq=1))

    elsewhere = write_pcd(tmp_path / "elsewhere.pcd", [(9.0, 9.0, 9.0)])
    _unchanged_after(repo, callback, notice(path=elsewhere, points=1, seq=2))
    # A relative path can never be under the root either.
    _unchanged_after(repo, callback, notice(path="map_cloud_3.pcd", points=1, seq=3))


def test_malformed_notices_are_ignored(wire, repo, shm):
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])
    callback(notice(path=path, points=1, seq=1))

    _unchanged_after(repo, callback, String(data="not json"))
    _unchanged_after(repo, callback, String(data="[]"))
    _unchanged_after(repo, callback, String(data=json.dumps({"seq": 2, "path": path})))
    _unchanged_after(
        repo,
        callback,
        String(data=json.dumps({"seq": "2", "path": path, "points": 1, "frame_id": "map"})),
    )


def test_a_file_that_is_not_a_pcd_keeps_the_current_map(wire, repo, shm):
    _, callback = wire
    path = write_pcd(shm / "map_cloud_1.pcd", [(1.0, 1.0, 1.0)])
    callback(notice(path=path, points=1, seq=1))

    junk = shm / "map_cloud_2.pcd"
    junk.write_bytes(b"\x00\x01\x02 definitely not a header")
    _unchanged_after(repo, callback, notice(path=str(junk), points=1, seq=2))

    truncated = shm / "map_cloud_3.pcd"
    full = (shm / "map_cloud_1.pcd").read_bytes()
    truncated.write_bytes(full[:-4])  # header promises one point, body is short
    _unchanged_after(repo, callback, notice(path=str(truncated), points=1, seq=3))


# ---- the parser on its own --------------------------------------------------


def test_parse_map_cloud_notice_reads_the_five_fields():
    got = parse_map_cloud_notice(
        json.dumps(
            {
                "seq": 12,
                "path": "/dev/shm/syncai_pgo/robot01/map_cloud_12.pcd",
                "points": 516043,
                "frame_id": "map",
                "stamp": {"sec": 1, "nanosec": 2},
            }
        )
    )
    assert got == MapCloudNotice(
        seq=12,
        path="/dev/shm/syncai_pgo/robot01/map_cloud_12.pcd",
        points=516043,
        frame_id="map",
    )


@pytest.mark.parametrize(
    "data",
    [
        "",
        "{",
        "42",
        json.dumps({"path": "", "points": 0, "frame_id": "map"}),  # no seq
        json.dumps({"seq": 1, "points": 0, "frame_id": "map"}),  # no path
        json.dumps({"seq": 1, "path": "", "frame_id": "map"}),  # no points
        json.dumps({"seq": 1, "path": "", "points": 0}),  # no frame_id
        json.dumps({"seq": -1, "path": "", "points": 0, "frame_id": "map"}),
        json.dumps({"seq": True, "path": "", "points": 0, "frame_id": "map"}),
        json.dumps({"seq": 1, "path": "", "points": "0", "frame_id": "map"}),
        json.dumps({"seq": 1, "path": None, "points": 0, "frame_id": "map"}),
        json.dumps({"seq": 1, "path": "", "points": 0, "frame_id": 3}),
    ],
)
def test_parse_map_cloud_notice_rejects_anything_else(data):
    with pytest.raises(ValueError):
        parse_map_cloud_notice(data)
