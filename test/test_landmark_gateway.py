"""Tests for LandmarkGateway: one detect is a poll of detect_landmark.

Same seams as test_map_gateway.py: a MagicMock node, hand-completed futures,
no ROS graph. The answers are queued per call, since what is under test is
which one ends the poll. The poll's real sleeps are kept, shrunk to
milliseconds through its own arguments.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")
# syncai_common — only present once the workspace is built and sourced.
pytest.importorskip("syncai_common")

from syncai_common.srv import DetectLandmark  # noqa: E402
from tf2_ros import LookupException  # noqa: E402

from syncai_backend.gateways import ros as ros_module  # noqa: E402
from syncai_backend.gateways.failure import Failure, failure_code  # noqa: E402
from syncai_backend.gateways.landmark.landmark import (  # noqa: E402
    MIN_CONSECUTIVE,
    SIGHTINGS,
    LandmarkGateway,
)

R = DetectLandmark.Response


class _Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result


CAMERA_FRAME = "robot01/camera_optical_frame"


def _seen(consecutive=MIN_CONSECUTIVE, ambiguous=False, x=1.5):
    # In the camera's frame, as an empty target_frame answers, at the image's stamp.
    response = R(error_code=R.NONE, consecutive=consecutive, ambiguous=ambiguous)
    response.pose.header.frame_id = CAMERA_FRAME
    response.pose.header.stamp.sec = 1794893969
    response.pose.header.stamp.nanosec = 250_000_000
    response.pose.pose.position.x = x
    response.age = 0.03
    response.reprojection_error_px = 0.2
    return response


def _failed(code, message):
    return R(error_code=code, error_msg=message)


class _TfBuffer:
    """map <- camera as a pure translation, or the error a lookup raises."""

    def __init__(self):
        self.translation = (10.0, 20.0, 0.5)
        self.error = None
        self.lookups = []

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        self.lookups.append((target_frame, source_frame, time, timeout))
        if self.error is not None:
            raise self.error
        x, y, z = self.translation
        return SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=x, y=y, z=z),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )


@pytest.fixture
def client():
    return MagicMock()


@pytest.fixture
def tf_buffer():
    return _TfBuffer()


@pytest.fixture
def landmark_gw(logger, client, tf_buffer) -> LandmarkGateway:
    node = MagicMock()
    # Keyed by name: a wrong srv_name is a KeyError in every test here.
    node.create_client.side_effect = (
        lambda srv_type, srv_name: {"detect_landmark": client}[srv_name]
    )
    return LandmarkGateway(logger=logger, node=node, tf_buffer=tf_buffer)


def _answers(client, *responses):
    """Queue the answers, the last one repeating for as long as it is asked."""
    queue = list(responses)

    def call_async(request):
        return _Future(queue.pop(0) if len(queue) > 1 else queue[0])

    client.wait_for_service.return_value = True
    client.call_async.side_effect = call_async


def _detect(landmark_gw, budget_s=0.05):
    return landmark_gw.detect("charger_a", budget_s=budget_s, poll_interval_s=0.001)


def test_wakes_the_detector_and_collects_steady_sightings_ambiguous_or_not(landmark_gw, client):
    _answers(
        client,
        _failed(R.SENSOR_STARTING, "waking"),
        _failed(R.NOT_DETECTED, "not seen since it woke"),
        _seen(consecutive=1),
        _seen(consecutive=MIN_CONSECUTIVE, ambiguous=True),
        _seen(consecutive=MIN_CONSECUTIVE + 1, x=2.0),
    )

    ok, message, sightings = _detect(landmark_gw, budget_s=5.0)

    assert ok and message == ""
    assert len(sightings) == SIGHTINGS
    # An ambiguous sighting is kept: its position is right.
    assert sightings[0].ambiguous and sightings[0].x == 11.5
    last = sightings[-1]
    # The camera-frame x carried into the map by the buffer's translation.
    assert (last.x, last.y, last.z) == (12.0, 20.0, 0.5)
    assert (last.camera_x, last.camera_y, last.camera_z) == (10.0, 20.0, 0.5)
    assert last.consecutive == MIN_CONSECUTIVE + 1
    assert (last.age, last.reprojection_error_px) == pytest.approx((0.03, 0.2))
    # Three answers before the streak, then one per sighting.
    assert client.call_async.call_count == 3 + SIGHTINGS


def test_a_budget_that_runs_out_keeps_the_sightings_it_has(landmark_gw, client):
    _answers(client, _seen(), _failed(R.NOT_DETECTED, "not seen for 0.5 s"))

    ok, message, sightings = _detect(landmark_gw)

    assert ok and message == ""
    assert len(sightings) == 1


def test_asks_for_the_pose_in_the_cameras_own_frame(landmark_gw, client):
    _answers(client, _seen())

    _detect(landmark_gw)

    request = client.call_async.call_args.args[0]
    assert request.landmark_id == "charger_a"
    assert request.target_frame == ""


def test_carries_the_sighting_into_the_map_at_the_images_stamp(landmark_gw, client, tf_buffer):
    _answers(client, _seen())

    ok, _, _ = _detect(landmark_gw)

    assert ok
    assert len(tf_buffer.lookups) == SIGHTINGS
    target_frame, source_frame, stamp, timeout = tf_buffer.lookups[0]
    assert (target_frame, source_frame) == ("map", CAMERA_FRAME)
    assert stamp.nanoseconds == 1794893969_250_000_000
    assert 0.0 < timeout.nanoseconds <= 1_000_000_000


def test_a_transform_that_fails_is_the_robots_fault(landmark_gw, client, tf_buffer):
    tf_buffer.error = LookupException("map does not exist")
    _answers(client, _seen())

    ok, message, sightings = _detect(landmark_gw, budget_s=5.0)

    assert not ok and sightings == []
    assert CAMERA_FRAME in message and "map does not exist" in message
    assert failure_code(message) is None
    # The kept answer ends the poll; asking again cannot fix the TF tree.
    assert client.call_async.call_count == 1


def test_an_unknown_landmark_fails_at_once(landmark_gw, client):
    _answers(client, _failed(R.UNKNOWN_LANDMARK, "known: charger_a"))

    ok, message, sightings = _detect(landmark_gw, budget_s=5.0)

    assert not ok and sightings == []
    assert failure_code(message) is Failure.LANDMARK_UNKNOWN
    assert message == "known: charger_a"
    assert client.call_async.call_count == 1


def test_never_seen_is_not_detected_with_the_detectors_sentence(landmark_gw, client):
    _answers(client, _failed(R.NOT_DETECTED, "'charger_a' not seen since it woke"))

    ok, message, _ = _detect(landmark_gw)

    assert not ok
    assert failure_code(message) is Failure.LANDMARK_NOT_DETECTED
    assert message == "'charger_a' not seen since it woke"
    assert client.call_async.call_count > 1


def test_a_sighting_that_never_steadies_is_not_detected(landmark_gw, client):
    _answers(client, _seen(consecutive=MIN_CONSECUTIVE - 1, ambiguous=True))

    ok, message, sightings = _detect(landmark_gw)

    assert not ok and sightings == []
    assert failure_code(message) is Failure.LANDMARK_NOT_DETECTED
    assert f"{MIN_CONSECUTIVE - 1} sighting(s)" in message


@pytest.mark.parametrize(
    "code, sentence",
    [
        (R.SENSOR_UNAVAILABLE, "no frame for 1.0 s"),
        (R.TRANSFORM_FAILED, "map -> camera_optical_frame failed"),
    ],
)
def test_a_detector_that_cannot_answer_is_the_robots_fault(landmark_gw, client, code, sentence):
    _answers(client, _failed(code, sentence))

    ok, message, _ = _detect(landmark_gw)

    assert not ok and message == sentence
    assert failure_code(message) is None


def test_service_not_available(landmark_gw, client):
    client.wait_for_service.return_value = False

    ok, message, sightings = _detect(landmark_gw)

    assert not ok and sightings == []
    assert "not available" in message
    assert failure_code(message) is None
    client.call_async.assert_not_called()


def test_no_answer_drops_each_pending_request(landmark_gw, client, monkeypatch):
    monkeypatch.setattr(ros_module, "wait_for_future", lambda future, timeout=None: False)
    _answers(client, _seen())

    ok, message, _ = _detect(landmark_gw)

    assert not ok and "Timeout" in message
    assert failure_code(message) is None
    assert client.remove_pending_request.call_count == client.call_async.call_count
