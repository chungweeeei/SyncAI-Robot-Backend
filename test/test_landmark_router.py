"""Tests for /api/v1/landmarks/detect — projection over a stubbed gateway.

The gateway's poll is its own test file; what the router owns is the refusals
before the detect, the status per failure, and the 6-DOF sighting flattened to
a waypoint's (x, y, z, theta) with the robot's floor as z.
"""

import math
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip("httpx")
pytest.importorskip("numpy")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.gateways.failure import Failure, fail  # noqa: E402
from syncai_backend.interfaces.rest.routers.landmark import (  # noqa: E402
    init_landmark_router,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)

URL = "/api/v1/landmarks/detect"


def _sighting(x=3.0, y=-1.0, z=0.4, yaw=math.pi / 2, pitch=0.0):
    # Yaw then pitch about the body y axis, as (x, y, z, w).
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(x=-sy * sp, y=cy * sp, z=sy * cp, w=cy * cp),
            )
        ),
        consecutive=4,
        age=0.05,
        reprojection_error_px=0.6,
    )


class _StubLandmarkGateway:
    def __init__(self):
        self.result = (True, "", _sighting())
        self.calls = []
        self.gate = None
        self.entered = threading.Event()

    def detect(self, landmark_id):
        self.calls.append(landmark_id)
        self.entered.set()
        if self.gate is not None:
            self.gate.wait(timeout=5)
        return self.result


class _StubRobotRepo:
    def __init__(self):
        self.state = self.localized(z=2.75)

    @staticmethod
    def localized(z, valid=True):
        return SimpleNamespace(
            localization_valid=valid,
            localization_status=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=z, yaw=0.0)
            ),
        )

    def get_robot_state(self):
        return self.state


class _StubCatalog:
    def __init__(self):
        self.active = "office"

    def active_name(self):
        return self.active


@pytest.fixture
def stubs(logger):
    gateway, robot_repo, catalog = _StubLandmarkGateway(), _StubRobotRepo(), _StubCatalog()
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_landmark_router(
            logger=logger,
            landmark_gw=gateway,
            robot_repo=robot_repo,
            map_catalog_repo=catalog,
        )
    )
    return SimpleNamespace(
        client=TestClient(app), gateway=gateway, robot_repo=robot_repo, catalog=catalog
    )


def test_detect_answers_a_waypoint_pose_on_the_loaded_map(stubs):
    response = stubs.client.post(URL)

    assert response.status_code == 200
    body = response.json()
    assert body["map"] == "office"
    assert (body["x"], body["y"]) == (3.0, -1.0)
    assert body["theta"] == pytest.approx(90.0)
    # The floor the robot stands on, not the marker's height on the wall.
    assert body["z"] == 2.75
    assert body["marker_z"] == 0.4
    assert body["consecutive"] == 4
    assert body["age"] == 0.05
    assert body["reprojection_error_px"] == 0.6
    assert stubs.gateway.calls == ["charger_a"]


def test_a_tilted_landmark_keeps_its_heading(stubs):
    stubs.gateway.result = (True, "", _sighting(yaw=-0.5, pitch=0.3))

    assert stubs.client.post(URL).json()["theta"] == pytest.approx(math.degrees(-0.5))


def test_a_landmark_facing_the_ceiling_is_not_a_detection(stubs):
    stubs.gateway.result = (True, "", _sighting(pitch=-math.pi / 2))

    response = stubs.client.post(URL)

    assert response.status_code == 409
    assert response.json()["code"] == "landmark_not_detected"


def test_the_landmark_can_be_named(stubs):
    assert stubs.client.post(URL, json={"landmark_id": "charger_b"}).status_code == 200
    assert stubs.gateway.calls == ["charger_b"]


def test_no_map_loaded_is_409_before_any_detect(stubs):
    stubs.catalog.active = None

    response = stubs.client.post(URL)

    assert response.status_code == 409
    assert response.json()["code"] == "no_map_loaded"
    assert stubs.gateway.calls == []


@pytest.mark.parametrize("state", [None, _StubRobotRepo.localized(z=0.0, valid=False)])
def test_not_localized_is_409_before_any_detect(stubs, state):
    stubs.robot_repo.state = state

    response = stubs.client.post(URL)

    assert response.status_code == 409
    assert response.json()["code"] == "not_localized"
    assert stubs.gateway.calls == []


def test_not_detected_is_409_with_the_detectors_sentence(stubs):
    stubs.gateway.result = (
        False,
        fail(Failure.LANDMARK_NOT_DETECTED, "'charger_a' not seen since it woke"),
        None,
    )

    response = stubs.client.post(URL)

    assert response.status_code == 409
    assert response.json() == {
        "detail": "'charger_a' not seen since it woke",
        "code": "landmark_not_detected",
    }


def test_unknown_landmark_is_400(stubs):
    stubs.gateway.result = (False, fail(Failure.LANDMARK_UNKNOWN, "known: charger_a"), None)

    response = stubs.client.post(URL, json={"landmark_id": "nope"})

    assert response.status_code == 400
    assert response.json()["code"] == "unknown_landmark"


def test_a_detector_that_is_down_is_502(stubs):
    stubs.gateway.result = (False, "detect_landmark is not available", None)

    response = stubs.client.post(URL)

    assert response.status_code == 502
    assert response.json() == {"detail": "detect_landmark is not available"}


def test_a_second_detect_while_one_runs_is_409(stubs):
    stubs.gateway.gate = threading.Event()
    first = {}
    thread = threading.Thread(target=lambda: first.update(response=stubs.client.post(URL)))
    thread.start()
    try:
        assert stubs.gateway.entered.wait(timeout=5)
        second = stubs.client.post(URL)
    finally:
        stubs.gateway.gate.set()
        thread.join(timeout=5)

    assert second.status_code == 409
    assert second.json()["code"] == "detect_running"
    assert first["response"].status_code == 200
    # The lock is released: the next one runs.
    assert stubs.client.post(URL).status_code == 200
