"""Tests for /api/v1/network/wifi — projection over a stubbed RobotGateway.

Small on purpose: the gateway methods are (success, message[, payload]) tuples,
so all the router owns is the projection and the status-code mapping — a failed
scan is the robot's fault (502), a failed connect is usually the caller's
credentials (400).
"""

from types import SimpleNamespace

import pytest

pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.interfaces.rest.routers.network import (  # noqa: E402
    init_network_router,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubRobotGateway:
    def __init__(self):
        self.scan_result = (
            True,
            "",
            [SimpleNamespace(bssid="aa:bb:cc:dd:ee:ff", ssid="net", rssi=-40)],
        )
        self.connect_result = (True, "")
        self.connect_calls = []
        self.list_result = (
            True,
            "",
            [
                SimpleNamespace(
                    uuid="u-1", name="net", ssid="net", active=True, last_used=1760000000
                ),
                SimpleNamespace(
                    uuid="u-2", name="Lab", ssid="lab-5g", active=False, last_used=0
                ),
            ],
        )
        self.activate_result = (True, "", "lab-5g")
        self.activate_calls = []
        self.delete_result = (True, "")
        self.delete_calls = []

    def scan_wifi_networks(self):
        return self.scan_result

    def connect_wifi(self, ssid, password):
        self.connect_calls.append((ssid, password))
        return self.connect_result

    def list_wifi_connections(self):
        return self.list_result

    def activate_wifi_connection(self, uuid):
        self.activate_calls.append(uuid)
        return self.activate_result

    def delete_wifi_connection(self, uuid):
        self.delete_calls.append(uuid)
        return self.delete_result


@pytest.fixture
def robot_gw():
    return _StubRobotGateway()


@pytest.fixture
def client(logger, robot_gw):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(init_network_router(logger=logger, robot_gw=robot_gw))
    return TestClient(app)


def test_scan_projects_the_networks(client):
    body = client.get("/api/v1/network/wifi/scan").json()

    assert body["networks"] == [
        {"bssid": "aa:bb:cc:dd:ee:ff", "ssid": "net", "rssi": -40}
    ]


def test_a_failed_scan_is_a_502(client, robot_gw):
    robot_gw.scan_result = (False, "scan_wifi service is not available", [])

    response = client.get("/api/v1/network/wifi/scan")

    assert response.status_code == 502
    assert "not available" in response.json()["detail"]


def test_connect_passes_credentials_through(client, robot_gw):
    body = client.post(
        "/api/v1/network/wifi/connect", json={"ssid": "net", "password": "secret"}
    ).json()

    assert robot_gw.connect_calls == [("net", "secret")]
    assert "net" in body["message"]


def test_a_failed_connect_is_a_400(client, robot_gw):
    # nmcli's "wrong password" lands here; the request itself was well formed
    # ROS-wise, but the credentials are the caller's to fix.
    robot_gw.connect_result = (False, "Secrets were required, but not provided")

    response = client.post(
        "/api/v1/network/wifi/connect", json={"ssid": "net", "password": "wrong"}
    )

    assert response.status_code == 400


def test_an_empty_ssid_is_rejected_at_the_boundary(client, robot_gw):
    response = client.post(
        "/api/v1/network/wifi/connect", json={"ssid": "", "password": ""}
    )

    assert response.status_code == 422
    assert robot_gw.connect_calls == []


def test_connections_project_with_an_iso_last_used(client):
    body = client.get("/api/v1/network/wifi/connections").json()

    assert body["connections"] == [
        {
            "uuid": "u-1",
            "name": "net",
            "ssid": "net",
            "active": True,
            "last_used_at": "2025-10-09T08:53:20Z",
        },
        # 0 is NetworkManager's "never", not the epoch.
        {
            "uuid": "u-2",
            "name": "Lab",
            "ssid": "lab-5g",
            "active": False,
            "last_used_at": None,
        },
    ]


def test_a_failed_connection_list_is_a_502(client, robot_gw):
    robot_gw.list_result = (False, "list_wifi_connections service is not available", [])

    assert client.get("/api/v1/network/wifi/connections").status_code == 502


def test_activate_names_the_network_it_joined(client, robot_gw):
    body = client.post("/api/v1/network/wifi/connections/u-2/activate").json()

    assert robot_gw.activate_calls == ["u-2"]
    assert body["message"] == "Successfully connected to lab-5g"


def test_a_failed_activate_is_a_400_with_the_robots_sentence(client, robot_gw):
    robot_gw.activate_result = (False, "Failed to connect to lab-5g: no network", "lab-5g")

    response = client.post("/api/v1/network/wifi/connections/u-2/activate")

    assert response.status_code == 400
    assert response.json()["detail"] == "Failed to connect to lab-5g: no network"


def test_delete_is_a_204(client, robot_gw):
    response = client.delete("/api/v1/network/wifi/connections/u-2")

    assert response.status_code == 204
    assert robot_gw.delete_calls == ["u-2"]


def test_forgetting_the_active_network_is_a_400(client, robot_gw):
    robot_gw.delete_result = (
        False,
        "Cannot forget net while the robot is connected to it",
    )

    response = client.delete("/api/v1/network/wifi/connections/u-1")

    assert response.status_code == 400
    assert "Cannot forget" in response.json()["detail"]
