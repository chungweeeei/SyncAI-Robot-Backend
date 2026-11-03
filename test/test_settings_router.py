"""Tests for /api/v1/settings/controller -- the projection and the status codes.

The saga is NodeSettingsService's and is tested in
test_node_settings.py; here the service runs on the same stub controller
so the codes come from the real refusals, not from a second stub's guesses.
"""

import pytest

pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.interfaces.rest.routers.settings import (  # noqa: E402
    init_settings_router,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)
from syncai_backend.services.controller_schema import CONTROLLER  # noqa: E402
from syncai_backend.services.node_settings import NodeSettingsService  # noqa: E402

# The stub controller and its fixtures, shared with the service's tests;
# pytest finds a fixture by the name it is imported under.
from test_node_settings import (  # noqa: E402, F401
    _StubController,
    controller,
    factory_file,
    repo,
    svc,
)

URL = "/api/v1/settings/controller"


@pytest.fixture
def client(logger, svc):  # noqa: F811
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(init_settings_router(logger=logger, settings_svcs={"controller": svc}))
    return TestClient(app)


def _field(body, field_id):
    return next(f for f in body["fields"] if f["id"] == field_id)


def test_get_returns_groups_and_fields_with_their_schema(client):
    response = client.get(URL)
    assert response.status_code == 200
    body = response.json()

    assert body["live_available"] is True
    assert [g["id"] for g in body["groups"]][:2] == ["speed", "arrival"]
    speed = _field(body, "max_speed")
    assert speed == {
        "id": "max_speed",
        "group": "speed",
        "label": "Maximum walking speed",
        "description": speed["description"],
        "kind": "number",
        "unit": "m/s",
        "min": 0.4,
        "max": 1.0,
        "step": 0.05,
        "apply": "live",
        "value": 0.8,
        "current": 0.8,
        "default": 0.8,
        "modified": False,
        "pending_restart": False,
    }
    # The engineering names stay on the robot's side of the wire.
    assert "targets" not in speed


def test_put_changes_and_returns_the_new_state(client, controller):  # noqa: F811
    response = client.put(URL, json={"values": {"max_speed": 0.6}})
    assert response.status_code == 200
    speed = _field(response.json(), "max_speed")
    assert (speed["value"], speed["modified"]) == (0.6, True)
    assert controller.params["FollowPath.vx_max"] == 0.6


def test_a_bad_value_is_a_400_sentence_not_a_422(client):
    response = client.put(URL, json={"values": {"max_speed": "fast"}})
    assert response.status_code == 400
    assert response.json() == {
        "detail": "Maximum walking speed must be a number",
        "code": "invalid_setting",
    }


def test_an_idle_field_while_navigating_is_a_409(client, controller):  # noqa: F811
    controller.navigating = True
    response = client.put(URL, json={"values": {"failure_tolerance": 4.0}})
    assert response.status_code == 409
    assert response.json()["code"] == "controller_busy"


def test_delete_resets_one_field_or_all(client, controller):  # noqa: F811
    client.put(URL, json={"values": {"max_speed": 0.6, "failure_tolerance": 4.0}})

    one = client.delete(URL, params={"id": "max_speed"})
    assert one.status_code == 200
    assert _field(one.json(), "max_speed")["modified"] is False
    assert _field(one.json(), "failure_tolerance")["modified"] is True

    every = client.delete(URL)
    assert every.status_code == 200
    assert not any(f["modified"] for f in every.json()["fields"])
    assert controller.params["failure_tolerance"] == 3.0


def test_with_the_controller_down_the_page_still_answers(logger, repo):  # noqa: F811
    down = _StubController()
    down.up = False
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_settings_router(
            logger=logger,
            settings_svcs={
                "controller": NodeSettingsService(
                    logger=logger,
                    schema=CONTROLLER,
                    gateway=down,
                    overlay_repo=repo,
                    subject="controller",
                )
            },
        )
    )
    body = TestClient(app).get(URL).json()

    assert body["live_available"] is False
    speed = _field(body, "max_speed")
    assert (speed["value"], speed["current"], speed["default"]) == (0.8, None, 0.8)


def test_each_page_has_its_own_path_and_an_unknown_one_is_a_404(client):
    """Only the pages registered exist: /planner here is not one."""
    assert client.get("/api/v1/settings/planner").status_code == 404
    assert client.get(URL).status_code == 200
