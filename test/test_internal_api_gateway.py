"""Tests for InternalApiGateway, the HTTP client for the robot's ``/internal`` service.

``httpx.MockTransport`` is the seam, as in test_tts_gateway.py, so the real
request path, the base_url joining and the JSON decoding all run.
"""

import pytest

pytest.importorskip("httpx")
pytest.importorskip("structlog")

import httpx  # noqa: E402
import structlog  # noqa: E402

from syncai_backend.gateways.internal_api.internal_api import (  # noqa: E402
    InternalApiGateway,
    init_internal_api_gateway,
)


_BASE_URL = "http://internal.test:8090"


def make_gateway(handler) -> InternalApiGateway:
    """A gateway whose transport is `handler` instead of a socket."""
    gateway = InternalApiGateway(logger=structlog.get_logger(), base_url=_BASE_URL)
    gateway._client = httpx.Client(
        transport=httpx.MockTransport(handler), base_url=_BASE_URL
    )
    return gateway


def summary(name, map_id):
    return {"id": map_id, "name": name, "active": False, "hasOctomap": True}


def test_find_map_id_matches_on_name():
    seen = []

    def handler(request):
        seen.append((request.method, request.url.path))
        return httpx.Response(
            200, json={"data": [summary("lobby", "uuid-1"), summary("hall", "uuid-2")]}
        )

    assert make_gateway(handler).find_map_id("hall") == (True, "", "uuid-2")
    assert seen == [("GET", "/internal/maps")]


def test_find_map_id_accepts_a_bare_list():
    def handler(request):
        return httpx.Response(200, json=[summary("hall", "uuid-2")])

    assert make_gateway(handler).find_map_id("hall") == (True, "", "uuid-2")


def test_find_map_id_is_none_for_an_unknown_name():
    def handler(request):
        return httpx.Response(200, json={"data": [summary("lobby", "uuid-1")]})

    assert make_gateway(handler).find_map_id("hall") == (True, "", None)


def test_find_map_id_fails_on_an_error_answer():
    def handler(request):
        return httpx.Response(
            500, json={"error": {"code": "SYSTEM_INTERNAL_ERROR", "message": "boom"}}
        )

    ok, message, map_id = make_gateway(handler).find_map_id("hall")

    assert ok is False and map_id is None
    assert message == "the internal service answered 500 SYSTEM_INTERNAL_ERROR: boom"


def test_find_map_id_fails_when_the_service_is_down():
    def handler(request):
        raise httpx.ConnectError("refused")

    ok, message, _ = make_gateway(handler).find_map_id("hall")

    assert ok is False
    assert "could not reach the internal service" in message


def test_activate_map_posts_to_the_maps_uuid():
    seen = []

    def handler(request):
        seen.append((request.method, request.url.path, request.content))
        return httpx.Response(
            201,
            json={"data": {"id": "uuid-2", "name": "hall", "switched": True, "message": "ok"}},
        )

    ok, message, payload = make_gateway(handler).activate_map("uuid-2")

    assert ok is True and message == ""
    assert payload["switched"] is True
    # No body: the uuid in the path is the whole request.
    assert seen == [("POST", "/internal/maps/uuid-2/activate", b"")]


def test_activate_map_waits_past_the_services_deadline():
    timeouts = []

    def handler(request):
        timeouts.append(request.extensions["timeout"])
        return httpx.Response(201, json={"data": {}})

    make_gateway(handler).activate_map("uuid-2")

    # Hanging up early cancels the switch on the service's side; it answers by 240 s.
    assert timeouts[0]["read"] > 240
    assert timeouts[0]["connect"] < 240


def test_activate_map_fails_on_an_error_answer():
    def handler(request):
        return httpx.Response(
            408, json={"error": {"code": "SYSTEM_REQUEST_TIMEOUT", "message": "late"}}
        )

    ok, message, payload = make_gateway(handler).activate_map("uuid-2")

    assert ok is False and payload is None
    assert "408 SYSTEM_REQUEST_TIMEOUT: late" in message


def test_init_reads_the_url_from_env(monkeypatch):
    monkeypatch.setenv("INTERNAL_API_URL", "http://10.0.0.5:9000/")

    gateway = init_internal_api_gateway(logger=structlog.get_logger())

    assert gateway._base_url == "http://10.0.0.5:9000"
