"""Tests for the sign-in route and the guard every other route sits behind.

What is pinned: the one account signs in and nothing else does, with one
sentence for either half wrong; a token opens a guarded route over a header
and a WebSocket over its query, and an expired, forged or missing one opens
neither; sign-in itself is reachable without one, as JSON for the console and
as the form /docs' Authorize dialog sends; a good token is renewed into a
fresh one in the same chain, and a chain is not renewed past its cap; the
real server's OpenAPI names that flow on every guarded route; and /docs asks
for the account alone.
"""

import time

import pytest

pytest.importorskip("httpx")
pytest.importorskip("jwt")
pytest.importorskip("python_multipart")

import jwt  # noqa: E402

from fastapi import APIRouter, Depends, FastAPI, WebSocket  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402
from starlette.websockets import WebSocketDisconnect  # noqa: E402

from syncai_backend.interfaces.rest.auth import (  # noqa: E402
    SIGN_IN_AGAIN,
    TOKEN_URL,
    AuthConfig,
    init_auth_guard,
    issue_token,
    renew_token,
)
from syncai_backend.interfaces.rest.routers.auth import init_auth_router  # noqa: E402
from syncai_backend.interfaces.rest.server import register_exception_handlers  # noqa: E402


CFG = AuthConfig(username="admin", password="s3cret", secret="k" * 32, expire_s=3600)


@pytest.fixture
def client(logger):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(init_auth_router(logger=logger, auth=CFG))

    # Stands in for the server's guarded routers: one route, one stream.
    guarded = APIRouter()

    @guarded.get("/api/v1/guarded")
    def guarded_route():
        return {"ok": True}

    @guarded.websocket("/api/v1/stream")
    async def stream(ws: WebSocket):
        await ws.accept()
        await ws.send_json({"ok": True})
        await ws.close()

    protected = APIRouter(dependencies=[Depends(init_auth_guard(CFG))])
    protected.include_router(guarded)
    app.include_router(protected)
    return TestClient(app)


def _sign_in(client) -> dict:
    res = client.post("/api/v1/auth/login", json={"username": "admin", "password": "s3cret"})
    assert res.status_code == 200
    return res.json()


def test_sign_in_answers_a_token_that_lasts_the_configured_time(client):
    body = _sign_in(client)
    assert body["token_type"] == "bearer"
    assert body["expires_in"] == 3600
    me = client.get(
        "/api/v1/auth/me", headers={"Authorization": f"Bearer {body['access_token']}"}
    )
    assert me.status_code == 200
    assert me.json() == {"username": "admin", "expires_at": body["expires_at"]}


@pytest.mark.parametrize(
    "username,password", [("admin", "wrong"), ("root", "s3cret"), ("", "")]
)
def test_a_wrong_half_is_refused_without_saying_which(client, username, password):
    res = client.post("/api/v1/auth/login", json={"username": username, "password": password})
    assert res.status_code == 401
    assert res.json() == {"detail": "Wrong username or password."}


def test_the_form_sign_in_answers_a_token_that_opens_a_guarded_route(client):
    # What the Authorize dialog of /docs posts: an OAuth2 password grant.
    res = client.post(
        TOKEN_URL, data={"grant_type": "password", "username": "admin", "password": "s3cret"}
    )
    assert res.status_code == 200
    body = res.json()
    assert body["token_type"] == "bearer"
    res = client.get(
        "/api/v1/guarded", headers={"Authorization": f"Bearer {body['access_token']}"}
    )
    assert res.status_code == 200


def test_the_form_sign_in_refuses_a_wrong_pair_as_the_json_one_does(client):
    res = client.post(TOKEN_URL, data={"username": "admin", "password": "wrong"})
    assert res.status_code == 401
    assert res.json() == {"detail": "Wrong username or password."}


def test_a_token_opens_a_guarded_route(client):
    token = _sign_in(client)["access_token"]
    res = client.get("/api/v1/guarded", headers={"Authorization": f"Bearer {token}"})
    assert res.status_code == 200


def test_no_token_is_a_401_in_the_house_shape(client):
    res = client.get("/api/v1/guarded")
    assert res.status_code == 401
    assert res.json() == {"detail": SIGN_IN_AGAIN}


def test_an_expired_token_is_refused(client):
    token, _ = issue_token(CFG, "admin", now=0)
    res = client.get("/api/v1/guarded", headers={"Authorization": f"Bearer {token}"})
    assert res.status_code == 401


def test_a_token_signed_with_another_key_is_refused(client):
    other = AuthConfig(username="admin", password="x", secret="o" * 32, expire_s=3600)
    token, _ = issue_token(other, "admin")
    res = client.get("/api/v1/guarded", headers={"Authorization": f"Bearer {token}"})
    assert res.status_code == 401


def test_a_good_token_is_renewed_into_a_fresh_one_that_opens_routes(client):
    first = _sign_in(client)
    res = client.post(
        "/api/v1/auth/refresh", headers={"Authorization": f"Bearer {first['access_token']}"}
    )
    assert res.status_code == 200
    body = res.json()
    assert body["token_type"] == "bearer"
    assert 0 < body["expires_in"] <= 3600
    res = client.get(
        "/api/v1/guarded", headers={"Authorization": f"Bearer {body['access_token']}"}
    )
    assert res.status_code == 200


def test_a_renewal_keeps_the_chain_s_start(client):
    # Signed in two hours ago, renewed ten minutes ago: the new token still
    # says when the password was given.
    began = int(time.time()) - 7200
    token, _ = issue_token(CFG, "admin", now=int(time.time()) - 600, auth_time=began)
    res = client.post("/api/v1/auth/refresh", headers={"Authorization": f"Bearer {token}"})
    assert res.status_code == 200
    claims = jwt.decode(res.json()["access_token"], CFG.secret, algorithms=["HS256"])
    assert claims["auth_time"] == began
    assert claims["sub"] == "admin"


def test_a_chain_is_not_renewed_past_its_cap():
    began = 1_000_000
    claims = {"sub": "admin", "iat": began, "auth_time": began}
    renew_token(CFG, claims, now=began + CFG.max_session_s - 1)
    with pytest.raises(Exception) as exc:
        renew_token(CFG, claims, now=began + CFG.max_session_s)
    assert str(exc.value) == SIGN_IN_AGAIN


def test_no_token_outlives_its_chain_s_cap():
    began = 1_000_000
    near_cap = began + CFG.max_session_s - 600
    _, expires_at = issue_token(CFG, "admin", now=near_cap, auth_time=began)
    assert expires_at == began + CFG.max_session_s


def test_a_token_from_before_auth_time_starts_its_chain_at_its_own_issue():
    claims = {"sub": "admin", "iat": 1_000_000}
    token, _ = renew_token(CFG, claims, now=1_000_100)
    assert jwt.decode(token, CFG.secret, algorithms=["HS256"], options={"verify_exp": False})[
        "auth_time"
    ] == 1_000_000


def test_renewal_needs_a_token_that_still_works(client):
    expired, _ = issue_token(CFG, "admin", now=0)
    for headers in ({}, {"Authorization": f"Bearer {expired}"}):
        res = client.post("/api/v1/auth/refresh", headers=headers)
        assert res.status_code == 401
        assert res.json() == {"detail": SIGN_IN_AGAIN}


def test_a_stream_opens_with_the_token_in_its_query(client):
    token = _sign_in(client)["access_token"]
    with client.websocket_connect(f"/api/v1/stream?token={token}") as ws:
        assert ws.receive_json() == {"ok": True}


def test_a_stream_without_a_token_is_closed_before_it_opens(client):
    with pytest.raises(WebSocketDisconnect) as exc:
        with client.websocket_connect("/api/v1/stream") as ws:
            ws.receive_json()
    assert exc.value.code == 1008


def test_every_route_of_the_real_server_but_sign_in_and_health_needs_a_token(logger):
    # The server itself, not the stand-in above: a router included into the
    # app beside `protected` (as the log export once was) fails here. Every
    # path in its OpenAPI is called without a token, its parameters filled
    # with junk: the guard answers before anything reads them.
    pytest.importorskip("temporalio")
    import re
    from unittest.mock import MagicMock

    from syncai_backend.interfaces.rest.server import RestDeps, init_rest_server

    deps = RestDeps(**{
        name: MagicMock() for name in RestDeps.__dataclass_fields__
    } | {"settings_svcs": {}, "auth": CFG, "robot_model": "g23"})
    client = TestClient(init_rest_server(logger=logger, deps=deps))

    open_paths = {"/health", "/api/v1/auth/login", TOKEN_URL}
    paths = client.app.openapi()["paths"]
    assert "/api/v1/logs/export" in paths
    unguarded = [
        (method.upper(), path)
        for path, methods in paths.items()
        if path not in open_paths
        for method in methods
        if client.request(method, re.sub(r"\{[^}]+\}", "x", path)).status_code != 401
    ]
    assert unguarded == []


def test_docs_is_open_and_is_the_stock_page_set_up_for_this_api(logger):
    # /docs is Swagger UI's page plus docs_page.py's settings and stylesheet:
    # the tags folded, the token kept over a reload, and a sign-in dialog
    # without the client id and secret this server never reads.
    pytest.importorskip("temporalio")
    from unittest.mock import MagicMock

    from syncai_backend.interfaces.rest.server import RestDeps, init_rest_server

    deps = RestDeps(**{
        name: MagicMock() for name in RestDeps.__dataclass_fields__
    } | {"settings_svcs": {}, "auth": CFG, "robot_model": "g23"})
    res = TestClient(init_rest_server(logger=logger, deps=deps)).get("/docs")

    assert res.status_code == 200
    assert "swagger-ui-bundle.js" in res.text and "/openapi.json" in res.text
    for setting in (
        '"docExpansion": "none"',
        '"persistAuthorization": true',
        '"displayRequestDuration": true',
    ):
        assert setting in res.text
    assert "tryItOutEnabled" not in res.text
    for field in ("password_type", "client_id_password", "client_secret_password"):
        assert f'label[for="{field}"]' in res.text
    assert ".scheme-container" in res.text
    assert res.text.index("<style>") < res.text.index("</head>")


def test_the_real_servers_openapi_names_the_sign_in_on_every_guarded_route(logger):
    # What puts the Authorize dialog in /docs and the token on each request
    # tried from it: the flow declared once, and required by every operation
    # but the three that are open.
    pytest.importorskip("temporalio")
    from unittest.mock import MagicMock

    from syncai_backend.interfaces.rest.server import RestDeps, init_rest_server

    deps = RestDeps(**{
        name: MagicMock() for name in RestDeps.__dataclass_fields__
    } | {"settings_svcs": {}, "auth": CFG, "robot_model": "g23"})
    spec = init_rest_server(logger=logger, deps=deps).openapi()

    (scheme_name, scheme), = spec["components"]["securitySchemes"].items()
    assert scheme == {
        "type": "oauth2",
        "flows": {"password": {"tokenUrl": TOKEN_URL, "scopes": {}}},
    }

    # Under a tag of its own, not Swagger UI's "default".
    assert spec["paths"]["/health"]["get"]["tags"] == ["Health"]

    open_paths = {"/health", "/api/v1/auth/login", TOKEN_URL}
    wrong = [
        (method.upper(), path)
        for path, methods in spec["paths"].items()
        for method, operation in methods.items()
        if (operation.get("security") == [{scheme_name: []}]) == (path in open_paths)
    ]
    assert wrong == []
