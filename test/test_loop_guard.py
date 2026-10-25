"""The conftest guard against disk I/O on the event loop, checked itself.

Every test runs under it; this one makes sure it still fires, so a green suite
keeps meaning "no async route reads the disk" rather than "the guard went
blind" (a starlette upgrade that runs routes elsewhere, a renamed package).
"""

import pytest

pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.helpers.system_config import (  # noqa: E402
    SYSTEM_INI_ENV,
    active_map_name,
)


@pytest.fixture
def client(logger, tmp_path, monkeypatch):
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    app = FastAPI()

    @app.get("/on-the-loop")
    async def on_the_loop():
        return {"map": active_map_name(logger)}

    @app.get("/in-the-threadpool")
    def in_the_threadpool():
        return {"map": active_map_name(logger)}

    return TestClient(app)


def test_a_plain_def_route_may_read_the_disk(client, no_disk_io_on_the_event_loop):
    assert client.get("/in-the-threadpool").json() == {"map": "full"}

    assert no_disk_io_on_the_event_loop == {}


def test_an_async_route_reading_the_disk_is_caught(client, no_disk_io_on_the_event_loop):
    assert client.get("/on-the-loop").json() == {"map": "full"}

    callers = list(no_disk_io_on_the_event_loop)
    assert callers and callers[0].startswith("helpers/system_config.py:")
    # Caught, which is the point; cleared so this test itself passes.
    no_disk_io_on_the_event_loop.clear()


def test_a_test_reading_the_disk_itself_is_not_caught(tmp_path,
                                                      no_disk_io_on_the_event_loop):
    """Only package code is held to it: an async test's own fixture files are
    the test's business."""
    import asyncio

    async def _read():
        return (tmp_path / "x").exists()

    assert asyncio.run(_read()) is False
    assert no_disk_io_on_the_event_loop == {}
