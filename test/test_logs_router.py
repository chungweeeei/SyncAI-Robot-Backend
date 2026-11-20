"""GET /api/v1/logs/export: the gateway's tar, spooled and streamed."""

import io
import tarfile

import pytest

pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.interfaces.rest.routers.logs import init_logs_router  # noqa: E402
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubLogBundleGateway:
    def __init__(self, scratch_dir):
        self.scratch_dir = scratch_dir

    def write_bundle(self, dest):
        with tarfile.open(fileobj=dest, mode="w") as tar:
            data = b"hello"
            info = tarfile.TarInfo("robot01-logs-20261120T000000Z/journal.log")
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
        return {"name": "robot01-logs-20261120T000000Z", "journal": {"included": True}}


@pytest.fixture
def client(logger, tmp_path):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_logs_router(logger=logger, log_bundle_gw=_StubLogBundleGateway(str(tmp_path)))
    )
    return TestClient(app)


def test_export_streams_the_tar_as_an_attachment(client, tmp_path):
    response = client.get("/api/v1/logs/export")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/x-tar"
    assert response.headers["content-disposition"] == (
        'attachment; filename="robot01-logs-20261120T000000Z.tar"'
    )
    assert response.headers["content-length"] == str(len(response.content))
    assert response.headers["cache-control"] == "no-store"
    with tarfile.open(fileobj=io.BytesIO(response.content)) as tar:
        member = tar.getmember("robot01-logs-20261120T000000Z/journal.log")
        assert tar.extractfile(member).read() == b"hello"
    # The spool is anonymous: nothing is left in the scratch directory.
    assert list(tmp_path.iterdir()) == []
