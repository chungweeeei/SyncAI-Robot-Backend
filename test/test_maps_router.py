"""Tests for the map catalogue routes (/api/v1/maps).

Same router as test_map_router.py — the two URL families were merged into
routers/map.py — but kept as its own file because the fixture differs: this one
needs a tmp_path maps tree and an INI override pinning the active map.

Same shape otherwise: the router is mounted on a bare FastAPI app with the
production exception handlers registered, so the domain-exception ->
status-code mapping is the real one. Repos are real, over a tmp_path maps tree and
in-memory SQLite.
"""

import builtins
import hashlib
import io
import json
import os
import shutil
import struct
import sys
import tarfile
import threading
import types
import zipfile

import pytest

pytest.importorskip("cv2")
pytest.importorskip("nav_msgs")
pytest.importorskip("httpx")
pytest.importorskip("yaml")

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.gateways.workflow.schema import TaskKind  # noqa: E402
from syncai_backend.helpers.system_config import SYSTEM_INI_ENV  # noqa: E402
from syncai_backend.interfaces.rest.routers import map as map_router_module  # noqa: E402
from syncai_backend.interfaces.rest.routers.map import init_map_router  # noqa: E402
from syncai_backend.repositories.mapping.mapping import (  # noqa: E402
    MAPPING_STATUS_TTL_S,
    MappingState,
    init_mapping_status_repo,
)
from syncai_backend.services import gridmap_conversion as conversion_module  # noqa: E402
from syncai_backend.services.gridmap_conversion import (  # noqa: E402
    GridmapConversionService,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubMapGateway:
    """Records reload_map / save_map calls instead of making ROS service calls.

    The one thing this suite cannot make real: a LoadMap client needs a live
    map_server on a DDS graph, a SaveMaps client a live pgo. The repos either
    side stay real. Note save_map writes nothing — a saved map's on-disk files
    are pgo's doing, so tests that need a map.pcd create it themselves.
    """

    def __init__(self):
        self.calls = []
        self.result = (True, "")
        self.save_calls = []
        self.save_result = (True, "")
        # Kept separate from save_*, deliberately: the tests below assert that
        # saving and resetting never reach into each other.
        self.reset_calls = []
        self.reset_result = (
            True,
            "Map discarded. The new one starts building once the lidar has "
            "re-levelled.",
        )
        # The third run act, separate for the same reason.
        self.start_calls = []
        self.start_result = (
            True,
            "Mapping started. Keep the robot still until the lidar has re-levelled.",
        )
        # The map-switch surface. `order` records every ROS step across both
        # clients in sequence, because for a switch the ordering *is* the
        # contract: the localizer moves before map_server so that the likeliest
        # failure leaves nothing changed.
        self.order = []
        self.swap_calls = []
        self.swap_result = (True, "")
        self.services_ready = True
        self.converged = True
        # The keepout mask's map_server. Recorded in `order` too: a switch has
        # to reload it *after* the INI write, the commit point.
        self.keepout_calls = []
        self.keepout_result = (True, "")

    def reload_map(self, yaml_path):
        self.calls.append(yaml_path)
        self.order.append(("reload_map", yaml_path))
        return self.result

    def reload_keepout(self, yaml_path):
        self.keepout_calls.append(yaml_path)
        self.order.append(("reload_keepout", yaml_path))
        return self.keepout_result

    def save_map(self, directory):
        self.save_calls.append(directory)
        return self.save_result

    def reset_mapping(self, reset_lio=True):
        self.reset_calls.append(reset_lio)
        return self.reset_result

    def start_mapping(self, reset_lio=True):
        self.start_calls.append(reset_lio)
        return self.start_result

    def nav_services_ready(self, timeout_sec=2.0):
        return self.services_ready

    def swap_localizer_map(self, pcd_path, x, y, yaw):
        self.swap_calls.append((pcd_path, x, y, yaw))
        self.order.append(("swap_localizer_map", pcd_path))
        return self.swap_result

    def localization_converged(self, timeout_s=3.0):
        return self.converged


class _StubWorkflowGateway:
    """Stands in for the Temporal visibility query behind the task_running gate.

    `error` is how the "Temporal is down" refusal is exercised: list_active_tasks
    raises for real in that case rather than returning an empty list, and the
    difference between those two is the whole point of the tasks_unknown code.
    """

    def __init__(self):
        self.tasks = []
        self.error = None

    async def list_active_tasks(self):
        if self.error is not None:
            raise self.error
        return self.tasks, "2026-09-16T00:00:00Z"


@pytest.fixture
def map_gw():
    return _StubMapGateway()


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


@pytest.fixture
def conversion_svc(logger):
    """The conversion service the router under test is wired to.

    One per test, which the module-level registry it replaced could not be: a
    test that marks a map as converting no longer needs a finally to undo it,
    and one that leaks an entry cannot reach the next test.
    """
    return GridmapConversionService(logger=logger)


def _mark_converting(conversion_svc, name):
    """Pretend a conversion for ``name`` is running in this process."""
    with conversion_svc._lock:
        conversion_svc._active.add(name)


def _clear_converting(conversion_svc, name):
    """Release the slot again, for the tests that assert on both states."""
    with conversion_svc._lock:
        conversion_svc._active.discard(name)


class _Clock:
    """The monotonic clock the status repo ages its sample on, moved by hand."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return _Clock()


@pytest.fixture
def mapping_status_repo(logger, clock):
    """pgo's latched run state, as the router sees it. Empty (unknown) unless
    a test latches a sample with ``_latch``."""
    return init_mapping_status_repo(logger=logger, now=clock)


def _latch(repo, state, key_poses=0, loop_closures=0):
    """Pretend pgo published ``state``, as the subscriber would."""
    repo.update(state=state, key_poses=key_poses, loop_closures=loop_closures, stamp=1.0)


@pytest.fixture
def client(
    logger,
    catalog_repo,
    map_repo,
    map_gw,
    workflow_gw,
    task_template_repo,
    conversion_svc,
    mapping_status_repo,
    tmp_path,
    monkeypatch,
):
    """A client whose active map is 'full', set through the INI env override."""
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_map_router(
            logger=logger,
            map_repo=map_repo,
            map_catalog_repo=catalog_repo,
            map_gw=map_gw,
            task_template_repo=task_template_repo,
            workflow_gw=workflow_gw,
            conversion_svc=conversion_svc,
            mapping_status_repo=mapping_status_repo,
        )
    )
    return TestClient(app)


_OCTET = {"Content-Type": "application/octet-stream"}


def _by_name(body):
    return {entry["name"]: entry for entry in body}


def _plant_sidecar(directory, payload):
    """Write a conversion record into a map directory, as a conversion would.

    Planted rather than produced by a real conversion for the status tests: the
    states worth pinning are the ones no single run can reach on demand — a
    record left behind by a process that no longer exists, and one written by a
    version of this code that had no status field.
    """
    path = directory / conversion_module.GRIDMAP_RECIPE_SIDECAR
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


# --- /api/v1/maps -----------------------------------------------------------


def test_list_returns_both_maps_sorted(client):
    response = client.get("/api/v1/maps")

    assert response.status_code == 200
    assert [entry["name"] for entry in response.json()] == ["full", "rawonly"]


def test_list_marks_only_the_ini_map_active(client):
    body = _by_name(client.get("/api/v1/maps").json())

    assert body["full"]["active"] is True
    assert body["rawonly"]["active"] is False


def test_list_reports_grid_geometry(client):
    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["grid"]["width"] == 6
    assert entry["grid"]["height"] == 4
    assert entry["grid"]["resolution"] == pytest.approx(0.05)
    assert entry["grid"]["origin"]["x"] == pytest.approx(-6.94)
    assert entry["grid"]["origin"]["yaw"] == pytest.approx(0.0)
    assert entry["thumbnail"] == "/api/v1/maps/full/thumbnail"
    assert entry["has_pointcloud"] is True
    assert entry["size_bytes"] > 0
    assert entry["modified_at"].endswith("Z")


def test_list_nulls_grid_for_an_unconverted_map(client):
    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid"] is None
    assert entry["thumbnail"] is None


# --- grid_status --------------------------------------------------------------
#
# The conversion-status surface. There is no job resource and no status
# endpoint: a client that starts a conversion watches this field on the
# catalogue, so every state a conversion can leave a map in has to be
# reachable through it — including the two that outlive the process.


def test_list_reports_ok_for_a_converted_map(client):
    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["grid_status"] == "ok"
    assert entry["grid_error"] is None
    assert entry["grid_converting"] is False


def test_list_reports_none_for_a_map_nobody_converted(client):
    """Distinct from `failed`, and that is the point of the enum: this one wants
    Build grid pressed, a failure wants its reason read first."""
    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid_status"] == "none"
    assert entry["grid_error"] is None


def test_list_reports_ok_for_a_sidecar_written_before_the_status_field(
    client, maps_dir
):
    """Every map converted before 2026-09 has a sidecar with no `status` key.
    The grid on disk is then the only evidence, and it says the run worked."""
    _plant_sidecar(maps_dir / "full", {"recipe": "z-band", "footprint_m2": 400.0})

    assert _by_name(client.get("/api/v1/maps").json())["full"]["grid_status"] == "ok"


def test_list_reports_ok_for_a_status_this_build_does_not_know(client, maps_dir):
    """A sidecar written by a newer backend, read after a rollback.

    GridRecordStatus.parse returns None for a value it cannot name, which puts
    this on the same path as a sidecar with no status at all: the grid on disk
    decides. The requirement is that it does not raise -- this read runs once
    per map on every catalogue listing, which every screen polls.
    """
    _plant_sidecar(
        maps_dir / "full", {"status": "cancelled", "recipe": "z-band"}
    )

    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["grid_status"] == "ok"
    assert entry["grid_error"] is None


def test_list_reports_none_for_an_unknown_status_over_no_grid(client, maps_dir):
    """The other half of the same fallthrough: no grid, so `none`, not `ok`."""
    _plant_sidecar(
        maps_dir / "rawonly", {"status": "cancelled", "recipe": "z-band"}
    )

    assert _by_name(client.get("/api/v1/maps").json())["rawonly"]["grid_status"] == "none"


def test_list_reports_a_failed_conversion_with_its_reason(client, maps_dir):
    """The whole point of the sidecar carrying a status: before it, this map was
    indistinguishable from one nobody had converted and the reason lived only in
    log/stack/<robot_id>/backend/current."""
    _plant_sidecar(
        maps_dir / "rawonly",
        {
            "status": "failed",
            "recipe": "z-band",
            "error": "obstacle band selected no points",
        },
    )

    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid_status"] == "failed"
    assert entry["grid_error"] == "obstacle band selected no points"
    assert entry["grid"] is None


def test_a_failed_reconversion_reports_failed_over_the_grid_it_left_behind(
    client, maps_dir
):
    """archive_gridmap *copies* the live grid aside rather than moving it, so the
    active map never has a window with no file — which means a failed re-convert
    leaves a loadable map whose grid is the old one. Reporting `ok` there would
    present that stale grid as the rebuild the operator asked for."""
    _plant_sidecar(
        maps_dir / "full",
        {"status": "failed", "recipe": "traversability", "error": "no ground"},
    )

    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["grid_status"] == "failed"
    assert entry["grid_error"] == "no ground"
    # Still loadable, and still listed with its geometry.
    assert entry["grid"] is not None


def test_list_reports_an_abandoned_conversion_as_interrupted(client, maps_dir):
    """A record saying `converting` with nothing in the registry behind it: the
    process running it is gone (a backend restart, or a switch_mode that tore
    down the byobu session the backend is a pane of). Nothing is coming to
    finish it, so the state has to say so rather than read as in-flight."""
    _plant_sidecar(maps_dir / "rawonly", {"status": "converting", "recipe": "z-band"})

    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid_status"] == "interrupted"
    assert entry["grid_converting"] is False
    assert entry["grid_error"] is None


def test_a_running_conversion_outranks_whatever_the_sidecar_says(
    client, conversion_svc, maps_dir
):
    """The registry is authoritative while this process is up. The sidecar write
    is best-effort, so a conversion whose record never landed — or landed as the
    previous run's failure — must still report as running."""
    _plant_sidecar(maps_dir / "rawonly", {"status": "failed", "error": "last time"})
    _mark_converting(conversion_svc, "rawonly")

    entry = _by_name(client.get("/api/v1/maps").json())["rawonly"]

    assert entry["grid_status"] == "converting"
    assert entry["grid_error"] is None


def test_list_survives_a_half_written_sidecar(client, maps_dir):
    """A conversion writes this file while the console's two-second catalogue
    poll reads it, so a torn read is expected traffic, not a corrupt map."""
    (maps_dir / "rawonly" / conversion_module.GRIDMAP_RECIPE_SIDECAR).write_text(
        '{"status": "conv'
    )

    response = client.get("/api/v1/maps")

    assert response.status_code == 200
    assert _by_name(response.json())["rawonly"]["grid_status"] == "none"


def test_list_counts_vertices_of_that_map_only(client, map_repo):
    map_repo.create_vertices(map="full", vertices=[
        {"name": "a", "type": "GENERAL", "x": 1.0, "y": 2.0, "theta": 0.0},
        {"name": "b", "type": "CHARGER", "x": 3.0, "y": 4.0, "theta": 90.0},
    ])
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "c", "type": "GENERAL", "x": 5.0, "y": 6.0, "theta": 0.0},
    ])

    body = _by_name(client.get("/api/v1/maps").json())

    assert body["full"]["vertex_count"] == 2
    assert body["rawonly"]["vertex_count"] == 1


# --- /api/v1/maps/{name} ----------------------------------------------------


def test_get_returns_one_summary(client):
    response = client.get("/api/v1/maps/full")

    assert response.status_code == 200
    assert response.json()["name"] == "full"


def test_get_missing_map_returns_404(client):
    assert client.get("/api/v1/maps/nosuchmap").status_code == 404


def test_get_unsafe_name_returns_400(client):
    assert client.get("/api/v1/maps/with%20space").status_code == 400


# --- /api/v1/maps/{name}/thumbnail ------------------------------------------


def test_thumbnail_returns_png(client):
    response = client.get("/api/v1/maps/full/thumbnail")

    assert response.status_code == 200
    assert response.headers["content-type"] == "image/png"
    # PNG magic; proves an image came back rather than an error body.
    assert response.content[:8] == b"\x89PNG\r\n\x1a\n"


def test_thumbnail_is_cached_until_the_file_changes(client, maps_dir, make_pgm):
    first = client.get("/api/v1/maps/full/thumbnail")
    again = client.get("/api/v1/maps/full/thumbnail")

    assert again.headers["etag"] == first.headers["etag"]
    assert again.content == first.content

    make_pgm(maps_dir / "full" / "gridmap.pgm", 9, 9, fill=0)
    third = client.get("/api/v1/maps/full/thumbnail")

    assert third.headers["etag"] != first.headers["etag"]
    assert third.content != first.content


def test_thumbnail_revalidates_to_304(client):
    tag = client.get("/api/v1/maps/full/thumbnail").headers["etag"]

    response = client.get(
        "/api/v1/maps/full/thumbnail", headers={"If-None-Match": tag}
    )

    assert response.status_code == 304


def test_thumbnail_404_when_the_map_has_none(client):
    assert client.get("/api/v1/maps/rawonly/thumbnail").status_code == 404


def test_thumbnail_404_when_the_gridmap_is_unreadable(client, maps_dir):
    """A torn file must be a 404, not a traceback."""
    (maps_dir / "full" / "gridmap.pgm").write_bytes(b"garbage")

    assert client.get("/api/v1/maps/full/thumbnail").status_code == 404


# --- /api/v1/maps/{name}/image ----------------------------------------------


def test_image_is_a_full_size_png(client):
    """Native resolution, unlike the thumbnail: 6x4 in, 6x4 out."""
    response = client.get("/api/v1/maps/full/image")

    assert response.status_code == 200
    assert response.headers["content-type"] == "image/png"
    decoded = cv2.imdecode(
        np.frombuffer(response.content, np.uint8), cv2.IMREAD_GRAYSCALE
    )
    assert decoded.shape == (4, 6)


def test_image_and_thumbnail_share_the_source_etag(client):
    """Both hash the .pgm, so a client can revalidate either against the other."""
    image = client.get("/api/v1/maps/full/image")
    thumbnail = client.get("/api/v1/maps/full/thumbnail")

    assert image.headers["etag"] == thumbnail.headers["etag"]


def test_image_revalidates_to_304(client):
    tag = client.get("/api/v1/maps/full/image").headers["etag"]

    response = client.get("/api/v1/maps/full/image", headers={"If-None-Match": tag})

    assert response.status_code == 304
    assert response.content == b""


def test_image_404_when_the_map_has_no_gridmap(client):
    assert client.get("/api/v1/maps/rawonly/image").status_code == 404


def test_image_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/image").status_code == 404


def test_image_404_when_the_gridmap_is_unreadable(client, maps_dir):
    (maps_dir / "full" / "gridmap.pgm").write_bytes(b"garbage")

    assert client.get("/api/v1/maps/full/image").status_code == 404


def test_image_etag_follows_content_not_mtime(client, maps_dir, make_pgm):
    """An edited gridmap keeps its dimensions, so it keeps its file size, and
    this filesystem hands out a coarse mtime — the tag has to be content-based
    or the editor would reload the pre-edit grid."""
    before = client.get("/api/v1/maps/full/image").headers["etag"]
    path = maps_dir / "full" / "gridmap.pgm"
    size_before = path.stat().st_size

    make_pgm(path, 6, 4, fill=0)
    after = client.get("/api/v1/maps/full/image")

    assert path.stat().st_size == size_before
    assert after.headers["etag"] != before
    assert after.status_code == 200


# --- PUT /api/v1/maps/{name}/grid -------------------------------------------


def _put_grid(client, name, body):
    return client.put(f"/api/v1/maps/{name}/grid", content=body, headers=_OCTET)


def _image_cells(client, name):
    response = client.get(f"/api/v1/maps/{name}/image")
    return cv2.imdecode(
        np.frombuffer(response.content, np.uint8), cv2.IMREAD_GRAYSCALE
    )


def test_save_grid_writes_the_cells(client, maps_dir):
    response = _put_grid(client, "full", b"\x00" * 24)

    assert response.status_code == 200
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes().startswith(b"P5\n6 4\n255\n")
    assert not _image_cells(client, "full").any()


def test_save_grid_reloads_the_active_map(client, map_gw):
    body = _put_grid(client, "full", b"\x00" * 24).json()

    assert body["active"] is True
    assert body["reloaded"] is True
    assert len(map_gw.calls) == 1

    # map_server resolves the yaml's relative image key against dirname() of the
    # string it was handed, unexpanded — so this must be absolute and ~-free.
    called = map_gw.calls[0]
    assert called.endswith("full/gridmap.yaml")
    assert called.startswith("/")
    assert "~" not in called


def test_save_grid_does_not_reload_an_inactive_map(
    client, map_gw, maps_dir, make_pgm, make_gridmap_yaml
):
    # Converted here rather than in the maps_dir fixture: a third gridmap there
    # would break the listing tests that assert exactly which maps have one.
    make_pgm(maps_dir / "rawonly" / "gridmap.pgm", 3, 2)
    make_gridmap_yaml(maps_dir / "rawonly" / "gridmap.yaml")

    body = _put_grid(client, "rawonly", b"\x00" * 6).json()

    assert body["active"] is False
    assert body["reloaded"] is False
    assert map_gw.calls == []


def test_save_grid_reports_a_failed_reload_without_failing_the_save(client, map_gw):
    """The bytes are on disk, so a 5xx would be a lie the operator acts on."""
    map_gw.result = (False, "map_server/load_map is not available")

    response = _put_grid(client, "full", b"\x00" * 24)

    assert response.status_code == 200
    body = response.json()
    assert body["active"] is True
    assert body["reloaded"] is False
    assert "map_server/load_map is not available" in body["message"]
    assert not _image_cells(client, "full").any()


def test_save_grid_rejects_a_wrong_length_body(client, maps_dir, map_gw):
    before = (maps_dir / "full" / "gridmap.pgm").read_bytes()

    response = _put_grid(client, "full", b"\x00" * 23)

    assert response.status_code == 400
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() == before
    assert map_gw.calls == []


def test_save_grid_refuses_while_a_conversion_is_running(
    client, conversion_svc, maps_dir
):
    """The one write path that used to lack the check rename/delete/activate make.

    The conversion thread os.replace()s the very files this route writes, so an
    edit saved mid-conversion vanished without a word, and the once-only raw
    snapshot could capture the half-finished grid as the original.
    """
    before = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    _mark_converting(conversion_svc, "full")

    response = _put_grid(client, "full", b"\x00" * 24)

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() == before
    assert not (maps_dir / "full" / "gridmap_raw.pgm").exists()


def test_save_grid_404_for_a_missing_map(client):
    assert _put_grid(client, "nosuchmap", b"\x00" * 24).status_code == 404


def test_save_grid_404_when_the_map_has_no_gridmap(client, maps_dir):
    assert _put_grid(client, "rawonly", b"\x00" * 24).status_code == 404
    assert not (maps_dir / "rawonly" / "gridmap.pgm").exists()


def test_save_grid_400_for_an_unsafe_name(client):
    assert _put_grid(client, "with%20space", b"\x00" * 24).status_code == 400


def test_save_grid_creates_the_raw_backup_once(client, maps_dir):
    pristine = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    raw = maps_dir / "full" / "gridmap_raw.pgm"

    _put_grid(client, "full", b"\x00" * 24)
    assert raw.read_bytes() == pristine

    _put_grid(client, "full", b"\xfe" * 24)
    assert raw.read_bytes() == pristine


def test_save_grid_etag_matches_the_image_etag(client):
    """The tag hashes the whole file, header included, on both sides."""
    body = _put_grid(client, "full", b"\x00" * 24)

    assert body.headers["etag"] == body.json()["etag"]
    assert client.get("/api/v1/maps/full/image").headers["etag"] == body.json()["etag"]


def test_save_grid_updates_the_thumbnail_without_an_eviction(client):
    """The write path deliberately does not touch the caches.

    _png_response re-reads and re-hashes the .pgm before consulting them, so a
    stale entry can never be served — this is the test that keeps that true.
    """
    before = client.get("/api/v1/maps/full/thumbnail")

    _put_grid(client, "full", b"\x00" * 24)
    after = client.get("/api/v1/maps/full/thumbnail")

    assert after.headers["etag"] != before.headers["etag"]
    assert after.content != before.content


def test_save_grid_accepts_a_missing_content_type(client, maps_dir):
    """A bare BufferSource fetch() sends no Content-Type; the save still lands.

    This test used to assert the opposite, on the premise that FastAPI would
    fall back to parsing the body as JSON without the header. The pinned
    FastAPI does no such thing: a ``bytes`` body parameter receives the raw
    payload whatever the Content-Type says, so the ``media_type`` on the Body
    is OpenAPI documentation, not enforcement. The length gate is what actually
    rejects a malformed body (test_save_grid_rejects_a_wrong_length_body), so
    tolerating the missing header loses nothing — and pinning tolerance keeps
    this from silently flipping again on the next FastAPI bump.
    """
    response = client.put("/api/v1/maps/full/grid", content=b"\x00" * 24)

    assert response.status_code == 200
    # The write really happened: the body is the 24 cells just sent.
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes()[-24:] == b"\x00" * 24


# --- /api/v1/maps/{name}/keepout --------------------------------------------


_KEEPOUT_HEADER = b"P5\n6 4\n255\n"


def _cell_centre(col, row, origin=(-6.94, -11.09), height=4, res=0.05):
    """World coordinates of the centre of cell (col, row), row 0 = top."""
    return {"x": origin[0] + (col + 0.5) * res, "y": origin[1] + (height - row - 0.5) * res}


def _rect_zone(zone_id=None, cols=(1, 3), rows=(1, 2)):
    """A rectangle over columns cols[0]..cols[1], rows rows[0]..rows[1] (inclusive)."""
    zone = {
        "points": [
            _cell_centre(cols[0], rows[1]),
            _cell_centre(cols[1], rows[1]),
            _cell_centre(cols[1], rows[0]),
            _cell_centre(cols[0], rows[0]),
        ]
    }
    if zone_id is not None:
        zone["id"] = zone_id
    return zone


def _put_keepout(client, name, zones):
    return client.put(f"/api/v1/maps/{name}/keepout", json={"zones": zones})


def _keepout_cells(maps_dir, name):
    body = (maps_dir / name / "keepout.pgm").read_bytes()
    assert body.startswith(_KEEPOUT_HEADER)
    return np.frombuffer(body[len(_KEEPOUT_HEADER):], np.uint8).reshape(4, 6)


def test_keepout_get_is_empty_for_a_map_never_drawn_on(client):
    body = client.get("/api/v1/maps/full/keepout").json()

    assert body == {"name": "full", "zones": [], "active": True}


def test_keepout_get_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/keepout").status_code == 404


def test_keepout_get_400_for_an_unsafe_name(client):
    assert client.get("/api/v1/maps/..%2Ffull/keepout").status_code in (400, 404)


def test_keepout_put_then_get_round_trips_in_order(client):
    zones = [_rect_zone("north"), _rect_zone("south", cols=(4, 5), rows=(3, 3))]

    saved = _put_keepout(client, "full", zones).json()
    fetched = client.get("/api/v1/maps/full/keepout").json()

    assert [zone["id"] for zone in saved["zones"]] == ["north", "south"]
    assert fetched["zones"] == saved["zones"]
    assert fetched["zones"][0]["points"] == zones[0]["points"]


def test_keepout_put_fills_in_missing_ids(client):
    body = _put_keepout(client, "full", [_rect_zone(), _rect_zone()]).json()

    ids = [zone["id"] for zone in body["zones"]]
    assert len(set(ids)) == 2
    assert all(len(zone_id) == 32 for zone_id in ids)  # uuid4().hex


def test_keepout_put_writes_the_three_files_and_paints_the_zone(client, maps_dir):
    response = _put_keepout(client, "full", [_rect_zone(cols=(1, 3), rows=(1, 2))])

    assert response.status_code == 200
    directory = maps_dir / "full"
    assert (directory / "keepout.yaml").read_text().startswith("image: keepout.pgm\n")
    assert json.loads((directory / "keepout.json").read_text())["version"] == 1
    cells = _keepout_cells(maps_dir, "full")
    expected = np.full((4, 6), 205, np.uint8)
    expected[1:3, 1:4] = 0
    np.testing.assert_array_equal(cells, expected)


def test_keepout_put_reloads_the_active_map(client, map_gw):
    body = _put_keepout(client, "full", [_rect_zone()]).json()

    assert body["active"] is True
    assert body["reloaded"] is True
    assert "reloaded the keepout filter" in body["message"]
    assert len(map_gw.keepout_calls) == 1
    called = map_gw.keepout_calls[0]
    assert called.endswith("full/keepout.yaml")
    assert called.startswith("/")
    assert "~" not in called
    # The gridmap's map_server is not asked to load a keepout mask.
    assert map_gw.calls == []


def test_keepout_put_does_not_reload_an_inactive_map(
    client, map_gw, maps_dir, make_pgm, make_gridmap_yaml
):
    make_pgm(maps_dir / "rawonly" / "gridmap.pgm", 3, 2)
    make_gridmap_yaml(maps_dir / "rawonly" / "gridmap.yaml")

    body = _put_keepout(client, "rawonly", [_rect_zone(cols=(0, 1), rows=(0, 1))]).json()

    assert body["active"] is False
    assert body["reloaded"] is False
    assert map_gw.keepout_calls == []
    assert (maps_dir / "rawonly" / "keepout.pgm").read_bytes().startswith(b"P5\n3 2\n255\n")


def test_keepout_put_reports_a_failed_reload_without_failing_the_save(
    client, map_gw, maps_dir
):
    """The zones are on disk, so a 5xx would be a lie the operator acts on."""
    map_gw.keepout_result = (False, "filter_mask_server/load_map is not available")

    response = _put_keepout(client, "full", [_rect_zone("z")])

    assert response.status_code == 200
    body = response.json()
    assert body["active"] is True
    assert body["reloaded"] is False
    assert "filter_mask_server/load_map is not available" in body["message"]
    assert client.get("/api/v1/maps/full/keepout").json()["zones"][0]["id"] == "z"


def test_keepout_put_empty_clears_to_an_all_unknown_mask(client, maps_dir, map_gw):
    _put_keepout(client, "full", [_rect_zone()])

    body = _put_keepout(client, "full", []).json()

    assert body["zones"] == []
    assert body["reloaded"] is True
    assert "Saved 0 forbidden zones" in body["message"]
    # The files stay -- deleting them would clear nothing in the running filter.
    assert (maps_dir / "full" / "keepout.yaml").is_file()
    assert (_keepout_cells(maps_dir, "full") == 205).all()
    assert len(map_gw.keepout_calls) == 2


@pytest.mark.parametrize(
    "zones,fragment",
    [
        ([{"points": [_cell_centre(0, 0), _cell_centre(1, 1)]}], "at least 3"),
        ([_rect_zone("dup"), _rect_zone("dup")], "more than once"),
        ([_rect_zone("")], "empty id"),
    ],
)
def test_keepout_put_400s_a_bad_zone_with_a_sentence(client, maps_dir, map_gw, zones, fragment):
    response = _put_keepout(client, "full", zones)

    assert response.status_code == 400
    assert fragment in response.json()["detail"]
    assert not (maps_dir / "full" / "keepout.json").exists()
    assert map_gw.keepout_calls == []


def test_keepout_put_400s_an_infinite_coordinate(client, maps_dir, map_gw):
    # Sent as raw JSON: 1e999 parses to inf, which no client library will
    # encode for us on purpose.
    response = client.put(
        "/api/v1/maps/full/keepout",
        content=(
            '{"zones": [{"points": '
            '[{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1e999, "y": 1}]}]}'
        ),
        headers={"Content-Type": "application/json"},
    )

    assert response.status_code == 400
    assert "finite" in response.json()["detail"]
    assert not (maps_dir / "full" / "keepout.json").exists()
    assert map_gw.keepout_calls == []


def test_keepout_put_refuses_while_a_conversion_is_running(client, conversion_svc, maps_dir):
    _mark_converting(conversion_svc, "full")

    response = _put_keepout(client, "full", [_rect_zone()])

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert not (maps_dir / "full" / "keepout.json").exists()


def test_keepout_put_404_when_the_map_has_no_gridmap(client, maps_dir):
    response = _put_keepout(client, "rawonly", [_rect_zone()])

    assert response.status_code == 404
    assert "grid/convert" in response.json()["detail"]
    assert not (maps_dir / "rawonly" / "keepout.pgm").exists()


def test_keepout_put_404_for_a_missing_map(client):
    assert _put_keepout(client, "nosuchmap", []).status_code == 404


# --- /api/v1/maps/{name}/pointcloud -----------------------------------------


def _unpack_cloud(payload):
    """Undo the wire format: [u32 count][f32 xyz * count]."""
    count = struct.unpack("<I", payload[:4])[0]
    xyz = np.frombuffer(payload[4:], dtype="<f4")
    return count, xyz.reshape(-1, 3)


def test_pointcloud_returns_the_packed_cloud(client):
    response = client.get("/api/v1/maps/full/pointcloud")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/octet-stream"

    count, points = _unpack_cloud(response.content)
    # The fixture's three points are >0.3 m apart, so none are voxel-merged.
    assert count == 3
    assert points.shape == (3, 3)


def test_pointcloud_payload_length_matches_the_count(client):
    """A short body would be read as garbage coordinates by the viewer."""
    payload = client.get("/api/v1/maps/full/pointcloud").content
    count = struct.unpack("<I", payload[:4])[0]

    assert len(payload) == 4 + count * 3 * 4


def test_pointcloud_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/pointcloud").status_code == 404


def test_pointcloud_404_when_the_map_has_no_pcd(client, maps_dir):
    (maps_dir / "full" / "map.pcd").unlink()

    assert client.get("/api/v1/maps/full/pointcloud").status_code == 404


def test_pointcloud_404_when_the_pcd_is_unreadable(client, maps_dir):
    """A torn .pcd must be a 404, not a traceback."""
    (maps_dir / "full" / "map.pcd").write_text("not a pcd at all\n")

    assert client.get("/api/v1/maps/full/pointcloud").status_code == 404


def test_pointcloud_is_recached_when_the_file_changes(client, maps_dir, make_pcd):
    first = client.get("/api/v1/maps/full/pointcloud").content

    make_pcd(
        maps_dir / "full" / "map.pcd",
        points=((0.0, 0.0, 0.0), (5.0, 5.0, 5.0)),
    )
    second = client.get("/api/v1/maps/full/pointcloud").content

    assert struct.unpack("<I", first[:4])[0] == 3
    assert struct.unpack("<I", second[:4])[0] == 2


# --- POST /api/v1/maps --------------------------------------------------------


def _post_map(client, payload):
    return client.post("/api/v1/maps", json=payload)


def test_create_map_saves_through_the_gateway(client, map_gw, catalog_repo):
    """The stub writes no map.pcd, so grid_pending honestly reports false."""
    response = _post_map(client, {"name": "newmap"})

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "newmap"
    assert body["has_pointcloud"] is True
    assert body["grid_pending"] is False
    # The gateway got the directory this router created, absolute.
    directory = catalog_repo.resolve_dir("newmap")
    assert map_gw.save_calls == [directory]
    assert os.path.isdir(directory)


def test_create_map_lists_afterwards_with_a_null_grid(client, maps_dir, make_pcd):
    _post_map(client, {"name": "newmap"})
    # Stand in for pgo: the stub gateway does not write files.
    make_pcd(maps_dir / "newmap" / "map.pcd")

    entry = _by_name(client.get("/api/v1/maps").json())["newmap"]

    assert entry["grid"] is None
    assert entry["has_pointcloud"] is True


def test_create_map_conflicts_with_an_existing_map(client, map_gw):
    response = _post_map(client, {"name": "full"})

    assert response.status_code == 409
    assert map_gw.save_calls == []


@pytest.mark.parametrize("name", ["../evil", "a/b", "", ".", "x" * 65])
def test_create_map_rejects_bad_names(client, map_gw, name):
    response = _post_map(client, {"name": name})

    # Length/emptiness die in the schema (422), separators in resolve_dir (400);
    # either way nothing reaches the gateway and no directory appears.
    assert response.status_code in (400, 422)
    assert map_gw.save_calls == []


def test_failed_save_unwinds_the_directory(client, map_gw, catalog_repo):
    map_gw.save_result = (False, "NO POSES!")

    response = _post_map(client, {"name": "newmap"})

    assert response.status_code == 502
    assert response.json()["detail"] == "NO POSES!"
    assert not os.path.exists(catalog_repo.resolve_dir("newmap"))


def test_create_map_does_not_reset_the_run(client, map_gw):
    # Saving and resetting are two deliberate acts. This is the test that keeps
    # anyone from "helpfully" folding the reset into the save, which would make
    # the common case (abandon a bad run without saving) impossible to express.
    _post_map(client, {"name": "newmap"})

    assert map_gw.reset_calls == []


def test_create_map_says_mapping_has_stopped(client):
    # A successful save ends the run on pgo's side (it goes idle and clears
    # /dev/shm itself); the sentence the console renders has to say so, or an
    # operator drives on into a map that is no longer being built.
    body = _post_map(client, {"name": "newmap"}).json()

    assert "Mapping has stopped" in body["message"]


def test_create_map_is_refused_while_idle(client, map_gw, catalog_repo, mapping_status_repo):
    # IDLE means "nothing banked" by definition under the state machine, so
    # this is a 409 with a code the console can act on ("press Start"), not
    # pgo's NO POSES! as a 502 that reads as a broken robot. Before any
    # directory is created.
    _latch(mapping_status_repo, MappingState.IDLE)

    response = _post_map(client, {"name": "newmap"})

    assert response.status_code == 409
    assert response.json()["code"] == "mapping_idle"
    assert map_gw.save_calls == []
    assert not os.path.exists(catalog_repo.resolve_dir("newmap"))


def test_create_map_is_refused_mid_transition(client, map_gw, mapping_status_repo):
    _latch(mapping_status_repo, MappingState.RESETTING)

    response = _post_map(client, {"name": "newmap"})

    assert response.status_code == 409
    assert response.json()["code"] == "mapping_busy"
    assert map_gw.save_calls == []


def test_create_map_asks_pgo_while_mapping_or_unknown(client, map_gw, mapping_status_repo):
    # Unknown (no sample) is the pre-status behaviour: pgo answers. MAPPING is
    # the normal case.
    assert _post_map(client, {"name": "one"}).status_code == 200
    _latch(mapping_status_repo, MappingState.MAPPING, key_poses=12)
    assert _post_map(client, {"name": "two"}).status_code == 200

    assert len(map_gw.save_calls) == 2


# --- POST /api/v1/mapping/reset -----------------------------------------------


def test_reset_mapping_calls_the_gateway(client, map_gw):
    response = client.post("/api/v1/mapping/reset")

    assert response.status_code == 200
    body = response.json()
    assert body["reset"] is True
    # The backend adds nothing: pgo's sentence is what the console renders.
    assert body["message"] == (
        "Map discarded. The new one starts building once the lidar has re-levelled."
    )
    # Always with the LIO front end — a graph-only reset would start the new map
    # wherever the old run had drifted to.
    assert map_gw.reset_calls == [True]


def test_reset_mapping_takes_no_body(client, map_gw):
    # Guards the console against a future required field: the button sends a
    # bodyless POST and must keep working.
    assert client.post("/api/v1/mapping/reset").status_code == 200
    assert map_gw.reset_calls == [True]


def test_reset_mapping_reports_the_gateway_refusal(client, map_gw):
    # The one an operator actually hits: pgo only exists in a mapping session.
    map_gw.reset_result = (
        False,
        "pgo/reset_mapping is not available — starting a new map needs the "
        "robot in MANUAL (mapping) mode.",
    )

    response = client.post("/api/v1/mapping/reset")

    assert response.status_code == 502
    assert "MANUAL" in response.json()["detail"]


def test_reset_mapping_does_not_touch_the_catalogue(client, map_gw, catalog_repo):
    # The route is about the *run*, not the map library — which is why it sits
    # under /api/v1/mapping/ and why a client has no cache to invalidate after
    # it. If this ever fails, the route has grown a disk side effect.
    before = {entry.name for entry in catalog_repo.list_maps()}

    assert client.post("/api/v1/mapping/reset").status_code == 200

    assert {entry.name for entry in catalog_repo.list_maps()} == before
    assert map_gw.save_calls == []


def test_reset_mapping_is_refused_while_idle(client, map_gw, mapping_status_repo):
    # Nothing to discard: pgo would refuse too, but from the latched status
    # the console gets a code it can act on rather than a 502.
    _latch(mapping_status_repo, MappingState.IDLE)

    response = client.post("/api/v1/mapping/reset")

    assert response.status_code == 409
    assert response.json()["code"] == "mapping_idle"
    assert map_gw.reset_calls == []


# --- POST /api/v1/mapping/start -----------------------------------------------


def test_start_mapping_calls_the_gateway(client, map_gw):
    response = client.post("/api/v1/mapping/start")

    assert response.status_code == 200
    body = response.json()
    assert body["started"] is True
    # pgo's sentence carries the stillness warning; the backend adds nothing.
    assert body["message"] == (
        "Mapping started. Keep the robot still until the lidar has re-levelled."
    )
    # Always with the LIO front end: the map's origin is the odometry origin.
    assert map_gw.start_calls == [True]


def test_start_mapping_takes_no_body(client, map_gw):
    assert client.post("/api/v1/mapping/start").status_code == 200
    assert map_gw.start_calls == [True]


def test_start_mapping_reports_the_gateway_refusal(client, map_gw):
    map_gw.start_result = (
        False,
        "pgo/start_mapping is not available — starting mapping needs the robot "
        "in MANUAL (mapping) mode.",
    )

    response = client.post("/api/v1/mapping/start")

    assert response.status_code == 502
    assert "MANUAL" in response.json()["detail"]


def test_start_mapping_is_refused_while_already_mapping(client, map_gw, mapping_status_repo):
    # pgo refuses this too; answering it from the latched status is what lets
    # the console disable the button instead of showing a 502.
    _latch(mapping_status_repo, MappingState.MAPPING, key_poses=3)

    response = client.post("/api/v1/mapping/start")

    assert response.status_code == 409
    assert response.json()["code"] == "mapping_running"
    assert map_gw.start_calls == []


def test_start_mapping_is_refused_mid_transition(client, map_gw, mapping_status_repo):
    _latch(mapping_status_repo, MappingState.RESETTING)

    response = client.post("/api/v1/mapping/start")

    assert response.status_code == 409
    assert response.json()["code"] == "mapping_busy"
    assert map_gw.start_calls == []


def test_start_mapping_asks_pgo_when_the_state_is_unknown_or_idle(
    client, map_gw, mapping_status_repo
):
    assert client.post("/api/v1/mapping/start").status_code == 200
    _latch(mapping_status_repo, MappingState.IDLE)
    assert client.post("/api/v1/mapping/start").status_code == 200

    assert map_gw.start_calls == [True, True]


def test_start_mapping_asks_pgo_once_the_latched_state_is_stale(
    client, map_gw, mapping_status_repo, clock
):
    # A MAPPING sample from a pgo that has since gone quiet (the session was
    # torn down) must not refuse forever: past the TTL the router lets pgo
    # answer, and pgo being absent is then the honest 502.
    _latch(mapping_status_repo, MappingState.MAPPING)
    clock.now += MAPPING_STATUS_TTL_S + 1.0

    assert client.post("/api/v1/mapping/start").status_code == 200
    assert map_gw.start_calls == [True]


def test_start_mapping_touches_nothing_else(client, map_gw, catalog_repo):
    before = {entry.name for entry in catalog_repo.list_maps()}

    assert client.post("/api/v1/mapping/start").status_code == 200

    assert {entry.name for entry in catalog_repo.list_maps()} == before
    assert map_gw.save_calls == []
    assert map_gw.reset_calls == []


# --- GET /api/v1/mapping ------------------------------------------------------


def test_mapping_status_is_unknown_before_pgo_has_said_anything(client):
    # The answer on every navigating robot: pgo exists only in a mapping
    # session. Not an error.
    response = client.get("/api/v1/mapping")

    assert response.status_code == 200
    assert response.json() == {"state": "unknown", "key_poses": 0, "loop_closures": 0}


def test_mapping_status_reports_the_latched_sample(client, mapping_status_repo):
    _latch(mapping_status_repo, MappingState.MAPPING, key_poses=42, loop_closures=3)

    assert client.get("/api/v1/mapping").json() == {
        "state": "mapping",
        "key_poses": 42,
        "loop_closures": 3,
    }


def test_mapping_status_goes_unknown_once_the_sample_is_stale(
    client, mapping_status_repo, clock
):
    _latch(mapping_status_repo, MappingState.IDLE)
    assert client.get("/api/v1/mapping").json()["state"] == "idle"

    clock.now += MAPPING_STATUS_TTL_S + 1.0

    assert client.get("/api/v1/mapping").json()["state"] == "unknown"


# --- PATCH /api/v1/maps/{name} ------------------------------------------------
#
# The client fixture pins the active map to 'full', so 'rawonly' is the map a
# rename is allowed to touch.


def _rename(client, name, new_name):
    return client.patch(f"/api/v1/maps/{name}", json={"name": new_name})


def test_rename_moves_the_directory_and_lists_under_the_new_name(
    client, maps_dir, map_repo, task_template_repo
):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "home", "type": "HOME", "x": 1.0, "y": 0.0, "theta": 0.0},
    ])
    task_template_repo.create_task_template(
        name="patrol", description="", map_name="rawonly", steps=[]
    )

    response = _rename(client, "rawonly", "hall")

    assert response.status_code == 200
    body = response.json()
    assert body["old_name"] == "rawonly"
    assert body["name"] == "hall"
    assert body["vertices_moved"] == 2
    assert body["templates_moved"] == 1
    assert "Renamed 'rawonly' to 'hall'" in body["message"]
    assert "Schedules already registered" in body["message"]

    assert not (maps_dir / "rawonly").exists()
    assert (maps_dir / "hall" / "map.pcd").is_file()
    listing = _by_name(client.get("/api/v1/maps").json())
    assert "rawonly" not in listing
    assert listing["hall"]["vertex_count"] == 2
    assert listing["hall"]["active"] is False
    assert map_repo.list_vertices(map="rawonly") == []
    assert len(map_repo.list_vertices(map="hall")) == 2


def test_rename_message_without_templates_has_no_schedule_caveat(client):
    body = _rename(client, "rawonly", "hall").json()

    assert body["templates_moved"] == 0
    assert "Schedules" not in body["message"]


def test_rename_refuses_the_active_map(client, maps_dir):
    response = _rename(client, "full", "hall")

    assert response.status_code == 409
    assert response.json()["code"] == "map_active"
    assert (maps_dir / "full" / "gridmap.pgm").is_file()
    assert not (maps_dir / "hall").exists()


def test_rename_refuses_while_a_conversion_is_running(client, conversion_svc, maps_dir):
    _mark_converting(conversion_svc, "rawonly")

    response = _rename(client, "rawonly", "hall")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert (maps_dir / "rawonly").is_dir()


def test_rename_refuses_a_taken_name(client, maps_dir):
    response = _rename(client, "rawonly", "full")

    assert response.status_code == 409
    assert response.json()["code"] == "name_taken"
    assert (maps_dir / "rawonly").is_dir()
    assert (maps_dir / "full" / "gridmap.pgm").is_file()


def test_rename_to_the_same_name_is_a_400(client, maps_dir):
    response = _rename(client, "rawonly", "rawonly")

    assert response.status_code == 400
    assert (maps_dir / "rawonly").is_dir()


@pytest.mark.parametrize("new_name", ["../evil", "a/b", "", ".", "x" * 65])
def test_rename_rejects_bad_new_names(client, maps_dir, new_name):
    response = _rename(client, "rawonly", new_name)

    # Length/emptiness die in the schema (422), separators in resolve_dir (400).
    assert response.status_code in (400, 422)
    assert (maps_dir / "rawonly").is_dir()


def test_rename_of_a_missing_map_is_a_404(client):
    assert _rename(client, "nope", "hall").status_code == 404


def test_rename_moves_the_directory_back_when_the_database_fails(
    client, maps_dir, map_repo, monkeypatch
):
    """The two stores cannot share a transaction, so the filesystem is undone."""
    def _boom(old_map, new_map, session=None):
        raise RuntimeError("database is away")

    monkeypatch.setattr(map_repo, "move_vertices", _boom)

    response = _rename(client, "rawonly", "hall")

    assert response.status_code == 502
    assert "left under its old name" in response.json()["detail"]
    assert (maps_dir / "rawonly" / "map.pcd").is_file()
    assert not (maps_dir / "hall").exists()


def test_rename_leaves_the_vertices_under_the_old_name_when_the_second_update_fails(
    client, maps_dir, map_repo, task_template_repo, monkeypatch
):
    """The two DB re-keys are one transaction.

    move_vertices used to commit on its own before rebind_map ran, so a failure
    in rebind_map moved the directory back while the vertices stayed keyed to
    the new name -- waypoints nobody could reach under either name.
    """
    map_repo.create_vertices(
        "rawonly", [dict(name="dock", type="dock", x=0.0, y=0.0, theta=0.0)]
    )

    def _boom(old_name, new_name, session=None):
        raise RuntimeError("templates table is away")

    monkeypatch.setattr(task_template_repo, "rebind_map", _boom)

    response = _rename(client, "rawonly", "hall")

    assert response.status_code == 502
    assert (maps_dir / "rawonly").is_dir() and not (maps_dir / "hall").exists()
    assert len(map_repo.list_vertices(map="rawonly")) == 1
    assert map_repo.list_vertices(map="hall") == []


def test_rename_drops_the_cached_renderings_of_the_old_name(client, maps_dir, make_pcd):
    """Warm the cloud cache under the old name, rename, and check nothing is left keyed on it."""
    # 'full' is active; give 'rawonly' a proper cloud and read it so it caches.
    make_pcd(maps_dir / "rawonly" / "map.pcd")
    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 200

    assert _rename(client, "rawonly", "hall").status_code == 200

    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 404
    assert client.get("/api/v1/maps/hall/pointcloud").status_code == 200


# --- DELETE /api/v1/maps/{name} ----------------------------------------------
#
# Same fixture geometry as the rename block: 'full' is the active map, so
# 'rawonly' is the one a delete is allowed to touch.


def _delete(client, name):
    return client.delete(f"/api/v1/maps/{name}")


def test_delete_removes_the_directory_and_its_vertices(client, maps_dir, map_repo):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "home", "type": "HOME", "x": 1.0, "y": 0.0, "theta": 0.0},
    ])
    # A vertex on another map must survive: the DELETE is filtered by map.
    map_repo.create_vertices(map="full", vertices=[
        {"name": "charger", "type": "CHARGER", "x": 2.0, "y": 0.0, "theta": 0.0},
    ])

    response = _delete(client, "rawonly")

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "rawonly"
    assert body["vertices_deleted"] == 2
    assert "Deleted 'rawonly' and 2 vertices" in body["message"]
    assert "Schedules already registered" in body["message"]

    assert not (maps_dir / "rawonly").exists()
    assert "rawonly" not in _by_name(client.get("/api/v1/maps").json())
    assert map_repo.list_vertices(map="rawonly") == []
    assert len(map_repo.list_vertices(map="full")) == 1


def test_delete_of_a_missing_map_is_a_404(client):
    assert _delete(client, "nope").status_code == 404


def test_delete_refuses_the_active_map(client, maps_dir):
    response = _delete(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "map_active"
    assert (maps_dir / "full" / "gridmap.pgm").is_file()


def test_delete_refuses_while_a_conversion_is_running(client, conversion_svc, maps_dir):
    _mark_converting(conversion_svc, "rawonly")

    response = _delete(client, "rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert (maps_dir / "rawonly" / "map.pcd").is_file()


def test_delete_refuses_while_a_task_template_is_bound(
    client, maps_dir, map_repo, task_template_repo
):
    """The one refusal that asks for work rather than a restart."""
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    task_template_repo.create_task_template(
        name="patrol", description="", map_name="rawonly", steps=[]
    )

    response = _delete(client, "rawonly")

    assert response.status_code == 409
    body = response.json()
    assert body["code"] == "template_bound"
    assert "'patrol'" in body["detail"]
    # Nothing was touched -- the refusal comes before either store is written.
    assert (maps_dir / "rawonly" / "map.pcd").is_file()
    assert len(map_repo.list_vertices(map="rawonly")) == 1


def test_delete_ignores_map_independent_templates(client, maps_dir, task_template_repo):
    """map_name IS NULL is a run-anywhere template; no map going away concerns it."""
    task_template_repo.create_task_template(
        name="stand up", description="", map_name=None, steps=[]
    )

    assert _delete(client, "rawonly").status_code == 200
    assert not (maps_dir / "rawonly").exists()


@pytest.mark.parametrize("name", ["../evil", "a/b", ".", "x" * 65])
def test_delete_rejects_bad_names(client, maps_dir, name):
    response = _delete(client, name)

    # Separators never reach the route (FastAPI does not match the path), the
    # rest die in resolve_dir or as a missing map. Nothing is ever a 200.
    assert response.status_code in (400, 404, 405, 422)
    assert (maps_dir / "rawonly").is_dir()


def test_delete_leaves_the_directory_when_the_database_fails(
    client, maps_dir, map_repo, monkeypatch
):
    """Rows first, rmtree last: a failed DELETE must not cost the map."""
    def _boom(map):
        raise RuntimeError("database is away")

    monkeypatch.setattr(map_repo, "delete_vertices", _boom)

    response = _delete(client, "rawonly")

    assert response.status_code == 502
    assert (maps_dir / "rawonly" / "map.pcd").is_file()


def test_delete_drops_the_cached_renderings(client, maps_dir, make_pcd):
    """Warm the cloud cache, delete, and check the entry does not outlive the map."""
    make_pcd(maps_dir / "rawonly" / "map.pcd")
    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 200

    assert _delete(client, "rawonly").status_code == 200

    # The caches are closure-local to init_map_router, so this is the only way
    # to observe them: a stale entry would answer 200 for a map that is gone.
    assert client.get("/api/v1/maps/rawonly/pointcloud").status_code == 404


# --- the background gridmap conversion ---------------------------------------
#
# The service is driven directly rather than through POST /api/v1/maps: the
# route answers as soon as the pcd is on disk and the conversion runs on a
# daemon thread, so going through the client would mean asserting against a
# race.


@pytest.fixture
def conversion_threads(monkeypatch):
    """Every Thread the code under test starts, plus anything that escaped one.

    Looking the thread up in ``threading.enumerate()`` by name is the obvious
    alternative and it is racy in both directions: a conversion that finishes
    before the lookup is already gone from the list, so "not found" cannot tell
    "done" from "never started".

    The ``excepthook`` half is what makes the failure cases mean anything. Every
    assertion there is about a file *not* appearing, and a thread that died on an
    unhandled exception satisfies that just as well as one that logged and
    returned — which is exactly how an ImportError raised outside the handler's
    try block passed as a green test once already.
    """
    class _Threads(list):
        """A list with room for the escaped-exception log."""

        escaped: list

    started = _Threads()
    escaped = []
    real_thread = threading.Thread

    def _record(*args, **kwargs):
        thread = real_thread(*args, **kwargs)
        started.append(thread)
        return thread

    monkeypatch.setattr(threading, "Thread", _record)
    monkeypatch.setattr(threading, "excepthook", lambda args: escaped.append(args))
    started.escaped = escaped
    return started


def _join(threads, timeout=10.0):
    """Join every recorded thread and assert none of them died on an exception."""
    for thread in threads:
        thread.join(timeout)
        assert not thread.is_alive(), f"{thread.name} did not finish"
    assert not threads.escaped, (
        "an exception escaped the conversion thread: "
        + ", ".join(f"{a.exc_type.__name__}: {a.exc_value}" for a in threads.escaped)
    )


@pytest.fixture
def fake_traversable(monkeypatch):
    """Stand in for helpers.traversable, which needs open3d.

    The route imports it *inside* the thread body, so injecting a module into
    ``sys.modules`` is enough — and that indirection is itself worth pinning: a
    module-level import would pull open3d into every backend start.
    """
    module = types.ModuleType("syncai_backend.helpers.traversable")
    module.calls = []
    module.raises = None

    def build_traversable_cloud(logger, pcd_path, *, segment, repair, debug_dir):
        module.calls.append(
            dict(pcd_path=pcd_path, segment=segment, repair=repair, debug_dir=debug_dir)
        )
        if module.raises is not None:
            raise module.raises
        # A 2 m x 2 m sheet at 5 cm, i.e. a plausible repaired floor.
        axis = np.arange(0.0, 2.0, 0.05)
        xx, yy = np.meshgrid(axis, axis)
        return np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])

    module.build_traversable_cloud = build_traversable_cloud
    monkeypatch.setitem(sys.modules, "syncai_backend.helpers.traversable", module)
    return module


def _sheet(size, step=0.5, z=0.0):
    """A flat square of points ``size`` metres on a side, as a list of xyz."""
    axis = np.arange(0.0, size, step)
    xx, yy = np.meshgrid(axis, axis)
    return [(float(x), float(y), z) for x, y in zip(xx.ravel(), yy.ravel())]


@pytest.fixture
def saved_map(maps_dir, make_pcd):
    """A map directory with a large-site map.pcd, as save_maps would leave it.

    Sized like a warehouse (60 m x 60 m of bbox) because the tests that use it
    exercise the *traversability* path — which since 2026-09 runs only on
    request, never by size, so these tests reach it through
    ``recipe_request="traversability"`` / the re-convert endpoint rather than by
    making the fixture big enough to trip a threshold that no longer exists.
    """
    directory = maps_dir / "newmap"
    os.makedirs(directory, exist_ok=True)
    # A Path, not a str: the make_pcd factory writes through Path.write_text.
    make_pcd(directory / "map.pcd", points=_sheet(60.0, step=2.0))
    return str(directory)


@pytest.fixture
def small_saved_map(maps_dir, make_pcd):
    """A map directory whose footprint puts it on the z-band side of the split.

    Carries a floor sheet at a **non-zero** z plus a block standing on it,
    because the z-band branch is the one under test here and it needs both: a
    floor to call free and something in the obstacle band to call occupied. The
    floor at -0.4 rather than 0 is what makes the recentring assertion mean
    something — held absolute, the shipped bands would land somewhere else.

    A block with real footprint rather than a single column, because
    ``despeckle_min_size`` is 12 and a column puts all its points in one cell:
    the obstacle survives the projection and is then deleted as speckle, which
    reads in the log as a conversion that simply found no obstacles.
    """
    directory = maps_dir / "smallmap"
    os.makedirs(directory, exist_ok=True)
    floor_z = -0.4
    points = _sheet(20.0, step=0.2, z=floor_z)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (5.0 + float(bx), 5.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(directory / "map.pcd", points=points)
    return str(directory)


def test_conversion_writes_a_gridmap_from_the_traversable_cloud(
    conversion_svc, saved_map, fake_traversable, conversion_threads
):
    started = conversion_svc.start(
        "newmap", saved_map, recipe_request="traversability"
    )
    _join(conversion_threads)

    assert started is True
    assert fake_traversable.calls[0]["pcd_path"] == os.path.join(saved_map, "map.pcd")
    # Both recipes are empty dicts: the pipeline runs at the helper's tuned
    # defaults, and passing {} is what says so rather than silently omitting it.
    assert fake_traversable.calls[0]["segment"] == {}
    assert fake_traversable.calls[0]["repair"] == {}
    # debug_dir off by default: the intermediates would inflate the size the
    # catalogue reports for every map.
    assert fake_traversable.calls[0]["debug_dir"] is None
    assert os.path.isfile(os.path.join(saved_map, "gridmap.pgm"))
    grid = yaml.safe_load(open(os.path.join(saved_map, "gridmap.yaml")))
    assert grid["image"] == "gridmap.pgm"


def test_conversion_passes_a_debug_dir_when_one_is_configured(
    logger, saved_map, fake_traversable, conversion_threads
):
    """The process-wide override is a constructor argument, not a constant.

    Set where the service is built, so turning the intermediates on for every
    conversion is visible in the wiring rather than done by reassigning a module
    attribute from wherever.
    """
    conversion_svc = GridmapConversionService(
        logger=logger, debug_subdir="traversable_debug"
    )

    conversion_svc.start("newmap", saved_map, recipe_request="traversability")
    _join(conversion_threads)

    assert fake_traversable.calls[0]["debug_dir"] == os.path.join(
        saved_map, "traversable_debug"
    )


def test_conversion_passes_a_debug_dir_when_the_request_asks(
    conversion_svc, saved_map, fake_traversable, conversion_threads
):
    """debug=True is the per-request tuning interface — the service is built
    without an override so ordinary conversions never inflate the catalogue's
    sizes."""
    conversion_svc.start(
        "newmap", saved_map, recipe_request="traversability", debug=True
    )
    _join(conversion_threads)

    assert fake_traversable.calls[0]["debug_dir"] == os.path.join(
        saved_map, "traversable_debug"
    )


def test_conversion_reports_a_failed_segmentation_instead_of_dying(
    conversion_svc, saved_map, fake_traversable, conversion_threads
):
    """A site whose intensity window selects no floor raises ValueError. The map
    ends up without a grid — the route has already answered 200 by then."""
    fake_traversable.raises = ValueError("intensity/normal gate selected no ground points")

    assert (
        conversion_svc.start("newmap", saved_map, recipe_request="traversability")
        is True
    )
    _join(conversion_threads)

    assert not os.path.exists(os.path.join(saved_map, "gridmap.pgm"))


# --- what a conversion records about itself -----------------------------------
#
# The sidecar is the only thing a conversion leaves that outlives its thread and
# its process, so it is also the only place an outcome can be read back from.
# These tests pin the three states it writes; the catalogue tests above pin how
# they are reported.


def test_a_conversion_records_itself_before_doing_any_work(
    conversion_svc, small_saved_map, conversion_threads, monkeypatch
):
    """The `converting` record has to land before the pipeline runs, not after.

    It is what an abandoned conversion is detected by, and a conversion is
    abandoned precisely when it never reaches its own ending — so a record
    written on the way out would be missing in the one case it exists for.
    Blocking inside measure_cloud is what makes "before" observable at all; the
    real thing is over in tens of seconds and nothing can be asserted mid-run.
    """
    entered = threading.Event()
    release = threading.Event()
    real_measure = conversion_module.measure_cloud

    def _blocking(bound, pcd_path):
        entered.set()
        assert release.wait(10.0), "the test never released the conversion"
        return real_measure(bound, pcd_path)

    monkeypatch.setattr(conversion_module, "measure_cloud", _blocking)

    conversion_svc.start("smallmap", small_saved_map)
    assert entered.wait(10.0), "the conversion thread never reached measure_cloud"

    mid_run = _sidecar(small_saved_map)
    assert mid_run["status"] == "converting"
    assert mid_run["recipe"] == "z-band"
    assert mid_run["started_at"].endswith("Z")
    # No ending recorded yet, which is exactly what `interrupted` keys off.
    assert "finished_at" not in mid_run

    release.set()
    _join(conversion_threads)

    assert _sidecar(small_saved_map)["status"] == "ok"


def test_a_successful_conversion_records_ok_and_keeps_the_diagnostics(
    conversion_svc, small_saved_map, conversion_threads
):
    conversion_svc.start("smallmap", small_saved_map)
    _join(conversion_threads)

    side = _sidecar(small_saved_map)
    assert side["status"] == "ok"
    assert side["started_at"].endswith("Z")
    assert side["finished_at"] >= side["started_at"]
    assert "error" not in side
    # The status is additive: everything the sidecar carried before it still has
    # to be there, because the recipe record is what tells two maps converted by
    # different recipes apart.
    assert side["recipe"] == "z-band"
    assert side["params"]["floor_z"] == pytest.approx(-0.4, abs=0.1)


def test_a_failed_conversion_records_the_reason(
    conversion_svc, saved_map, fake_traversable, conversion_threads
):
    """The same sentence the log line carries, put where a client can read it."""
    fake_traversable.raises = ValueError("intensity/normal gate selected no ground points")

    conversion_svc.start("newmap", saved_map, recipe_request="traversability")
    _join(conversion_threads)

    side = _sidecar(saved_map)
    assert side["status"] == "failed"
    assert "selected no ground points" in side["error"]
    assert side["recipe"] == "traversability"
    assert side["finished_at"].endswith("Z")
    assert not os.path.exists(os.path.join(saved_map, "gridmap.pgm"))


def test_a_missing_open3d_is_recorded_as_a_failure_naming_the_fix(
    conversion_svc, saved_map, conversion_threads, monkeypatch
):
    """An environment fault, not a bad cloud: re-converting this map will fail
    identically until the container is fixed, so the operator has to be told
    which of the two it is rather than being invited to retry."""
    real_import = builtins.__import__

    def _no_open3d(name, *args, **kwargs):
        if name == "syncai_backend.helpers.traversable":
            raise ImportError("No module named 'open3d'")
        return real_import(name, *args, **kwargs)

    monkeypatch.delitem(sys.modules, "syncai_backend.helpers.traversable", raising=False)
    monkeypatch.setattr(builtins, "__import__", _no_open3d)

    conversion_svc.start("newmap", saved_map, recipe_request="traversability")
    _join(conversion_threads)

    side = _sidecar(saved_map)
    assert side["status"] == "failed"
    assert "open3d" in side["error"]
    assert "pip3 install" in side["hint"]


def test_a_failed_conversion_releases_the_slot(
    conversion_svc, saved_map, fake_traversable, conversion_threads
):
    """The registry entry must not outlive the thread, or the map is stuck
    unconvertible until a backend restart."""
    fake_traversable.raises = ValueError("no ground")
    conversion_svc.start("newmap", saved_map, recipe_request="traversability")
    _join(conversion_threads)

    assert not conversion_svc.is_converting("newmap")
    # And a second attempt is accepted rather than 409'd.
    fake_traversable.raises = None
    assert (
        conversion_svc.start("newmap", saved_map, recipe_request="traversability")
        is True
    )
    _join(conversion_threads)
    assert os.path.isfile(os.path.join(saved_map, "gridmap.pgm"))


def test_a_running_conversion_conflicts(conversion_svc, saved_map):
    """Two threads writing the same gridmap.pgm would interleave outputs."""
    from syncai_backend.exceptions import ConflictError

    _mark_converting(conversion_svc, "newmap")

    with pytest.raises(ConflictError) as exc:
        conversion_svc.start("newmap", saved_map)
    assert exc.value.code == "conversion_running"


def test_conversion_survives_open3d_being_absent(
    conversion_svc, saved_map, conversion_threads, monkeypatch
):
    """ImportError is caught on its own: uncaught it would kill the thread and
    land as a bare traceback with nothing naming the map being saved."""
    real_import = builtins.__import__

    def _no_open3d(name, *args, **kwargs):
        if name == "syncai_backend.helpers.traversable":
            raise ImportError("No module named 'open3d'")
        return real_import(name, *args, **kwargs)

    monkeypatch.delitem(sys.modules, "syncai_backend.helpers.traversable", raising=False)
    monkeypatch.setattr(builtins, "__import__", _no_open3d)

    assert (
        conversion_svc.start("newmap", saved_map, recipe_request="traversability")
        is True
    )
    _join(conversion_threads)

    assert not os.path.exists(os.path.join(saved_map, "gridmap.pgm"))


def _sidecar(directory):
    path = os.path.join(directory, conversion_module.GRIDMAP_RECIPE_SIDECAR)
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


@pytest.fixture
def glassy_saved_map(maps_dir, make_pcd):
    """The conference failure in miniature: a huge bbox over a small floor.

    A dense 20 m floor sheet at -0.4 with a block on it (the real hall), plus a
    handful of points 4 m up and 60 m out — out-of-hall structure seen through
    glass. The bbox comes out ~3600 m² while the floor stays ~400 m², which is
    exactly the shape that used to trip the removed footprint threshold into the
    traversability recipe and produce the 59%-occupied blob.
    """
    directory = maps_dir / "glassy"
    os.makedirs(directory, exist_ok=True)
    floor_z = -0.4
    points = _sheet(20.0, step=0.2, z=floor_z)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (5.0 + float(bx), 5.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    points += [(60.0, 60.0, 4.0), (58.0, 61.0, 4.5), (61.0, 58.0, 5.0)]
    make_pcd(directory / "map.pcd", points=points)
    return str(directory)


def test_the_default_recipe_is_z_band_whatever_the_footprint(
    conversion_svc,
    glassy_saved_map,
    fake_traversable,
    conversion_threads,
):
    """The conference regression: a glass-inflated bbox must not change the
    recipe, because no recipe is picked by size any more.

    The empty ``calls`` list is the assertion that matters twice over: the
    traversability pipeline did not run, and — since the open3d import sits
    inside that branch — was never even imported.
    """
    conversion_svc.start("glassy", glassy_saved_map)
    _join(conversion_threads)

    assert fake_traversable.calls == []
    side = _sidecar(glassy_saved_map)
    assert side["recipe"] == "z-band"
    # Both diagnostics are recorded, and their divergence is the fingerprint of
    # a glass-inflated cloud: a big bbox over little floor.
    assert side["footprint_m2"] > 3000.0
    assert side["floor_area_m2"] < 600.0
    assert "threshold_m2" not in side


def test_the_z_band_sidecar_carries_both_area_diagnostics(
    conversion_svc,
    small_saved_map,
    fake_traversable,
    conversion_threads,
):
    conversion_svc.start("smallmap", small_saved_map)
    _join(conversion_threads)

    assert fake_traversable.calls == []
    side = _sidecar(small_saved_map)
    assert side["recipe"] == "z-band"
    # A 20 m sheet at 0.2 m spacing: bbox and covered floor agree to within the
    # 0.5 m measuring cell's boundary over-count.
    assert side["footprint_m2"] == pytest.approx(400.0, rel=0.15)
    assert side["floor_area_m2"] == pytest.approx(400.0, rel=0.15)
    assert "recipe_override" not in side
    # No poses.txt in the fixture, so the connectivity filter must not claim to
    # have run.
    assert "pose_filter" not in side["params"]


def test_a_requested_traversability_conversion_records_the_override(
    conversion_svc,
    saved_map,
    fake_traversable,
    conversion_threads,
):
    override = {"requested": "traversability", "picked_by": "test", "reason": None}
    conversion_svc.start(
        "newmap",
        saved_map,
        recipe_request="traversability",
        override=override,
    )
    _join(conversion_threads)

    assert len(fake_traversable.calls) == 1
    side = _sidecar(saved_map)
    assert side["recipe"] == "traversability"
    assert side["recipe_override"] == override


def test_grid_overrides_reach_the_traversable_projection_and_the_sidecar(
    conversion_svc,
    saved_map,
    fake_traversable,
    conversion_threads,
):
    """gap_fill_size is the acknowledged per-site knob (dp1f's bottom aisle);
    an override must land in the conversion and be readable off the sidecar."""
    conversion_svc.start(
        "newmap",
        saved_map,
        recipe_request="traversability",
        grid_overrides={"gap_fill_size": 1.0},
    )
    _join(conversion_threads)

    side = _sidecar(saved_map)
    assert side["params"]["grid"] == {"gap_fill_size": 1.0}


def test_the_z_band_recipe_produces_a_trinary_map(
    conversion_svc,
    small_saved_map,
    fake_traversable,
    conversion_threads,
):
    """The reason the split exists: unknown survives, and unknown is recoverable.

    A traversability output has no unknown cells at all — everything the cloud
    does not cover is wall, permanently, since costmap_layer.cpp:90 can lower a
    NO_INFORMATION master cell but never a LETHAL one.
    """
    conversion_svc.start("smallmap", small_saved_map)
    _join(conversion_threads)

    grid = cv2.imread(os.path.join(small_saved_map, "gridmap.pgm"), cv2.IMREAD_UNCHANGED)
    present = set(np.unique(grid).tolist())
    assert 205 in present, "no unknown cells — this is not a trinary map"
    assert 254 in present, "no free cells — the floor band missed the floor"
    assert 0 in present, "no occupied cells — the obstacle band missed the column"


def test_the_z_band_bands_are_recentred_on_the_measured_floor(
    conversion_svc,
    small_saved_map,
    fake_traversable,
    conversion_threads,
):
    """Absolute bands are a per-site guess; z=0 is only the lidar mount height.

    The fixture's floor is at -0.4, so every band must come back shifted by
    roughly that much from the offsets — not at the constants the fleet's older
    maps were built with. No poses.txt in this fixture, so this is the
    ``global`` fallback of the default ``local`` reference, and the sidecar
    has to say so.
    """
    conversion_svc.start("smallmap", small_saved_map)
    _join(conversion_threads)

    params = _sidecar(small_saved_map)["params"]
    assert params["floor_reference"] == "global"
    assert "poses.txt" in params["floor_reference_fallback"]
    floor_z = params["floor_z"]
    assert floor_z == pytest.approx(-0.4, abs=0.1)
    for key, offset in conversion_module.GRIDMAP_BANDS_ABOVE_FLOOR.items():
        assert params[key] == pytest.approx(offset + floor_z, abs=0.01)
    # The floor band actually brackets the fixture's floor, which is the whole
    # point of measuring it rather than trusting the constant.
    assert params["floor_zmin"] < -0.4 < params["floor_zmax"]


# --- the local floor reference through the conversion --------------------------


@pytest.fixture
def drifting_saved_map(maps_dir, make_pcd):
    """0917_TP1F_test1 in miniature: a floor that rises 0.8 m along 40 m of x,
    the way a LIO map drifts across a large venue, with keyframes riding it 0.5 m
    up and a block near the low end so the obstacle band is not empty."""
    directory = maps_dir / "drifting"
    os.makedirs(directory)
    xs = np.arange(0.0, 40.0, 0.2)
    ys = np.arange(0.0, 6.0, 0.2)
    xx, yy = np.meshgrid(xs, ys)
    points = [(float(x), float(y), 0.02 * float(x)) for x, y in zip(xx.ravel(), yy.ravel())]
    # Sampled at 2 cm, not the grid's 5 cm: points spaced exactly one cell apart
    # land on cell boundaries, float32 rounding splits them unevenly, and the
    # block comes out dashed and is despeckled away.
    block = np.arange(0.0, 1.0, 0.02)
    points += [
        (1.0 + float(bx), 1.0 + float(by), 0.02 * (1.0 + float(bx)) + float(h))
        for bx in block
        for by in block
        for h in (0.5, 1.0)
    ]
    make_pcd(directory / "map.pcd", points=points)
    (directory / "poses.txt").write_text(
        "".join(
            f"{i}.pcd {x} 3.0 {0.02 * x + 0.5} 1 0 0 0\n"
            for i, x in enumerate(np.arange(2.0, 40.0, 2.0))
        )
    )
    return str(directory)


def _cell_reader(directory):
    grid = cv2.imread(os.path.join(directory, "gridmap.pgm"), cv2.IMREAD_UNCHANGED)
    with open(os.path.join(directory, "gridmap.yaml"), "r", encoding="utf-8") as handle:
        meta = yaml.safe_load(handle)
    res, (ox, oy, _) = meta["resolution"], meta["origin"]

    def cell(x, y):
        # pgm row 0 is max y — the writer's flip.
        return grid[grid.shape[0] - 1 - int((y - oy) / res), int((x - ox) / res)]

    return cell


def test_the_z_band_recipe_follows_a_drifting_floor_by_default(
    conversion_svc, drifting_saved_map, fake_traversable, conversion_threads
):
    """The TP1F regression: half a venue's floor drifting out of a band placed
    off one floor level. With the default reference the far, drifted end is
    free like the near one, the bands are recorded as the plain offsets, and
    the sidecar carries the local-floor stats that say how far the site drifted."""
    conversion_svc.start("drifting", drifting_saved_map)
    _join(conversion_threads)

    side = _sidecar(drifting_saved_map)
    params = side["params"]
    assert params["floor_reference"] == "local"
    assert "floor_reference_fallback" not in params
    for key, offset in conversion_module.GRIDMAP_BANDS_ABOVE_FLOOR.items():
        assert params[key] == offset
    assert params["local_floor"]["keyframes_measured"] == 19
    assert params["local_floor"]["pose_z_spread"] > 0.5
    assert params["local_floor"]["lidar_height"] == pytest.approx(0.5, abs=0.1)
    # The global estimate is still recorded, as the diagnostic it now is.
    assert "floor_z" in params
    # And the pose filter still ran off the same trajectory.
    assert params["pose_filter"]["applied"] == 1

    cell = _cell_reader(drifting_saved_map)
    assert cell(2.0, 3.0) == 254
    assert cell(38.0, 3.0) == 254, "the drifted end of the floor is not free"
    assert cell(1.5, 1.5) == 0, "the block on the floor was lost"


def test_floor_reference_global_bands_against_the_single_measured_floor(
    conversion_svc, drifting_saved_map, fake_traversable, conversion_threads
):
    """The escape hatch is the pre-2026-09 conversion, and on this fixture it
    reproduces the failure: the far end leaves the floor band."""
    conversion_svc.start("drifting", drifting_saved_map, floor_reference="global")
    _join(conversion_threads)

    params = _sidecar(drifting_saved_map)["params"]
    assert params["floor_reference"] == "global"
    assert "floor_reference_fallback" not in params
    assert "local_floor" not in params
    for key, offset in conversion_module.GRIDMAP_BANDS_ABOVE_FLOOR.items():
        assert params[key] == pytest.approx(offset + params["floor_z"], abs=0.01)

    cell = _cell_reader(drifting_saved_map)
    assert cell(2.0, 3.0) == 254
    assert cell(38.0, 3.0) != 254


def test_an_unmeasurable_local_floor_falls_back_to_global_and_says_so(
    conversion_svc, drifting_saved_map, fake_traversable, conversion_threads
):
    """poses.txt from some other map: no keyframe sees any cloud. The map still
    converts — against the global level — and the sidecar records why."""
    with open(os.path.join(drifting_saved_map, "poses.txt"), "w", encoding="utf-8") as handle:
        handle.write("0.pcd 500.0 500.0 1.0 1 0 0 0\n1.pcd 600.0 600.0 1.0 1 0 0 0\n")

    conversion_svc.start("drifting", drifting_saved_map)
    _join(conversion_threads)

    side = _sidecar(drifting_saved_map)
    assert side["status"] == "ok"
    params = side["params"]
    assert params["floor_reference"] == "global"
    assert "mismatched" in params["floor_reference_fallback"]
    assert os.path.isfile(os.path.join(drifting_saved_map, "gridmap.pgm"))


def test_an_unknown_floor_reference_is_refused_before_anything_is_touched(
    conversion_svc, drifting_saved_map, conversion_threads
):
    archived = []
    with pytest.raises(ValueError, match="floor_reference"):
        conversion_svc.start(
            "drifting",
            drifting_saved_map,
            floor_reference="nearest",
            archive=lambda: archived.append(1),
        )

    assert archived == []
    assert conversion_svc.is_converting("drifting") is False


def test_conversion_is_skipped_without_a_pcd(
    conversion_svc,
    maps_dir,
    conversion_threads,
):
    """pgo reported success but wrote nothing: report False, start no thread."""
    directory = str(maps_dir / "newmap")
    os.makedirs(directory, exist_ok=True)

    assert conversion_svc.start("newmap", directory) is False
    assert list(conversion_threads) == []


# --- the pose-connectivity filter through the conversion ----------------------


def test_z_band_conversion_reverts_free_space_the_poses_cannot_reach(
    conversion_svc,
    maps_dir,
    make_pcd,
    conversion_threads,
):
    """The glass-leak regression in miniature: two floor sheets 8 m apart, poses
    on one. The undriven, unconnected sheet must come back unknown — 205, not 0,
    so real driving could still clear it."""
    directory = maps_dir / "twosheets"
    os.makedirs(directory)
    floor_z = -0.4
    points = _sheet(4.0, step=0.2, z=floor_z)
    points += [(12.0 + x, y, z) for x, y, z in _sheet(4.0, step=0.2, z=floor_z)]
    # A block on the driven sheet: the z-band conversion refuses a cloud with an
    # empty obstacle band.
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (1.0 + float(bx), 1.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(directory / "map.pcd", points=points)
    # Keyframes ride 0.5 m above the floor, as a lidar does.
    (directory / "poses.txt").write_text(
        "".join(
            f"{i}.pcd {x} {y} {floor_z + 0.5} 1 0 0 0\n"
            for i, (x, y) in enumerate([(2.5, 2.5), (3.0, 2.0), (2.0, 3.0)])
        )
    )

    conversion_svc.start("twosheets", str(directory))
    _join(conversion_threads)

    side = _sidecar(str(directory))
    stats = side["params"]["pose_filter"]
    assert stats["applied"] == 1
    assert stats["reverted_free_cells"] > 0
    assert stats["components_kept"] >= 1

    grid = cv2.imread(str(directory / "gridmap.pgm"), cv2.IMREAD_UNCHANGED)
    meta = yaml.safe_load((directory / "gridmap.yaml").read_text())
    res, (ox, oy, _) = meta["resolution"], meta["origin"]

    def cell(x, y):
        # pgm row 0 is max y — the writer's flip.
        return grid[grid.shape[0] - 1 - int((y - oy) / res), int((x - ox) / res)]

    assert cell(2.5, 2.5) == 254, "the driven sheet lost its free space"
    assert cell(14.0, 2.0) == 205, "the unreachable sheet is still free"


def test_z_band_conversion_survives_a_malformed_poses_txt(
    conversion_svc,
    maps_dir,
    make_pcd,
    conversion_threads,
):
    """A broken poses.txt costs the filter, never the gridmap."""
    directory = maps_dir / "badposes"
    os.makedirs(directory)
    floor_z = -0.4
    points = _sheet(4.0, step=0.2, z=floor_z)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (1.0 + float(bx), 1.0 + float(by), floor_z + float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(directory / "map.pcd", points=points)
    (directory / "poses.txt").write_text("not a pose line\n")

    conversion_svc.start("badposes", str(directory))
    _join(conversion_threads)

    assert os.path.isfile(directory / "gridmap.pgm")
    assert "pose_filter" not in _sidecar(str(directory))["params"]


# --- POST /api/v1/maps/{name}/grid/convert ------------------------------------


def _post_convert(client, name, payload=None):
    return client.post(f"/api/v1/maps/{name}/grid/convert", json=payload or {})


def test_convert_endpoint_runs_the_requested_traversability_recipe(
    client, saved_map, fake_traversable, conversion_threads, map_gw
):
    response = _post_convert(
        client, "newmap", {"recipe": "traversability", "reason": "big warehouse"}
    )
    _join(conversion_threads)

    assert response.status_code == 200
    body = response.json()
    assert body["started"] is True
    assert body["recipe"] == "traversability"
    assert len(fake_traversable.calls) == 1
    side = _sidecar(saved_map)
    assert side["recipe"] == "traversability"
    assert side["recipe_override"]["requested"] == "traversability"
    assert side["recipe_override"]["reason"] == "big warehouse"
    assert side["recipe_override"]["picked_by"] == (
        "POST /api/v1/maps/newmap/grid/convert"
    )
    # newmap is not the active map ('full' is): no reload.
    assert map_gw.calls == []


def test_convert_endpoint_defaults_to_z_band(
    client, maps_dir, saved_map, make_pcd, fake_traversable, conversion_threads
):
    """An empty body re-converts with the safe default, whatever the size."""
    # The 60 m sheet alone has nothing in the obstacle band, so give it a block.
    points = _sheet(60.0, step=2.0)
    block = np.arange(0.0, 1.0, 0.05)
    points += [
        (1.0 + float(bx), 1.0 + float(by), float(h))
        for bx in block
        for by in block
        for h in np.arange(0.5, 1.5, 0.25)
    ]
    make_pcd(maps_dir / "newmap" / "map.pcd", points=points)

    response = _post_convert(client, "newmap")
    _join(conversion_threads)

    assert response.status_code == 200
    assert response.json()["recipe"] == "z-band"
    assert fake_traversable.calls == []
    assert _sidecar(saved_map)["recipe"] == "z-band"


def test_convert_endpoint_reloads_the_active_map(
    client, map_gw, conversion_threads
):
    """'full' is the active map; a re-convert that map_server never hears about
    would leave it serving the grid the operator just replaced."""
    response = _post_convert(client, "full")
    _join(conversion_threads)

    assert response.status_code == 200
    assert len(map_gw.calls) == 1
    assert map_gw.calls[0].endswith("full/gridmap.yaml")


def test_convert_endpoint_archives_the_previous_grid(
    client, maps_dir, conversion_threads
):
    previous = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    previous_yaml = (maps_dir / "full" / "gridmap.yaml").read_bytes()

    response = _post_convert(client, "full")
    _join(conversion_threads)

    assert response.status_code == 200
    assert "gridmap_prev.pgm" in response.json()["message"]
    assert (maps_dir / "full" / "gridmap_prev.pgm").read_bytes() == previous
    assert (maps_dir / "full" / "gridmap_prev.yaml").read_bytes() == previous_yaml
    # The new conversion really replaced the grid (the fixture's 6x4 became the
    # pcd's real extent).
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() != previous


def test_convert_endpoint_archives_the_previous_recipe_record(
    client, maps_dir, conversion_threads
):
    """The incoming conversion overwrites the sidecar with its own `converting`
    record before it does any work, so the outgoing one has to move aside with
    the grid it describes — otherwise a re-convert that then fails leaves a map
    serving the archived grid with nothing on disk saying how it was made."""
    _plant_sidecar(
        maps_dir / "full",
        {"status": "ok", "recipe": "z-band", "params": {"floor_z": -0.4}},
    )

    response = _post_convert(client, "full")
    _join(conversion_threads)

    assert response.status_code == 200
    prev = json.loads(
        (maps_dir / "full" / "gridmap_prev.recipe.json").read_text(encoding="utf-8")
    )
    assert prev["recipe"] == "z-band"
    assert prev["params"] == {"floor_z": -0.4}
    # ...and the live record is the new run's, not the archived one.
    assert _sidecar(str(maps_dir / "full"))["status"] == "ok"
    assert "params" in _sidecar(str(maps_dir / "full"))


def test_convert_endpoint_refuses_to_discard_hand_edits(
    client, maps_dir, make_pgm, conversion_threads
):
    """A raw snapshot differing from the live grid means operator work; silently
    re-converting over it would destroy it."""
    make_pgm(maps_dir / "full" / "gridmap_raw.pgm", 6, 4, fill=0)

    response = _post_convert(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "gridmap_hand_edited"
    assert list(conversion_threads) == []


def test_convert_endpoint_overwrites_hand_edits_only_on_confirmation(
    client, maps_dir, make_pgm, conversion_threads
):
    make_pgm(maps_dir / "full" / "gridmap_raw.pgm", 6, 4, fill=0)
    edited = (maps_dir / "full" / "gridmap.pgm").read_bytes()

    response = _post_convert(client, "full", {"overwrite_edits": True})
    _join(conversion_threads)

    assert response.status_code == 200
    # The edited grid survives as the prev generation...
    assert (maps_dir / "full" / "gridmap_prev.pgm").read_bytes() == edited
    # ...its stale raw snapshot moved aside with it, so the next editor save
    # snapshots the *new* conversion rather than trusting a wrong-era raw...
    assert not (maps_dir / "full" / "gridmap_raw.pgm").exists()
    assert (maps_dir / "full" / "gridmap_prev_raw.pgm").exists()
    # ...and the live grid is the fresh conversion.
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() != edited


def test_convert_endpoint_conflicts_while_a_conversion_runs(client, conversion_svc):
    _mark_converting(conversion_svc, "full")

    response = _post_convert(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"


def test_grid_status_is_converting_while_the_slot_is_held(client, conversion_svc):
    """Both fields, read from one sample of the registry: `grid_converting` is a
    deprecated alias of `grid_status == "converting"`, and an alias that can
    disagree with what it aliases is worse than no alias."""
    _mark_converting(conversion_svc, "full")

    entry = _by_name(client.get("/api/v1/maps").json())["full"]
    assert entry["grid_status"] == "converting"
    assert entry["grid_converting"] is True

    _clear_converting(conversion_svc, "full")

    entry = _by_name(client.get("/api/v1/maps").json())["full"]
    assert entry["grid_status"] == "ok"
    assert entry["grid_converting"] is False


def test_convert_endpoint_400s_without_a_pcd(client, maps_dir, conversion_threads):
    (maps_dir / "full" / "map.pcd").unlink()

    response = _post_convert(client, "full")

    assert response.status_code == 400
    assert list(conversion_threads) == []


def test_convert_endpoint_404s_for_a_missing_map(client):
    assert _post_convert(client, "nosuchmap").status_code == 404


def test_convert_endpoint_rejects_cross_recipe_parameters(client, conversion_threads):
    z_with_gap = _post_convert(client, "full", {"recipe": "z-band", "gap_fill_size": 1.0})
    trav_with_bands = _post_convert(
        client,
        "full",
        {"recipe": "traversability", "z_band_offsets": {"zmax": 3.0}},
    )

    assert z_with_gap.status_code == 400
    assert trav_with_bands.status_code == 400
    assert list(conversion_threads) == []


def test_convert_endpoint_rejects_a_floor_reference_for_traversability(
    client, conversion_threads
):
    response = _post_convert(
        client, "full", {"recipe": "traversability", "floor_reference": "global"}
    )

    assert response.status_code == 400
    assert "floor_reference" in response.json()["detail"]
    assert list(conversion_threads) == []


def test_convert_endpoint_422s_an_unknown_floor_reference(client, conversion_threads):
    response = _post_convert(client, "full", {"floor_reference": "nearest"})

    assert response.status_code == 422
    assert list(conversion_threads) == []


def test_convert_endpoint_records_a_requested_floor_reference(
    client, drifting_saved_map, fake_traversable, conversion_threads
):
    """``global`` asked for explicitly: the conversion uses it and the override
    record names it, so a later reader can tell a choice from a fallback."""
    response = _post_convert(
        client, "drifting", {"floor_reference": "global", "reason": "compare"}
    )
    _join(conversion_threads)

    assert response.status_code == 200
    side = _sidecar(drifting_saved_map)
    assert side["params"]["floor_reference"] == "global"
    assert "floor_reference_fallback" not in side["params"]
    assert side["recipe_override"]["param_overrides"] == {"floor_reference": "global"}


def test_convert_endpoint_defaults_the_floor_reference_to_local(
    client, drifting_saved_map, fake_traversable, conversion_threads
):
    """An empty body neither names a reference nor records one as an override:
    the default is the service's, and the sidecar's params say which ran."""
    response = _post_convert(client, "drifting")
    _join(conversion_threads)

    assert response.status_code == 200
    side = _sidecar(drifting_saved_map)
    assert side["params"]["floor_reference"] == "local"
    assert side["recipe_override"]["param_overrides"] == {}


def test_convert_endpoint_rejects_inverted_merged_bands(client, conversion_threads):
    """The override merges over the recipe's other end, so one wrong number can
    invert a band — that must die here, not half a minute later in the thread."""
    response = _post_convert(
        client, "full", {"recipe": "z-band", "z_band_offsets": {"floor_zmax": -0.5}}
    )

    assert response.status_code == 400
    assert "floor band is inverted" in response.json()["detail"]
    assert list(conversion_threads) == []


def test_convert_endpoint_422s_an_out_of_range_gap_fill(client, conversion_threads):
    response = _post_convert(
        client, "full", {"recipe": "traversability", "gap_fill_size": 5.0}
    )

    assert response.status_code == 422
    assert list(conversion_threads) == []


def test_convert_endpoint_passes_debug_and_overrides_through(
    client, saved_map, fake_traversable, conversion_threads
):
    response = _post_convert(
        client,
        "newmap",
        {"recipe": "traversability", "gap_fill_size": 1.2, "debug": True},
    )
    _join(conversion_threads)

    assert response.status_code == 200
    assert fake_traversable.calls[0]["debug_dir"] == os.path.join(
        saved_map, "traversable_debug"
    )
    side = _sidecar(saved_map)
    assert side["params"]["grid"] == {"gap_fill_size": 1.2}
    assert side["recipe_override"]["param_overrides"] == {"gap_fill_size": 1.2}


# --- POST /api/v1/maps/{name}/activate --------------------------------------
#
# The fixture INI pins `full` active, and `full` is the only fixture map with a
# gridmap, so most switch tests point the INI at `rawonly` first and switch *to*
# `full`. Writing the file rather than re-parameterising the fixture is
# deliberate: `active_map_name` re-reads the INI on every call, which is the
# property the route depends on and therefore one worth exercising.

_INTERPOLATED_INI = (
    "[system]\nrobot_id: robot01\n\n"
    "# which map the stack loads\n"
    "[map]\n"
    "name: rawonly\n"
    "pcd: map/%(name)s/map.pcd\n"
    "map: map/%(name)s/gridmap.yaml\n\n"
    "[initial_pose]\n"
    "x: 3.25\ny: -1.5\nz: 0.0\nyaw: 1.57\n"
)


def _point_ini_at(monkeypatch, tmp_path, text):
    ini = tmp_path / "system.ini"
    ini.write_text(text)
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    return ini


def _activate(client, name):
    return client.post(f"/api/v1/maps/{name}/activate")


def test_activate_the_active_map_is_a_noop(client, map_gw):
    response = _activate(client, "full")

    assert response.status_code == 200
    body = response.json()
    assert body["switched"] is False
    assert body["previous"] == "full"
    # The point of the no-op: a good registration is not thrown away.
    assert map_gw.order == []


def test_activate_unknown_map_is_a_404(client, map_gw):
    assert _activate(client, "nosuchmap").status_code == 404
    assert map_gw.order == []


def test_activate_refuses_a_map_with_no_gridmap(client, map_gw, monkeypatch, tmp_path):
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: full\n")

    response = _activate(client, "rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "grid_missing"
    assert map_gw.order == []


def test_activate_refuses_a_map_with_no_pointcloud(
    client, map_gw, maps_dir, make_pgm, make_gridmap_yaml, monkeypatch, tmp_path
):
    gridonly = maps_dir / "gridonly"
    gridonly.mkdir()
    make_pgm(gridonly / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(gridonly / "gridmap.yaml", origin=(0.0, 0.0, 0.0))

    response = _activate(client, "gridonly")

    assert response.status_code == 409
    assert response.json()["code"] == "pointcloud_missing"
    assert map_gw.order == []


def test_activate_refuses_while_a_conversion_runs(
    client, conversion_svc, map_gw, monkeypatch, tmp_path
):
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")

    _mark_converting(conversion_svc, "full")

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    assert map_gw.order == []


def test_activate_refuses_while_a_task_is_running(
    client, map_gw, workflow_gw, monkeypatch, tmp_path
):
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")
    workflow_gw.tasks = [types.SimpleNamespace(id="task-7")]

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "task-7" in response.json()["detail"]
    assert map_gw.order == []


def test_activate_refuses_when_temporal_cannot_be_reached(
    client, map_gw, workflow_gw, monkeypatch, tmp_path
):
    """A Temporal outage must not be read as "nothing is running"."""
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")
    workflow_gw.error = RuntimeError("connection refused")

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert map_gw.order == []


# --- The map a running job holds ----------------------------------------------


def _job(map_name=None, kind=None):
    from datetime import datetime, timezone

    from syncai_backend.gateways.workflow.schema import ActiveTask, TaskSource

    return ActiveTask(
        id="robot01-task-9",
        run_id="run-9",
        status="IN_PROGRESS",
        started_at=datetime(2026, 10, 4, tzinfo=timezone.utc),
        source=TaskSource.DIRECT,
        kind=kind,
        map_name=map_name,
    )


# Every edit the planner, the costmaps or a job's MOVE steps read from, each
# on 'full' -- the client fixture's loaded map.
_EDITS = {
    "floor plan": lambda client: _put_grid(client, "full", b"\x00" * 24),
    "forbidden zones": lambda client: _put_keepout(client, "full", [_rect_zone()]),
    "rebuild": lambda client: _post_convert(client, "full"),
}


@pytest.mark.parametrize("edit", list(_EDITS))
def test_an_edit_waits_for_the_job_driving_on_the_map(client, map_gw, workflow_gw, edit):
    workflow_gw.tasks = [_job(map_name="full")]

    response = _EDITS[edit](client)

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "running a job on this map" in response.json()["detail"]
    # Refused before anything reached the running stack.
    assert map_gw.calls == []


def test_a_run_older_than_the_stamp_holds_the_loaded_map(client, map_gw, workflow_gw):
    workflow_gw.tasks = [_job(map_name=None, kind=None)]

    response = _put_grid(client, "full", b"\x00" * 24)

    assert response.json()["code"] == "task_running"


def test_a_job_that_drives_nowhere_does_not_hold_the_map(
    client, map_gw, workflow_gw
):
    workflow_gw.tasks = [_job(map_name=None, kind=TaskKind.STANDUP)]

    assert _put_grid(client, "full", b"\x00" * 24).status_code == 200


def test_an_edit_is_refused_when_running_jobs_cannot_be_known(
    client, map_gw, workflow_gw
):
    workflow_gw.error = RuntimeError("connection refused")

    response = _put_keepout(client, "full", [_rect_zone()])

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert map_gw.calls == []


def test_a_map_that_is_not_loaded_is_edited_without_asking(
    client, maps_dir, workflow_gw, make_pgm, make_gridmap_yaml
):
    # A running job can only hold the loaded map (activate refuses while one
    # runs), so an outage of the task service must not block this edit.
    make_pgm(maps_dir / "rawonly" / "gridmap.pgm", 3, 2)
    make_gridmap_yaml(maps_dir / "rawonly" / "gridmap.yaml")
    workflow_gw.error = RuntimeError("connection refused")

    assert _put_grid(client, "rawonly", b"\x00" * 6).status_code == 200


def test_activate_refuses_when_the_nav_stack_is_not_up(
    client, map_gw, monkeypatch, tmp_path
):
    """How "the robot is in mapping mode" is detected."""
    _point_ini_at(monkeypatch, tmp_path, "[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")
    map_gw.services_ready = False

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "stack_not_ready"
    assert map_gw.order == []


def test_activate_moves_the_localizer_before_map_server(
    client, map_gw, maps_dir, monkeypatch, tmp_path
):
    ini = _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)

    response = _activate(client, "full")

    assert response.status_code == 200
    body = response.json()
    assert body["switched"] is True
    assert body["name"] == "full"
    assert body["previous"] == "rawonly"
    assert body["localized"] is True

    # The ordering is the contract: the localizer's failure is the clean one, so
    # it goes first.
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "reload_keepout",
    ]
    assert map_gw.order[0][1] == str(maps_dir / "full" / "map.pcd")
    assert map_gw.order[1][1] == str(maps_dir / "full" / "gridmap.yaml")
    assert map_gw.order[2][1] == str(maps_dir / "full" / "keepout.yaml")
    # Zeroed, not carried over from the old map's frame.
    assert map_gw.swap_calls[0][1:] == (0.0, 0.0, 0.0)

    written = ini.read_text()
    assert "name: full" in written
    # The whole reason the writer edits lines instead of round-tripping through
    # ConfigParser: expanding this would pin pcd/map at the *old* map forever.
    assert "pcd: map/%(name)s/map.pcd" in written
    assert "map: map/%(name)s/gridmap.yaml" in written
    assert "# which map the stack loads" in written
    # A pose measured in rawonly's frame means nothing in full's.
    assert "x: 0.0" in written and "yaw: 0.0" in written
    assert "3.25" not in written


def test_activate_puts_the_localizer_back_when_map_server_refuses(
    client, map_gw, maps_dir, monkeypatch, tmp_path
):
    ini = _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.result = (False, "map_server rejected gridmap.yaml")

    response = _activate(client, "full")

    assert response.status_code == 502
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "swap_localizer_map",
    ]
    # Back onto the map it came from, not left straddling two.
    assert map_gw.order[2][1] == str(maps_dir / "rawonly" / "map.pcd")
    assert "name: rawonly" in ini.read_text()


def test_activate_rolls_back_both_steps_when_the_ini_cannot_be_written(
    client, map_gw, maps_dir, make_pcd, make_pgm, make_gridmap_yaml,
    monkeypatch, tmp_path,
):
    """The INI write is preflighted, so reaching it and failing must undo the rest.

    `previous` here is a fully converted map, unlike the pcd-only `rawonly` the
    other switch tests come from: a map the stack actually launched on must have
    had a gridmap.yaml, and it is the rollback of *both* ROS steps that this
    covers.
    """
    prior = maps_dir / "prior"
    prior.mkdir()
    make_pcd(prior / "map.pcd")
    make_pgm(prior / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(prior / "gridmap.yaml", origin=(0.0, 0.0, 0.0))
    _point_ini_at(
        monkeypatch, tmp_path, _INTERPOLATED_INI.replace("name: rawonly", "name: prior")
    )

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(map_router_module, "set_active_map", _boom)

    response = _activate(client, "full")

    assert response.status_code == 502
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "swap_localizer_map",
        "reload_map",
    ]
    assert map_gw.order[2][1] == str(prior / "map.pcd")
    assert map_gw.order[3][1] == str(prior / "gridmap.yaml")


def test_activate_says_so_when_the_old_grid_cannot_be_restored(
    client, map_gw, monkeypatch, tmp_path
):
    """Rolling back onto a map with no gridmap.yaml leaves a mismatch worth naming."""
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(map_router_module, "set_active_map", _boom)

    response = _activate(client, "full")

    assert response.status_code == 502
    detail = response.json()["detail"]
    assert "no gridmap.yaml to put back" in detail
    # The localizer still went back; only the grid could not.
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "swap_localizer_map",
    ]


def test_activate_refuses_when_the_ini_is_not_writable(
    client, map_gw, monkeypatch, tmp_path
):
    ini = _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    os.chmod(ini, 0o444)
    try:
        response = _activate(client, "full")
    finally:
        os.chmod(ini, 0o644)

    assert response.status_code == 409
    assert response.json()["code"] == "ini_not_writable"
    assert map_gw.order == []


def test_activate_reports_an_unconverged_localizer(
    client, map_gw, monkeypatch, tmp_path
):
    """A swap that lands but does not converge is a success with a warning."""
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.converged = False

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert body["localized"] is False
    assert "set an initial pose" in body["message"]


def test_activate_reports_an_unreachable_localizer_check(
    client, map_gw, monkeypatch, tmp_path
):
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.converged = None

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert body["localized"] is None
    assert "could not be asked" in body["message"]


def test_activate_writes_a_blank_mask_for_a_map_never_booted_into(
    client, map_gw, maps_dir, monkeypatch, tmp_path
):
    """filter_mask_server holds the old map's zones until it is handed this one's."""
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    assert not (maps_dir / "full" / "keepout.yaml").exists()

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert body["keepout_reloaded"] is True
    assert "forbidden zone" not in body["message"]  # nothing to say about zero zones
    # All unknown, the same blank the nav session writes at boot -- and written
    # into the map's own directory so the next boot of it finds the pair.
    cells = (maps_dir / "full" / "keepout.pgm").read_bytes()[len(b"P5\n6 4\n255\n"):]
    assert cells == bytes([205]) * 24
    assert map_gw.keepout_calls == [str(maps_dir / "full" / "keepout.yaml")]


def test_activate_carries_zones_drawn_while_the_map_was_inactive(
    client, map_gw, maps_dir, monkeypatch, tmp_path
):
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    _put_keepout(client, "full", [_rect_zone("a"), _rect_zone("b", cols=(5, 5), rows=(3, 3))])
    map_gw.keepout_calls.clear()
    written = (maps_dir / "full" / "keepout.pgm").read_bytes()

    body = _activate(client, "full").json()

    assert body["keepout_reloaded"] is True
    assert "Its 2 forbidden zones are active." in body["message"]
    assert map_gw.keepout_calls == [str(maps_dir / "full" / "keepout.yaml")]
    # An existing mask is loaded as-is, not regenerated.
    assert (maps_dir / "full" / "keepout.pgm").read_bytes() == written


def test_activate_reports_a_failed_keepout_reload_without_failing_the_switch(
    client, map_gw, monkeypatch, tmp_path
):
    ini = _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.keepout_result = (False, "filter_mask_server rejected keepout.yaml")

    response = _activate(client, "full")

    assert response.status_code == 200
    body = response.json()
    assert body["switched"] is True
    assert body["keepout_reloaded"] is False
    assert "filter_mask_server rejected keepout.yaml" in body["message"]
    # The switch itself is recorded; the mask is the only thing left behind.
    assert "name: full" in ini.read_text()


def test_activate_reloads_the_keepout_only_after_the_ini_is_written(
    client, map_gw, monkeypatch, tmp_path
):
    """A rolled-back switch must not hand the mask server the map it backed out of."""
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(map_router_module, "set_active_map", _boom)

    assert _activate(client, "full").status_code == 502
    assert map_gw.keepout_calls == []
    assert "reload_keepout" not in [step for step, _ in map_gw.order]


def test_activate_noop_has_no_keepout_verdict(client, map_gw):
    body = _activate(client, "full").json()

    assert body["switched"] is False
    assert body["keepout_reloaded"] is None
    assert map_gw.keepout_calls == []


# --- export / import ------------------------------------------------------------
#
# Same fixture geometry as rename and delete: 'full' is the active map, so
# 'rawonly' is the one an import is allowed to replace.

_MANIFEST = "syncai_map.json"


def _export(client, name, fmt=None):
    query = "" if fmt is None else f"?format={fmt}"
    return client.get(f"/api/v1/maps/{name}/export{query}")


def _import(client, data, name=None):
    query = "" if name is None else f"?name={name}"
    return client.post(f"/api/v1/maps/import{query}", content=data, headers=_OCTET)


def _hidden_dirs(maps_dir):
    return sorted(p.name for p in maps_dir.iterdir() if p.name.startswith(".import-"))


def _tree(directory):
    """{relpath: bytes} of every file under ``directory``."""
    out = {}
    for dirpath, _dirs, files in os.walk(directory):
        for filename in files:
            full = os.path.join(dirpath, filename)
            out[os.path.relpath(full, directory)] = open(full, "rb").read()
    return out


def _zip_of(files, manifest):
    """A hand-built zip: ``files`` is {relpath: bytes}; manifest a dict or None."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for rel, data in files.items():
            archive.writestr(rel, data)
        if manifest is not None:
            archive.writestr(_MANIFEST, json.dumps(manifest))
    return buffer.getvalue()


def _manifest_for(files, name="handmade", vertices=()):
    return {
        "format": "syncai-map",
        "version": 1,
        "name": name,
        "exported_at": "2026-10-03T00:00:00Z",
        "files": {rel: hashlib.md5(data, usedforsecurity=False).hexdigest()
                  for rel, data in files.items()},
        "vertices": list(vertices),
    }


def _handmade(files=None, name="handmade", vertices=(), manifest=True):
    files = {"map.pcd": b"# .PCD\nDATA ascii\n0 0 0\n"} if files is None else files
    return _zip_of(files, _manifest_for(files, name, vertices) if manifest else None)


@pytest.fixture
def exportable(maps_dir, map_repo, make_pcd):
    """'full' with a patch, a debug cloud and two vertices -- a real-looking map."""
    (maps_dir / "full" / "patches").mkdir()
    make_pcd(maps_dir / "full" / "patches" / "000001.pcd")
    (maps_dir / "full" / "traversable_debug").mkdir()
    (maps_dir / "full" / "traversable_debug" / "step1.pcd").write_bytes(b"debug")
    map_repo.create_vertices(map="full", vertices=[
        {"name": "dock", "type": "CHARGER", "x": 1.5, "y": -2.0, "theta": 90.0},
        {"name": "home", "type": "HOME", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    return maps_dir / "full"


@pytest.mark.parametrize("fmt, media, opener", [
    ("zip", "application/zip", lambda b: zipfile.ZipFile(io.BytesIO(b)).namelist()),
    ("tar.gz", "application/gzip",
     lambda b: tarfile.open(fileobj=io.BytesIO(b), mode="r:gz").getnames()),
])
def test_export_round_trips_through_import(client, maps_dir, map_repo, exportable,
                                           fmt, media, opener):
    response = _export(client, "full", fmt)

    assert response.status_code == 200
    assert response.headers["content-type"] == media
    assert response.headers["content-disposition"] == f'attachment; filename="full.{fmt}"'
    names = opener(response.content)
    assert names[-1] == _MANIFEST
    assert "patches/000001.pcd" in names
    assert not any(n.startswith("traversable_debug") for n in names)

    imported = _import(client, response.content, name="copy")

    assert imported.status_code == 201, imported.text
    body = imported.json()
    assert body["name"] == "copy"
    assert body["replaced"] is False
    assert body["files"] == 4  # map.pcd, gridmap.pgm, gridmap.yaml, the patch
    assert body["vertices_created"] == 2
    assert body["vertices_deleted"] == 0
    assert "Imported 'copy'" in body["message"]

    expected = {k: v for k, v in _tree(exportable).items()
                if not k.startswith("traversable_debug")}
    assert _tree(maps_dir / "copy") == expected
    assert _hidden_dirs(maps_dir) == []

    copied = {v.name: v for v in map_repo.list_vertices(map="copy")}
    originals = {v.name: v for v in map_repo.list_vertices(map="full")}
    assert set(copied) == {"dock", "home"}
    for name, vertex in copied.items():
        assert vertex.id != originals[name].id
        assert (vertex.type, vertex.x, vertex.y, vertex.theta) == (
            originals[name].type, originals[name].x, originals[name].y,
            originals[name].theta)

    listing = _by_name(client.get("/api/v1/maps").json())
    assert listing["copy"]["vertex_count"] == 2
    assert listing["copy"]["active"] is False


def test_export_defaults_to_zip(client):
    response = _export(client, "full")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/zip"
    assert response.content[:2] == b"PK"


def test_export_unknown_format_is_422(client):
    assert _export(client, "full", "rar").status_code == 422


def test_export_404_for_a_missing_map(client):
    assert _export(client, "nosuchmap").status_code == 404


def test_export_refuses_while_a_conversion_is_running(client, conversion_svc):
    _mark_converting(conversion_svc, "rawonly")

    response = _export(client, "rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"


def test_export_of_the_active_map_is_allowed(client):
    assert _export(client, "full").status_code == 200


def test_export_manifest_carries_md5s_and_vertices(client, exportable):
    payload = _export(client, "full").content
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        manifest = json.loads(archive.read(_MANIFEST))
        pcd = archive.read("map.pcd")

    assert manifest["format"] == "syncai-map"
    assert manifest["version"] == 1
    assert manifest["name"] == "full"
    assert manifest["files"]["map.pcd"] == hashlib.md5(pcd, usedforsecurity=False).hexdigest()
    assert _MANIFEST not in manifest["files"]
    assert {v["name"] for v in manifest["vertices"]} == {"dock", "home"}
    assert all("id" not in v for v in manifest["vertices"])


# import refusals: each leaves the maps tree exactly as it was.


def _assert_untouched(maps_dir, names=("full", "rawonly")):
    assert sorted(p.name for p in maps_dir.iterdir() if p.is_dir()) == sorted(names)
    assert _hidden_dirs(maps_dir) == []


def test_import_without_a_manifest_is_400(client, maps_dir):
    response = _import(client, _handmade(manifest=False))

    assert response.status_code == 400
    assert _MANIFEST in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_with_an_md5_mismatch_is_400_and_sweeps_the_staging_dir(client, maps_dir):
    files = {"map.pcd": b"original"}
    manifest = _manifest_for(files)
    payload = _zip_of({"map.pcd": b"tampered"}, manifest)

    response = _import(client, payload)

    assert response.status_code == 400
    assert "md5 mismatch for 'map.pcd'" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_with_an_unlisted_file_is_400(client, maps_dir):
    files = {"map.pcd": b"pcd"}
    payload = _zip_of(dict(files, **{"extra.sh": b"#!/bin/sh"}), _manifest_for(files))

    response = _import(client, payload)

    assert response.status_code == 400
    assert "Not listed in the manifest: 'extra.sh'" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_with_a_listed_file_missing_is_400(client, maps_dir):
    files = {"map.pcd": b"pcd", "gridmap.pgm": b"pgm"}
    payload = _zip_of({"map.pcd": b"pcd"}, _manifest_for(files))

    response = _import(client, payload)

    assert response.status_code == 400
    assert "missing from the archive: 'gridmap.pgm'" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_with_a_traversing_member_is_400(client, maps_dir):
    files = {"map.pcd": b"pcd", "../evil": b"x"}
    response = _import(client, _zip_of(files, _manifest_for({"map.pcd": b"pcd"})))

    assert response.status_code == 400
    assert "../evil" in response.json()["detail"]
    _assert_untouched(maps_dir)
    assert not (maps_dir.parent / "evil").exists()


def test_import_with_a_symlink_member_is_400(client, maps_dir):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        info = tarfile.TarInfo("map.pcd")
        info.size = 3
        archive.addfile(info, io.BytesIO(b"pcd"))
        link = tarfile.TarInfo("link")
        link.type = tarfile.SYMTYPE
        link.linkname = "/etc/passwd"
        archive.addfile(link)
        manifest = json.dumps(_manifest_for({"map.pcd": b"pcd"})).encode()
        info = tarfile.TarInfo(_MANIFEST)
        info.size = len(manifest)
        archive.addfile(info, io.BytesIO(manifest))

    response = _import(client, buffer.getvalue())

    assert response.status_code == 400
    assert "link" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_of_an_unsupported_container_is_400(client, maps_dir):
    response = _import(client, b"this is not an archive")

    assert response.status_code == 400
    assert "Unsupported archive" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_without_a_pointcloud_is_400(client, maps_dir):
    response = _import(client, _handmade(files={"gridmap.pgm": b"P5"}))

    assert response.status_code == 400
    assert "map.pcd" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_with_a_bad_name_override_is_400(client, maps_dir):
    response = _import(client, _handmade(), name="../escape")

    assert response.status_code == 400
    _assert_untouched(maps_dir)


def test_import_with_a_bad_manifest_name_and_no_override_is_400(client, maps_dir):
    response = _import(client, _handmade(name="has space"))

    assert response.status_code == 400
    assert "Invalid map name" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_with_an_unknown_vertex_type_is_400(client, maps_dir):
    vertices = [{"name": "x", "type": "TELEPORTER", "x": 0, "y": 0, "theta": 0}]
    response = _import(client, _handmade(vertices=vertices))

    assert response.status_code == 400
    assert "TELEPORTER" in response.json()["detail"]
    _assert_untouched(maps_dir)


def test_import_refuses_when_the_disk_is_too_full(client, maps_dir, catalog_repo, monkeypatch):
    monkeypatch.setattr(catalog_repo, "free_bytes", lambda: 0)

    response = _import(client, _handmade())

    assert response.status_code == 409
    assert response.json()["code"] == "disk_low"
    _assert_untouched(maps_dir)


def test_import_without_a_name_uses_the_manifest_name(client, maps_dir, map_repo):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    payload = _export(client, "rawonly").content
    before = _tree(maps_dir / "rawonly")
    shutil.rmtree(maps_dir / "rawonly")
    map_repo.delete_vertices("rawonly")

    response = _import(client, payload)

    assert response.status_code == 201, response.text
    assert response.json()["name"] == "rawonly"
    assert response.json()["replaced"] is False
    assert _tree(maps_dir / "rawonly") == before
    assert [v.name for v in map_repo.list_vertices(map="rawonly")] == ["dock"]


def test_import_with_no_vertices_creates_none(client, maps_dir, map_repo):
    response = _import(client, _handmade(name="bare"))

    assert response.status_code == 201, response.text
    assert response.json()["vertices_created"] == 0
    assert map_repo.list_vertices(map="bare") == []
    assert (maps_dir / "bare" / "map.pcd").is_file()


def test_import_sweeps_orphaned_vertex_rows_of_a_deleted_directory(client, map_repo):
    map_repo.create_vertices(map="ghost", vertices=[
        {"name": "stale", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    vertices = [{"name": "fresh", "type": "HOME", "x": 1.0, "y": 1.0, "theta": 0.0}]

    response = _import(client, _handmade(name="ghost", vertices=vertices))

    assert response.status_code == 201, response.text
    body = response.json()
    assert body["replaced"] is False
    assert body["vertices_deleted"] == 1
    assert [v.name for v in map_repo.list_vertices(map="ghost")] == ["fresh"]


# replacing an existing map


def test_import_replaces_an_existing_map(client, maps_dir, map_repo, exportable):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "old1", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "old2", "type": "GENERAL", "x": 1.0, "y": 0.0, "theta": 0.0},
        {"name": "old3", "type": "GENERAL", "x": 2.0, "y": 0.0, "theta": 0.0},
    ])
    (maps_dir / "rawonly" / "poses.txt").write_text("stale\n")
    payload = _export(client, "full").content

    response = _import(client, payload, name="rawonly")

    assert response.status_code == 201, response.text
    body = response.json()
    assert body["name"] == "rawonly"
    assert body["replaced"] is True
    assert body["vertices_created"] == 2
    assert body["vertices_deleted"] == 3
    assert "Replaced 'rawonly'" in body["message"]
    assert "3 previous vertices removed" in body["message"]

    expected = {k: v for k, v in _tree(exportable).items()
                if not k.startswith("traversable_debug")}
    assert _tree(maps_dir / "rawonly") == expected  # poses.txt is gone
    assert _hidden_dirs(maps_dir) == []
    assert {v.name for v in map_repo.list_vertices(map="rawonly")} == {"dock", "home"}
    # The source map is untouched.
    assert len(map_repo.list_vertices(map="full")) == 2


def test_import_refuses_to_replace_the_active_map(client, maps_dir):
    before = _tree(maps_dir / "full")

    response = _import(client, _handmade(), name="full")

    assert response.status_code == 409
    assert response.json()["code"] == "map_active"
    assert "replaced" in response.json()["detail"]
    assert _tree(maps_dir / "full") == before
    _assert_untouched(maps_dir)


def test_import_refuses_to_replace_a_converting_map(client, maps_dir, conversion_svc):
    _mark_converting(conversion_svc, "rawonly")

    response = _import(client, _handmade(), name="rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "conversion_running"
    _assert_untouched(maps_dir)


def test_import_refuses_to_replace_a_template_bound_map(client, maps_dir, task_template_repo):
    task_template_repo.create_task_template(
        name="patrol", description="", map_name="rawonly", steps=[]
    )
    before = _tree(maps_dir / "rawonly")

    response = _import(client, _handmade(), name="rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "template_bound"
    assert "'patrol'" in response.json()["detail"]
    assert _tree(maps_dir / "rawonly") == before
    _assert_untouched(maps_dir)


def test_import_rolls_a_new_map_back_when_the_vertex_insert_fails(
    client, maps_dir, map_repo, monkeypatch
):
    def _boom(*args, **kwargs):
        raise RuntimeError("database away")

    monkeypatch.setattr(map_repo, "create_vertices", _boom)
    vertices = [{"name": "v", "type": "GENERAL", "x": 0, "y": 0, "theta": 0}]

    response = _import(client, _handmade(name="copy", vertices=vertices))

    assert response.status_code == 502
    assert "removed again" in response.json()["detail"]
    _assert_untouched(maps_dir)
    assert map_repo.list_vertices(map="copy") == []


def test_import_puts_the_previous_map_back_when_the_vertex_insert_fails(
    client, maps_dir, map_repo, monkeypatch
):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "keep", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    before = _tree(maps_dir / "rawonly")

    def _boom(*args, **kwargs):
        raise RuntimeError("database away")

    monkeypatch.setattr(map_repo, "create_vertices", _boom)
    vertices = [{"name": "v", "type": "GENERAL", "x": 0, "y": 0, "theta": 0}]

    response = _import(client, _handmade(name="rawonly", vertices=vertices))

    assert response.status_code == 502
    assert "put back" in response.json()["detail"]
    assert _tree(maps_dir / "rawonly") == before
    _assert_untouched(maps_dir)
    # The DELETE was in the same transaction as the failed insert.
    assert [v.name for v in map_repo.list_vertices(map="rawonly")] == ["keep"]


def test_import_drops_the_cached_pointcloud_of_the_map_it_replaces(client, make_pcd, tmp_path):
    # The fixture's rawonly cloud holds a single point.
    first = client.get("/api/v1/maps/rawonly/pointcloud").content
    assert struct.unpack("<I", first[:4])[0] == 1

    other = tmp_path / "other.pcd"
    make_pcd(other, points=((0.0, 0.0, 0.0), (5.0, 5.0, 5.0)))
    response = _import(client, _handmade(files={"map.pcd": other.read_bytes()}), name="rawonly")
    assert response.status_code == 201, response.text

    second = client.get("/api/v1/maps/rawonly/pointcloud").content
    assert struct.unpack("<I", second[:4])[0] == 2


def test_pointcloud_serves_a_single_point_cloud(client):
    """rawonly's fixture cloud is one point; it used to 404 on a reshape error."""
    response = client.get("/api/v1/maps/rawonly/pointcloud")

    assert response.status_code == 200
    count, points = _unpack_cloud(response.content)
    assert count == 1
    assert points.shape == (1, 3)


def test_import_message_names_no_endpoint(client):
    """The console shows the sentence as-is; operator copy names pages, not routes."""
    body = _import(client, _handmade(name="bare")).json()

    assert "/api/" not in body["message"]
    assert "Maps page" in body["message"]


def test_import_logs_its_phase_timings(client, capsys):
    """The journal, not an estimate, is what decides whether import goes async."""
    assert _import(client, _handmade(name="bare")).status_code == 201

    out = capsys.readouterr().out
    line = next(line for line in out.splitlines() if "Imported map" in line)
    for field in ("inspect_ms=", "extract_ms=", "db_ms="):
        assert field in line


# --- the robot-side 3D map ---------------------------------------------------
#
# syncai_mapping's build_octomap writes octomap.{bt,recipe.json} and the two
# layer PCDs into the map directory, from the robot container, minutes after
# the save. Nothing here starts or watches it, so every state is planted: the
# sidecar as the build writes it, the layers as PCL writes pcl::PointXYZ.

_ROAD = ((0.0, 0.0, -0.45), (0.1, 0.0, -0.45), (0.2, 0.0, -0.45), (0.3, 0.0, -0.45))
_WALLS = ((1.0, 0.0, 0.05), (1.0, 0.0, 0.15))


def _iso_ago(seconds):
    from datetime import datetime, timedelta, timezone

    stamp = datetime.now(timezone.utc) - timedelta(seconds=seconds)
    return stamp.replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _plant_octomap(directory, make_pcl_pcd, road=_ROAD, walls=_WALLS):
    make_pcl_pcd(directory / "octomap_road.pcd", points=road)
    make_pcl_pcd(directory / "octomap_occupied.pcd", points=walls)
    (directory / "octomap.bt").write_bytes(b"# Octomap OcTree binary file\n" + b"\0" * 64)


def _plant_octomap_sidecar(directory, payload):
    path = directory / "octomap.recipe.json"
    path.write_text(json.dumps(payload) if not isinstance(payload, str) else payload)
    return path


def _octomap_entry(client, name="rawonly"):
    return _by_name(client.get("/api/v1/maps").json())[name]


def test_list_reports_no_3d_map_by_default(client):
    entry = _octomap_entry(client)

    assert entry["octomap_status"] == "none"
    assert entry["octomap_error"] is None
    assert entry["octomap_resolution"] is None


def test_list_reports_ok_for_layers_without_a_sidecar(client, maps_dir, make_pcl_pcd):
    """Files on disk are the evidence, as for a gridmap with no record."""
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)

    entry = _octomap_entry(client)

    assert entry["octomap_status"] == "ok"
    assert entry["octomap_resolution"] is None


def test_list_reports_the_recorded_resolution(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "ok", "started_at": _iso_ago(300), "params": {"resolution": 0.1},
    })

    entry = _octomap_entry(client)

    assert entry["octomap_status"] == "ok"
    assert entry["octomap_resolution"] == pytest.approx(0.1)


def test_list_reports_a_fresh_build_as_converting(client, maps_dir):
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "converting", "started_at": _iso_ago(60), "params": {"resolution": 0.1},
    })

    entry = _octomap_entry(client)

    assert entry["octomap_status"] == "converting"
    assert entry["octomap_resolution"] is None
    # Independent of the gridmap's own surface.
    assert entry["grid_status"] == "none"


def test_list_reports_a_stale_build_as_interrupted(client, maps_dir):
    """No registry to ask in another container: a build still 'converting'
    after the threshold was killed, and nothing will finish it."""
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "converting",
        "started_at": _iso_ago(map_router_module.OCTOMAP_STALE_AFTER_S + 60),
    })

    assert _octomap_entry(client)["octomap_status"] == "interrupted"


def test_list_ages_a_build_without_started_at_by_the_sidecar_mtime(client, maps_dir):
    import time as time_module

    path = _plant_octomap_sidecar(maps_dir / "rawonly", {"status": "converting"})
    assert _octomap_entry(client)["octomap_status"] == "converting"

    old = time_module.time() - map_router_module.OCTOMAP_STALE_AFTER_S - 60
    os.utime(path, (old, old))

    assert _octomap_entry(client)["octomap_status"] == "interrupted"


def test_list_reports_a_failed_build_with_its_reason(client, maps_dir):
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "failed", "error": "cannot read patches/12.pcd",
    })

    entry = _octomap_entry(client)

    assert entry["octomap_status"] == "failed"
    assert entry["octomap_error"] == "cannot read patches/12.pcd"


def test_list_failed_outranks_leftover_layers(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    _plant_octomap_sidecar(maps_dir / "rawonly", {"status": "failed", "error": "boom"})

    assert _octomap_entry(client)["octomap_status"] == "failed"


def test_list_ok_record_with_a_layer_missing_is_none(client, maps_dir, make_pcl_pcd):
    """A hand-deleted layer: the route could not serve it, so do not offer it."""
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    _plant_octomap_sidecar(maps_dir / "rawonly", {"status": "ok"})
    (maps_dir / "rawonly" / "octomap_occupied.pcd").unlink()

    assert _octomap_entry(client)["octomap_status"] == "none"


def test_list_survives_a_half_written_3d_map_sidecar(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    _plant_octomap_sidecar(maps_dir / "rawonly", '{"status": "conv')

    response = client.get("/api/v1/maps")

    assert response.status_code == 200
    assert _by_name(response.json())["rawonly"]["octomap_status"] == "ok"


def test_map_detail_carries_the_3d_map_fields(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    _plant_octomap_sidecar(maps_dir / "rawonly", {"status": "ok", "params": {"resolution": 0.05}})

    body = client.get("/api/v1/maps/rawonly").json()

    assert body["octomap_status"] == "ok"
    assert body["octomap_error"] is None
    assert body["octomap_resolution"] == pytest.approx(0.05)


@pytest.mark.parametrize("layer, planted", [("road", _ROAD), ("occupied", _WALLS)])
def test_octomap_layer_is_served_packed_and_unmerged(
    client, maps_dir, make_pcl_pcd, layer, planted
):
    """Points 0.1 m apart survive: these are voxel centres, and the 0.3 m merge
    /pointcloud applies would collapse a 0.1 m floor."""
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)

    response = client.get(f"/api/v1/maps/rawonly/octomap/{layer}")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/octet-stream"
    assert len(response.content) == 4 + len(planted) * 12
    count, points = _unpack_cloud(response.content)
    assert count == len(planted)
    assert np.allclose(points, planted)


def test_octomap_layer_is_capped(client, maps_dir, make_pcl_pcd, monkeypatch):
    monkeypatch.setattr(map_router_module, "OCTOMAP_MAX_POINTS", 2)
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)

    count, _ = _unpack_cloud(client.get("/api/v1/maps/rawonly/octomap/road").content)

    assert count == 2


def test_octomap_layer_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/octomap/road").status_code == 404


def test_octomap_layer_404_when_never_built(client):
    response = client.get("/api/v1/maps/rawonly/octomap/road")

    assert response.status_code == 404
    assert response.json()["detail"] == "Map 'rawonly' has no 3D map."


def test_octomap_layer_409_while_the_robot_builds_it(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "converting", "started_at": _iso_ago(10),
    })

    response = client.get("/api/v1/maps/rawonly/octomap/road")

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_converting"


def test_octomap_layer_404_carries_the_failure(client, maps_dir):
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "failed", "error": "road layer is empty",
    })

    response = client.get("/api/v1/maps/rawonly/octomap/road")

    assert response.status_code == 404
    assert "road layer is empty" in response.json()["detail"]


def test_octomap_layer_404_when_interrupted(client, maps_dir):
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "converting",
        "started_at": _iso_ago(map_router_module.OCTOMAP_STALE_AFTER_S + 60),
    })

    response = client.get("/api/v1/maps/rawonly/octomap/occupied")

    assert response.status_code == 404
    assert "not finished" in response.json()["detail"]


def test_octomap_layer_404_when_the_pcd_is_unreadable(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    (maps_dir / "rawonly" / "octomap_road.pcd").write_text("not a pcd at all\n")

    response = client.get("/api/v1/maps/rawonly/octomap/road")

    assert response.status_code == 404
    assert "octomap_road.pcd" in response.json()["detail"]


def test_octomap_unknown_layer_is_422(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)

    assert client.get("/api/v1/maps/rawonly/octomap/walls").status_code == 422


def test_octomap_layer_is_recached_when_the_file_changes(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    first = client.get("/api/v1/maps/rawonly/octomap/road").content

    make_pcl_pcd(maps_dir / "rawonly" / "octomap_road.pcd", points=_ROAD[:1])
    second = client.get("/api/v1/maps/rawonly/octomap/road").content

    assert struct.unpack("<I", first[:4])[0] == len(_ROAD)
    assert struct.unpack("<I", second[:4])[0] == 1


def test_rename_drops_the_cached_3d_map_of_the_old_name(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    assert client.get("/api/v1/maps/rawonly/octomap/road").status_code == 200

    assert _rename(client, "rawonly", "hall").status_code == 200

    assert client.get("/api/v1/maps/rawonly/octomap/road").status_code == 404
    assert client.get("/api/v1/maps/hall/octomap/road").status_code == 200


def test_delete_drops_the_cached_3d_map(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "rawonly", make_pcl_pcd)
    assert client.get("/api/v1/maps/rawonly/octomap/occupied").status_code == 200

    assert _delete(client, "rawonly").status_code == 200

    assert client.get("/api/v1/maps/rawonly/octomap/occupied").status_code == 404


def _building(maps_dir, name="rawonly"):
    _plant_octomap_sidecar(maps_dir / name, {"status": "converting", "started_at": _iso_ago(5)})


def test_rename_refuses_while_the_robot_builds_the_3d_map(client, maps_dir):
    _building(maps_dir)

    response = _rename(client, "rawonly", "hall")

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_converting"
    assert (maps_dir / "rawonly").is_dir()
    assert not (maps_dir / "hall").exists()


def test_delete_refuses_while_the_robot_builds_the_3d_map(client, maps_dir):
    _building(maps_dir)

    response = _delete(client, "rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_converting"
    assert (maps_dir / "rawonly" / "map.pcd").is_file()


def test_import_refuses_to_replace_a_map_whose_3d_map_is_building(client, maps_dir):
    _building(maps_dir)

    response = _import(client, _handmade(), name="rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_converting"


def test_delete_goes_ahead_once_the_build_is_interrupted(client, maps_dir):
    """Nothing is coming to write into the directory any more."""
    _plant_octomap_sidecar(maps_dir / "rawonly", {
        "status": "converting",
        "started_at": _iso_ago(map_router_module.OCTOMAP_STALE_AFTER_S + 60),
    })

    assert _delete(client, "rawonly").status_code == 200
    assert not (maps_dir / "rawonly").exists()


def test_activate_does_not_wait_for_the_3d_map(client, map_gw, maps_dir, monkeypatch, tmp_path):
    """Display-only: nothing in the nav stack reads it."""
    _point_ini_at(monkeypatch, tmp_path, _INTERPOLATED_INI)
    _building(maps_dir, "full")

    response = _activate(client, "full")

    assert response.status_code == 200
    assert response.json()["switched"] is True


def test_export_leaves_the_octree_out_but_keeps_the_layers(client, maps_dir, make_pcl_pcd):
    _plant_octomap(maps_dir / "full", make_pcl_pcd)
    _plant_octomap_sidecar(maps_dir / "full", {"status": "ok", "params": {"resolution": 0.1}})

    response = _export(client, "full", "zip")

    assert response.status_code == 200
    names = zipfile.ZipFile(io.BytesIO(response.content)).namelist()
    assert "octomap.bt" not in names
    assert {"octomap_road.pcd", "octomap_occupied.pcd", "octomap.recipe.json"} <= set(names)
