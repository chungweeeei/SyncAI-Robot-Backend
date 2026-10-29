"""Tests for the map catalogue routes (/api/v1/maps).

Same router as test_map_router.py — the two URL families were merged into
routers/map.py — but kept as its own file because the fixture differs: this one
needs a tmp_path maps tree and an INI override pinning the active map.

Same shape otherwise: the router is mounted on a bare FastAPI app with the
production exception handlers registered, so the domain-exception ->
status-code mapping is the real one. Repos are real, over a tmp_path maps tree and
in-memory SQLite.
"""

import asyncio
import hashlib
import io
import json
from datetime import datetime, timedelta, timezone
import os
import shutil
import struct
import tarfile
import threading
import types
import zipfile

import pytest

pytest.importorskip("nav_msgs")
pytest.importorskip("httpx")

import numpy as np  # noqa: E402
from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.tasks.schema import TaskKind  # noqa: E402
from syncai_backend.helpers.system_config import SYSTEM_INI_ENV  # noqa: E402
from syncai_backend.interfaces.rest.routers.map import init_map_router  # noqa: E402
# The import limit is read where the import route lives.
from syncai_backend.interfaces.rest.routers.maps import archive as map_router_module  # noqa: E402
from syncai_backend.repositories.mapping.mapping import (  # noqa: E402
    MAPPING_STATUS_TTL_S,
    MappingState,
    init_mapping_status_repo,
)
from syncai_backend.services import map_switch as map_switch_module  # noqa: E402
from syncai_backend.services.map_import import MapImportService  # noqa: E402
from syncai_backend.services.map_switch import MapSwitchService  # noqa: E402
from syncai_backend.services.map_usage import JobGate, MapUsagePolicy  # noqa: E402
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)


class _StubMapGateway:
    """Records reload_octomap / save_map calls instead of making ROS service calls.

    The one thing this suite cannot make real: a LoadMap client needs a live
    Octo3D on a DDS graph, a SaveMaps client a live pgo. The repos either
    side stay real. Note save_map writes nothing — a saved map's on-disk files
    are pgo's doing, so tests that need a map.pcd create it themselves.
    """

    def __init__(self):
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
        # contract: the localizer moves before Octo3D so that the likeliest
        # failure leaves nothing changed.
        self.order = []
        self.swap_calls = []
        self.swap_result = (True, "")
        self.services_ready = True
        self.converged = True
        # Octo3D's forbidden zones (keepout.json), reloaded on the active map's save.
        self.keepout_3d_calls = []
        self.keepout_3d_result = (True, "")
        # The 3D planner (Octo3D). `octomap_results` queues one answer per call
        # (default: success), so a test can fail the load and let the rollback
        # succeed.
        self.planner_3d = True
        self.octomap_calls = []
        self.octomap_results = []
        # The kidnap recovery. `auto_localize_gate`, when set, holds the call
        # until the test releases it -- how a second press is caught mid-search.
        self.auto_localize_calls = 0
        self.auto_localize_result = (
            True,
            "matched",
            types.SimpleNamespace(
                x=1.5, y=-2.0, z=0.1, yaw=1.5707963, fitness=0.02,
                inlier_ratio=0.8, candidates=480,
            ),
        )
        self.auto_localize_entered = threading.Event()
        self.auto_localize_gate = None

    def reload_keepout_3d(self, json_path):
        self.keepout_3d_calls.append(json_path)
        self.order.append(("reload_keepout_3d", json_path))
        return self.keepout_3d_result

    def planner_3d_available(self, timeout_sec=2.0):
        return self.planner_3d

    def reload_octomap(self, bt_path):
        self.octomap_calls.append(bt_path)
        self.order.append(("reload_octomap", bt_path))
        return self.octomap_results.pop(0) if self.octomap_results else (True, "")

    def save_map(self, directory):
        self.save_calls.append(directory)
        return self.save_result

    def reset_mapping(self, reset_lio=True):
        self.reset_calls.append(reset_lio)
        return self.reset_result

    def start_mapping(self, reset_lio=True):
        self.start_calls.append(reset_lio)
        return self.start_result

    def localizer_ready(self, timeout_sec=2.0):
        return self.services_ready

    def swap_localizer_map(self, pcd_path, x, y, yaw):
        self.swap_calls.append((pcd_path, x, y, yaw))
        self.order.append(("swap_localizer_map", pcd_path))
        return self.swap_result

    def localization_converged(self, timeout_s=3.0):
        return self.converged

    def auto_localize(self):
        self.auto_localize_calls += 1
        self.auto_localize_entered.set()
        if self.auto_localize_gate is not None:
            self.auto_localize_gate.wait(timeout=5)
        return self.auto_localize_result


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
    mapping_status_repo,
    tmp_path,
    monkeypatch,
):
    """A client whose active map is 'full', set through the INI env override."""
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    catalog_repo.load_active()

    map_usage = MapUsagePolicy(
        map_catalog_repo=catalog_repo,
        task_template_repo=task_template_repo,
    )
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_map_router(
            logger=logger,
            map_repo=map_repo,
            map_catalog_repo=catalog_repo,
            map_gw=map_gw,
            task_template_repo=task_template_repo,
            mapping_status_repo=mapping_status_repo,
            map_switch_svc=MapSwitchService(
                logger=logger,
                map_catalog_repo=catalog_repo,
                map_gw=map_gw,
            ),
            map_usage=map_usage,
            job_gate=JobGate(
                logger=logger, map_catalog_repo=catalog_repo, workflow_gw=workflow_gw
            ),
            map_import_svc=MapImportService(
                logger=logger,
                map_repo=map_repo,
                map_catalog_repo=catalog_repo,
                map_usage=map_usage,
            ),
        )
    )
    return TestClient(app)


_OCTET = {"Content-Type": "application/octet-stream"}


def _by_name(body):
    return {entry["name"]: entry for entry in body}


# --- /api/v1/maps -----------------------------------------------------------


def test_list_returns_both_maps_sorted(client):
    response = client.get("/api/v1/maps")

    assert response.status_code == 200
    assert [entry["name"] for entry in response.json()] == ["full", "rawonly"]


def test_list_marks_only_the_ini_map_active(client):
    body = _by_name(client.get("/api/v1/maps").json())

    assert body["full"]["active"] is True
    assert body["rawonly"]["active"] is False


def test_list_reports_the_map_on_disk(client):
    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    assert entry["has_pointcloud"] is True
    assert entry["size_bytes"] > 0
    assert entry["modified_at"].endswith("Z")
    assert entry["octomap_status"] == "ok"


def test_list_carries_no_2d_gridmap_fields(client, maps_dir, make_pgm, make_gridmap_yaml):
    """The stack is 3D-only (2026-10-28): a map saved before that may still
    carry a gridmap, which is a file like any other now, not a field."""
    make_pgm(maps_dir / "full" / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(maps_dir / "full" / "gridmap.yaml")

    entry = _by_name(client.get("/api/v1/maps").json())["full"]

    for key in ("grid", "thumbnail", "grid_status", "grid_error", "grid_converting"):
        assert key not in entry


def test_list_counts_vertices_of_that_map_only(client, map_repo):
    map_repo.create_vertices(map="full", vertices=[
        {"name": "a", "type": "MOVE", "x": 1.0, "y": 2.0, "theta": 0.0},
        {"name": "b", "type": "CHARGE", "x": 3.0, "y": 4.0, "theta": 90.0},
    ])
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "c", "type": "MOVE", "x": 5.0, "y": 6.0, "theta": 0.0},
    ])

    body = _by_name(client.get("/api/v1/maps").json())

    assert body["full"]["vertex_count"] == 2
    assert body["rawonly"]["vertex_count"] == 1


def test_list_counts_every_map_in_one_query(client, map_repo, monkeypatch):
    """One GROUP BY for the listing, not a count per map; a map with no
    vertices (no row in the grouping) still reads 0."""

    def per_map(*args, **kwargs):
        raise AssertionError("the listing must not count map by map")

    monkeypatch.setattr(map_repo, "count_vertices", per_map)
    map_repo.create_vertices(map="full", vertices=[
        {"name": "a", "type": "MOVE", "x": 1.0, "y": 2.0, "theta": 0.0},
    ])

    body = _by_name(client.get("/api/v1/maps").json())

    assert body["full"]["vertex_count"] == 1
    assert body["rawonly"]["vertex_count"] == 0


# --- /api/v1/maps/{name} ----------------------------------------------------


@pytest.mark.parametrize("method, path", [
    ("GET", "/api/v1/maps/full/vertices"),
    ("GET", "/api/v1/maps/full/keepout"),
    ("GET", "/api/v1/maps/full/pointcloud"),
    ("GET", "/api/v1/maps/full/floors"),
    ("GET", "/api/v1/maps/full/export"),
])
def test_a_route_that_only_needs_the_map_to_exist_does_not_read_it_whole(
    client, catalog_repo, monkeypatch, method, path
):
    """get_map walks every file of the map (hundreds under patches/) and opens
    its sidecars; these routes only refused an unknown name with it, then read
    what they needed themselves -- a 304 paid that walk on every poll."""
    monkeypatch.setattr(
        catalog_repo, "get_map", lambda name: pytest.fail(f"{path} read the whole map")
    )

    response = client.request(method, path)

    assert response.status_code < 500, response.text


@pytest.mark.parametrize("path", [
    "/api/v1/maps/nosuchmap/vertices",
    "/api/v1/maps/nosuchmap/keepout",
    "/api/v1/maps/nosuchmap/export",
])
def test_an_unknown_map_is_still_the_same_404(client, path):
    response = client.get(path)

    assert response.status_code == 404
    assert response.json()["detail"] == "No map named 'nosuchmap' on this robot."


def test_get_returns_one_summary(client):
    response = client.get("/api/v1/maps/full")

    assert response.status_code == 200
    assert response.json()["name"] == "full"


def test_get_missing_map_returns_404(client):
    assert client.get("/api/v1/maps/nosuchmap").status_code == 404


def test_get_unsafe_name_returns_400(client):
    assert client.get("/api/v1/maps/with%20space").status_code == 400


# --- /api/v1/maps/{name}/keepout --------------------------------------------


def _rect_zone(zone_id=None, x=(0.0, 1.0), y=(0.0, 1.0)):
    """A rectangle in map-frame metres."""
    zone = {
        "points": [
            {"x": x[0], "y": y[0]},
            {"x": x[1], "y": y[0]},
            {"x": x[1], "y": y[1]},
            {"x": x[0], "y": y[1]},
        ]
    }
    if zone_id is not None:
        zone["id"] = zone_id
    return zone


def _put_keepout(client, name, zones):
    return client.put(f"/api/v1/maps/{name}/keepout", json={"zones": zones})


def test_keepout_get_is_empty_for_a_map_never_drawn_on(client):
    body = client.get("/api/v1/maps/full/keepout").json()

    assert body == {"name": "full", "zones": [], "active": True}


def test_keepout_get_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/keepout").status_code == 404


def test_keepout_get_400_for_an_unsafe_name(client):
    assert client.get("/api/v1/maps/..%2Ffull/keepout").status_code in (400, 404)


def test_keepout_put_then_get_round_trips_in_order(client):
    zones = [_rect_zone("north"), _rect_zone("south", x=(4.0, 5.0), y=(3.0, 3.5))]

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


def test_keepout_put_writes_the_json_alone(client, maps_dir):
    """No keepout.pgm / keepout.yaml: the 2D mask went with the 2D stack."""
    response = _put_keepout(client, "full", [_rect_zone()])

    assert response.status_code == 200
    directory = maps_dir / "full"
    assert json.loads((directory / "keepout.json").read_text())["version"] == 1
    assert not (directory / "keepout.pgm").exists()
    assert not (directory / "keepout.yaml").exists()


def test_keepout_put_reloads_the_3d_planner_on_the_active_map(client, map_gw):
    body = _put_keepout(client, "full", [_rect_zone()]).json()

    assert body["active"] is True
    assert body["reloaded"] is True
    assert "reloaded the 3D planner" in body["message"]
    assert len(map_gw.keepout_3d_calls) == 1
    called = map_gw.keepout_3d_calls[0]
    assert called.endswith("full/keepout.json")
    assert called.startswith("/")
    assert "~" not in called
    # The tree itself is not rebuilt for a zone edit.
    assert map_gw.octomap_calls == []


def test_keepout_put_does_not_reload_an_inactive_map(client, map_gw, maps_dir, make_bt):
    make_bt(maps_dir / "rawonly" / "octomap.bt")

    body = _put_keepout(client, "rawonly", [_rect_zone()]).json()

    assert body["active"] is False
    assert body["reloaded"] is False
    assert map_gw.keepout_3d_calls == []
    assert (maps_dir / "rawonly" / "keepout.json").is_file()


def test_keepout_put_reports_a_failed_reload_without_failing_the_save(client, map_gw):
    """The zones are on disk, so a 5xx would be a lie the operator acts on."""
    map_gw.keepout_3d_result = (False, "the 3D planner has no 3D map loaded")

    response = _put_keepout(client, "full", [_rect_zone("z")])

    assert response.status_code == 200
    body = response.json()
    assert body["active"] is True
    assert body["reloaded"] is False
    assert "the 3D planner has no 3D map loaded" in body["message"]
    assert client.get("/api/v1/maps/full/keepout").json()["zones"][0]["id"] == "z"


def test_keepout_put_empty_clears_the_zones(client, maps_dir, map_gw):
    _put_keepout(client, "full", [_rect_zone()])

    body = _put_keepout(client, "full", []).json()

    assert body["zones"] == []
    assert body["reloaded"] is True
    assert "Saved 0 forbidden zones" in body["message"]
    # Rewritten with none, never deleted.
    assert json.loads((maps_dir / "full" / "keepout.json").read_text())["zones"] == []
    assert len(map_gw.keepout_3d_calls) == 2


@pytest.mark.parametrize(
    "zones,fragment",
    [
        ([{"points": [{"x": 0, "y": 0}, {"x": 1, "y": 1}]}], "at least 3"),
        ([_rect_zone("dup"), _rect_zone("dup")], "more than once"),
        ([_rect_zone("")], "empty id"),
    ],
)
def test_keepout_put_400s_a_bad_zone_with_a_sentence(client, maps_dir, map_gw, zones, fragment):
    response = _put_keepout(client, "full", zones)

    assert response.status_code == 400
    assert fragment in response.json()["detail"]
    assert not (maps_dir / "full" / "keepout.json").exists()
    assert map_gw.keepout_3d_calls == []


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
    assert map_gw.keepout_3d_calls == []


def test_keepout_put_404_when_the_map_has_no_3d_map(client, maps_dir):
    response = _put_keepout(client, "rawonly", [_rect_zone()])

    assert response.status_code == 404
    assert "octomap.bt" in response.json()["detail"]
    assert not (maps_dir / "rawonly" / "keepout.json").exists()


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


# --- octomap_status -----------------------------------------------------------
#
# The 3D map's status surface, read off disk alone: the
# tree is the robot's clean_map's to write, a detached process this backend
# never sees, so there is no registry to reconcile with.


def _plant_clean(directory, payload):
    """Write a map cleaning record, as pgo_node and clean_map would."""
    path = directory / "map_clean.recipe.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _iso(moment):
    return moment.isoformat(timespec="seconds").replace("+00:00", "Z")


def _now_iso():
    return _iso(datetime.now(timezone.utc))


def _old_iso():
    """Older than the stale threshold: a clean that was killed hard."""
    return _iso(datetime.now(timezone.utc) - timedelta(hours=1))


def _clean_payload(
    status,
    *,
    octomap=True,
    written=None,
    octomap_error=None,
    started_at=None,
    error=None,
):
    payload = {
        "status": status,
        "started_at": _now_iso() if started_at is None else started_at,
        "params": {"voxel_size": 0.1, "octomap": octomap},
    }
    if status == "ok":
        measurements = {"octomap_written": bool(written)}
        if octomap_error is not None:
            measurements["octomap_error"] = octomap_error
        payload["measurements"] = measurements
    if error is not None:
        payload["error"] = error
    return payload


def _entry(client, name="full"):
    return _by_name(client.get("/api/v1/maps").json())[name]


def test_list_reports_no_3d_map_for_a_map_without_one(client):
    entry = _entry(client, "rawonly")

    assert entry["octomap_status"] == "none"
    assert entry["octomap_error"] is None
    assert entry["octomap_version"] is None
    assert entry["octomap_resolution"] is None


def test_list_reports_ok_with_the_version_and_resolution_of_a_tree_on_disk(
    client, maps_dir, make_bt
):
    tree = make_bt(maps_dir / "full" / "octomap.bt", res=0.1)

    entry = _entry(client)

    assert entry["octomap_status"] == "ok"
    assert entry["octomap_error"] is None
    assert entry["octomap_version"] == f"{tree.stat().st_size}-{tree.stat().st_mtime_ns}"
    assert entry["octomap_resolution"] == pytest.approx(0.1)


def test_a_tree_on_disk_is_ok_whatever_the_sidecar_says(client, maps_dir, make_bt):
    """A re-clean leaves the old tree in place until the new one lands."""
    make_bt(maps_dir / "full" / "octomap.bt")
    _plant_clean(maps_dir / "full", _clean_payload("converting"))

    assert _entry(client)["octomap_status"] == "ok"


def test_list_reports_converting_while_the_clean_runs(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("converting"))

    entry = _entry(client, "rawonly")

    assert entry["octomap_status"] == "converting"
    assert entry["octomap_version"] is None


def test_list_reports_converting_for_a_clean_with_no_start_time(client, maps_dir):
    """Nothing to age it by; the timeout is the conversion's problem, not this field's."""
    payload = _clean_payload("converting")
    del payload["started_at"]
    _plant_clean(maps_dir / "rawonly", payload)

    assert _entry(client, "rawonly")["octomap_status"] == "converting"


def test_list_reports_interrupted_for_a_clean_that_never_finished(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("converting", started_at=_old_iso()))

    assert _entry(client, "rawonly")["octomap_status"] == "interrupted"


def test_list_reports_failed_with_the_reason_when_the_tree_was_dropped(client, maps_dir):
    """map.pcd was cleaned; the tree ran out of memory and was not written."""
    _plant_clean(
        maps_dir / "rawonly",
        _clean_payload("ok", written=False, octomap_error="out of memory"),
    )

    entry = _entry(client, "rawonly")

    assert entry["octomap_status"] == "failed"
    assert entry["octomap_error"] == "out of memory"


def test_list_reports_failed_with_a_sentence_when_the_sidecar_has_none(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("ok", written=False))

    entry = _entry(client, "rawonly")

    assert entry["octomap_status"] == "failed"
    assert entry["octomap_error"]


def test_list_reports_failed_with_the_cleans_reason_when_the_clean_failed(
    client, maps_dir
):
    _plant_clean(maps_dir / "rawonly", _clean_payload("failed", error="no readable patch"))

    entry = _entry(client, "rawonly")

    assert entry["octomap_status"] == "failed"
    assert entry["octomap_error"] == "no readable patch"


def test_list_reports_none_when_the_clean_never_asked_for_a_tree(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("failed", octomap=False, error="x"))
    assert _entry(client, "rawonly")["octomap_status"] == "none"

    _plant_clean(maps_dir / "rawonly", _clean_payload("converting", octomap=False))
    assert _entry(client, "rawonly")["octomap_status"] == "none"


def test_list_reports_none_for_a_written_tree_that_is_gone(client, maps_dir):
    """A save into the same directory deletes the previous tree first."""
    _plant_clean(maps_dir / "rawonly", _clean_payload("ok", written=True))

    assert _entry(client, "rawonly")["octomap_status"] == "none"


def test_list_survives_a_torn_clean_sidecar(client, maps_dir):
    (maps_dir / "rawonly" / "map_clean.recipe.json").write_text('{"status": "conv')

    assert _entry(client, "rawonly")["octomap_status"] == "none"


# --- GET /api/v1/maps/{name}/octomap ------------------------------------------


def test_octomap_serves_the_tree_byte_for_byte(client, maps_dir, make_bt):
    tree = make_bt(maps_dir / "full" / "octomap.bt")

    response = client.get("/api/v1/maps/full/octomap")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/octet-stream"
    assert response.content == tree.read_bytes()


def test_octomap_etag_is_the_catalogues_version(client, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt")
    version = _entry(client)["octomap_version"]

    response = client.get("/api/v1/maps/full/octomap")

    assert response.headers["etag"] == f'"{version}"'
    assert response.headers["cache-control"] == "no-cache"


def test_octomap_revalidates_to_304(client, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt")
    tag = client.get("/api/v1/maps/full/octomap").headers["etag"]

    response = client.get("/api/v1/maps/full/octomap", headers={"If-None-Match": tag})

    assert response.status_code == 304
    assert response.content == b""


def test_octomap_etag_follows_a_replaced_file(client, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt", body=b"\x02\x00")
    first = client.get("/api/v1/maps/full/octomap").headers["etag"]

    make_bt(maps_dir / "full" / "octomap.bt", body=b"\x02\x00\x02\x00")
    response = client.get("/api/v1/maps/full/octomap", headers={"If-None-Match": first})

    assert response.status_code == 200
    assert response.headers["etag"] != first
    assert response.content == (maps_dir / "full" / "octomap.bt").read_bytes()


def test_octomap_404_for_a_map_without_one(client):
    response = client.get("/api/v1/maps/rawonly/octomap")

    assert response.status_code == 404
    assert "no 3D map" in response.json()["detail"]


def test_octomap_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/octomap").status_code == 404


def test_octomap_409_while_the_clean_builds_it(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("converting"))

    response = client.get("/api/v1/maps/rawonly/octomap")

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_converting"


def test_octomap_404_carries_the_cleans_reason(client, maps_dir):
    _plant_clean(
        maps_dir / "rawonly",
        _clean_payload("ok", written=False, octomap_error="out of memory"),
    )

    response = client.get("/api/v1/maps/rawonly/octomap")

    assert response.status_code == 404
    assert "out of memory" in response.json()["detail"]


def test_octomap_404_for_an_interrupted_build(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("converting", started_at=_old_iso()))

    response = client.get("/api/v1/maps/rawonly/octomap")

    assert response.status_code == 404
    assert "never finished" in response.json()["detail"]


# --- /api/v1/maps/{name}/octomap/edits ----------------------------------------
#
# The ground patches. A record beside the tree, applied to it on the robot;
# the backend writes the file and nothing else.


def _put_edits(client, name, keys):
    return client.put(f"/api/v1/maps/{name}/octomap/edits", json={"fill_key": keys})


def test_octomap_edits_get_is_empty_for_a_map_never_patched(client):
    response = client.get("/api/v1/maps/full/octomap/edits")

    assert response.status_code == 200
    assert response.json() == {
        "name": "full",
        "res": None,
        "fill_key": [],
        "created_at": None,
    }


def test_octomap_edits_get_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/octomap/edits").status_code == 404


def test_octomap_edits_put_then_get_round_trips(client, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt", res=0.1)
    keys = [[32770, 32768, 32763], [32771, 32768, 32763]]

    response = _put_edits(client, "full", keys)

    assert response.status_code == 200
    assert response.json()["count"] == 2
    assert "2 ground patches" in response.json()["message"]
    body = client.get("/api/v1/maps/full/octomap/edits").json()
    assert body["fill_key"] == keys
    assert body["res"] == pytest.approx(0.1)
    assert body["created_at"].endswith("Z")


def test_octomap_edits_put_writes_the_viewers_document(client, maps_dir, make_bt):
    """The file is what the robot's apply tool reads; its shape is the contract."""
    make_bt(maps_dir / "full" / "octomap.bt", res=0.1)

    _put_edits(client, "full", [[32770, 32768, 32763]])

    document = json.loads((maps_dir / "full" / "octomap_edits.json").read_text())
    assert document["format"] == "syncai-octomap-edits"
    assert document["version"] == 1
    # The tree's resolution, read off its header -- the body carries none.
    assert document["res"] == pytest.approx(0.1)
    assert document["fill_key"] == [[32770, 32768, 32763]]
    assert document["fill_xyz"] == [[0.25, 0.05, -0.45]]
    assert not [p.name for p in (maps_dir / "full").iterdir() if p.name.endswith(".tmp")]


def test_octomap_edits_put_empty_clears(client, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt")
    _put_edits(client, "full", [[1, 2, 3]])

    response = _put_edits(client, "full", [])

    assert response.status_code == 200
    assert response.json()["count"] == 0
    assert "Cleared" in response.json()["message"]
    assert client.get("/api/v1/maps/full/octomap/edits").json()["fill_key"] == []


def test_octomap_edits_put_404_without_a_tree(client, maps_dir):
    response = _put_edits(client, "rawonly", [[1, 2, 3]])

    assert response.status_code == 404
    assert not (maps_dir / "rawonly" / "octomap_edits.json").exists()


def test_octomap_edits_put_409_while_the_tree_builds(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("converting"))

    response = _put_edits(client, "rawonly", [[1, 2, 3]])

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_converting"


def test_octomap_edits_put_409_for_a_tree_without_a_readable_header(client, maps_dir):
    """The keys are meaningless without the tree's resolution."""
    (maps_dir / "full" / "octomap.bt").write_bytes(b"not a tree")

    response = _put_edits(client, "full", [[1, 2, 3]])

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_unreadable"
    assert not (maps_dir / "full" / "octomap_edits.json").exists()


@pytest.mark.parametrize(
    "keys, fragment",
    [
        ([[1, 2, 70000]], "outside the 3D map"),
        ([[1, 2, 3], [1, 2, 9]], "same column"),
    ],
)
def test_octomap_edits_put_400s_a_bad_key_with_a_sentence(
    client, maps_dir, make_bt, keys, fragment
):
    make_bt(maps_dir / "full" / "octomap.bt")

    response = _put_edits(client, "full", keys)

    assert response.status_code == 400
    assert fragment in response.json()["detail"]
    assert not (maps_dir / "full" / "octomap_edits.json").exists()


def test_octomap_edits_put_422s_a_key_that_is_not_three_ints(client, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt")

    assert _put_edits(client, "full", [[1, 2]]).status_code == 422
    assert _put_edits(client, "full", [[1, 2, "x"]]).status_code == 422


def test_octomap_edits_put_refuses_while_a_job_drives_on_the_map(
    client, maps_dir, make_bt, workflow_gw
):
    make_bt(maps_dir / "full" / "octomap.bt")
    workflow_gw.tasks = [types.SimpleNamespace(id="task-7", map_in_use=lambda active: "full")]

    response = _put_edits(client, "full", [[1, 2, 3]])

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert not (maps_dir / "full" / "octomap_edits.json").exists()


def test_octomap_edits_put_refuses_when_temporal_cannot_be_asked(
    client, maps_dir, make_bt, workflow_gw
):
    make_bt(maps_dir / "full" / "octomap.bt")
    workflow_gw.error = RuntimeError("temporal down")

    response = _put_edits(client, "full", [[1, 2, 3]])

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"


def test_octomap_edits_put_on_an_idle_map_does_not_ask_temporal(
    client, maps_dir, make_bt, workflow_gw
):
    make_bt(maps_dir / "rawonly" / "octomap.bt")
    workflow_gw.error = RuntimeError("temporal down")

    assert _put_edits(client, "rawonly", [[1, 2, 3]]).status_code == 200


# --- POST /api/v1/maps --------------------------------------------------------


def _post_map(client, payload):
    return client.post("/api/v1/maps", json=payload)


def test_create_map_saves_through_the_gateway(client, map_gw, catalog_repo):
    response = _post_map(client, {"name": "newmap"})

    assert response.status_code == 200
    body = response.json()
    assert body["name"] == "newmap"
    assert body["has_pointcloud"] is True
    assert "grid_pending" not in body
    assert "3D map" in body["message"]
    # The operator's sentence names no route.
    assert "/api/" not in body["message"]
    # The gateway got the directory this router created, absolute.
    directory = catalog_repo.resolve_dir("newmap")
    assert map_gw.save_calls == [directory]
    assert os.path.isdir(directory)


def test_create_map_ignores_the_2d_era_build_choices(client, map_gw):
    """An older console still sends `grid` / `octomap`; the robot always
    builds the 3D map, and nothing refuses a body that asks for neither."""
    response = _post_map(client, {"name": "newmap", "grid": False, "octomap": False})

    assert response.status_code == 200
    assert len(map_gw.save_calls) == 1


def test_create_map_lists_afterwards_while_the_3d_map_builds(client, maps_dir, make_pcd):
    _post_map(client, {"name": "newmap"})
    # Stand in for pgo: the stub gateway does not write files.
    make_pcd(maps_dir / "newmap" / "map.pcd")

    entry = _by_name(client.get("/api/v1/maps").json())["newmap"]

    assert entry["has_pointcloud"] is True
    assert entry["octomap_status"] == "none"


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
        {"name": "dock", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "home", "type": "MOVE", "x": 1.0, "y": 0.0, "theta": 0.0},
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
    assert (maps_dir / "full" / "octomap.bt").is_file()
    assert not (maps_dir / "hall").exists()


def test_rename_refuses_a_taken_name(client, maps_dir):
    response = _rename(client, "rawonly", "full")

    assert response.status_code == 409
    assert response.json()["code"] == "name_taken"
    assert (maps_dir / "rawonly").is_dir()
    assert (maps_dir / "full" / "octomap.bt").is_file()


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
        {"name": "dock", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "home", "type": "MOVE", "x": 1.0, "y": 0.0, "theta": 0.0},
    ])
    # A vertex on another map must survive: the DELETE is filtered by map.
    map_repo.create_vertices(map="full", vertices=[
        {"name": "charger", "type": "CHARGE", "x": 2.0, "y": 0.0, "theta": 0.0},
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
    assert (maps_dir / "full" / "octomap.bt").is_file()


def test_delete_refuses_while_a_task_template_is_bound(
    client, maps_dir, map_repo, task_template_repo
):
    """The one refusal that asks for work rather than a restart."""
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "dock", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
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


# --- POST /api/v1/maps/{name}/activate --------------------------------------
#
# The fixture INI pins `full` active, and `full` is the only fixture map with a
# 3D map (octomap.bt), so most switch tests point the INI at `rawonly` first and
# switch *to* `full`. Each test writes a real INI and reloads the catalogue from it (the
# robot booting on that file), since the switch rewrites that file in place and
# the tests read back what it wrote.

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


def _ini_naming(name):
    return f"[system]\nrobot_id: robot01\n\n[map]\nname: {name}\n"


def _point_ini_at(catalog_repo, monkeypatch, tmp_path, text):
    ini = tmp_path / "system.ini"
    ini.write_text(text)
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    # The repo read its INI when it was built; this is the robot booting on it.
    catalog_repo.load_active()
    return ini


def _activate(client, name):
    return client.post(f"/api/v1/maps/{name}/activate")


def _off_the_loop(calls, label, real):
    """Wrap ``real`` to record whether it ran on a thread with a running event
    loop (the uvicorn loop, in production) or in a worker thread."""

    def wrapper(*args, **kwargs):
        try:
            asyncio.get_running_loop()
            calls.append((label, "ON THE EVENT LOOP"))
        except RuntimeError:
            calls.append((label, "worker thread"))
        return real(*args, **kwargs)

    return wrapper


@pytest.mark.parametrize("name, expected_status", [("full", 200), ("nosuchmap", 404)])
def test_activate_reads_the_map_and_the_ini_off_the_event_loop(
    client, catalog_repo, monkeypatch, name, expected_status
):
    """activate_map is async (it awaits Temporal). plan() stats the map's files
    and reads the INI; on the loop, a slow eMMC froze
    teleop and the telemetry streams for that long. 'full' is the active map
    (the no-op answer), 'nosuchmap' the 404 raised from the worker thread."""
    calls = []
    monkeypatch.setattr(
        catalog_repo, "exists", _off_the_loop(calls, "exists", catalog_repo.exists)
    )
    monkeypatch.setattr(
        catalog_repo,
        "active_name",
        _off_the_loop(calls, "active_name", catalog_repo.active_name),
    )
    monkeypatch.setattr(
        map_switch_module.MapSwitchService,
        "plan",
        _off_the_loop(calls, "plan", map_switch_module.MapSwitchService.plan),
    )

    response = _activate(client, name)

    assert response.status_code == expected_status, response.text
    assert calls, "nothing was read"
    assert all(where == "worker thread" for _, where in calls), calls


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


def test_activate_refuses_a_map_with_no_3d_map(
    catalog_repo, client, map_gw, monkeypatch, tmp_path
):
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _ini_naming("full"))

    response = _activate(client, "rawonly")

    assert response.status_code == 409
    assert response.json()["code"] == "octomap_missing"
    assert map_gw.order == []


def test_activate_refuses_a_map_with_no_pointcloud(
    client, map_gw, maps_dir, make_bt, monkeypatch, tmp_path
):
    treeonly = maps_dir / "treeonly"
    treeonly.mkdir()
    make_bt(treeonly / "octomap.bt")

    response = _activate(client, "treeonly")

    assert response.status_code == 409
    assert response.json()["code"] == "pointcloud_missing"
    assert map_gw.order == []


def test_activate_refuses_while_a_task_is_running(
    catalog_repo, client, map_gw, workflow_gw, monkeypatch, tmp_path
):
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _ini_naming("rawonly"))
    workflow_gw.tasks = [types.SimpleNamespace(id="task-7")]

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "task-7" in response.json()["detail"]
    assert map_gw.order == []


def test_activate_refuses_when_temporal_cannot_be_reached(
    catalog_repo, client, map_gw, workflow_gw, monkeypatch, tmp_path
):
    """A Temporal outage must not be read as "nothing is running"."""
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _ini_naming("rawonly"))
    workflow_gw.error = RuntimeError("connection refused")

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert map_gw.order == []


# --- The map a running job holds ----------------------------------------------


def _job(map_name=None, kind=None):
    from datetime import datetime, timezone

    from syncai_backend.tasks.schema import ActiveTask, TaskSource

    return ActiveTask(
        id="robot01-task-9",
        run_id="run-9",
        status="IN_PROGRESS",
        started_at=datetime(2026, 10, 4, tzinfo=timezone.utc),
        source=TaskSource.DIRECT,
        kind=kind,
        map_name=map_name,
    )


# Every edit the planner or a job's MOVE steps read from, each on 'full' -- the
# client fixture's loaded map.
_EDITS = {
    "forbidden zones": lambda client: _put_keepout(client, "full", [_rect_zone()]),
    "ground patches": lambda client: _put_edits(client, "full", [[32768, 32768, 32768]]),
}


@pytest.mark.parametrize("edit", list(_EDITS))
def test_an_edit_waits_for_the_job_driving_on_the_map(client, map_gw, workflow_gw, edit):
    workflow_gw.tasks = [_job(map_name="full")]

    response = _EDITS[edit](client)

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "running a job on this map" in response.json()["detail"]
    # Refused before anything reached the running stack.
    assert map_gw.keepout_3d_calls == []


def test_a_run_older_than_the_stamp_holds_the_loaded_map(client, map_gw, workflow_gw):
    workflow_gw.tasks = [_job(map_name=None, kind=None)]

    response = _put_keepout(client, "full", [_rect_zone()])

    assert response.json()["code"] == "task_running"


def test_a_job_that_drives_nowhere_does_not_hold_the_map(
    client, map_gw, workflow_gw
):
    workflow_gw.tasks = [_job(map_name=None, kind=TaskKind.STANDUP)]

    assert _put_keepout(client, "full", [_rect_zone()]).status_code == 200


def test_an_edit_is_refused_when_running_jobs_cannot_be_known(
    client, map_gw, workflow_gw
):
    workflow_gw.error = RuntimeError("connection refused")

    response = _put_keepout(client, "full", [_rect_zone()])

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert map_gw.keepout_3d_calls == []


def test_a_map_that_is_not_loaded_is_edited_without_asking(
    client, maps_dir, workflow_gw, make_bt
):
    # A running job can only hold the loaded map (activate refuses while one
    # runs), so an outage of the task service must not block this edit.
    make_bt(maps_dir / "rawonly" / "octomap.bt")
    workflow_gw.error = RuntimeError("connection refused")

    assert _put_keepout(client, "rawonly", [_rect_zone()]).status_code == 200


def test_activate_refuses_when_the_nav_stack_is_not_up(
    catalog_repo, client, map_gw, monkeypatch, tmp_path
):
    """How "the robot is in mapping mode" is detected."""
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _ini_naming("rawonly"))
    map_gw.services_ready = False

    response = _activate(client, "full")

    assert response.status_code == 409
    assert response.json()["code"] == "stack_not_ready"
    assert map_gw.order == []


def test_activate_moves_the_localizer_before_the_3d_planner(
    catalog_repo, client, map_gw, maps_dir, monkeypatch, tmp_path
):
    ini = _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)

    response = _activate(client, "full")

    assert response.status_code == 200
    body = response.json()
    assert body["switched"] is True
    assert body["name"] == "full"
    assert body["previous"] == "rawonly"
    assert body["localized"] is True
    # Everything that asks which map is loaded now hears the new one.
    assert catalog_repo.active_name() == "full"

    # The ordering is the contract: the localizer's failure is the clean one, so
    # it goes first.
    # No keepout call: Octo3D read the map's keepout.json with the tree.
    assert [step for step, _ in map_gw.order] == ["swap_localizer_map", "reload_octomap"]
    assert map_gw.order[0][1] == str(maps_dir / "full" / "map.pcd")
    assert map_gw.order[1][1] == str(maps_dir / "full" / "octomap.bt")
    # Zeroed, not carried over from the old map's frame.
    assert map_gw.swap_calls[0][1:] == (0.0, 0.0, 0.0)

    written = ini.read_text()
    assert "name: full" in written
    # The whole reason the writer edits lines instead of round-tripping through
    # ConfigParser: expanding this would pin pcd/map at the *old* map forever.
    # (`map:` is a 2D-era key nothing reads now; the writer leaves it alone.)
    assert "pcd: map/%(name)s/map.pcd" in written
    assert "map: map/%(name)s/gridmap.yaml" in written
    assert "# which map the stack loads" in written
    # A pose measured in rawonly's frame means nothing in full's.
    assert "x: 0.0" in written and "yaw: 0.0" in written
    assert "3.25" not in written


def test_activate_rolls_back_both_steps_when_the_ini_cannot_be_written(
    catalog_repo, client, map_gw, maps_dir, make_pcd, make_bt, monkeypatch, tmp_path,
):
    """The INI write is preflighted, so reaching it and failing must undo the rest.

    `previous` here is a navigable map, unlike the tree-less `rawonly` the
    other switch tests come from: a map the stack actually launched on has its
    octomap.bt, and it is the rollback of *both* ROS steps that this covers.
    """
    prior = maps_dir / "prior"
    prior.mkdir()
    make_pcd(prior / "map.pcd")
    make_bt(prior / "octomap.bt")
    _point_ini_at(
        catalog_repo,
        monkeypatch,
        tmp_path,
        _INTERPOLATED_INI.replace("name: rawonly", "name: prior"),
    )

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(
        "syncai_backend.repositories.map.catalog.set_active_map", _boom
    )

    response = _activate(client, "full")

    assert response.status_code == 502
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_octomap",
        "swap_localizer_map",
        "reload_octomap",
    ]
    assert map_gw.order[2][1] == str(prior / "map.pcd")
    assert map_gw.order[3][1] == str(prior / "octomap.bt")
    # Nothing was recorded, so the loaded map is still the one the INI names.
    assert catalog_repo.active_name() == "prior"


def test_activate_refuses_when_the_ini_is_not_writable(
    catalog_repo, client, map_gw, monkeypatch, tmp_path
):
    ini = _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)
    os.chmod(ini, 0o444)
    try:
        response = _activate(client, "full")
    finally:
        os.chmod(ini, 0o644)

    assert response.status_code == 409
    assert response.json()["code"] == "ini_not_writable"
    assert map_gw.order == []


def test_activate_reports_an_unconverged_localizer(
    catalog_repo, client, map_gw, monkeypatch, tmp_path
):
    """A swap that lands but does not converge is a success with a warning."""
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.converged = False

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert body["localized"] is False
    assert "set an initial pose" in body["message"]


def test_activate_reports_an_unreachable_localizer_check(
    catalog_repo, client, map_gw, monkeypatch, tmp_path
):
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.converged = None

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert body["localized"] is None
    assert "could not be asked" in body["message"]


def test_activate_names_the_zones_the_3d_planner_read_with_the_tree(
    catalog_repo, client, map_gw, maps_dir, monkeypatch, tmp_path
):
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)
    _put_keepout(client, "full", [_rect_zone("a"), _rect_zone("b", x=(5.0, 6.0))])

    body = _activate(client, "full").json()

    assert body["switched"] is True
    assert "Its 2 forbidden zones are active." in body["message"]
    # Drawn while inactive, so never sent; the load_map read them.
    assert map_gw.keepout_3d_calls == []
    for key in ("map_kind", "keepout_reloaded"):
        assert key not in body


# --- keepout: floor bands (2026-10-23) --------------------------------------------
#
# Each zone may carry a name and a z band (the floor it forbids); the robot's 3D
# planner reads keepout.json itself.


def _banded(zone, name="rack", z_min=-0.5, z_max=0.5):
    return {**zone, "name": name, "z_min": z_min, "z_max": z_max}


def test_keepout_round_trips_name_and_band_and_omits_unset_ones(client, maps_dir):
    zones = [_banded(_rect_zone("a")), _rect_zone("b")]
    saved = _put_keepout(client, "full", zones).json()
    fetched = client.get("/api/v1/maps/full/keepout").json()

    assert saved["zones"] == fetched["zones"]
    assert fetched["zones"][0]["name"] == "rack"
    assert fetched["zones"][0]["z_min"] == -0.5
    assert fetched["zones"][0]["z_max"] == 0.5
    assert fetched["zones"][1]["z_min"] is None
    # On disk an unset key is absent.
    document = json.loads((maps_dir / "full" / "keepout.json").read_text())
    assert set(document["zones"][0]) == {"id", "points", "name", "z_min", "z_max"}
    assert set(document["zones"][1]) == {"id", "points"}


@pytest.mark.parametrize(
    "band,fragment",
    # Raw JSON text: 1e999 parses to inf, which no client library will encode
    # for us on purpose (and Python would print it as the non-JSON "inf").
    [
        ('"z_min": 2.0, "z_max": 1.0', "above z_max"),
        ('"z_min": null, "z_max": 1e999', "not a finite height"),
    ],
)
def test_keepout_put_400s_a_bad_band(client, maps_dir, band, fragment):
    body = json.dumps({"zones": [_rect_zone("a")]})[:-3]  # strip "}]}" to splice the band in
    response = client.put(
        "/api/v1/maps/full/keepout",
        content=body + ", " + band + "}]}",
        headers={"Content-Type": "application/json"},
    )

    assert response.status_code == 400
    assert fragment in response.json()["detail"]
    assert not (maps_dir / "full" / "keepout.json").exists()


def test_keepout_on_an_inactive_map_keeps_its_bands(client, maps_dir, make_pcd, make_bt):
    directory = maps_dir / "three"
    directory.mkdir()
    make_pcd(directory / "map.pcd")
    make_bt(directory / "octomap.bt")

    response = _put_keepout(client, "three", [_banded(_rect_zone("a"), z_min=4.5, z_max=5.5)])

    assert response.status_code == 200
    assert response.json()["reloaded"] is False  # not the active map
    assert json.loads((directory / "keepout.json").read_text())["zones"][0]["z_min"] == 4.5
    assert not (directory / "keepout.pgm").exists()
    assert not (directory / "keepout.yaml").exists()
    assert client.get("/api/v1/maps/three/keepout").json()["zones"][0]["z_max"] == 5.5


# --- activate: the 3D planner (Octo3D) ---------------------------------------------
#
# Every map is loaded into the robot's 3D planner, Octo3D, which every goal plans
# on (2026-10-28: the stack is 3D-only). A failed load puts the previous map's
# tree back, or none when the previous map had none.


@pytest.fixture
def three_d(maps_dir, make_pcd, make_bt):
    """A second navigable map: map.pcd + octomap.bt."""
    directory = maps_dir / "three"
    directory.mkdir()
    make_pcd(directory / "map.pcd")
    make_bt(directory / "octomap.bt")
    return directory


def test_activate_refuses_on_a_stack_without_the_3d_planner(
    catalog_repo, client, map_gw, three_d, monkeypatch, tmp_path
):
    ini = _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.planner_3d = False

    response = _activate(client, "three")

    assert response.status_code == 409
    assert response.json()["code"] == "planner_3d_unavailable"
    assert map_gw.order == []
    assert "name: rawonly" in ini.read_text()


def test_activate_rolls_back_when_the_3d_planner_will_not_load(
    catalog_repo, client, map_gw, maps_dir, three_d, make_bt, monkeypatch, tmp_path
):
    """Octo3D drops its map before building, so a failed load must put the old one back."""
    make_bt(maps_dir / "rawonly" / "octomap.bt")
    ini = _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.octomap_results = [(False, "the 3D planner could not read octomap.bt")]

    response = _activate(client, "three")

    assert response.status_code == 502
    assert "could not read octomap.bt" in response.json()["detail"]
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_octomap",
        "swap_localizer_map",
        "reload_octomap",
    ]
    assert map_gw.order[2][1] == str(maps_dir / "rawonly" / "map.pcd")
    # Back onto the previous map's own tree, not left without one.
    assert map_gw.order[3][1] == str(maps_dir / "rawonly" / "octomap.bt")
    assert "name: rawonly" in ini.read_text()


def test_activate_rolls_the_3d_planner_back_to_none_for_a_previous_map_without_one(
    catalog_repo, client, map_gw, three_d, monkeypatch, tmp_path
):
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(
        "syncai_backend.repositories.map.catalog.set_active_map", _boom
    )

    response = _activate(client, "three")

    assert response.status_code == 502
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_octomap",
        "swap_localizer_map",
        "reload_octomap",
    ]
    # rawonly has no tree: the planner goes back to having none.
    assert map_gw.octomap_calls == [str(three_d / "octomap.bt"), ""]


def test_activate_says_so_when_the_3d_planner_cannot_be_put_back(
    catalog_repo, client, map_gw, three_d, monkeypatch, tmp_path
):
    _point_ini_at(catalog_repo, monkeypatch, tmp_path, _INTERPOLATED_INI)
    map_gw.octomap_results = [
        (False, "the 3D planner could not read octomap.bt"),
        (False, "Timeout waiting for planner_server/Octo3D/load_map response"),
    ]

    response = _activate(client, "three")

    assert response.status_code == 502
    assert "3D planner could not be put back" in response.json()["detail"]


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
        {"name": "dock", "type": "CHARGE", "x": 1.5, "y": -2.0, "theta": 90.0},
        {"name": "home", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
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
    assert body["files"] == 3  # map.pcd, octomap.bt, the patch
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


def test_export_streams_from_a_spool_and_declares_its_length(client, maps_dir, exportable):
    """Built into an anonymous file and streamed, not held in memory: the
    length is still declared, and nothing is left in map/ afterwards."""
    before = sorted(p.name for p in maps_dir.iterdir())

    response = _export(client, "full")

    assert response.status_code == 200
    assert int(response.headers["content-length"]) == len(response.content)
    assert zipfile.ZipFile(io.BytesIO(response.content)).namelist()[-1] == _MANIFEST
    assert sorted(p.name for p in maps_dir.iterdir()) == before


def test_an_import_over_the_limit_by_its_declared_length_is_413(
    client, maps_dir, catalog_repo, monkeypatch
):
    """Refused on Content-Length, before a byte is spooled: no spool is even
    opened (the streamed count would also say 413, so this is what tells the
    early refusal apart)."""
    monkeypatch.setattr(map_router_module, "MAP_IMPORT_MAX_BYTES", 1024)
    monkeypatch.setattr(
        catalog_repo, "scratch_file", lambda: pytest.fail("spooled a refused upload")
    )

    response = _import(client, _handmade(files={"map.pcd": b"x" * 4096}))

    assert response.status_code == 413
    assert response.json()["code"] == "archive_too_large"
    assert _hidden_dirs(maps_dir) == []
    assert not (maps_dir / "handmade").exists()


def test_an_import_over_the_limit_without_a_declared_length_is_413(
    client, maps_dir, monkeypatch
):
    """Chunked: no Content-Length to trust, so the bytes received are counted."""
    monkeypatch.setattr(map_router_module, "MAP_IMPORT_MAX_BYTES", 1024)
    archive = _handmade(files={"map.pcd": b"x" * 4096})

    def _chunked():
        for start in range(0, len(archive), 512):
            yield archive[start:start + 512]

    response = client.post("/api/v1/maps/import", content=_chunked(), headers=_OCTET)

    assert response.status_code == 413
    assert response.json()["code"] == "archive_too_large"
    assert not (maps_dir / "handmade").exists()


def test_an_import_leaves_nothing_but_the_map_in_map_dir(client, maps_dir):
    before = {p.name for p in maps_dir.iterdir()}

    assert _import(client, _handmade()).status_code == 201

    assert {p.name for p in maps_dir.iterdir()} == before | {"handmade"}


def test_export_defaults_to_zip(client):
    response = _export(client, "full")

    assert response.status_code == 200
    assert response.headers["content-type"] == "application/zip"
    assert response.content[:2] == b"PK"


def test_export_unknown_format_is_422(client):
    assert _export(client, "full", "rar").status_code == 422


def test_export_404_for_a_missing_map(client):
    assert _export(client, "nosuchmap").status_code == 404


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
    files = {"map.pcd": b"pcd", "octomap.bt": b"bt"}
    payload = _zip_of({"map.pcd": b"pcd"}, _manifest_for(files))

    response = _import(client, payload)

    assert response.status_code == 400
    assert "missing from the archive: 'octomap.bt'" in response.json()["detail"]
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
    response = _import(client, _handmade(files={"octomap.bt": b"bt"}))

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
        {"name": "dock", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
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
        {"name": "stale", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    vertices = [{"name": "fresh", "type": "MOVE", "x": 1.0, "y": 1.0, "theta": 0.0}]

    response = _import(client, _handmade(name="ghost", vertices=vertices))

    assert response.status_code == 201, response.text
    body = response.json()
    assert body["replaced"] is False
    assert body["vertices_deleted"] == 1
    assert [v.name for v in map_repo.list_vertices(map="ghost")] == ["fresh"]


# replacing an existing map


def test_import_replaces_an_existing_map(client, maps_dir, map_repo, exportable):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "old1", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "old2", "type": "MOVE", "x": 1.0, "y": 0.0, "theta": 0.0},
        {"name": "old3", "type": "MOVE", "x": 2.0, "y": 0.0, "theta": 0.0},
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
    vertices = [{"name": "v", "type": "MOVE", "x": 0, "y": 0, "theta": 0}]

    response = _import(client, _handmade(name="copy", vertices=vertices))

    assert response.status_code == 502
    assert "removed again" in response.json()["detail"]
    _assert_untouched(maps_dir)
    assert map_repo.list_vertices(map="copy") == []


def test_import_puts_the_previous_map_back_when_the_vertex_insert_fails(
    client, maps_dir, map_repo, monkeypatch
):
    map_repo.create_vertices(map="rawonly", vertices=[
        {"name": "keep", "type": "MOVE", "x": 0.0, "y": 0.0, "theta": 0.0},
    ])
    before = _tree(maps_dir / "rawonly")

    def _boom(*args, **kwargs):
        raise RuntimeError("database away")

    monkeypatch.setattr(map_repo, "create_vertices", _boom)
    vertices = [{"name": "v", "type": "MOVE", "x": 0, "y": 0, "theta": 0}]

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


# --- POST /api/v1/localization/auto -----------------------------------------


def _auto_localize(client):
    return client.post("/api/v1/localization/auto")


def test_auto_localize_answers_the_pose_in_degrees(client, map_gw):
    response = _auto_localize(client)

    assert response.status_code == 200
    body = response.json()
    assert body["localized"] is True
    assert body["message"] == "matched"
    assert (body["x"], body["y"]) == pytest.approx((1.5, -2.0))
    assert body["theta"] == pytest.approx(90.0, abs=1e-4)
    assert body["inlier_ratio"] == pytest.approx(0.8)
    assert body["candidates"] == 480
    assert map_gw.auto_localize_calls == 1


def test_auto_localize_failure_is_a_502_with_the_localizers_sentence(client, map_gw):
    map_gw.auto_localize_result = (
        False,
        "search failed: ambiguous, a distinct pose scores alike",
        types.SimpleNamespace(
            x=0.0, y=0.0, z=0.0, yaw=0.0, fitness=0.0, inlier_ratio=0.0,
            candidates=480,
        ),
    )

    response = _auto_localize(client)

    assert response.status_code == 502
    assert "ambiguous" in response.json()["detail"]


def test_auto_localize_outside_auto_mode_is_a_502(client, map_gw):
    map_gw.auto_localize_result = (
        False,
        "auto_localize is not available — finding the robot needs it in AUTO "
        "(navigation) mode.",
        None,
    )

    response = _auto_localize(client)

    assert response.status_code == 502
    assert "AUTO" in response.json()["detail"]


def test_auto_localize_refuses_while_a_task_is_running(client, map_gw, workflow_gw):
    workflow_gw.tasks = [types.SimpleNamespace(id="task-7")]

    response = _auto_localize(client)

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "task-7" in response.json()["detail"]
    assert map_gw.auto_localize_calls == 0


def test_auto_localize_refuses_when_temporal_cannot_be_reached(
    client, map_gw, workflow_gw
):
    workflow_gw.error = RuntimeError("connection refused")

    response = _auto_localize(client)

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert map_gw.auto_localize_calls == 0


def test_a_second_auto_localize_mid_search_is_a_409(client, map_gw):
    map_gw.auto_localize_gate = threading.Event()
    first = {}
    worker = threading.Thread(target=lambda: first.update(r=_auto_localize(client)))
    worker.start()
    try:
        assert map_gw.auto_localize_entered.wait(timeout=5)

        second = _auto_localize(client)

        assert second.status_code == 409
        assert second.json()["code"] == "localize_running"
    finally:
        map_gw.auto_localize_gate.set()
        worker.join(timeout=5)

    assert first["r"].status_code == 200
    assert map_gw.auto_localize_calls == 1
    # Released after the answer: the next press goes through.
    map_gw.auto_localize_gate = None
    assert _auto_localize(client).status_code == 200


def test_auto_localize_releases_the_lock_after_a_failure(client, map_gw):
    map_gw.auto_localize_result = (False, "did not converge", None)
    assert _auto_localize(client).status_code == 502

    map_gw.auto_localize_result = (
        True, "matched", types.SimpleNamespace(
            x=0.0, y=0.0, z=0.0, yaw=0.0, fitness=0.0, inlier_ratio=1.0,
            candidates=1,
        ),
    )
    assert _auto_localize(client).status_code == 200


# --- /api/v1/maps/{name}/floors -----------------------------------------------
#
# The storeys of a map: detected from poses.txt, replaced by a saved list.


def _plant_poses(directory, *segments):
    """``(level, count)`` segments, each walked along the same 20 m of x, so
    the storeys are one over the other; ``(level, count, x)`` starts a
    segment's walk at ``x`` instead."""
    lines = []
    for level, count, *at in segments:
        x0 = at[0] if at else 0
        lines.extend(
            f"{len(lines) + i}.pcd {x0 + (i % 100) * 0.2:.1f} 0 {level} 1 0 0 0"
            for i in range(count)
        )
    (directory / "poses.txt").write_text("\n".join(lines) + "\n")


def test_floors_are_detected_from_the_trajectory(client, maps_dir):
    _plant_poses(maps_dir / "full", (0.0, 400), (2.5, 10), (5.3, 300))

    body = client.get("/api/v1/maps/full/floors").json()

    assert body == {
        "name": "full",
        "floors": [
            {"name": "1F", "level": 0.0, "low": 0.0, "high": 0.0},
            {"name": "2F", "level": 5.3, "low": 5.3, "high": 5.3},
        ],
        "detected": [
            {"name": "1F", "level": 0.0, "low": 0.0, "high": 0.0},
            {"name": "2F", "level": 5.3, "low": 5.3, "high": 5.3},
        ],
        "saved": False,
    }


def test_floors_are_empty_for_one_storey_or_no_trajectory(client, maps_dir):
    assert client.get("/api/v1/maps/rawonly/floors").json()["floors"] == []

    _plant_poses(maps_dir / "full", (0.0, 400))
    assert client.get("/api/v1/maps/full/floors").json()["floors"] == []


def test_two_heights_walked_in_different_places_are_one_floor(client, maps_dir):
    # The foot and the top of a long slope: a storey apart, over nothing.
    _plant_poses(maps_dir / "full", (0.0, 400, 0), (-3.0, 300, 60))

    assert client.get("/api/v1/maps/full/floors").json()["floors"] == []


def test_floors_404_for_a_missing_map(client):
    assert client.get("/api/v1/maps/nosuchmap/floors").status_code == 404


def test_saved_floors_replace_the_detected_ones(client, maps_dir):
    _plant_poses(maps_dir / "full", (0.0, 400), (5.3, 300))

    response = client.put(
        "/api/v1/maps/full/floors",
        json={"floors": [{"name": " Lobby ", "level": 5.3}, {"name": "B1", "level": -0.1}]},
    )

    assert response.status_code == 200
    # Sorted lowest first, names trimmed.
    assert response.json()["floors"] == [
        {"name": "B1", "level": -0.1, "low": None, "high": None},
        {"name": "Lobby", "level": 5.3, "low": None, "high": None},
    ]
    body = client.get("/api/v1/maps/full/floors").json()
    assert body["saved"] is True
    assert [floor["name"] for floor in body["floors"]] == ["B1", "Lobby"]
    assert [floor["name"] for floor in body["detected"]] == ["1F", "2F"]
    document = json.loads((maps_dir / "full" / "floors.json").read_text())
    assert document["format"] == "syncai-map-floors"
    assert document["version"] == 1


def test_a_saved_floor_keeps_the_heights_walked_on_it(client, maps_dir):
    # A detected list saved under new names keeps its ranges; a range that
    # does not hold its level is dropped, and that floor keeps its level.
    response = client.put(
        "/api/v1/maps/full/floors",
        json={
            "floors": [
                {"name": "G", "level": -0.2, "low": -3.0, "high": 0.4},
                {"name": "Deck", "level": 3.9, "low": 4.0, "high": 4.5},
            ]
        },
    )

    assert response.status_code == 200
    assert client.get("/api/v1/maps/full/floors").json()["floors"] == [
        {"name": "G", "level": -0.2, "low": -3.0, "high": 0.4},
        {"name": "Deck", "level": 3.9, "low": None, "high": None},
    ]


def test_an_empty_save_is_one_floor_and_still_saved(client, maps_dir):
    _plant_poses(maps_dir / "full", (0.0, 400), (5.3, 300))

    response = client.put("/api/v1/maps/full/floors", json={"floors": []})

    assert response.status_code == 200
    assert "one floor" in response.json()["message"]
    body = client.get("/api/v1/maps/full/floors").json()
    assert body["floors"] == [] and body["saved"] is True


@pytest.mark.parametrize(
    "floors, phrase",
    [
        ([{"name": "  ", "level": 0.0}], "needs a name"),
        ([{"name": "x" * 17, "level": 0.0}], "too long"),
        ([{"name": "1F", "level": 0.0}, {"name": "1f", "level": 5.0}], "Two floors"),
        ([{"name": "1F", "level": 0.0}, {"name": "2F", "level": 0.5}], "less than 1 m"),
    ],
)
def test_floors_put_refuses_a_list_it_cannot_draw(client, floors, phrase):
    response = client.put("/api/v1/maps/full/floors", json={"floors": floors})

    assert response.status_code == 400
    assert phrase in response.json()["detail"]


def test_an_unreadable_floors_file_falls_back_to_detection(client, maps_dir):
    _plant_poses(maps_dir / "full", (0.0, 400), (5.3, 300))
    (maps_dir / "full" / "floors.json").write_text("{not json")

    body = client.get("/api/v1/maps/full/floors").json()

    assert body["saved"] is False
    assert [floor["name"] for floor in body["floors"]] == ["1F", "2F"]


# --- the robot's map cleaning holds its map directory -----------------------------
#
# For minutes after a save the robot's clean_map rewrites map.pcd and builds
# octomap.bt in place. Moving, removing, replacing or archiving the directory
# under it would leave a half-built map, so each is refused while the cleaning
# sidecar says it is running -- and only then: a clean killed hard (stale) or
# finished does not hold the map.


def _cleaning(maps_dir, name="rawonly", **kwargs):
    _plant_clean(maps_dir / name, _clean_payload("converting", **kwargs))


_WHILE_CLEANING = {
    "rename": lambda client: _rename(client, "rawonly", "hall"),
    "delete": lambda client: _delete(client, "rawonly"),
    "export": lambda client: _export(client, "rawonly"),
    "replace by import": lambda client: _import(client, _handmade(), name="rawonly"),
}


@pytest.mark.parametrize("operation", list(_WHILE_CLEANING))
def test_a_map_the_robot_is_still_cleaning_is_held(client, maps_dir, operation):
    _cleaning(maps_dir)

    response = _WHILE_CLEANING[operation](client)

    assert response.status_code == 409, response.text
    assert response.json()["code"] == "octomap_converting"
    assert "still building the 3D map of 'rawonly'" in response.json()["detail"]
    assert (maps_dir / "rawonly" / "map.pcd").is_file()
    assert not (maps_dir / "hall").exists()


@pytest.mark.parametrize("operation", list(_WHILE_CLEANING))
def test_a_clean_killed_hard_does_not_hold_the_map(client, maps_dir, operation):
    _cleaning(maps_dir, started_at=_old_iso())

    response = _WHILE_CLEANING[operation](client)

    assert response.status_code < 300, response.text


def test_a_finished_clean_does_not_hold_the_map(client, maps_dir):
    _plant_clean(maps_dir / "rawonly", _clean_payload("ok", written=True))

    assert _rename(client, "rawonly", "hall").status_code == 200
