"""Tests for MapSwitchService: the live switch's steps and their undos.

The HTTP side (refusal codes, the response body, the Temporal gate) is in
test_maps_router.py. These drive the service directly, which is what pulling
the saga out of the router bought: the order of the ROS calls *is* the
contract, and here it is asserted without a client in between.
"""

import pytest

pytest.importorskip("syncai_common")

from syncai_backend.exceptions import ConflictError, UpstreamError  # noqa: E402
from syncai_backend.helpers.system_config import SYSTEM_INI_ENV  # noqa: E402
from syncai_backend.services.map_switch import MapSwitchService  # noqa: E402

from conftest import StubInternalApiGateway  # noqa: E402


class _RecordingMapGateway:
    """Every ROS step the switch takes, in order; each answers ``ok`` unless
    named in ``fail``."""

    def __init__(self, planner_3d=True):
        self.order = []
        self.fail = set()
        self.planner_3d = planner_3d

    def _step(self, step, arg):
        self.order.append((step, arg))
        return (False, f"{step} refused") if step in self.fail else (True, "")

    def localizer_ready(self, timeout_sec=2.0):
        return True

    def planner_3d_available(self, timeout_sec=2.0):
        return self.planner_3d

    def swap_localizer_map(self, pcd_path, x, y, yaw):
        return self._step("swap_localizer_map", pcd_path)

    def reload_octomap(self, bt_path):
        return self._step("reload_octomap", bt_path)

    def localization_converged(self, timeout_s=3.0):
        return True


@pytest.fixture
def prior(maps_dir, make_pcd, make_bt, catalog_repo, tmp_path, monkeypatch):
    """A navigable map named ``prior``, which the INI says is loaded."""
    directory = maps_dir / "prior"
    directory.mkdir()
    make_pcd(directory / "map.pcd")
    make_bt(directory / "octomap.bt")
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: prior\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    catalog_repo.load_active()
    return directory


def _service(logger, catalog_repo, map_gw, internal_api_gw=None):
    return MapSwitchService(
        logger=logger,
        map_catalog_repo=catalog_repo,
        map_gw=map_gw,
        internal_api_gw=internal_api_gw or StubInternalApiGateway(),
    )


def test_plan_is_none_for_the_loaded_map(logger, catalog_repo, prior):
    svc = _service(logger, catalog_repo, _RecordingMapGateway())

    assert svc.plan("prior") is None


def test_plan_refuses_a_map_with_no_tree(logger, catalog_repo, prior):
    svc = _service(logger, catalog_repo, _RecordingMapGateway())

    with pytest.raises(ConflictError) as caught:
        svc.plan("rawonly")

    assert caught.value.code == "octomap_missing"


def test_a_switch_runs_the_localizer_then_the_3d_planner(
    logger, catalog_repo, maps_dir, prior
):
    map_gw = _RecordingMapGateway()
    svc = _service(logger, catalog_repo, map_gw)

    outcome = svc.execute(svc.plan("full"))

    assert outcome.previous == "prior"
    assert map_gw.order == [
        ("swap_localizer_map", str(maps_dir / "full" / "map.pcd")),
        ("reload_octomap", str(maps_dir / "full" / "octomap.bt")),
    ]
    assert catalog_repo.active_name() == "full"


def test_a_stack_without_the_3d_planner_is_refused_before_anything_moves(
    logger, catalog_repo, prior
):
    map_gw = _RecordingMapGateway(planner_3d=False)
    svc = _service(logger, catalog_repo, map_gw)

    with pytest.raises(ConflictError) as caught:
        svc.execute(svc.plan("full"))

    assert caught.value.code == "planner_3d_unavailable"
    assert map_gw.order == []


def test_a_localizer_refusal_undoes_nothing(logger, catalog_repo, prior):
    map_gw = _RecordingMapGateway()
    map_gw.fail.add("swap_localizer_map")
    svc = _service(logger, catalog_repo, map_gw)

    with pytest.raises(UpstreamError):
        svc.execute(svc.plan("full"))

    assert [step for step, _ in map_gw.order] == ["swap_localizer_map"]


def test_an_octomap_refusal_undoes_every_step_in_the_order_done(
    logger, catalog_repo, maps_dir, prior
):
    map_gw = _RecordingMapGateway()
    map_gw.fail.add("reload_octomap")
    svc = _service(logger, catalog_repo, map_gw)

    with pytest.raises(UpstreamError):
        svc.execute(svc.plan("full"))

    assert map_gw.order == [
        ("swap_localizer_map", str(maps_dir / "full" / "map.pcd")),
        ("reload_octomap", str(maps_dir / "full" / "octomap.bt")),
        ("swap_localizer_map", str(prior / "map.pcd")),
        # Octo3D put back on the previous map's tree.
        ("reload_octomap", str(prior / "octomap.bt")),
    ]
    assert catalog_repo.active_name() == "prior"


def test_an_ini_failure_undoes_every_step_and_says_what_was_put_back(
    logger, catalog_repo, prior, monkeypatch
):
    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(
        "syncai_backend.repositories.map.catalog.set_active_map", _boom
    )
    map_gw = _RecordingMapGateway()
    svc = _service(logger, catalog_repo, map_gw)

    with pytest.raises(UpstreamError) as caught:
        svc.execute(svc.plan("full"))

    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_octomap",
        "swap_localizer_map",
        "reload_octomap",
    ]
    assert "read-only file system" in str(caught.value)
    assert "The localizer was put back on 'prior'." in str(caught.value)


def test_the_internal_service_is_told_after_the_ini(
    logger, catalog_repo, prior, internal_api_gw
):
    svc = _service(logger, catalog_repo, _RecordingMapGateway(), internal_api_gw)

    svc.execute(svc.plan("full"))

    assert internal_api_gw.activated == ["id-full"]


def test_a_map_the_internal_service_does_not_know_is_refused_before_anything_moves(
    logger, catalog_repo, prior, internal_api_gw
):
    internal_api_gw.maps = {"prior"}
    map_gw = _RecordingMapGateway()
    svc = _service(logger, catalog_repo, map_gw, internal_api_gw)

    with pytest.raises(ConflictError) as caught:
        svc.execute(svc.plan("full"))

    assert caught.value.code == "map_not_registered"
    assert map_gw.order == []
    assert internal_api_gw.activated == []


def test_a_failed_lookup_moves_nothing(logger, catalog_repo, prior, internal_api_gw):
    internal_api_gw.lookup_error = "connection refused"
    map_gw = _RecordingMapGateway()
    svc = _service(logger, catalog_repo, map_gw, internal_api_gw)

    with pytest.raises(UpstreamError, match="connection refused"):
        svc.execute(svc.plan("full"))

    assert map_gw.order == []


def test_a_failed_internal_activate_undoes_nothing(
    logger, catalog_repo, prior, internal_api_gw
):
    internal_api_gw.activate_error = "the internal service answered 503"
    map_gw = _RecordingMapGateway()
    svc = _service(logger, catalog_repo, map_gw, internal_api_gw)

    outcome = svc.execute(svc.plan("full"))

    assert [step for step, _ in map_gw.order] == ["swap_localizer_map", "reload_octomap"]
    assert catalog_repo.active_name() == "full"
    assert "answered 503" in outcome.message


def test_a_failed_ini_write_never_tells_the_internal_service(
    logger, catalog_repo, prior, internal_api_gw, monkeypatch
):
    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(
        "syncai_backend.repositories.map.catalog.set_active_map", _boom
    )
    svc = _service(logger, catalog_repo, _RecordingMapGateway(), internal_api_gw)

    with pytest.raises(UpstreamError):
        svc.execute(svc.plan("full"))

    assert internal_api_gw.activated == []
