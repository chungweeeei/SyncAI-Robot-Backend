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
from syncai_backend.services.gridmap_conversion import (  # noqa: E402
    GridmapConversionService,
)
from syncai_backend.services.map_switch import MapSwitchService  # noqa: E402
from syncai_backend.services.map_usage import MapUsagePolicy  # noqa: E402


class _RecordingMapGateway:
    """Every ROS step the switch takes, in order; each answers ``ok`` unless
    named in ``fail``."""

    def __init__(self, planner_3d=False, map_server=True):
        self.order = []
        self.fail = set()
        self.planner_3d = planner_3d
        self.map_server = map_server

    def _step(self, step, arg):
        self.order.append((step, arg))
        return (False, f"{step} refused") if step in self.fail else (True, "")

    def localizer_ready(self, timeout_sec=2.0):
        return True

    def map_server_ready(self, timeout_sec=2.0):
        return self.map_server

    def planner_3d_available(self, timeout_sec=2.0):
        return self.planner_3d

    def swap_localizer_map(self, pcd_path, x, y, yaw):
        return self._step("swap_localizer_map", pcd_path)

    def reload_map(self, yaml_path):
        return self._step("reload_map", yaml_path)

    def reload_octomap(self, bt_path):
        return self._step("reload_octomap", bt_path)

    def reload_keepout(self, yaml_path):
        return self._step("reload_keepout", yaml_path)

    def localization_converged(self, timeout_s=3.0):
        return True


@pytest.fixture
def prior(maps_dir, make_pcd, make_pgm, make_gridmap_yaml, catalog_repo, tmp_path,
          monkeypatch):
    """A converted map named ``prior``, which the INI says is loaded."""
    directory = maps_dir / "prior"
    directory.mkdir()
    make_pcd(directory / "map.pcd")
    make_pgm(directory / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(directory / "gridmap.yaml", origin=(0.0, 0.0, 0.0))
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: prior\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    catalog_repo.load_active()
    return directory


def _service(logger, catalog_repo, map_gw):
    return MapSwitchService(
        logger=logger,
        map_catalog_repo=catalog_repo,
        map_gw=map_gw,
        map_usage=MapUsagePolicy(
            map_catalog_repo=catalog_repo,
            conversion_svc=GridmapConversionService(logger=logger),
            # The switch never asks about templates.
            task_template_repo=None,
        ),
    )


def _plan(svc, catalog_repo, name):
    return svc.plan(catalog_repo.get_map(name), grid_usable=True, grid_error=None)


def test_plan_is_none_for_the_loaded_map(logger, catalog_repo, prior):
    svc = _service(logger, catalog_repo, _RecordingMapGateway())

    assert _plan(svc, catalog_repo, "prior") is None


def test_plan_refuses_a_map_with_no_grid_and_no_tree(logger, catalog_repo, prior):
    svc = _service(logger, catalog_repo, _RecordingMapGateway())

    with pytest.raises(ConflictError) as caught:
        svc.plan(catalog_repo.get_map("rawonly"), grid_usable=False, grid_error=None)

    assert caught.value.code == "grid_missing"


def test_a_switch_runs_localizer_grid_octomap_then_the_mask(
    logger, catalog_repo, maps_dir, prior
):
    map_gw = _RecordingMapGateway(planner_3d=True)
    svc = _service(logger, catalog_repo, map_gw)

    outcome = svc.execute(_plan(svc, catalog_repo, "full"))

    assert outcome.map_kind == "2d"
    assert outcome.previous == "prior"
    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "reload_octomap",
        "reload_keepout",
    ]
    # A 2D map unloads the 3D planner.
    assert map_gw.order[2] == ("reload_octomap", "")
    assert catalog_repo.active_name() == "full"


def test_a_localizer_refusal_undoes_nothing(logger, catalog_repo, prior):
    map_gw = _RecordingMapGateway(planner_3d=True)
    map_gw.fail.add("swap_localizer_map")
    svc = _service(logger, catalog_repo, map_gw)

    with pytest.raises(UpstreamError):
        svc.execute(_plan(svc, catalog_repo, "full"))

    assert [step for step, _ in map_gw.order] == ["swap_localizer_map"]


def test_an_octomap_refusal_undoes_every_step_in_the_order_done(
    logger, catalog_repo, maps_dir, prior
):
    map_gw = _RecordingMapGateway(planner_3d=True)
    map_gw.fail.add("reload_octomap")
    svc = _service(logger, catalog_repo, map_gw)

    with pytest.raises(UpstreamError):
        svc.execute(_plan(svc, catalog_repo, "full"))

    assert map_gw.order == [
        ("swap_localizer_map", str(maps_dir / "full" / "map.pcd")),
        ("reload_map", str(maps_dir / "full" / "gridmap.yaml")),
        ("reload_octomap", ""),
        ("swap_localizer_map", str(prior / "map.pcd")),
        ("reload_map", str(prior / "gridmap.yaml")),
        # Octo3D put back as the previous map had it: no tree.
        ("reload_octomap", ""),
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
        svc.execute(_plan(svc, catalog_repo, "full"))

    assert [step for step, _ in map_gw.order] == [
        "swap_localizer_map",
        "reload_map",
        "swap_localizer_map",
        "reload_map",
    ]
    assert "read-only file system" in str(caught.value)
    assert "The localizer was put back on 'prior'." in str(caught.value)
