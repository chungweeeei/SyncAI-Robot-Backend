"""Tests for ControllerSettingsService: the change saga over the live controller
and the overlay file.

A stub stands in for controller_server's parameter services, with the
behaviours the saga exists for: undotted parameters refused mid-goal, a node
that reports success but holds another value, a node that is not running. The
overlay is a real file in tmp_path.
"""

import pytest

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.failure import Failure, fail
from syncai_backend.repositories.params.overlay import (
    CONTROLLER_NODE_KEY,
    ParamOverlayRepo,
)
from syncai_backend.services.controller_schema import FIELDS
from syncai_backend.services.controller_settings import ControllerSettingsService

FACTORY = {
    name: value
    for name, value in {
        # Every target, at the shipped values.
        "FollowPath.vx_max": 0.8,
        "FollowPathRPP3D.desired_linear_vel": 0.8,
        "FollowPath.max_linear_accel": 1.0,
        "FollowPath.max_angular_accel": 3.2,
        "goal_checker.xy_goal_tolerance": 0.1,
        "FollowPath.GoalAngleCritic.threshold_to_consider": 0.1,
        "goal_checker.yaw_goal_tolerance": 0.2,
        "FollowPath.approach_dist": 0.5,
        "progress_checker.required_movement_radius": 0.1,
        "progress_checker.movement_time_allowance": 30.0,
        "failure_tolerance": 3.0,
        "FollowPath.PathFollowCritic.cost_weight": 5.0,
        "FollowPath.CostCritic.cost_weight": 3.81,
        "FollowPath.TwirlingCritic.cost_weight": 5.0,
        "FollowPath.stair_linear_vel": 0.5,
        "FollowPathRPP3D.stair_linear_vel": 0.5,
        "FollowPath.stair_standoff": 1.0,
        "FollowPathRPP3D.stair_standoff": 1.0,
        "Docking.v_linear_max": 0.4,
        "Undocking.v_linear_max": 0.3,
        "dock_goal_checker.xy_goal_tolerance": 0.05,
        # Read by the rules only.
        "FollowPath.walk_vx_min": 0.3,
        "FollowPath.approach_decel": 0.4,
        "FollowPath.goal_stop_decel": 0.5,
        "FollowPath.approach_floor_dist": 0.2,
        "FollowPath.stair_enter_dist": 1.0,
        "FollowPath.stair_exit_ahead_dist": 3.5,
        "FollowPathRPP3D.stair_enter_dist": 1.0,
        "FollowPathRPP3D.stair_exit_ahead_dist": 3.5,
        "Docking.v_linear_min": 0.05,
        "Undocking.v_linear_min": 0.05,
    }.items()
}


def test_the_fixture_covers_every_target():
    assert all(name in FACTORY for f in FIELDS for name in f.targets)


class _StubController:
    """controller_server's parameter services, in memory."""

    def __init__(self):
        self.params = dict(FACTORY)
        self.up = True
        self.navigating = False
        # Names whose set reports success but leaves the old value.
        self.ignores = set()
        self.refuse = None
        self.sets = []

    def available(self, timeout_sec=2.0):
        return self.up

    def get(self, names):
        if not self.up:
            return False, "controller_server is not running", {}
        return True, "", {name: self.params.get(name) for name in names}

    def set_atomically(self, values):
        if not values:
            return True, ""
        if not self.up:
            return False, "controller_server is not running"
        self.sets.append(dict(values))
        if self.navigating and any("." not in name for name in values):
            return False, fail(Failure.CONTROLLER_BUSY, "not while navigating")
        if self.refuse:
            return False, fail(Failure.PARAMETER_REFUSED, self.refuse)
        for name, value in values.items():
            if name not in self.ignores:
                self.params[name] = value
        return True, ""


@pytest.fixture
def factory_file(tmp_path):
    import yaml

    from syncai_backend.repositories.params.overlay import nest

    path = tmp_path / "factory.yaml"
    path.write_text(
        yaml.safe_dump({CONTROLLER_NODE_KEY: {"ros__parameters": nest(FACTORY)}})
    )
    return str(path)


@pytest.fixture
def repo(logger, tmp_path, factory_file):
    return ParamOverlayRepo(
        logger=logger,
        overlay_path=str(tmp_path / "params" / "robot" / "controller_server.yaml"),
        factory_path=factory_file,
        node_key=CONTROLLER_NODE_KEY,
    )


@pytest.fixture
def controller():
    return _StubController()


@pytest.fixture
def svc(logger, controller, repo):
    return ControllerSettingsService(logger=logger, gateway=controller, overlay_repo=repo)


def _field(snapshot, field_id):
    return next(state for state in snapshot.fields if state.setting.id == field_id)


def test_snapshot_reports_live_default_and_saved(svc, controller):
    controller.params["FollowPath.vx_max"] = 0.7
    snap = svc.snapshot()

    assert snap.live_available
    speed = _field(snap, "max_speed")
    assert (speed.value, speed.current, speed.default, speed.persisted) == (
        0.7,
        0.7,
        0.8,
        None,
    )


def test_a_live_change_sets_every_target_and_is_saved(svc, controller, repo):
    snap = svc.apply({"max_speed": 0.6})

    assert controller.params["FollowPath.vx_max"] == 0.6
    assert controller.params["FollowPathRPP3D.desired_linear_vel"] == 0.6
    assert repo.read() == {
        "FollowPath.vx_max": 0.6,
        "FollowPathRPP3D.desired_linear_vel": 0.6,
    }
    assert _field(snap, "max_speed").persisted == 0.6


def test_a_json_int_is_sent_as_a_float(svc, controller):
    svc.apply({"failure_tolerance": 4})

    sent = controller.sets[-1]["failure_tolerance"]
    assert sent == 4.0 and isinstance(sent, float)


def test_server_parameters_go_in_a_set_of_their_own_first(svc, controller):
    svc.apply({"failure_tolerance": 4.0, "max_speed": 0.6})

    assert list(controller.sets[0]) == ["failure_tolerance"]
    assert set(controller.sets[1]) == {
        "FollowPath.vx_max",
        "FollowPathRPP3D.desired_linear_vel",
    }


def test_an_idle_field_mid_goal_is_a_409_and_nothing_changes(svc, controller, repo):
    controller.navigating = True

    with pytest.raises(ConflictError) as exc:
        svc.apply({"failure_tolerance": 4.0, "max_speed": 0.6})

    assert exc.value.code == "controller_busy"
    assert controller.params["failure_tolerance"] == 3.0
    assert controller.params["FollowPath.vx_max"] == 0.8
    assert repo.read() == {}


def test_a_live_field_mid_goal_goes_through(svc, controller):
    controller.navigating = True
    svc.apply({"max_speed": 0.6})
    assert controller.params["FollowPath.vx_max"] == 0.6


def test_a_refused_plugin_set_undoes_the_server_set(svc, controller, repo):
    """The plugin batch is refused after the server batch went through; the
    server's value goes back, and nothing is saved."""
    plain_set = controller.set_atomically
    refused = []

    def refuse_first_plugin_set(values):
        if any("." in name for name in values) and not refused:
            refused.append(values)
            return False, fail(Failure.PARAMETER_REFUSED, "v_linear_min above v_linear_max")
        return plain_set(values)

    controller.set_atomically = refuse_first_plugin_set

    with pytest.raises(BadRequestError) as exc:
        svc.apply({"failure_tolerance": 4.0, "max_speed": 0.6})

    assert exc.value.code == "parameter_refused"
    assert controller.params["failure_tolerance"] == 3.0
    assert controller.params["FollowPath.vx_max"] == 0.8
    assert repo.read() == {}


def test_a_set_the_node_did_not_take_is_undone_and_not_saved(svc, controller, repo):
    controller.ignores = {"FollowPathRPP3D.desired_linear_vel"}

    with pytest.raises(UpstreamError):
        svc.apply({"max_speed": 0.6})

    assert controller.params["FollowPath.vx_max"] == 0.8
    assert repo.read() == {}


def test_a_restart_field_is_saved_but_not_set(svc, controller, repo):
    snap = svc.apply({"stair_standoff": 1.2})

    assert controller.sets == []
    assert controller.params["FollowPath.stair_standoff"] == 1.0
    assert repo.read()["FollowPath.stair_standoff"] == 1.2
    standoff = _field(snap, "stair_standoff")
    assert standoff.value == 1.2
    assert standoff.current == 1.0
    assert standoff.pending_restart


def test_with_the_controller_down_a_change_is_saved_for_its_start(svc, controller, repo):
    controller.up = False
    snap = svc.apply({"max_speed": 0.6})

    assert controller.sets == []
    assert repo.read()["FollowPath.vx_max"] == 0.6
    assert not snap.live_available
    assert _field(snap, "max_speed").value == 0.6


def test_a_failed_overlay_write_undoes_the_live_set(svc, controller, repo):
    def broken_write(values):
        raise OSError("read-only file system")

    repo.write = broken_write

    with pytest.raises(ConflictError) as exc:
        svc.apply({"max_speed": 0.6})

    assert exc.value.code == "params_not_writable"
    assert controller.params["FollowPath.vx_max"] == 0.8


def test_a_rule_refuses_before_anything_is_set(svc, controller, repo):
    with pytest.raises(BadRequestError, match="final approach") as exc:
        svc.apply({"xy_goal_tolerance": 0.3})

    assert exc.value.code == "invalid_setting"
    assert controller.sets == []
    assert repo.read() == {}


def test_a_rule_reads_the_saved_value_of_a_field_not_in_the_request(svc, repo):
    """The approach distance is not in the request; its saved value decides."""
    svc.apply({"approach_dist": 0.2})
    with pytest.raises(BadRequestError, match="final approach"):
        svc.apply({"xy_goal_tolerance": 0.2})


def test_unknown_and_out_of_range_values_are_400s(svc):
    with pytest.raises(BadRequestError, match="Unknown setting"):
        svc.apply({"wz_max": 1.0})
    with pytest.raises(BadRequestError, match="at most"):
        svc.apply({"max_speed": 2.0})
    with pytest.raises(BadRequestError):
        svc.apply({})


def test_reset_one_field_restores_its_factory_value(svc, controller, repo):
    svc.apply({"max_speed": 0.6, "failure_tolerance": 4.0})
    snap = svc.reset(["max_speed"])

    assert controller.params["FollowPath.vx_max"] == 0.8
    assert controller.params["FollowPathRPP3D.desired_linear_vel"] == 0.8
    assert repo.read() == {"failure_tolerance": 4.0}
    assert _field(snap, "max_speed").persisted is None


def test_reset_all_removes_the_overlay_but_keeps_keys_not_on_the_page(
    svc, controller, repo
):
    repo.write({"FollowPath.vx_max": 0.6, "FollowPath.lookahead_dist": 0.9})
    svc.reset()

    assert repo.read() == {"FollowPath.lookahead_dist": 0.9}


def test_reset_without_defaults_is_refused(logger, controller, tmp_path):
    repo = ParamOverlayRepo(
        logger=logger,
        overlay_path=str(tmp_path / "o.yaml"),
        factory_path=str(tmp_path / "missing.yaml"),
        node_key=CONTROLLER_NODE_KEY,
    )
    svc = ControllerSettingsService(logger=logger, gateway=controller, overlay_repo=repo)

    with pytest.raises(ConflictError) as exc:
        svc.reset()
    assert exc.value.code == "defaults_unavailable"


def test_an_unreadable_overlay_refuses_a_change(svc, repo):
    import os

    os.makedirs(os.path.dirname(repo.overlay_path))
    with open(repo.overlay_path, "w") as handle:
        handle.write("a: [\n")

    with pytest.raises(ConflictError) as exc:
        svc.apply({"max_speed": 0.6})
    assert exc.value.code == "params_unreadable"


def test_one_change_at_a_time(svc):
    svc._changing.acquire()
    try:
        with pytest.raises(ConflictError) as exc:
            svc.apply({"max_speed": 0.6})
        assert exc.value.code == "settings_busy"
    finally:
        svc._changing.release()
