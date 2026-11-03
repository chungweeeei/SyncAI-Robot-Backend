"""Tests for the settings schemas (controller, planner): fields, coercion, rules.

The last test checks each schema against the workspace's factory params file,
the one place a renamed parameter shows up before a robot answers "no
parameter" -- set ``SYNCAI_CONTROLLER_PARAMS`` / ``SYNCAI_PLANNER_PARAMS`` to
the files to run it.
"""

import os

import pytest

from syncai_backend.repositories.params.overlay import (
    CONTROLLER_NODE_KEY,
    PLANNER_NODE_KEY,
    _node_params,
)
from syncai_backend.services.controller_schema import CONTROLLER
from syncai_backend.services.planner_schema import PLANNER
from syncai_backend.services.settings_schema import (
    APPLY_IDLE,
    APPLY_LIVE,
    APPLY_RESTART,
    KIND_BOOLEAN,
    coerce,
)

SCHEMAS = pytest.mark.parametrize("schema", [CONTROLLER, PLANNER], ids=["controller", "planner"])
FIELDS_BY_ID = CONTROLLER.by_id
RULES = CONTROLLER.rules


@SCHEMAS
def test_field_ids_are_unique_and_groups_exist(schema):
    assert len(schema.by_id) == len(schema.fields)
    group_ids = {g.id for g in schema.groups}
    assert all(f.group in group_ids for f in schema.fields)
    assert all(any(f.group == g.id for f in schema.fields) for g in schema.groups)


@SCHEMAS
def test_no_parameter_is_written_by_two_fields(schema):
    targets = [name for f in schema.fields for name in f.targets]
    assert len(targets) == len(set(targets))


@SCHEMAS
def test_apply_matches_where_the_node_takes_the_parameter(schema):
    for f in schema.fields:
        assert f.apply in (APPLY_LIVE, APPLY_IDLE, APPLY_RESTART)
        undotted = ["." not in name for name in f.targets]
        # controller_server's own parameters are the ones refused mid-goal,
        # and a field may not mix them with plugin parameters.
        assert len(set(undotted)) == 1, f.id
        if undotted[0] and f.apply != APPLY_RESTART:
            assert f.apply == APPLY_IDLE, f.id


@SCHEMAS
def test_every_rule_names_existing_fields(schema):
    for rule in schema.rules:
        assert rule.fields
        assert all(field_id in schema.by_id for field_id in rule.fields)


def test_every_planner_field_applies_live():
    """Octo3D refuses a build-time key, so the page offers only live ones."""
    assert all(f.apply == APPLY_LIVE for f in PLANNER.fields)


def test_coerce_takes_a_boolean_only_as_a_boolean():
    shortcut = PLANNER.by_id["shortcut"]
    assert shortcut.kind == KIND_BOOLEAN
    assert coerce(shortcut, False) is False
    with pytest.raises(ValueError, match="true or false"):
        coerce(shortcut, 1)


def test_coerce_makes_an_int_a_float():
    """controller_server has no dynamic typing: an int for a double is refused."""
    value = coerce(FIELDS_BY_ID["failure_tolerance"], 4)
    assert value == 4.0 and isinstance(value, float)


@pytest.mark.parametrize(
    "raw, message",
    [
        (True, "must be a number"),
        ("0.5", "must be a number"),
        (None, "must be a number"),
        (float("nan"), "finite"),
        (0.1, "at least 0.4 m/s"),
        (1.5, "at most 1.0 m/s"),
    ],
)
def test_coerce_refuses_with_the_operators_sentence(raw, message):
    with pytest.raises(ValueError, match=message):
        coerce(FIELDS_BY_ID["max_speed"], raw)


def _check(field_id, values):
    sentences = [
        rule.check(values.get)
        for rule in RULES
        if field_id in rule.fields
    ]
    return [s for s in sentences if s]


def test_a_speed_cap_at_the_walking_floor_is_refused():
    assert _check(
        "max_speed", {"FollowPath.vx_max": 0.4, "FollowPath.walk_vx_min": 0.4}
    )
    assert not _check(
        "max_speed", {"FollowPath.vx_max": 0.5, "FollowPath.walk_vx_min": 0.4}
    )


def test_an_unknown_value_never_refuses():
    assert not _check("max_speed", {"FollowPath.vx_max": 0.4})


def test_the_arrival_tolerance_stays_under_the_approach_floor():
    values = {
        "goal_checker.xy_goal_tolerance": 0.25,
        "FollowPath.approach_floor_dist": 0.2,
        "FollowPath.approach_dist": 0.5,
    }
    assert _check("xy_goal_tolerance", values)
    values["goal_checker.xy_goal_tolerance"] = 0.15
    assert not _check("xy_goal_tolerance", values)


def test_the_stair_standoff_rule_checks_both_controllers():
    values = {
        "FollowPath.stair_standoff": 1.0,
        "FollowPath.stair_enter_dist": 1.0,
        "FollowPath.stair_exit_ahead_dist": 3.5,
        "FollowPathRPP3D.stair_standoff": 2.0,
        "FollowPathRPP3D.stair_enter_dist": 1.0,
        "FollowPathRPP3D.stair_exit_ahead_dist": 2.5,
    }
    assert len(_check("stair_standoff", values)) == 1


@pytest.mark.parametrize(
    "schema, env, key",
    [
        (CONTROLLER, "SYNCAI_CONTROLLER_PARAMS", CONTROLLER_NODE_KEY),
        (PLANNER, "SYNCAI_PLANNER_PARAMS", PLANNER_NODE_KEY),
    ],
    ids=["controller", "planner"],
)
def test_every_target_is_in_the_factory_params_file(schema, env, key):
    if not os.environ.get(env):
        pytest.skip(f"{env} names no factory params file")
    factory = _node_params(os.environ[env], key)
    missing = [name for f in schema.fields for name in f.targets if name not in factory]
    assert not missing
    for f in schema.fields:
        for name in f.targets:
            # A field writes one type to every target; the file must agree.
            if f.kind == KIND_BOOLEAN:
                assert isinstance(factory[name], bool), (f.id, name)
                continue
            assert isinstance(factory[name], float), (f.id, name)
            assert f.min <= factory[name] <= f.max, (f.id, name, factory[name])
