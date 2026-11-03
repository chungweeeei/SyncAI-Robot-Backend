"""Tests for ParamOverlayRepo: the runtime params overlay as rcl will read it.

What is pinned is the file's shape (a ROS 2 params file under the wildcard
node key, so the controller launch can load it after the factory file), that
it holds only what it was given, that a float stays a float on disk (rcl reads
``1`` as an int and the controller refuses an int for a double), and that no
overlay at all is the factory state.
"""

import os

import yaml

from syncai_backend.repositories.params.overlay import (
    BACKUP_SUFFIX,
    CONTROLLER_NODE_KEY,
    ParamOverlayRepo,
    flatten,
    init_controller_overlay_repo,
    nest,
)

FACTORY = """\
# a comment the factory file keeps
/**/controller_server:
  ros__parameters:
    failure_tolerance: 3.0
    goal_checker:
      xy_goal_tolerance: 0.1
    FollowPath:
      vx_max: 0.8
      CostCritic:
        cost_weight: 3.81
/**/local_costmap:
  ros__parameters:
    width: 4
"""


def _repo(logger, tmp_path, factory=FACTORY):
    factory_path = tmp_path / "factory.yaml"
    if factory is not None:
        factory_path.write_text(factory)
    return ParamOverlayRepo(
        logger=logger,
        overlay_path=str(tmp_path / "params" / "robot" / "controller_server.yaml"),
        factory_path=str(factory_path),
        node_key=CONTROLLER_NODE_KEY,
    )


def test_flatten_and_nest_are_inverse():
    tree = {"a": 1.0, "FollowPath": {"vx_max": 0.8, "CostCritic": {"cost_weight": 2.0}}}
    flat = flatten(tree)
    assert flat == {
        "a": 1.0,
        "FollowPath.vx_max": 0.8,
        "FollowPath.CostCritic.cost_weight": 2.0,
    }
    assert nest(flat) == tree


def test_factory_reads_only_the_named_node(logger, tmp_path):
    assert _repo(logger, tmp_path).factory() == {
        "failure_tolerance": 3.0,
        "goal_checker.xy_goal_tolerance": 0.1,
        "FollowPath.vx_max": 0.8,
        "FollowPath.CostCritic.cost_weight": 3.81,
    }


def test_a_missing_factory_file_is_no_defaults_not_an_error(logger, tmp_path):
    assert _repo(logger, tmp_path, factory=None).factory() == {}


def test_no_overlay_reads_empty(logger, tmp_path):
    assert _repo(logger, tmp_path).read() == {}


def test_write_creates_a_params_file_rcl_can_load(logger, tmp_path):
    repo = _repo(logger, tmp_path)
    repo.write({"FollowPath.vx_max": 0.7, "failure_tolerance": 4.0})

    with open(repo.overlay_path) as handle:
        text = handle.read()
    assert text.startswith("# Runtime parameter overlay")
    assert yaml.safe_load(text) == {
        "/**/controller_server": {
            "ros__parameters": {"FollowPath": {"vx_max": 0.7}, "failure_tolerance": 4.0}
        }
    }
    assert repo.read() == {"FollowPath.vx_max": 0.7, "failure_tolerance": 4.0}


def test_a_whole_float_stays_a_float_on_disk(logger, tmp_path):
    repo = _repo(logger, tmp_path)
    repo.write({"failure_tolerance": 4.0})

    with open(repo.overlay_path) as handle:
        assert "failure_tolerance: 4.0" in handle.read()
    assert isinstance(repo.read()["failure_tolerance"], float)


def test_writing_nothing_removes_the_overlay_and_keeps_a_backup(logger, tmp_path):
    repo = _repo(logger, tmp_path)
    repo.write({"FollowPath.vx_max": 0.7})
    repo.write({})

    assert not os.path.exists(repo.overlay_path)
    assert repo.read() == {}
    with open(repo.overlay_path + BACKUP_SUFFIX) as handle:
        assert "vx_max: 0.7" in handle.read()


def test_a_rewrite_keeps_the_previous_file_as_backup(logger, tmp_path):
    repo = _repo(logger, tmp_path)
    repo.write({"FollowPath.vx_max": 0.7})
    repo.write({"FollowPath.vx_max": 0.6})

    assert repo.read() == {"FollowPath.vx_max": 0.6}
    with open(repo.overlay_path + BACKUP_SUFFIX) as handle:
        assert "vx_max: 0.7" in handle.read()


def test_no_temp_file_is_left_behind(logger, tmp_path):
    repo = _repo(logger, tmp_path)
    repo.write({"FollowPath.vx_max": 0.7})
    repo.write({"FollowPath.vx_max": 0.6})

    assert sorted(os.listdir(os.path.dirname(repo.overlay_path))) == [
        "controller_server.yaml",
        "controller_server.yaml" + BACKUP_SUFFIX,
    ]


def test_a_malformed_overlay_is_an_error_not_empty(logger, tmp_path):
    """Read as empty, the next write would drop every other saved setting."""
    repo = _repo(logger, tmp_path)
    os.makedirs(os.path.dirname(repo.overlay_path))
    with open(repo.overlay_path, "w") as handle:
        handle.write("a: [\n")

    try:
        repo.read()
    except ValueError:
        pass
    else:
        raise AssertionError("a malformed overlay must raise")


def test_the_overlay_path_is_the_one_the_controller_launch_loads(logger, tmp_path):
    repo = init_controller_overlay_repo(
        logger=logger,
        robot_id="robot01",
        params_dir=str(tmp_path),
        factory_path=str(tmp_path / "f.yaml"),
    )
    assert repo.overlay_path == str(tmp_path / "robot01" / "controller_server.yaml")
    assert repo.node_key == "/**/controller_server"
