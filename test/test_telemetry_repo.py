"""Tests for TelemetryRepo: the single-slot caches behind the telemetry WS.

The seq-gating convention (return None unless newer than ``after_seq``) is what
lets the 20 Hz WebSocket loop poll without resending unchanged samples, and the
path TTL is the only thing that ever clears a finished route from the
operator's map — the ROS side never publishes an empty plan (see the module
comment on ``_PATH_TTL_S``). Time is controlled by patching ``time.monotonic``
at the repo's import site, so the 6 s TTL is exercised without sleeping.
"""

from unittest.mock import patch

import pytest

from syncai_backend.repositories.telemetry.telemetry import (
    _DOCKING_PLAN_TTL_S,
    _LOCAL_COSTMAP_TTL_S,
    _LOCAL_PLAN_TTL_S,
    _PATH_TTL_S,
    TelemetryRepo,
)


MONOTONIC = "syncai_backend.repositories.telemetry.telemetry.time.monotonic"


@pytest.fixture
def repo(logger) -> TelemetryRepo:
    return TelemetryRepo(logger)


class TestSeqGating:
    def test_empty_repo_answers_none(self, repo):
        assert repo.get_pose() is None
        assert repo.get_joints() is None
        assert repo.get_path() is None

    def test_pose_is_sent_once_per_sample(self, repo):
        repo.update_pose(x=1.0, y=2.0, z=0.0, yaw_deg=90.0, stamp=10.0)

        sample = repo.get_pose(after_seq=0)
        assert (sample.x, sample.yaw_deg) == (1.0, 90.0)
        # The WS loop hands back the seq it sent; the same sample must not
        # be offered twice.
        assert repo.get_pose(after_seq=sample.seq) is None

        repo.update_pose(x=1.1, y=2.0, z=0.0, yaw_deg=91.0, stamp=10.05)
        assert repo.get_pose(after_seq=sample.seq).x == 1.1

    def test_streams_advance_independently(self, repo):
        # A joints frame must not force a resend of an unchanged pose.
        repo.update_pose(x=1.0, y=2.0, z=0.0, yaw_deg=0.0, stamp=10.0)
        pose_seq = repo.get_pose().seq

        repo.update_joints(joints={"FL_HipX_joint": 0.5}, stamp=10.0)

        assert repo.get_pose(after_seq=pose_seq) is None
        assert repo.get_joints().joints == {"FL_HipX_joint": 0.5}


class TestPathTtl:
    def test_a_fresh_path_replays_until_consumed(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0, 0.0), (1.0, 1.0, 0.0)), stamp=10.0)
            sample = repo.get_path(after_seq=0)
            assert sample.points == ((0.0, 0.0, 0.0), (1.0, 1.0, 0.0))
            assert repo.get_path(after_seq=sample.seq) is None

    def test_a_stale_path_expires_into_one_empty_sample(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0, 0.0),), stamp=10.0)
            seq = repo.get_path(after_seq=0).seq

        # Past the TTL the silence is turned into an explicit "no route":
        # exactly one empty sample with a bumped seq, so every connected
        # client erases the band it is drawing — then quiet.
        with patch(MONOTONIC, return_value=100.0 + _PATH_TTL_S + 1.0):
            expired = repo.get_path(after_seq=seq)
            assert expired.points == ()
            assert expired.seq == seq + 1
            assert repo.get_path(after_seq=expired.seq) is None

    def test_expiry_is_idempotent(self, repo):
        # Once empty, the branch must not keep bumping seq on every poll —
        # that would re-send "no route" to every client at 20 Hz forever.
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0, 0.0),), stamp=10.0)

        with patch(MONOTONIC, return_value=200.0):
            expired = repo.get_path(after_seq=0)
            assert expired.points == ()
            assert repo.get_path(after_seq=expired.seq) is None
            assert repo.get_path(after_seq=expired.seq) is None

    def test_a_replan_revives_the_route(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0, 0.0),), stamp=10.0)
            first = repo.get_path(after_seq=0)

        with patch(MONOTONIC, return_value=200.0):
            expired = repo.get_path(after_seq=first.seq)
            assert expired.points == ()

            # The planner replans: the fresh route supersedes the expiry.
            repo.update_path(points=((5.0, 5.0, 0.0),), stamp=110.0)
            revived = repo.get_path(after_seq=expired.seq)
            assert revived.points == ((5.0, 5.0, 0.0),)


class TestDockingPlan:
    def test_the_docking_plan_is_its_own_slot(self, repo):
        # A dock's clear must not erase the route, nor the reverse.
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0, 0.0), (1.0, 1.0, 0.0)), stamp=10.0)
            path_seq = repo.get_path().seq

            repo.update_docking_plan(points=(), stamp=11.0)

            assert repo.get_path(after_seq=path_seq) is None
            assert repo.get_docking_plan().points == ()

    def test_a_docking_plan_expires_on_its_own_ttl(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_docking_plan(points=((0.0, 0.0, 0.0),), stamp=10.0)
            seq = repo.get_docking_plan().seq

        # Shorter than the route's: the plan comes at 2 Hz, not 1 Hz.
        assert _DOCKING_PLAN_TTL_S < _PATH_TTL_S
        with patch(MONOTONIC, return_value=100.0 + _DOCKING_PLAN_TTL_S + 0.1):
            expired = repo.get_docking_plan(after_seq=seq)
            assert expired.points == ()
            assert repo.get_docking_plan(after_seq=expired.seq) is None


def _costmap(repo, stamp=10.0, data=b"\x00\xfe\xff\x10"):
    repo.update_local_costmap(
        resolution=0.05,
        width=2,
        height=2,
        origin=(1.0, 2.0, 0.5),
        data=data,
        stamp=stamp,
    )


class TestLocalCostmap:
    def test_the_costmap_is_kept_as_given(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            _costmap(repo)
            sample = repo.get_local_costmap()
            assert repo.get_local_costmap(after_seq=sample.seq) is None

        assert (sample.width, sample.height, sample.resolution) == (2, 2, 0.05)
        assert sample.origin == (1.0, 2.0, 0.5)
        assert sample.data == b"\x00\xfe\xff\x10"

    def test_a_costmap_that_stops_expires_into_one_empty_sample(self, repo):
        # The controller died: the console must not keep a frozen picture of
        # the robot's surroundings.
        with patch(MONOTONIC, return_value=100.0):
            _costmap(repo)
            seq = repo.get_local_costmap().seq

        with patch(MONOTONIC, return_value=100.0 + _LOCAL_COSTMAP_TTL_S + 0.1):
            expired = repo.get_local_costmap(after_seq=seq)
            assert expired.width == 0 and expired.data == b""
            assert repo.get_local_costmap(after_seq=expired.seq) is None


class TestLocalPlan:
    def test_the_local_plan_is_its_own_slot(self, repo):
        with patch(MONOTONIC, return_value=100.0):
            repo.update_path(points=((0.0, 0.0, 0.0), (1.0, 1.0, 0.0)), stamp=10.0)
            path_seq = repo.get_path().seq

            repo.update_local_plan(points=((0.0, 0.0), (0.2, 0.1)), stamp=11.0)

            assert repo.get_path(after_seq=path_seq) is None
            assert repo.get_local_plan().points == ((0.0, 0.0), (0.2, 0.1))

    def test_a_local_plan_expires_within_a_second(self, repo):
        # MPPI stops publishing on a stair and in the final rotation; the last
        # trajectory must go with it, not hang where the robot is not going.
        with patch(MONOTONIC, return_value=100.0):
            repo.update_local_plan(points=((0.0, 0.0),), stamp=10.0)
            seq = repo.get_local_plan().seq

        assert _LOCAL_PLAN_TTL_S <= 1.0
        with patch(MONOTONIC, return_value=100.0 + _LOCAL_PLAN_TTL_S + 0.1):
            assert repo.get_local_plan(after_seq=seq).points == ()
