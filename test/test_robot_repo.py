"""Unit tests for RobotRepo: the slot the ``robot_state`` topic fills.

No ROS graph: the clock is injected, so the ageing is tested by moving time
rather than waiting (the pattern of test_mapping_status_repo.py). The ageing is
the behaviour worth pinning. syncai_robot_state can die while this process
lives on, and until a read aged the sample out its last word went on answering
for it: a state chip showing IDLE for a robot nobody was reporting on, a
navigation goal sent on a confirmation nobody was vouching for.
"""

import pytest

pytest.importorskip("syncai_common")

from syncai_common.msg import RobotStatus  # noqa: E402

from syncai_backend.repositories.robot.robot import (  # noqa: E402
    ROBOT_STATE_MAX_AGE_S,
    init_robot_repo,
    status_name,
)


class _Clock:
    def __init__(self):
        self.now = 500.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return _Clock()


@pytest.fixture
def repo(logger, clock):
    return init_robot_repo(logger=logger, now=clock)


def test_empty_until_a_sample_lands(repo):
    assert repo.get_robot_state() is None
    # Never heard from: no age either, which is how "not yet" is told from
    # "went silent".
    assert repo.sample_age() is None
    assert repo.status() is None
    assert repo.state_message() == ""


def test_a_fresh_sample_is_returned_as_stored(repo, clock, make_robot_state):
    msg = make_robot_state(state=RobotStatus.ERROR, state_message="A task failed.")
    repo.update_robot_state(state=msg)
    clock.now += 1.0

    assert repo.get_robot_state() is msg
    assert repo.sample_age() == pytest.approx(1.0)
    assert repo.status() == "ERROR"
    assert repo.state_message() == "A task failed."


def test_a_sample_older_than_the_limit_reads_as_none(repo, clock, make_robot_state):
    repo.update_robot_state(state=make_robot_state(state=RobotStatus.IDLE))
    clock.now += ROBOT_STATE_MAX_AGE_S
    # At exactly the limit it is still believed; one tick past it, not.
    assert repo.get_robot_state() is not None
    assert repo.status() == "IDLE"
    clock.now += 0.001

    assert repo.get_robot_state() is None


def test_every_default_read_agrees_on_a_stale_sample(repo, clock, make_robot_state):
    # One answer for every reader: the state route, the task guard, the
    # landmark route. None of them may be handed the last word of a robot
    # that stopped reporting.
    repo.update_robot_state(
        state=make_robot_state(state=RobotStatus.ERROR, state_message="A task failed.")
    )
    clock.now += ROBOT_STATE_MAX_AGE_S + 4.5

    assert repo.get_robot_state() is None
    assert repo.status() is None
    assert repo.state_message() == ""
    # Only the age survives, for the sentence that says how long.
    sample = repo.sample()
    assert sample.state is None
    assert sample.age_s == pytest.approx(ROBOT_STATE_MAX_AGE_S + 4.5)
    assert repo.sample_age() == pytest.approx(ROBOT_STATE_MAX_AGE_S + 4.5)


def test_a_new_sample_restarts_the_clock(repo, clock, make_robot_state):
    repo.update_robot_state(state=make_robot_state(state=RobotStatus.IDLE))
    clock.now += ROBOT_STATE_MAX_AGE_S + 10.0
    assert repo.get_robot_state() is None

    # The state node came back (a mode switch rebuilds it): UNINITIALIZED again.
    repo.update_robot_state(state=make_robot_state(state=RobotStatus.UNINITIALIZED))

    assert repo.status() == "UNINITIALIZED"
    assert repo.sample_age() == pytest.approx(0.0)


def test_a_later_sample_replaces_the_earlier_one(repo, make_robot_state):
    repo.update_robot_state(state=make_robot_state(state=RobotStatus.IDLE))
    later = make_robot_state(state=RobotStatus.RUNNING)
    repo.update_robot_state(state=later)

    assert repo.get_robot_state() is later


def test_the_limit_is_three_periods_of_the_shipped_publish_rate():
    # syncai_robot_state publishes at 1 Hz, best-effort: three periods, so two
    # frames dropped in a row are not a dead robot. Change the rate and this
    # moves with it.
    assert ROBOT_STATE_MAX_AGE_S == 3.0


def test_the_default_clock_is_the_process_monotonic_clock(logger, make_robot_state):
    # What main.py builds: no injected clock, and a sample just stored is fresh.
    repo = init_robot_repo(logger=logger)
    repo.update_robot_state(state=make_robot_state())

    assert repo.get_robot_state() is not None
    assert 0.0 <= repo.sample_age() < ROBOT_STATE_MAX_AGE_S


def test_an_unknown_code_is_a_name_not_a_key_error():
    assert status_name(99) == "UNKNOWN"
