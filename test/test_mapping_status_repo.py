"""Unit tests for MappingStatusRepo: the slot GET /api/v1/mapping reads.

No ROS: the clock is injected, so the TTL is tested by moving time rather than
waiting. The one behaviour worth pinning is the ageing -- the other single-slot
caches never expire, and this one does because the backend container outlives
the mapping session that latched the sample.
"""

import pytest

from syncai_backend.repositories.mapping.mapping import (
    MAPPING_STATUS_TTL_S,
    MappingState,
    init_mapping_status_repo,
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
    return init_mapping_status_repo(logger=logger, now=clock)


def test_empty_until_a_sample_lands(repo):
    assert repo.get() is None


def test_a_fresh_sample_is_returned_whole(repo, clock):
    repo.update(state=MappingState.MAPPING, key_poses=7, loop_closures=1, stamp=12.5)

    status = repo.get()

    assert status is not None
    assert status.state is MappingState.MAPPING
    assert (status.key_poses, status.loop_closures, status.stamp) == (7, 1, 12.5)
    assert status.received_at == clock.now


def test_a_sample_older_than_the_ttl_reads_as_none(repo, clock):
    repo.update(state=MappingState.IDLE, key_poses=0, loop_closures=0, stamp=1.0)
    clock.now += MAPPING_STATUS_TTL_S
    # At exactly the TTL it is still believed; one tick past it, not.
    assert repo.get() is not None
    clock.now += 0.001

    assert repo.get() is None


def test_a_new_sample_restarts_the_clock(repo, clock):
    repo.update(state=MappingState.IDLE, key_poses=0, loop_closures=0, stamp=1.0)
    clock.now += MAPPING_STATUS_TTL_S + 1.0
    assert repo.get() is None

    repo.update(state=MappingState.MAPPING, key_poses=1, loop_closures=0, stamp=2.0)

    assert repo.get().state is MappingState.MAPPING


def test_a_later_sample_replaces_the_earlier_one(repo):
    # Single slot: the save's IDLE must win over the MAPPING it follows.
    repo.update(state=MappingState.MAPPING, key_poses=40, loop_closures=2, stamp=1.0)
    repo.update(state=MappingState.IDLE, key_poses=0, loop_closures=0, stamp=2.0)

    status = repo.get()

    assert status.state is MappingState.IDLE
    assert status.key_poses == 0
