"""Unit tests for MapRepo: vertex CRUD against the SQLite-backed fixture.

The repo used to also cache the live map topic's OccupancyGrid; that went with
the endpoints that read it, and with it this module's need for nav_msgs.
"""

import pytest

import uuid

from syncai_backend.exceptions import ConflictError

_MISSING_ID = uuid.UUID("00000000-0000-0000-0000-000000000000")


def _create(repo, name="v", type="GENERAL", map="warehouse", x=1.0, y=2.0,
            theta=0.0):
    """Create a single vertex through the batch API and return it."""
    return repo.create_vertices(map=map, vertices=[{
        "name": name, "type": type, "x": x, "y": y, "theta": theta,
    }])[0]


def test_create_returns_persisted_vertex(map_repo):
    vertex = _create(map_repo, name="dock", x=3.0, y=-1.5, theta=90.0)

    assert vertex.id is not None
    assert vertex.name == "dock"
    assert vertex.type == "GENERAL"
    assert vertex.map == "warehouse"
    assert (vertex.x, vertex.y, vertex.theta) == (3.0, -1.5, 90.0)
    assert vertex.created_at is not None
    assert vertex.updated_at is not None


def test_create_vertices_batch_persists_all_in_order(map_repo):
    created = map_repo.create_vertices(map="warehouse", vertices=[
        {"name": "a", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
        {"name": "b", "type": "ARTIFACT", "x": 1.0, "y": 1.0, "theta": 0.0},
    ])

    assert [v.name for v in created] == ["a", "b"]
    assert all(isinstance(v.id, uuid.UUID) for v in created)
    assert {v.id for v in map_repo.list_vertices()} == {c.id for c in created}


def test_create_rejects_a_name_the_map_already_has(map_repo):
    _create(map_repo, name="dock")

    with pytest.raises(ConflictError) as caught:
        _create(map_repo, name="dock", x=9.0)

    assert caught.value.code == "vertex_name_taken"
    assert "dock" in str(caught.value)
    assert len(map_repo.list_vertices(map="warehouse")) == 1


def test_create_allows_the_same_name_on_a_different_map(map_repo):
    # The constraint is (map, name), not name: every map is expected to have
    # its own "dock", and forbidding that would be the wrong fix.
    first = _create(map_repo, name="dock", map="warehouse")
    second = _create(map_repo, name="dock", map="lab")

    assert first.id != second.id
    assert {v.map for v in map_repo.list_vertices()} == {"warehouse", "lab"}


def test_create_batch_rejects_a_name_repeated_within_the_request(map_repo):
    with pytest.raises(ConflictError) as caught:
        map_repo.create_vertices(map="warehouse", vertices=[
            {"name": "a", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
            {"name": "a", "type": "GENERAL", "x": 1.0, "y": 1.0, "theta": 0.0},
        ])

    assert caught.value.code == "vertex_name_taken"
    # Batch inserts are all-or-nothing, so the non-colliding row is gone too.
    assert map_repo.list_vertices() == []


def test_create_batch_inserts_nothing_when_one_name_collides(map_repo):
    _create(map_repo, name="taken")

    with pytest.raises(ConflictError):
        map_repo.create_vertices(map="warehouse", vertices=[
            {"name": "fresh", "type": "GENERAL", "x": 0.0, "y": 0.0, "theta": 0.0},
            {"name": "taken", "type": "GENERAL", "x": 1.0, "y": 1.0, "theta": 0.0},
        ])

    assert [v.name for v in map_repo.list_vertices()] == ["taken"]


def test_get_returns_vertex_and_none_when_missing(map_repo):
    created = _create(map_repo)

    fetched = map_repo.get_vertex(created.id)
    assert fetched is not None
    assert fetched.id == created.id

    assert map_repo.get_vertex(_MISSING_ID) is None


def test_list_vertices_orders_by_creation_time(map_repo):
    first = _create(map_repo, name="a")
    second = _create(map_repo, name="b")

    vertices = map_repo.list_vertices()
    assert [v.id for v in vertices] == [first.id, second.id]


def test_list_vertices_filters_by_map_and_type(map_repo):
    _create(map_repo, name="g1", type="GENERAL", map="warehouse")
    _create(map_repo, name="a1", type="ARTIFACT", map="warehouse")
    _create(map_repo, name="g2", type="GENERAL", map="office")

    assert len(map_repo.list_vertices(map="warehouse")) == 2
    assert len(map_repo.list_vertices(map="office")) == 1
    assert len(map_repo.list_vertices(type="GENERAL")) == 2
    assert len(map_repo.list_vertices(map="warehouse", type="ARTIFACT")) == 1
    assert map_repo.list_vertices(map="does-not-exist") == []


def test_count_vertices_matches_list_under_the_same_filters(map_repo):
    _create(map_repo, name="g1", type="GENERAL", map="warehouse")
    _create(map_repo, name="a1", type="ARTIFACT", map="warehouse")
    _create(map_repo, name="g2", type="GENERAL", map="office")

    for filters in (
        {},
        {"map": "warehouse"},
        {"map": "office"},
        {"type": "GENERAL"},
        {"map": "warehouse", "type": "ARTIFACT"},
        {"map": "does-not-exist"},
    ):
        assert map_repo.count_vertices(**filters) == len(
            map_repo.list_vertices(**filters)
        ), filters


def test_count_vertices_is_zero_not_none_on_an_empty_table(map_repo):
    # count(*) returns a row even with nothing to count; callers render this
    # straight into a response field, so it must be an int.
    assert map_repo.count_vertices(map="warehouse") == 0


def test_get_vertices_returns_only_the_requested_ids(map_repo):
    wanted = _create(map_repo, name="a", map="warehouse")
    _create(map_repo, name="b", map="warehouse")

    fetched = map_repo.get_vertices(map="warehouse", vertex_ids=[wanted.id])

    assert [v.id for v in fetched] == [wanted.id]


def test_get_vertices_excludes_ids_belonging_to_another_map(map_repo):
    # The map filter is what makes absence from the result mean "not a vertex
    # of this map" -- task templates report a cross-map reference from it.
    elsewhere = _create(map_repo, name="g2", map="office")

    assert map_repo.get_vertices(map="warehouse", vertex_ids=[elsewhere.id]) == []


def test_get_vertices_ignores_unknown_ids_and_an_empty_request(map_repo):
    known = _create(map_repo, name="a", map="warehouse")

    fetched = map_repo.get_vertices(
        map="warehouse", vertex_ids=[known.id, _MISSING_ID]
    )
    assert [v.id for v in fetched] == [known.id]

    assert map_repo.get_vertices(map="warehouse", vertex_ids=[]) == []


def test_update_changes_fields(map_repo):
    created = _create(map_repo, name="old", x=1.0)

    updated = map_repo.update_vertex(created.id, name="new", x=5.0)

    assert updated is not None
    assert updated.name == "new"
    assert updated.x == 5.0
    # Untouched fields are preserved.
    assert updated.y == 2.0


def test_update_ignores_unknown_and_none_fields(map_repo):
    created = _create(map_repo, name="keep")

    updated = map_repo.update_vertex(
        created.id, name=None, bogus="value", theta=45.0
    )

    assert updated is not None
    assert updated.name == "keep"  # None ignored
    assert updated.theta == 45.0
    assert not hasattr(updated, "bogus")


def test_update_rejects_a_rename_onto_a_taken_name(map_repo):
    _create(map_repo, name="dock")
    other = _create(map_repo, name="desk")

    with pytest.raises(ConflictError) as caught:
        map_repo.update_vertex(other.id, name="dock")

    assert caught.value.code == "vertex_name_taken"
    assert map_repo.get_vertex(other.id).name == "desk"


def test_update_allows_renaming_a_vertex_to_the_name_it_already_has(map_repo):
    # A row is not its own duplicate; a no-op PUT must not 409.
    created = _create(map_repo, name="dock")

    updated = map_repo.update_vertex(created.id, name="dock", x=7.0)

    assert updated.name == "dock"
    assert updated.x == 7.0


def test_update_allows_a_name_taken_only_on_another_map(map_repo):
    _create(map_repo, name="dock", map="lab")
    created = _create(map_repo, name="desk", map="warehouse")

    assert map_repo.update_vertex(created.id, name="dock").name == "dock"


def test_update_missing_returns_none(map_repo):
    assert map_repo.update_vertex(_MISSING_ID, name="x") is None


def test_delete_removes_vertex(map_repo):
    created = _create(map_repo)

    assert map_repo.delete_vertex(created.id) is True
    assert map_repo.get_vertex(created.id) is None


def test_delete_missing_returns_false(map_repo):
    assert map_repo.delete_vertex(_MISSING_ID) is False


def test_move_vertices_rekeys_only_that_map(map_repo):
    a = _create(map_repo, name="a", map="old")
    b = _create(map_repo, name="b", map="old")
    other = _create(map_repo, name="c", map="other")

    moved = map_repo.move_vertices("old", "new")

    assert moved == 2
    assert {v.id for v in map_repo.list_vertices(map="new")} == {a.id, b.id}
    assert map_repo.list_vertices(map="old") == []
    assert map_repo.get_vertex(other.id).map == "other"


def test_move_vertices_bumps_updated_at_and_keeps_ids(map_repo):
    """A Core UPDATE skips ``onupdate``; the repo sets updated_at itself."""
    created = _create(map_repo, map="old")
    # Read back before moving: SQLite hands timestamps back naive, so the
    # comparison has to be between two rows that took the same path.
    before = map_repo.get_vertex(created.id)

    map_repo.move_vertices("old", "new")

    after = map_repo.get_vertex(created.id)
    assert after.id == before.id
    assert after.created_at == before.created_at
    assert after.updated_at >= before.updated_at


def test_move_vertices_in_a_caller_transaction_is_committed_by_the_block(map_repo):
    """The rename cascade's contract: the block, not the method, commits.

    Isolation itself is not asserted here on purpose -- the test engine is a
    single-connection SQLite StaticPool, so a second session shares the
    connection and sees uncommitted rows. The rollback test below is what pins
    that nothing lands when the block fails.
    """
    created = _create(map_repo, map="old")

    with map_repo.transaction(op="test") as session:
        assert map_repo.move_vertices("old", "new", session=session) == 1

    assert map_repo.get_vertex(created.id).map == "new"


def test_move_vertices_in_a_failed_transaction_is_rolled_back(map_repo):
    created = _create(map_repo, map="old")

    with pytest.raises(RuntimeError):
        with map_repo.transaction(op="test") as session:
            map_repo.move_vertices("old", "new", session=session)
            raise RuntimeError("the second update failed")

    assert map_repo.get_vertex(created.id).map == "old"


def test_move_vertices_of_an_unknown_map_moves_nothing(map_repo):
    _create(map_repo, map="warehouse")

    assert map_repo.move_vertices("ghost", "new") == 0
    assert len(map_repo.list_vertices(map="warehouse")) == 1
