"""connect_to_postgres: the engine it builds.

The connection itself is not made here (no PostgreSQL in the suite); what is
pinned is how the pool is configured, because the two settings below are the
difference between a postgres restart being invisible and a run of 500s.
"""

import pytest

pytest.importorskip("sqlalchemy_utils")

import structlog  # noqa: E402

from syncai_backend.database import postgres  # noqa: E402


def _build_engine(monkeypatch):
    """Capture create_engine's arguments; pretend the database exists."""
    captured = {}

    def fake_create_engine(url, **kwargs):
        captured["url"] = url
        captured.update(kwargs)
        return _FakeEngine(url)

    monkeypatch.setattr(postgres, "create_engine", fake_create_engine)
    monkeypatch.setattr(postgres, "database_exists", lambda url: True)
    postgres.connect_to_postgres(structlog.get_logger(), robot_id="robot01")
    return captured


class _FakeEngine:
    def __init__(self, url):
        self.url = url


def test_the_pool_pings_a_connection_before_handing_it_out(monkeypatch):
    """Idle pooled connections outlive a postgres restart without knowing it;
    without pre_ping each one failed its next request, one 500 per connection."""
    assert _build_engine(monkeypatch)["pool_pre_ping"] is True


def test_connecting_is_bounded(monkeypatch):
    """libpq's default connect_timeout is minutes; a host that does not answer
    must fail the request, not park the threadpool."""
    args = _build_engine(monkeypatch)["connect_args"]

    assert args["connect_timeout"] == postgres.CONNECT_TIMEOUT_S
    assert 0 < postgres.CONNECT_TIMEOUT_S <= 10


def test_the_database_is_the_robots(monkeypatch):
    assert _build_engine(monkeypatch)["url"].endswith("/robot01_db")
