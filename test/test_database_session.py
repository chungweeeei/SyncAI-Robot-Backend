"""database/session.py: the factory and the log-and-reraise every repo shares."""

from unittest.mock import MagicMock

import pytest
from sqlalchemy import create_engine, text

from syncai_backend.database.session import logged_session, session_factory
from syncai_backend.exceptions import ConflictError


@pytest.fixture
def factory():
    return session_factory(create_engine("sqlite://"))


def test_the_factory_keeps_rows_readable_after_commit(factory):
    session = factory()
    assert session.expire_on_commit is False
    assert session.autoflush is False
    session.close()


def test_a_clean_block_logs_nothing(factory):
    logger = MagicMock()

    with logged_session(factory, logger, "[Repo][op]") as session:
        assert session.execute(text("SELECT 1")).scalar() == 1

    logger.error.assert_not_called()


def test_a_failure_is_logged_with_its_traceback_and_reraised(factory):
    logger = MagicMock()

    with pytest.raises(RuntimeError, match="boom"):
        with logged_session(factory, logger, "[MapRepo][create_vertices]"):
            raise RuntimeError("boom")

    logger.error.assert_called_once_with(
        "[MapRepo][create_vertices] database operation failed", exc_info=True
    )


def test_a_quiet_exception_passes_unlogged(factory):
    """A duplicate name is a 409 the operator caused, not a DB fault."""
    logger = MagicMock()

    with pytest.raises(ConflictError):
        with logged_session(factory, logger, "[MapRepo][op]", quiet=(ConflictError,)):
            raise ConflictError("taken")

    logger.error.assert_not_called()
