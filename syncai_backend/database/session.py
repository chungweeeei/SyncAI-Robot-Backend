"""How a repository opens a session, shared by every repository on the engine.

One factory configuration (so a repo cannot quietly differ in how its rows
outlive a commit) and one log-and-reraise wrapper (the SQLAlchemy error carries
the failed statement; the prefix says which repo and which operation).
"""

from collections.abc import Generator
from contextlib import contextmanager
from typing import Tuple, Type

import structlog
from sqlalchemy import Engine
from sqlalchemy.orm import Session, sessionmaker


def session_factory(engine: Engine) -> sessionmaker:
    """The repositories' factory: no autoflush, and rows stay readable after
    commit (``expire_on_commit=False``), since they are returned to the router
    after the session closes."""
    return sessionmaker(
        autocommit=False,
        autoflush=False,
        expire_on_commit=False,
        bind=engine,
    )


@contextmanager
def logged_session(
    factory: sessionmaker,
    logger: structlog.stdlib.BoundLogger,
    where: str,
    quiet: Tuple[Type[BaseException], ...] = (),
) -> Generator[Session, None, None]:
    """A session that logs (with traceback) whatever escapes it, then re-raises.

    ``where`` prefixes the line, e.g. ``[MapRepo][create_vertices]``. ``quiet``
    exceptions pass through unlogged: a domain outcome (a duplicate name is a
    409), not a database fault.
    """
    with factory() as session:
        try:
            yield session
        except quiet:
            raise
        except Exception:
            logger.error(f"{where} database operation failed", exc_info=True)
            raise
