import os
import time
import structlog

from sqlalchemy import Engine, create_engine, text

from sqlalchemy_utils import create_database, database_exists

MAX_RETRIES = 20
RETRY_INTERVAL = 5

# libpq's connect_timeout, for every connection the pool opens after startup.
# Without it a postgres host that does not answer holds the request for libpq's
# own default (minutes). The startup retries above do not use it: they go
# through sqlalchemy_utils, which opens its own connection from the URL.
CONNECT_TIMEOUT_S = 5


def _execute(engine: Engine, clause: str, raise_error: bool = True):

    with engine.connect() as conn:
        try:
            conn.execute(text(clause))
            conn.commit()
        except Exception as e:
            if "psycopg2.errors.DuplicateColumn" in str(e):
                return

            if raise_error:
                raise e


def connect_to_postgres(logger: structlog.stdlib.BoundLogger, robot_id: str) -> Engine:
    pg_user = os.getenv("POSTGRES_USER", "syncrobotic")
    pg_password = os.getenv("POSTGRES_PASSWORD", "syncrobotic")
    pg_host = os.getenv("POSTGRES_HOST", "localhost")
    pg_port = int(os.getenv("POSTGRES_PORT", 5432))
    pg_db = f"{robot_id}_db"

    engine = create_engine(
        f"postgresql://{pg_user}:{pg_password}@{pg_host}:{pg_port}/{pg_db}",
        pool_size=5,
        max_overflow=10,
        # Pooled connections outlive a postgres restart (its own compose stack,
        # restarted independently of this process) without knowing it: the
        # next checkout of each dead one failed its request with "server
        # closed the connection unexpectedly" -- a 500 per pooled connection,
        # then fine again, which reads as a random fault. pre_ping sends a
        # SELECT 1 on checkout and, when that fails, drops every connection
        # older than the failure and reconnects, transparently. ~0.1 ms on lo.
        pool_pre_ping=True,
        connect_args={"connect_timeout": CONNECT_TIMEOUT_S},
    )

    # attempt to connect to the database, retrying if it fails
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            if not database_exists(url=engine.url):
                create_database(url=engine.url)
            return engine
        except Exception as err:
            logger.warning(
                "Connection attempt failed",
                component="Postgres",
                attempt=attempt,
                max_retries=MAX_RETRIES,
                error=str(err),
            )
            if attempt == MAX_RETRIES:
                raise err
            time.sleep(RETRY_INTERVAL)
