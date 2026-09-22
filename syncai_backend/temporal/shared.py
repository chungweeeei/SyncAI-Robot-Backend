import os

# Temporal server frontend gRPC endpoint. Defaults to the local docker-compose
# service (`temporal:7233` inside the compose network / `127.0.0.1:7233` on host).
DEFAULT_TEMPORAL_ADDRESS = "127.0.0.1:7233"


def temporal_server_url() -> str:
    """The Temporal address, read from the environment at call time.

    A function rather than a module constant on purpose. This used to be
    ``TEMPORAL_SERVER_URL = os.getenv(...)`` evaluated at import, and main.py
    imports temporal.worker (which imports this) on line 11 -- before its own
    ``dotenv.load_dotenv()`` on line 52. A TEMPORAL_ADDRESS that lived only in
    ``.env`` was therefore never seen and both clients silently used the
    default; it worked in deployments only because compose also injected the
    variable into the real environment. Every caller here runs from inside
    SyncAIBackend.__init__ or later, well after load_dotenv, so reading at call
    time sees the value.
    """
    return os.getenv("TEMPORAL_ADDRESS", DEFAULT_TEMPORAL_ADDRESS)
