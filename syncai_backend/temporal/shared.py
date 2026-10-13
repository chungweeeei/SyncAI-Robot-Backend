import os

# Temporal frontend gRPC endpoint; defaults to the local docker-compose service.
DEFAULT_TEMPORAL_ADDRESS = "127.0.0.1:7233"


def temporal_server_url() -> str:
    """The Temporal address, read from the environment at call time.

    A function, not a module constant: main.py imports this before its own
    ``dotenv.load_dotenv()``, so an import-time read never saw a TEMPORAL_ADDRESS
    that lived only in ``.env``. Every caller runs after load_dotenv.
    """
    return os.getenv("TEMPORAL_ADDRESS", DEFAULT_TEMPORAL_ADDRESS)
