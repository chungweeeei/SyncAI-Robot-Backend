"""The console's sign-in: one operator account, a signed token, and the guard.

There is one account, `admin`, whose name and password come from the
environment; there is no user table and no way to add one over the API, by
design for now. A sign-in answers a JWT (HS256) that the console sends on every
request, and which expires on its own: the token is the whole session, so there
is nothing to revoke server-side and a sign-out is the console forgetting it.
"""

import hmac
import os
import secrets
import time
from dataclasses import dataclass

import jwt
import structlog
from fastapi import WebSocketException, status
from starlette.requests import HTTPConnection

from syncai_backend.exceptions import UnauthorizedError


_DEFAULT_USERNAME = "admin"
_DEFAULT_PASSWORD = "admin"
_DEFAULT_EXPIRE_S = 3600
_ALGORITHM = "HS256"

# Said for every refusal of a request without a usable token, so the answer
# never tells a caller whether the token was missing, forged or merely old.
SIGN_IN_AGAIN = "Your session has ended. Sign in again."


@dataclass(frozen=True)
class AuthConfig:
    username: str
    password: str
    secret: str
    expire_s: int

    @classmethod
    def from_env(cls, logger: structlog.stdlib.BoundLogger) -> "AuthConfig":
        """Read once at startup, after load_dotenv.

        Without JWT_SECRET the key is drawn fresh per process, so a backend
        restart signs every console out. That is a fair price on a robot,
        where restarts are rare, and it means no default secret ships that
        forges tokens on every robot at once.
        """
        password = os.getenv("AUTH_PASSWORD", _DEFAULT_PASSWORD)
        if password == _DEFAULT_PASSWORD:
            logger.warning("AUTH_PASSWORD is unset or the default; change it")
        secret = os.getenv("JWT_SECRET") or secrets.token_urlsafe(32)
        return cls(
            username=os.getenv("AUTH_USERNAME", _DEFAULT_USERNAME),
            password=password,
            secret=secret,
            expire_s=int(os.getenv("JWT_EXPIRE_SECONDS", str(_DEFAULT_EXPIRE_S))),
        )


def check_credentials(cfg: AuthConfig, username: str, password: str) -> bool:
    # compare_digest on both, and both always compared, so the answer's timing
    # does not say which of the two was wrong.
    name_ok = hmac.compare_digest(username.encode(), cfg.username.encode())
    password_ok = hmac.compare_digest(password.encode(), cfg.password.encode())
    return name_ok and password_ok


def issue_token(
    cfg: AuthConfig, username: str, now: "float | None" = None
) -> tuple[str, int]:
    """A signed token for ``username`` and its expiry, in epoch seconds."""
    issued = int(time.time() if now is None else now)
    expires_at = issued + cfg.expire_s
    token = jwt.encode(
        {"sub": username, "iat": issued, "exp": expires_at},
        cfg.secret,
        algorithm=_ALGORITHM,
    )
    return token, expires_at


def verify_token(cfg: AuthConfig, token: str) -> dict:
    """The token's claims, or UnauthorizedError when it is not one of ours."""
    try:
        # The algorithm list is pinned: a token naming another (`none`) is refused.
        return jwt.decode(
            token, cfg.secret, algorithms=[_ALGORITHM], options={"require": ["exp", "sub"]}
        )
    except jwt.InvalidTokenError as exc:
        raise UnauthorizedError(SIGN_IN_AGAIN) from exc


def _bearer(conn: HTTPConnection) -> "str | None":
    header = conn.headers.get("authorization", "")
    scheme, _, value = header.partition(" ")
    if scheme.lower() == "bearer" and value:
        return value.strip()
    # A browser cannot set a header on a WebSocket, so the streams carry the
    # token in the query; nothing else should need to.
    return conn.query_params.get("token")


def init_auth_guard(cfg: AuthConfig):
    """The dependency every route but sign-in and /health sits behind.

    It takes an HTTPConnection rather than a Request so the one function guards
    the WebSocket streams too, where a refusal has to be a close before the
    accept instead of a 401.
    """

    def require_auth(conn: HTTPConnection) -> dict:
        token = _bearer(conn)
        try:
            if not token:
                raise UnauthorizedError(SIGN_IN_AGAIN)
            return verify_token(cfg, token)
        except UnauthorizedError:
            if conn.scope["type"] == "websocket":
                raise WebSocketException(
                    code=status.WS_1008_POLICY_VIOLATION, reason=SIGN_IN_AGAIN
                )
            raise

    return require_auth
