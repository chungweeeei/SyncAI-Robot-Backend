import structlog
from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field

from syncai_backend.exceptions import UnauthorizedError
from syncai_backend.interfaces.rest.auth import (
    AuthConfig,
    check_credentials,
    init_auth_guard,
    issue_token,
)


class LoginRequest(BaseModel):
    username: str = Field(..., max_length=128)
    password: str = Field(..., max_length=256)


class LoginResponse(BaseModel):
    access_token: str = Field(..., description="Sent back as `Authorization: Bearer <token>`.")
    token_type: str = Field("bearer")
    expires_at: int = Field(..., description="When the token stops working, in epoch seconds.")
    expires_in: int = Field(
        ...,
        description=(
            "Seconds the token lasts from now. What a client times its sign-out by: "
            "a robot's clock and the browser's need not agree."
        ),
    )


class SessionResponse(BaseModel):
    username: str
    expires_at: int


def init_auth_router(logger: structlog.stdlib.BoundLogger, auth: AuthConfig) -> APIRouter:
    auth_router = APIRouter(prefix="", tags=["Auth"])
    require_auth = init_auth_guard(auth)

    @auth_router.post("/api/v1/auth/login", response_model=LoginResponse)
    def login(request: LoginRequest):
        if not check_credentials(auth, request.username, request.password):
            logger.warning("Sign-in refused", username=request.username)
            # One sentence for either half wrong: which one was is the
            # guesser's next clue, not the operator's.
            raise UnauthorizedError("Wrong username or password.")
        token, expires_at = issue_token(auth, request.username)
        logger.info("Signed in", username=request.username)
        return LoginResponse(
            access_token=token, expires_at=expires_at, expires_in=auth.expire_s
        )

    @auth_router.get("/api/v1/auth/me", response_model=SessionResponse)
    def me(claims: dict = Depends(require_auth)):
        """Whose token this is, and until when: how a client checks that a
        token it kept still opens the robot, without touching anything else."""
        return SessionResponse(username=claims["sub"], expires_at=claims["exp"])

    return auth_router
