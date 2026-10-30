"""The robot's internal REST service (``/internal/...``), over HTTP.

It keeps its own record of the maps, keyed by a uuid that survives a rename,
and its own idea of which one is active. The map switch tells it after the
switch is recorded (services/map_switch.py).

Returns ``(success, message, payload)`` like the other HTTP gateway (tts);
the service's ``{"error": {"code", "message"}}`` body becomes the sentence.
``httpx.Client`` is safe to share across the uvicorn threadpool.
"""

import os
from typing import Optional, Tuple

import httpx
import structlog


# Host networking, so 127.0.0.1 is the robot's own loopback; INTERNAL_API_URL
# overrides it.
_DEFAULT_BASE_URL = "http://127.0.0.1:8090"

# Connecting is local; a slow answer is the service thinking, not the network.
_CONNECT_TIMEOUT_S = 5.0

_REQUEST_TIMEOUT_S = 10.0

# The service's activate deadline is 240 s, and hanging up early cancels the
# switch where it stands, so wait past it for its own answer.
_ACTIVATE_TIMEOUT_S = 250.0


class InternalApiGateway:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        base_url: str = _DEFAULT_BASE_URL,
    ):
        self._logger = logger
        self._base_url = base_url.rstrip("/")

        self._client = httpx.Client(
            base_url=self._base_url,
            timeout=httpx.Timeout(_REQUEST_TIMEOUT_S, connect=_CONNECT_TIMEOUT_S),
        )

        # The URL comes from env, so log which address we will try.
        self._logger.info("[InternalApiGateway] Using internal service", url=self._base_url)

    # --- HTTP ---------------------------------------------------------------

    def _call(
        self, method: str, path: str, timeout: float
    ) -> Tuple[bool, str, Optional[httpx.Response]]:
        """One request, with every failure already turned into a sentence.

        Connecting is always bounded by ``_CONNECT_TIMEOUT_S``: httpx takes a
        per-request timeout *instead of* the client's, and a bare float would
        let an unanswered SYN wait the whole 250 s of an activate.
        """
        try:
            response = self._client.request(
                method,
                path,
                timeout=httpx.Timeout(timeout, connect=_CONNECT_TIMEOUT_S),
            )
        except httpx.ConnectTimeout:
            return (
                False,
                f"could not reach the internal service at {self._base_url} within "
                f"{_CONNECT_TIMEOUT_S:.0f}s",
                None,
            )
        except httpx.TimeoutException:
            return (
                False,
                f"the internal service at {self._base_url} did not answer within "
                f"{timeout:.0f}s",
                None,
            )
        except httpx.RequestError as exc:
            return (
                False,
                f"could not reach the internal service at {self._base_url}: {exc}",
                None,
            )

        if not response.is_success:
            return False, _error_message(response), None

        return True, "", response

    # --- Maps ---------------------------------------------------------------

    def find_map_id(self, name: str) -> Tuple[bool, str, Optional[str]]:
        """The uuid the service knows map ``name`` by.

        ``(True, "", None)`` when the service answered but has no such map;
        ``success`` is False only when it could not be asked.
        """
        ok, message, response = self._call("GET", "/internal/maps", _REQUEST_TIMEOUT_S)
        if not ok:
            return False, message, None

        try:
            maps = _data(response.json())
        except ValueError as exc:
            return False, f"the internal service returned an unreadable map list: {exc}", None
        if not isinstance(maps, list):
            return False, "the internal service returned an unreadable map list", None

        for summary in maps:
            if isinstance(summary, dict) and summary.get("name") == name:
                map_id = summary.get("id")
                if isinstance(map_id, str) and map_id:
                    return True, "", map_id
        return True, "", None

    def activate_map(self, map_id: str) -> Tuple[bool, str, Optional[dict]]:
        """Switch the service onto map ``map_id``; blocks up to its 240 s deadline.

        The payload is its ActivateMapResponse (``id``, ``name``, ``switched``,
        ``previous``, ``localized``, ``message``).
        """
        self._logger.info("[InternalApiGateway] Activating map", map_id=map_id)
        ok, message, response = self._call(
            "POST", f"/internal/maps/{map_id}/activate", _ACTIVATE_TIMEOUT_S
        )
        if not ok:
            return False, message, None

        try:
            payload = _data(response.json())
        except ValueError:
            payload = None
        if not isinstance(payload, dict):
            # The switch was answered with a success status; an unreadable body
            # does not undo it.
            payload = {}
        return True, "", payload


def _data(body):
    """The service wraps payloads in ``{"data": ...}``; accept a bare one too."""
    if isinstance(body, dict) and "data" in body:
        return body["data"]
    return body


def _error_message(response: httpx.Response) -> str:
    """``{"error": {"code", "message"}}`` as one sentence; the status otherwise."""
    code = message = None
    try:
        error = response.json().get("error")
    except (ValueError, AttributeError):
        error = None
    if isinstance(error, dict):
        code = error.get("code") if isinstance(error.get("code"), str) else None
        message = error.get("message") if isinstance(error.get("message"), str) else None

    text = f"the internal service answered {response.status_code}"
    if code:
        text += f" {code}"
    detail = message or response.text[:200]
    if detail:
        text += f": {detail}"
    return text


def init_internal_api_gateway(
    logger: structlog.stdlib.BoundLogger, base_url: Optional[str] = None
) -> InternalApiGateway:
    # Read here, not at import, so it runs after main.py's load_dotenv().
    resolved = base_url or os.getenv("INTERNAL_API_URL") or _DEFAULT_BASE_URL
    return InternalApiGateway(logger=logger, base_url=resolved)
