"""Speech out, over HTTP to the syncai_tts service.

The engine (kokoro-onnx + ``aplay``) runs in its own container (the
``SyncAI-TTS`` repo), which owns the speaker and serialises utterances in a
FIFO queue, so two processes cannot talk over each other.

**There is deliberately no lock in this file**: this process holds neither the
model nor the speaker. ``httpx.Client`` is safe to share across the uvicorn
threadpool and the Temporal activity thread.

Returns ``(success, message, payload)``, with caller-fixable failures tagged
via :mod:`syncai_backend.gateways.failure`; the service publishes matching
``code`` strings (``errors.py`` in the TTS repo).
"""

import os
import time
from typing import Callable, Dict, List, Optional, Tuple

import httpx
import structlog

from syncai_backend.gateways.failure import Failure, fail


# The compose service name; TTS_SERVICE_URL overrides it (e.g. 127.0.0.1:8080).
_DEFAULT_BASE_URL = "http://syncai_tts:8080"

# Connecting is local; a slow answer is the service thinking, not the network.
_CONNECT_TIMEOUT_S = 5.0

# Generous: the first request may wait on the service loading its ~310 MB model.
_REQUEST_TIMEOUT_S = 60.0

# `speak` holds the response until playback ends, so this covers an utterance
# plus whatever is queued ahead of it.
_SPEAK_TIMEOUT_S = 180.0

# `start_speak` waits only for (inline) synthesis. Public because the SPEAK
# activity cannot heartbeat during this POST: its heartbeat_timeout must
# outlast it (pinned in test_activities.py).
SPEAK_START_TIMEOUT_S = _REQUEST_TIMEOUT_S

# Stopping playback is quick on the service's side; slower means not answering.
_CANCEL_TIMEOUT_S = 10.0

# Backstop for waiting on a job. Under the SPEAK activity's 5-minute
# start_to_close, so this gives up with a sentence before Temporal does.
_SPEAK_DEADLINE_S = 240.0

# How often a playing job is read. SPEAK heartbeats once per read and a cancel
# arrives only on a heartbeat's reply, so keep this at or under the worker's
# HEARTBEAT_THROTTLE_MAX (pinned in test_activities.py).
SPEAK_POLL_INTERVAL_S = 0.25

# The service's codes this side answers differently, keyed by the service's
# literal strings (only `unknown_voice` is shared on purpose). Everything else
# is the robot's problem and stays an untagged 502.
_FAILURE_BY_CODE = {
    "unknown_voice": Failure.UNKNOWN_VOICE,
    "queue_full": Failure.TTS_QUEUE_FULL,
}

_TERMINAL_STATUSES = frozenset({"done", "failed", "cancelled"})


class TtsGateway:
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
        self._logger.info("[TtsGateway] Using speech service", url=self._base_url)

    # --- HTTP ---------------------------------------------------------------

    def _call(
        self, method: str, path: str, timeout: float, **kwargs
    ) -> Tuple[bool, str, Optional[httpx.Response]]:
        """One request, with every failure already turned into a sentence.

        ``timeout`` bounds the whole exchange; connecting is always bounded by
        ``_CONNECT_TIMEOUT_S``. Spelled out per request because httpx takes a
        per-request timeout *instead of* the client's, and a bare float sets
        connect to that float too -- which made an unreachable address (a wrong
        IP, a host that is off, a dropped port) wait the full 60-180 s before
        failing, holding the SPEAK activity's slot and the REST threadpool for
        as long. A refused connection is quick either way; this is for the one
        that never answers the SYN.
        """
        try:
            response = self._client.request(
                method,
                path,
                timeout=httpx.Timeout(timeout, connect=_CONNECT_TIMEOUT_S),
                **kwargs,
            )
        except httpx.ConnectTimeout:
            # Told apart from a slow answer: the operator's fix is the address
            # or the container, not patience.
            return (
                False,
                f"could not reach the speech service at {self._base_url} within "
                f"{_CONNECT_TIMEOUT_S:.0f}s",
                None,
            )
        except httpx.TimeoutException:
            return (
                False,
                f"the speech service at {self._base_url} did not answer within "
                f"{timeout:.0f}s",
                None,
            )
        except httpx.RequestError as exc:
            # Usually the container is down or TTS_SERVICE_URL is wrong.
            return (
                False,
                f"could not reach the speech service at {self._base_url}: {exc}",
                None,
            )

        if response.status_code >= 400:
            return False, self._error_message(response), None

        return True, "", response

    def _error_message(self, response: httpx.Response) -> str:
        """Lift the service's ``{detail, code}`` body into our own convention.

        The code is re-tagged because callers read :func:`failure_code`, not
        the sentence.
        """
        detail = ""
        code = None
        try:
            body = response.json()
        except ValueError:
            body = None
        if isinstance(body, dict):
            detail = str(body.get("detail") or "")
            code = _FAILURE_BY_CODE.get(body.get("code"))

        if not detail:
            detail = (
                f"the speech service answered {response.status_code} "
                f"{response.reason_phrase}".strip()
            )

        return fail(code, detail) if code is not None else detail

    # --- API ----------------------------------------------------------------

    def list_voices(self) -> Tuple[bool, str, List[str]]:
        success, message, response = self._call(
            "GET", "/api/v1/voices", timeout=_REQUEST_TIMEOUT_S
        )
        if not success:
            return False, message, []

        try:
            voices = list(response.json()["voices"])
        except (ValueError, KeyError, TypeError) as exc:
            return False, f"the speech service returned an unreadable voice list: {exc}", []

        return True, "", voices

    def synthesize(
        self, text: str, voice: str = "af_heart", speed: float = 1.0
    ) -> Tuple[bool, str, bytes]:
        """Render text to a mono 16-bit WAV. Nothing reaches the speaker.

        An unknown voice comes back tagged ``Failure.UNKNOWN_VOICE``.
        """
        success, message, response = self._call(
            "POST",
            "/api/v1/synthesize",
            timeout=_REQUEST_TIMEOUT_S,
            json=self._payload(text=text, voice=voice, speed=speed),
        )
        if not success:
            return False, message, b""

        return True, "", response.content

    def speak(
        self, text: str, voice: str = "af_heart", speed: float = 1.0
    ) -> Tuple[bool, str, Optional[float]]:
        """Speak on the robot speaker, blocking until playback finishes.

        ``wait=true`` asks the service to hold the response; it answers 202
        with a non-terminal job if its own wait expires, hence the rare poll.

        For ``POST /api/v1/tts/speak``. The SPEAK activity must heartbeat and
        be cancellable, so it drives the job itself (``start_speak``,
        ``await_speak``, ``cancel_speak``).
        """
        success, message, response = self._call(
            "POST",
            "/api/v1/speak",
            timeout=_SPEAK_TIMEOUT_S,
            json={**self._payload(text=text, voice=voice, speed=speed), "wait": True},
        )
        if not success:
            return False, message, None

        success, message, job = self._job_of(response)
        if not success:
            return False, message, None

        success, message, job = self.await_speak(job)
        if not success:
            return False, message, None

        return self.speak_outcome(job)

    def start_speak(
        self, text: str, voice: str = "af_heart", speed: float = 1.0
    ) -> Tuple[bool, str, Optional[dict]]:
        """Queue an utterance and return its job without waiting for playback.

        Synthesis is inline, so an unknown voice is answered here, as in
        ``speak``.
        """
        success, message, response = self._call(
            "POST",
            "/api/v1/speak",
            timeout=SPEAK_START_TIMEOUT_S,
            json={**self._payload(text=text, voice=voice, speed=speed), "wait": False},
        )
        if not success:
            return False, message, None

        return self._job_of(response)

    def await_speak(
        self, job: dict, on_poll: Optional[Callable[[str], None]] = None
    ) -> Tuple[bool, str, Optional[dict]]:
        """Poll ``job`` until it stops moving, or we run out of patience.

        ``on_poll`` gets the status before every wait; SPEAK passes
        ``activity.heartbeat``, which is how a cancel reaches it mid-utterance.
        Bounded by ``_SPEAK_DEADLINE_S``.
        """
        deadline = time.monotonic() + _SPEAK_DEADLINE_S

        while job.get("status") not in _TERMINAL_STATUSES:
            if time.monotonic() >= deadline:
                self._logger.warning(
                    "[TtsGateway] Gave up waiting for an utterance",
                    job_id=job.get("id"),
                    status=job.get("status"),
                )
                return (
                    False,
                    f"the utterance was still {job.get('status')!r} after "
                    f"{_SPEAK_DEADLINE_S:.0f}s; the speech queue is backed up",
                    None,
                )

            if on_poll is not None:
                on_poll(job.get("status"))

            time.sleep(SPEAK_POLL_INTERVAL_S)

            success, message, response = self._call(
                "GET", f"/api/v1/speak/{job.get('id')}", timeout=_REQUEST_TIMEOUT_S
            )
            if not success:
                return False, message, None
            success, message, job = self._job_of(response)
            if not success:
                return False, message, None

        return True, "", job

    def cancel_speak(self, job_id: str) -> Tuple[bool, str]:
        """Stop a job, queued or playing.

        Cancelling a finished job is not an error, so racing the end is fine.
        """
        success, message, _ = self._call(
            "DELETE", f"/api/v1/speak/{job_id}", timeout=_CANCEL_TIMEOUT_S
        )
        return success, message

    @staticmethod
    def speak_outcome(job: dict) -> Tuple[bool, str, Optional[float]]:
        """A terminal job as ``speak``'s ``(success, message, duration)``."""
        status = job.get("status")
        if status == "done":
            # Always a float: SpeakResponse.duration is required, so a missing
            # value would turn a spoken utterance into a 500.
            duration = job.get("duration")
            return True, "", float(duration) if duration is not None else 0.0

        detail = job.get("error") or f"playback ended {status!r}"
        code = _FAILURE_BY_CODE.get(job.get("code"))
        return False, (fail(code, detail) if code is not None else detail), None

    # --- Internals ----------------------------------------------------------

    @staticmethod
    def _payload(text: str, voice: str, speed: float) -> Dict[str, object]:
        return {"text": text, "voice": voice, "speed": speed}

    @staticmethod
    def _job_of(response: httpx.Response) -> Tuple[bool, str, Optional[dict]]:
        try:
            return True, "", response.json()
        except ValueError as exc:
            return False, f"the speech service returned an unreadable job: {exc}", None


def init_tts_gateway(
    logger: structlog.stdlib.BoundLogger, base_url: Optional[str] = None
) -> TtsGateway:
    # Read here, not at import, so it runs after main.py's load_dotenv().
    resolved = base_url or os.getenv("TTS_SERVICE_URL") or _DEFAULT_BASE_URL
    return TtsGateway(logger=logger, base_url=resolved)
