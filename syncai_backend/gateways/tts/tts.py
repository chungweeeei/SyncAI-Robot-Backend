"""Speech out, over HTTP to the syncai_tts service.

This used to be the speech engine itself: a kokoro-onnx session and an ``aplay``
subprocess living in this process. It is now a client, and the engine runs in
its own container (the ``SyncAI-TTS`` repo). Two things moved it.

**The speaker needs exactly one owner.** "One aplay stream on the pcm node at a
time" was enforced here by a ``threading.Lock``, which holds only while every
caller is in this process. Moving the Temporal worker into a process of its own
breaks that: two ``TtsGateway`` instances hold two locks and nothing stops a
scheduled SPEAK step from talking over a manual ``POST /api/v1/tts/speak``. The
service puts the lock back in front of the single piece of hardware.

**The pins had nothing to do with ROS.** ``onnxruntime==1.18.1`` is pinned for
the Orin's offlined cores, and in here that pin also had to survive alongside
the ROS ecosystem's numpy ceiling — which is why ``kokoro-onnx`` had to be
installed ``--no-deps``. None of that is this repo's problem any more, and the
backend process sheds a ~310 MB model from its address space.

**There is deliberately no lock left in this file.** Both of the ones that were
here guarded resources this process no longer holds: the inference session and
the speaker. Serialisation is the service's job now, and it is a FIFO queue
rather than a lock, so utterances also come out in the order they were accepted.
``httpx.Client`` is safe to share across the uvicorn threadpool and the Temporal
activity thread, and pooling connections is the point of holding one.

The return shape is unchanged — ``(success, message, payload)`` with the one
caller-fixable failure tagged via :mod:`syncai_backend.gateways.failure` — so
``routers/tts.py`` and the SPEAK activity did not change when this was swapped.
The service publishes the same ``code`` strings for exactly that reason; see
``errors.py`` in the TTS repo.
"""

import os
import time
from typing import Dict, List, Optional, Tuple

import httpx
import structlog

from syncai_backend.gateways.failure import Failure, fail


# Same host, different container. Overridden with TTS_SERVICE_URL; the compose
# stack sets it to the service name.
_DEFAULT_BASE_URL = "http://syncai_tts:8080"

# Connecting is local and should be immediate; a slow answer is the service
# thinking, not the network.
_CONNECT_TIMEOUT_S = 5.0

# Generous because the service loads its ~310 MB session on the first request
# that needs it (TTS_PRELOAD makes that a background thread at its startup, but
# a request arriving first still waits on the same lock).
_REQUEST_TIMEOUT_S = 60.0

# `speak` asks the service to hold the response until playback finishes, so this
# has to cover an utterance plus whatever is queued ahead of it. The service
# computes its own wait from the rendered duration; this is only the transport's
# patience with that.
_SPEAK_TIMEOUT_S = 180.0

# Backstop for the whole of `speak`, polling included. Under the SPEAK
# activity's 5-minute start_to_close, so this gateway gives up with a sentence
# before Temporal gives up on the activity.
_SPEAK_DEADLINE_S = 240.0
_POLL_INTERVAL_S = 0.5

# The service's codes that this side answers differently, mapped onto ours.
# Two namespaces that happen to agree on the first entry and need not on the
# second, so the keys are the service's literal strings rather than our enum's
# values: `unknown_voice` is shared on purpose and documented as such in both
# repos, while what the service calls a full queue is a name this side is free
# to choose for its own wire contract.
#
# Everything else the service can report — a missing model, a wedged device,
# aplay not installed — is the robot's problem and stays an untagged 502,
# exactly as it did when those failures were raised in this file.
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

        # Same breadcrumb rationale as the model path this replaces: the URL is
        # an env var nobody sees in a failure message otherwise, and "could not
        # reach the speech service" is a question about which address we tried.
        self._logger.info("[TtsGateway] Using speech service", url=self._base_url)

    # --- HTTP ---------------------------------------------------------------

    def _call(
        self, method: str, path: str, timeout: float, **kwargs
    ) -> Tuple[bool, str, Optional[httpx.Response]]:
        """One request, with every failure already turned into a sentence."""
        try:
            response = self._client.request(method, path, timeout=timeout, **kwargs)
        except httpx.TimeoutException:
            return (
                False,
                f"the speech service at {self._base_url} did not answer within "
                f"{timeout:.0f}s",
                None,
            )
        except httpx.RequestError as exc:
            # Connection refused is the common one: the container is down, or
            # TTS_SERVICE_URL names somewhere it is not.
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

        The code is re-tagged rather than passed through as prose, because this
        side's callers read :func:`failure_code`, not the sentence — the router
        picks 400/409 off it and the SPEAK activity picks retryability off the
        same one.
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

        An unknown voice comes back tagged ``Failure.UNKNOWN_VOICE`` — the one
        failure here that is the caller's to fix, so the router answers 400
        instead of its uniform 502 and the SPEAK activity does not retry it.
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

        ``wait=true`` asks the service to hold the response, which keeps this
        method's contract exactly what it was when it ran ``aplay`` itself. The
        service answers 202 with a non-terminal job if its own wait expires
        first (a long queue ahead of this utterance), so the poll below is the
        rare path rather than the normal one.

        The service's job API is also pollable and cancellable, which is what a
        future ``execute_speak`` should use directly: it would let the activity
        heartbeat, which it cannot do while it sits in this one blocking call.
        """
        success, message, response = self._call(
            "POST",
            "/api/v1/speak",
            timeout=_SPEAK_TIMEOUT_S,
            json={**self._payload(text=text, voice=voice, speed=speed), "wait": True},
        )
        if not success:
            return False, message, None

        try:
            job = response.json()
        except ValueError as exc:
            return False, f"the speech service returned an unreadable job: {exc}", None

        success, message, job = self._await_terminal(job)
        if not success:
            return False, message, None

        status = job.get("status")
        if status == "done":
            # Always a float on success. The service always sends one, but the
            # router's SpeakResponse.duration is a required field, so a body
            # that ever omitted it would turn a spoken utterance into a 500.
            duration = job.get("duration")
            return True, "", float(duration) if duration is not None else 0.0

        detail = job.get("error") or f"playback ended {status!r}"
        code = _FAILURE_BY_CODE.get(job.get("code"))
        return False, (fail(code, detail) if code is not None else detail), None

    # --- Internals ----------------------------------------------------------

    @staticmethod
    def _payload(text: str, voice: str, speed: float) -> Dict[str, object]:
        return {"text": text, "voice": voice, "speed": speed}

    def _await_terminal(
        self, job: dict
    ) -> Tuple[bool, str, Optional[dict]]:
        """Poll the job until it stops moving, or we run out of patience.

        Only reached when the service's own wait expired, which means something
        is queued ahead of this utterance. Bounded by ``_SPEAK_DEADLINE_S`` so a
        SPEAK activity gets a sentence from here rather than being killed by its
        ``start_to_close``.
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

            time.sleep(_POLL_INTERVAL_S)

            success, message, response = self._call(
                "GET", f"/api/v1/speak/{job.get('id')}", timeout=_REQUEST_TIMEOUT_S
            )
            if not success:
                return False, message, None
            try:
                job = response.json()
            except ValueError as exc:
                return False, f"the speech service returned an unreadable job: {exc}", None

        return True, "", job


def init_tts_gateway(
    logger: structlog.stdlib.BoundLogger, base_url: Optional[str] = None
) -> TtsGateway:
    # Read here rather than at import, which is the rule the rest of this
    # package's env reading follows (and the one temporal/shared.py breaks): the
    # factory runs inside SyncAIBackend.__init__, well after main.py's
    # load_dotenv(), so a value living only in .env is actually seen.
    resolved = base_url or os.getenv("TTS_SERVICE_URL") or _DEFAULT_BASE_URL
    return TtsGateway(logger=logger, base_url=resolved)
