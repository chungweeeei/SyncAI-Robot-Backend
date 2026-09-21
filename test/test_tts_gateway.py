"""Tests for TtsGateway, which is now an HTTP client for the syncai_tts service.

What this file used to pin — that two callers could not be inside ``aplay`` at
once — is no longer this repo's guarantee to make. The speaker moved to the
service, and its queue is tested there, against a fake ``aplay``. What is left
here is the seam: the requests this gateway sends, and the translation of the
service's ``{detail, code}`` bodies back into this repo's
``(success, message, payload)`` convention.

That translation is the part worth testing, because two consumers act on it.
``routers/tts.py`` picks 400 / 409 / 502 off the code, and the SPEAK activity
picks retryability off the same one. A code that fails to survive the trip over
HTTP silently turns a typo'd voice into a 502 that Temporal then retries three
times.

``httpx.MockTransport`` is the seam rather than a stubbed client, so the real
httpx request path, the base_url joining and the JSON decoding all run.
"""

import json

import pytest

pytest.importorskip("httpx")
pytest.importorskip("structlog")

import httpx  # noqa: E402
import structlog  # noqa: E402

from syncai_backend.gateways.failure import Failure, failure_code  # noqa: E402
from syncai_backend.gateways.tts import tts as tts_module  # noqa: E402
from syncai_backend.gateways.tts.tts import TtsGateway, init_tts_gateway  # noqa: E402


_BASE_URL = "http://tts.test:8080"
_WAV = b"RIFF-fake-wav"


def make_gateway(handler, base_url: str = _BASE_URL) -> TtsGateway:
    """A gateway whose transport is `handler` instead of a socket."""
    gateway = TtsGateway(logger=structlog.get_logger(), base_url=base_url)
    gateway._client = httpx.Client(
        transport=httpx.MockTransport(handler), base_url=base_url
    )
    return gateway


def json_response(status_code: int, body: dict) -> httpx.Response:
    return httpx.Response(status_code, json=body)


def job(status: str = "done", **overrides) -> dict:
    body = {
        "id": "job-1",
        "status": status,
        "text": "hello",
        "voice": "af_heart",
        "speed": 1.0,
        "duration": 1.25,
        "queue_position": None,
        "queued_at": "2026-09-21T03:07:25.114Z",
        "started_at": None,
        "finished_at": None,
        "error": None,
        "code": None,
    }
    body.update(overrides)
    return body


@pytest.fixture
def recorder():
    """Collects the requests the gateway makes, so their shape can be asserted."""
    return []


# --- Happy paths -------------------------------------------------------------


def test_voices_are_read_off_the_service(recorder):
    def handler(request):
        recorder.append(request)
        return json_response(200, {"voices": ["af_heart", "am_adam"]})

    success, message, voices = make_gateway(handler).list_voices()

    assert success, message
    assert voices == ["af_heart", "am_adam"]
    assert recorder[0].url.path == "/api/v1/voices"


def test_synthesize_passes_the_parameters_and_returns_the_bytes(recorder):
    def handler(request):
        recorder.append(request)
        return httpx.Response(200, content=_WAV, headers={"content-type": "audio/wav"})

    success, message, wav = make_gateway(handler).synthesize(
        text="hello", voice="am_adam", speed=1.5
    )

    assert success, message
    assert wav == _WAV

    request = recorder[0]
    assert request.method == "POST"
    assert request.url.path == "/api/v1/synthesize"
    assert json.loads(request.content) == {
        "text": "hello",
        "voice": "am_adam",
        "speed": 1.5,
    }


def test_speak_asks_the_service_to_hold_the_response(recorder):
    """`wait=true` is what keeps this method's contract what it was when it ran
    aplay itself, so routers/tts.py and the SPEAK activity did not change."""

    def handler(request):
        recorder.append(request)
        return json_response(200, job("done", duration=2.5))

    success, message, duration = make_gateway(handler).speak(text="hello")

    assert success, message
    assert duration == 2.5

    body = json.loads(recorder[0].content)
    assert body["wait"] is True
    assert recorder[0].url.path == "/api/v1/speak"


# --- Error translation -------------------------------------------------------


def test_an_unknown_voice_survives_the_trip_as_a_code(recorder):
    """The one failure shared verbatim between the two repos.

    The router answers 400 off this code and the SPEAK activity marks the
    attempt non-retryable off the same one. If the tag were dropped in
    translation both would silently fall back to "the robot is broken, retry".
    """

    def handler(request):
        return json_response(
            400, {"detail": "unknown voice: 'af_nope'", "code": "unknown_voice"}
        )

    success, message, wav = make_gateway(handler).synthesize(
        text="hi", voice="af_nope"
    )

    assert not success
    assert failure_code(message) is Failure.UNKNOWN_VOICE
    assert "af_nope" in message
    assert wav == b""


def test_a_full_speech_queue_is_tagged_so_the_router_can_answer_409():
    """New with the service: while the speaker was a lock in this process, a
    ninth caller blocked instead of being refused."""

    def handler(request):
        return json_response(
            409,
            {"detail": "8 utterances are already waiting", "code": "queue_full"},
        )

    success, message, duration = make_gateway(handler).speak(text="hi")

    assert not success
    assert failure_code(message) is Failure.TTS_QUEUE_FULL
    assert duration is None


def test_a_failure_the_robot_owns_carries_no_code():
    """Missing weights, a wedged speaker, aplay absent: all still plain 502s,
    exactly as they were when this file raised them itself."""

    def handler(request):
        return json_response(
            503,
            {"detail": "kokoro model file missing: /models/...", "code": "model_unavailable"},
        )

    success, message, voices = make_gateway(handler).list_voices()

    assert not success
    assert failure_code(message) is None
    assert "kokoro model file missing" in message
    assert voices == []


def test_a_service_that_is_not_running_is_named_in_the_message():
    """Connection refused is the likely failure, and "which address did we try"
    is the first question the operator has."""

    def handler(request):
        raise httpx.ConnectError("Connection refused", request=request)

    success, message, voices = make_gateway(handler).list_voices()

    assert not success
    assert _BASE_URL in message
    assert failure_code(message) is None


def test_a_service_that_does_not_answer_says_so():
    def handler(request):
        raise httpx.ReadTimeout("timed out", request=request)

    success, message, wav = make_gateway(handler).synthesize(text="hi")

    assert not success
    assert "did not answer" in message
    assert _BASE_URL in message


def test_an_error_body_that_is_not_json_still_reads_as_a_sentence():
    """A 502 from something in front of the service — a proxy, a wrong port —
    must not surface as a JSONDecodeError traceback."""

    def handler(request):
        return httpx.Response(502, content=b"<html>Bad Gateway</html>")

    success, message, voices = make_gateway(handler).list_voices()

    assert not success
    assert "502" in message


def test_a_success_body_that_is_not_json_is_reported_not_raised():
    def handler(request):
        return httpx.Response(200, content=b"not json")

    success, message, voices = make_gateway(handler).list_voices()

    assert not success
    assert "unreadable" in message


# --- Job outcomes ------------------------------------------------------------


def test_a_failed_job_reports_the_services_own_reason():
    def handler(request):
        return json_response(
            200,
            job("failed", error="aplay failed: audio open error", code="playback_failed"),
        )

    success, message, duration = make_gateway(handler).speak(text="hi")

    assert not success
    assert "audio open error" in message
    # playback_failed is the robot's problem: no code, so the router's 502.
    assert failure_code(message) is None
    assert duration is None


def test_a_cancelled_job_is_a_failure_with_a_sentence():
    def handler(request):
        return json_response(
            200, job("cancelled", error="cancelled during playback")
        )

    success, message, duration = make_gateway(handler).speak(text="hi")

    assert not success
    assert "cancelled" in message
    assert duration is None


def test_speak_polls_when_the_services_own_wait_expires(monkeypatch, recorder):
    """202 with a non-terminal job means something is queued ahead of this one.

    The rare path, but the one that decides whether `speak()` is honest about
    having blocked until the utterance finished.
    """
    monkeypatch.setattr(tts_module, "_POLL_INTERVAL_S", 0.0)
    statuses = iter(["playing", "playing", "done"])

    def handler(request):
        recorder.append(request)
        if request.method == "POST":
            return json_response(202, job("queued"))
        return json_response(200, job(next(statuses)))

    success, message, duration = make_gateway(handler).speak(text="hi")

    assert success, message
    assert duration == 1.25
    assert [r.method for r in recorder] == ["POST", "GET", "GET", "GET"]
    assert recorder[1].url.path == "/api/v1/speak/job-1"


def test_speak_gives_up_before_the_activity_does(monkeypatch):
    """Bounded so a SPEAK step gets a sentence from here rather than being
    killed by its own start_to_close."""
    monkeypatch.setattr(tts_module, "_POLL_INTERVAL_S", 0.0)
    monkeypatch.setattr(tts_module, "_SPEAK_DEADLINE_S", 0.0)

    def handler(request):
        return json_response(202, job("queued"))

    success, message, duration = make_gateway(handler).speak(text="hi")

    assert not success
    assert "backed up" in message
    assert duration is None


def test_the_service_going_away_mid_utterance_is_reported(monkeypatch):
    monkeypatch.setattr(tts_module, "_POLL_INTERVAL_S", 0.0)

    def handler(request):
        if request.method == "POST":
            return json_response(202, job("queued"))
        raise httpx.ConnectError("Connection refused", request=request)

    success, message, duration = make_gateway(handler).speak(text="hi")

    assert not success
    assert _BASE_URL in message


# --- Wiring ------------------------------------------------------------------


def test_the_service_url_comes_from_the_environment(monkeypatch):
    monkeypatch.setenv("TTS_SERVICE_URL", "http://tts:9999/")

    gateway = init_tts_gateway(logger=structlog.get_logger())

    # Trailing slash trimmed, so paths join predictably and the URL reads the
    # same in every error message.
    assert gateway._base_url == "http://tts:9999"


def test_a_missing_service_url_falls_back_to_the_local_default(monkeypatch):
    monkeypatch.delenv("TTS_SERVICE_URL", raising=False)

    gateway = init_tts_gateway(logger=structlog.get_logger())

    assert gateway._base_url == "http://127.0.0.1:8080"
