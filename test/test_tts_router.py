"""Tests for /api/v1/tts — projection over a stubbed TtsGateway.

The gateway itself is not exercised here; it is an HTTP client for the
syncai_tts service and its own tests live in test_tts_gateway.py. What the
router owns is the request validation, the WAV passthrough, and the status-code
mapping:

- an unknown voice is the caller's typo (400),
- a full speech queue is a backlog the caller can wait out or stop adding to
  (409, with a machine-readable code beside the detail),
- everything else that fails — the speech service unreachable, its weights
  missing, a wedged speaker — is the robot's problem (502).

The stub tags those two failures the way the real gateway does, by re-tagging
the code the service sent. The router reads the code rather than the sentence,
so an untagged message is by design just another 502 (pinned below).
"""

import pytest

pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_backend.gateways.failure import Failure, fail  # noqa: E402
from syncai_backend.interfaces.rest.routers.tts import init_tts_router  # noqa: E402
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)

# Not a real WAV — the router forwards the gateway's bytes untouched, so any
# marker proves the passthrough.
_WAV = b"RIFF-fake-wav"


class _StubTtsGateway:
    def __init__(self):
        self.synthesize_result = (True, "", _WAV)
        self.speak_result = (True, "", 1.25)
        self.voices_result = (True, "", ["af_heart", "am_adam"])
        self.synthesize_calls = []
        self.speak_calls = []

    def synthesize(self, text, voice, speed):
        self.synthesize_calls.append((text, voice, speed))
        return self.synthesize_result

    def speak(self, text, voice, speed):
        self.speak_calls.append((text, voice, speed))
        return self.speak_result

    def list_voices(self):
        return self.voices_result


@pytest.fixture
def tts_gw():
    return _StubTtsGateway()


@pytest.fixture
def client(logger, tts_gw):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(init_tts_router(logger=logger, tts_gw=tts_gw))
    return TestClient(app)


def test_synthesize_returns_the_wav_bytes(client, tts_gw):
    response = client.post("/api/v1/tts/synthesize", json={"text": "hello"})

    assert response.status_code == 200
    assert response.headers["content-type"] == "audio/wav"
    assert response.content == _WAV
    # Defaults applied by the request model, not by the gateway.
    assert tts_gw.synthesize_calls == [("hello", "af_heart", 1.0)]


def test_an_unknown_voice_is_a_400(client, tts_gw):
    tts_gw.synthesize_result = (
        False,
        fail(Failure.UNKNOWN_VOICE, "unknown voice: 'af_nope'"),
        b"",
    )

    response = client.post(
        "/api/v1/tts/synthesize", json={"text": "hello", "voice": "af_nope"}
    )

    assert response.status_code == 400
    assert "af_nope" in response.json()["detail"]


def test_a_failure_that_carries_no_code_is_a_502(client, tts_gw):
    """The sentence is not the contract: only the code moves a failure off 502.

    Pins the half of the rule that is easy to lose — a gateway failure nobody
    tagged stays the robot's problem even when its prose reads like the
    caller's fault.
    """
    tts_gw.synthesize_result = (False, "unknown voice: 'af_nope'", b"")

    response = client.post(
        "/api/v1/tts/synthesize", json={"text": "hello", "voice": "af_nope"}
    )

    assert response.status_code == 502


def test_a_full_speech_queue_is_a_409_with_a_code(client, tts_gw):
    """Not a 502: the robot is fine, there is just already a backlog of speech.

    The console's next step is to wait or stop queueing, which is not the next
    step for a 502, and it branches on the code rather than on the sentence —
    the ConflictError.code precedent.
    """
    tts_gw.speak_result = (
        False,
        fail(Failure.TTS_QUEUE_FULL, "8 utterances are already waiting"),
        None,
    )

    response = client.post("/api/v1/tts/speak", json={"text": "hi"})

    assert response.status_code == 409
    body = response.json()
    assert body["code"] == "tts_queue_full"
    assert "already waiting" in body["detail"]


def test_an_unreachable_speech_service_is_a_502(client, tts_gw):
    tts_gw.synthesize_result = (
        False,
        "could not reach the speech service at http://tts:8080: Connection refused",
        b"",
    )

    response = client.post("/api/v1/tts/synthesize", json={"text": "hello"})

    assert response.status_code == 502


def test_speak_passes_parameters_through(client, tts_gw):
    body = client.post(
        "/api/v1/tts/speak", json={"text": "hi", "voice": "am_adam", "speed": 1.5}
    ).json()

    assert tts_gw.speak_calls == [("hi", "am_adam", 1.5)]
    assert body["duration"] == 1.25


def test_a_failed_playback_is_a_502(client, tts_gw):
    tts_gw.speak_result = (False, "aplay not found — alsa-utils is not installed", None)

    response = client.post("/api/v1/tts/speak", json={"text": "hi"})

    assert response.status_code == 502


def test_empty_text_is_rejected_at_the_boundary(client, tts_gw):
    response = client.post("/api/v1/tts/speak", json={"text": ""})

    assert response.status_code == 422
    assert tts_gw.speak_calls == []


def test_out_of_range_speed_is_rejected_at_the_boundary(client, tts_gw):
    response = client.post("/api/v1/tts/speak", json={"text": "hi", "speed": 5.0})

    assert response.status_code == 422
    assert tts_gw.speak_calls == []


def test_voices_are_listed(client):
    body = client.get("/api/v1/tts/voices").json()

    assert body["voices"] == ["af_heart", "am_adam"]
