"""Tests for TtsGateway's locking.

The router's tests stub this gateway wholesale, so nothing exercised the one
thing it actually owns: which callers are allowed to touch the speaker at the
same time. Both seams here are cheap -- the kokoro session is a fake object
assigned onto the gateway (so ``_ensure_loaded`` short-circuits and the 310 MB
.onnx is never opened) and ``aplay`` is a recorder patched over
``subprocess.run`` -- so no model, no ALSA device and no ROS are needed.

The regression these pin: ``_lock`` used to be released at the end of
``synthesize()``, leaving the subprocess unguarded. Synthesis takes a few
hundred milliseconds and playback takes as long as the utterance, so a Temporal
SPEAK step and a manual POST /api/v1/tts/speak could both be inside aplay at
once -- two streams into one pcm node.
"""

import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("numpy")
pytest.importorskip("structlog")

import numpy as np  # noqa: E402
import structlog  # noqa: E402

from syncai_backend.gateways.failure import Failure, failure_code  # noqa: E402
from syncai_backend.gateways.tts import tts as tts_module  # noqa: E402
from syncai_backend.gateways.tts.tts import TtsGateway  # noqa: E402


class _FakeKokoro:
    """Stands in for the loaded ONNX session."""

    def __init__(self, synth_delay: float = 0.0):
        self._synth_delay = synth_delay

    def get_voices(self):
        return ["af_heart", "am_adam"]

    def create(self, text, voice, speed):
        if self._synth_delay:
            time.sleep(self._synth_delay)
        # A tenth of a second of silence at kokoro's native rate.
        return np.zeros(2400, dtype=np.float32), 24000


class _RecordingAplay:
    """Counts how many callers are inside the subprocess at the same time."""

    def __init__(self, duration: float = 0.05):
        self._duration = duration
        self._lock = threading.Lock()
        self.inside = 0
        self.max_inside = 0
        self.calls = 0
        self.devices = []

    def __call__(self, argv, input=None, capture_output=False, timeout=None):
        with self._lock:
            self.inside += 1
            self.max_inside = max(self.max_inside, self.inside)
            self.calls += 1
            self.devices.append(argv[argv.index("-D") + 1])
        try:
            time.sleep(self._duration)
        finally:
            with self._lock:
                self.inside -= 1
        return SimpleNamespace(returncode=0, stderr=b"")


@pytest.fixture
def gateway():
    gw = TtsGateway(logger=structlog.get_logger())
    gw._kokoro = _FakeKokoro()
    return gw


@pytest.fixture
def aplay(monkeypatch):
    recorder = _RecordingAplay()
    monkeypatch.setattr(tts_module.subprocess, "run", recorder)
    return recorder


def test_playback_is_serialised_across_concurrent_speakers(gateway, aplay):
    """Four callers, one speaker: never more than one aplay at a time."""
    results = []
    results_lock = threading.Lock()

    def _speak():
        outcome = gateway.speak(text="hello", voice="af_heart")
        with results_lock:
            results.append(outcome)

    threads = [threading.Thread(target=_speak) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=10.0)

    assert all(not thread.is_alive() for thread in threads)
    assert aplay.calls == 4
    assert aplay.max_inside == 1
    assert [success for success, _, _ in results] == [True] * 4


def test_a_synthesize_only_caller_is_not_blocked_by_playback(gateway, aplay):
    """The two locks are distinct: holding the speaker does not stop synthesis.

    Deterministic on purpose -- the playback lock is taken by the test rather
    than by a racing thread, so this proves the split rather than timing.
    """
    with gateway._playback_lock:
        success, message, wav_bytes = gateway.synthesize(text="hi", voice="af_heart")

    assert success, message
    assert wav_bytes.startswith(b"RIFF")
    assert aplay.calls == 0


def test_speak_reports_the_utterance_length(gateway, aplay):
    success, message, duration = gateway.speak(text="hi", voice="af_heart")

    assert success, message
    assert duration == pytest.approx(0.1, abs=0.01)
    assert aplay.calls == 1


def test_an_unknown_voice_never_reaches_the_speaker(gateway, aplay):
    """Tagged, not just worded: the router and the SPEAK activity read the code.

    The prose is asserted too, because it is what the operator reads, but the
    code is the contract — see gateways/failure.py.
    """
    success, message, duration = gateway.speak(text="hi", voice="nosuchvoice")

    assert not success
    assert failure_code(message) is Failure.UNKNOWN_VOICE
    assert "nosuchvoice" in message
    assert duration is None
    assert aplay.calls == 0
