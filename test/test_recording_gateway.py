"""Unit tests for RecordingGateway's stop ladder.

No ``ros2``: the recorder is a stub ``Popen`` whose exit is scripted against a
fake clock, and the bag directory is a real tmp dir the stub writes into, so the
SIGINT rung's stall detection is tested by moving time rather than waiting.

What is pinned is the regression behind dp1f_1006 (2026-10-06): a recorder
that is still writing after SIGINT -- zstd-compressing its last split -- keeps
its SIGINT past the old fixed 15 s, and is escalated only once its directory
stops changing.
"""

import signal
import subprocess
from datetime import datetime, timezone
from unittest.mock import MagicMock

import pytest

from syncai_backend.gateways.recording import recording
from syncai_backend.gateways.recording.recording import _Active, _terminate


class _Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class _Recorder:
    """A Popen stand-in. Each wait() is one poll interval of fake time.

    ``write_until`` is how long after SIGINT it keeps growing a file in the bag
    directory; ``exit_at`` is when it exits on its own (None: never, until a
    SIGTERM/SIGKILL, which ends it at once).
    """

    def __init__(self, clock, path, write_until, exit_at):
        self._clock = clock
        self._path = path
        self._write_until = write_until
        self._exit_at = exit_at
        self.signals = []
        self.returncode = None

    def poll(self):
        return self.returncode

    def send_signal(self, sig):
        self.signals.append(sig)

    def terminate(self):
        self.signals.append(signal.SIGTERM)
        self.returncode = -signal.SIGTERM

    def kill(self):
        self.signals.append(signal.SIGKILL)
        self.returncode = -signal.SIGKILL

    def wait(self, timeout=None):
        if self.returncode is not None:
            return self.returncode
        self._clock.now += timeout
        if self._clock.now <= self._write_until:
            with open(self._path / "bag_1.db3.zstd", "ab") as f:
                f.write(b"x")
        if self._exit_at is not None and self._clock.now >= self._exit_at:
            self.returncode = 0
            return 0
        raise subprocess.TimeoutExpired("ros2", timeout)


@pytest.fixture
def clock(monkeypatch):
    c = _Clock()
    monkeypatch.setattr(recording.time, "monotonic", c.monotonic)
    return c


def _active(path, process):
    return _Active(
        name="bag",
        path=str(path),
        topics=["/robot01/livox/lidar"],
        started_at=datetime.now(timezone.utc),
        started_monotonic=0.0,
        compression=True,
        process=process,
    )


def test_a_recorder_still_compressing_keeps_its_sigint(tmp_path, clock):
    # 60 s of writing, past the old fixed 15 s cut, then a clean exit.
    proc = _Recorder(clock, tmp_path, write_until=60.0, exit_at=62.0)

    assert _terminate(MagicMock(), _active(tmp_path, proc)) == "sigint"
    assert proc.signals == [signal.SIGINT]


def test_a_recorder_that_stops_writing_is_escalated(tmp_path, clock):
    proc = _Recorder(clock, tmp_path, write_until=30.0, exit_at=None)

    assert _terminate(MagicMock(), _active(tmp_path, proc)) == "sigterm"
    assert proc.signals == [signal.SIGINT, signal.SIGTERM]
    # Escalated one stall window after the last write, not before.
    assert 30.0 + recording._SIGINT_STALL_TIMEOUT <= clock.now
    assert clock.now <= 30.0 + recording._SIGINT_STALL_TIMEOUT + 2.0


def test_a_silent_recorder_is_escalated_after_the_stall_window(tmp_path, clock):
    proc = _Recorder(clock, tmp_path, write_until=-1.0, exit_at=None)

    assert _terminate(MagicMock(), _active(tmp_path, proc)) == "sigterm"
    assert clock.now == pytest.approx(recording._SIGINT_STALL_TIMEOUT, abs=1.0)


def test_the_sigint_budget_bounds_a_recorder_that_never_stops_writing(
    tmp_path, clock
):
    proc = _Recorder(clock, tmp_path, write_until=float("inf"), exit_at=None)

    assert _terminate(MagicMock(), _active(tmp_path, proc)) == "sigterm"
    assert clock.now == pytest.approx(recording._SIGINT_MAX_TIMEOUT, abs=1.0)


def test_a_missing_bag_directory_reads_as_no_progress(tmp_path, clock):
    proc = _Recorder(clock, tmp_path / "nope", write_until=-1.0, exit_at=None)

    assert _terminate(MagicMock(), _active(tmp_path / "nope", proc)) == "sigterm"


def test_an_exited_recorder_is_not_signalled(tmp_path, clock):
    proc = _Recorder(clock, tmp_path, write_until=-1.0, exit_at=None)
    proc.returncode = 1

    assert _terminate(MagicMock(), _active(tmp_path, proc)) == "already_exited"
    assert proc.signals == []
