import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from typing import Dict, List, NamedTuple, Optional, Tuple

import structlog

from syncai_backend.gateways.failure import Failure, fail


# The LIO inputs: replaying them is the only way to recover a mapping run that
# ended without ``save_maps``. Relative, like every topic name here.
DEFAULT_TOPICS = ("livox/lidar", "livox/imu")

# One split: with less free, a start cannot finish the file it opens. Refusing
# is recoverable; a full partition takes postgres and the stack down with it.
MIN_FREE_BYTES = 2 * 1024 * 1024 * 1024

# What each split is cut at, so a long run copies off the robot in pieces.
MAX_BAG_SIZE_BYTES = 2000000000

# Reserved: GET /api/v1/recordings/active would otherwise shadow a recording
# directory legitimately called that.
RESERVED_NAMES = frozenset({"active"})

# The stop ladder. SIGINT is the only signal rosbag2 answers by flushing; any
# other leaves no metadata.yaml (unplayable until `ros2 bag reindex`).
#
# The SIGINT rung is a stall timeout, not a deadline: with compression on,
# rosbag2 first zstd-compresses the open split (~50 s for 2 GB on an Orin). A
# fixed 15 s deadline once SIGKILLed it mid-compression and truncated the bag,
# so escalate only after _SIGINT_STALL_TIMEOUT of no writes.
# _SIGINT_MAX_TIMEOUT bounds the blocking POST.
_SIGINT_STALL_TIMEOUT = 15.0
_SIGINT_MAX_TIMEOUT = 300.0
_SIGINT_POLL_INTERVAL = 1.0
_SIGTERM_TIMEOUT = 5.0
_SIGKILL_TIMEOUT = 2.0

# How long a fresh recorder must stay alive. Bad arguments kill it within
# milliseconds; catching that makes it the POST's error, not a silent 201.
_LIVENESS_PROBE = 0.5


class _Active(NamedTuple):
    """The one recording this backend is running."""

    name: str
    path: str
    topics: List[str]
    started_at: datetime
    started_monotonic: float
    compression: bool
    process: subprocess.Popen


class RecordingGateway:
    """``ros2 bag record`` as a supervised child process.

    A subprocess rather than ``rosbag2_py``, so the lidar recorder does not
    share this executor or hold the GIL; a wedged recorder takes nothing down.

    Two properties of the child are load-bearing:

      - **No** ``start_new_session``: ``switch_mode`` kills the byobu session
        this backend runs in, and a recorder in its own session would survive
        unstoppable. Sharing it means the bag reads ``interrupted``.
      - stdout/stderr are **inherited**, not piped: an undrained pipe
        deadlocks the recorder at 64 KiB.

    One recording at a time; the slot is claimed under the lock before the
    spawn, so there is no check-then-start race.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_id: str,
        records_dir: str,
    ):
        self._logger = logger
        self._robot_id = robot_id
        self._records_dir = records_dir

        # Covers the slot, not the child; never held across stop()'s wait, so
        # status() stays answerable.
        self._lock = threading.Lock()
        self._active: Optional[_Active] = None

    # --- Helpers ------------------------------------------------------------

    def _resolve_topics(self, topics: List[str]) -> List[str]:
        """Expand relative topic names under this robot's namespace.

        ``livox/lidar`` becomes ``/robot01/livox/lidar``; absolute names (e.g.
        ``/tf``, ``/tf_static``) pass through untouched.
        """
        resolved = []
        for topic in topics:
            stripped = topic.strip()
            if stripped.startswith("/"):
                resolved.append(stripped)
            else:
                resolved.append(f"/{self._robot_id}/{stripped}")
        return resolved

    def _reap(self) -> None:
        """Drop the slot if the child is gone. Caller must hold the lock.

        A recorder can die on its own; without this every later start would
        answer 409 against a process that no longer exists.
        """
        active = self._active
        if active is None or active.process.poll() is None:
            return

        self._logger.warning(
            "[RecordingGateway] The recorder exited on its own",
            recording=active.name,
            returncode=active.process.returncode,
        )
        self._active = None

    def _describe(self, active: _Active) -> Dict:
        return {
            "name": active.name,
            "path": active.path,
            "topics": list(active.topics),
            "started_at": active.started_at,
            "elapsed_seconds": time.monotonic() - active.started_monotonic,
            "compression": active.compression,
            "size_bytes": _dir_size(active.path),
        }

    # --- API ----------------------------------------------------------------

    def active_name(self) -> Optional[str]:
        """The live recording's name, or None. Cheap: no disk walk."""
        with self._lock:
            self._reap()
            return self._active.name if self._active is not None else None

    def status(self) -> Optional[Dict]:
        """What is being recorded right now, or None."""
        with self._lock:
            self._reap()
            active = self._active
        # Outside the lock: _describe walks the bag directory.
        return self._describe(active) if active is not None else None

    def start(
        self, name: str, topics: List[str], compression: bool
    ) -> Tuple[bool, str, Dict]:
        """Spawn a recorder. Returns (success, message, payload).

        Checks run under the lock that claims the slot. The directory is not
        created here: the recorder's ``-o`` refuses an existing path, which
        backs up the router's name check.
        """
        resolved = self._resolve_topics(topics)
        path = os.path.join(self._records_dir, name)

        with self._lock:
            self._reap()
            if self._active is not None:
                return (
                    False,
                    fail(
                        Failure.RECORDING_RUNNING,
                        f"already recording '{self._active.name}'",
                    ),
                    {"name": self._active.name},
                )

            free = _free_bytes(self._records_dir)
            if free < MIN_FREE_BYTES:
                return (
                    False,
                    fail(
                        Failure.DISK_LOW,
                        f"only {free / 1e9:.1f} GB free where recordings are "
                        f"written, and one bag file is "
                        f"{MAX_BAG_SIZE_BYTES / 1e9:.0f} GB — delete a "
                        "recording first",
                    ),
                    {"free_bytes": free},
                )

            command = _build_command(path, resolved, compression)
            try:
                # No start_new_session, no PIPE: see the class docstring. cwd is
                # fixed so nothing depends on where the backend was started.
                process = subprocess.Popen(command, cwd=os.path.expanduser("~"))
            except FileNotFoundError:
                return (
                    False,
                    "ros2 not found on PATH — the backend was started without a "
                    "sourced ROS 2 environment",
                    {},
                )
            except OSError as exc:
                return False, f"could not start ros2 bag record: {exc}", {}

            # The liveness probe (see _LIVENESS_PROBE).
            try:
                process.wait(timeout=_LIVENESS_PROBE)
                return (
                    False,
                    f"ros2 bag record exited immediately (code "
                    f"{process.returncode}) — see the backend log for its own "
                    "error; a bad topic name or an unwritable record/ is the "
                    "usual cause",
                    {},
                )
            except subprocess.TimeoutExpired:
                pass

            active = _Active(
                name=name,
                path=path,
                topics=resolved,
                started_at=datetime.now(timezone.utc),
                started_monotonic=time.monotonic(),
                compression=compression,
                process=process,
            )
            self._active = active

        self._logger.info(
            "[RecordingGateway] Recording started",
            recording=name,
            topics=resolved,
            compression=compression,
            pid=process.pid,
        )
        return True, "", self._describe(active)

    def stop(self) -> Tuple[bool, str, Dict]:
        """Stop the live recorder and wait for it to flush its metadata.

        Blocking: seconds, ~a minute with compression, up to
        _SIGINT_MAX_TIMEOUT; hence a plain ``def`` route. The caller wants to
        know whether the bag is playable.

        The slot is released before the signal, so a hung stop cannot wedge
        later starts.
        """
        with self._lock:
            self._reap()
            active = self._active
            self._active = None

        if active is None:
            return False, "nothing is being recorded", {}

        stopped_by = _terminate(self._logger, active)

        metadata = os.path.join(active.path, "metadata.yaml")
        complete = os.path.isfile(metadata)
        if not complete:
            self._logger.warning(
                "[RecordingGateway] The bag has no metadata.yaml",
                recording=active.name,
                stopped_by=stopped_by,
            )

        self._logger.info(
            "[RecordingGateway] Recording stopped",
            recording=active.name,
            stopped_by=stopped_by,
            duration_seconds=time.monotonic() - active.started_monotonic,
        )
        return (
            True,
            "",
            {
                "name": active.name,
                "path": active.path,
                "topics": list(active.topics),
                "started_at": active.started_at,
                "elapsed_seconds": time.monotonic() - active.started_monotonic,
                "size_bytes": _dir_size(active.path),
                "stopped_by": stopped_by,
                "complete": complete,
            },
        )


def _build_command(path: str, topics: List[str], compression: bool) -> List[str]:
    """The argv, kept out of start() so a test can read it without a spawn."""
    command = [
        "ros2",
        "bag",
        "record",
        "-o",
        path,
        "--max-bag-size",
        str(MAX_BAG_SIZE_BYTES),
    ]
    if compression:
        # File mode: message mode compresses on the writer's path and drops
        # messages under load.
        command += [
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
        ]
    return command + list(topics)


def _terminate(logger: structlog.stdlib.BoundLogger, active: _Active) -> str:
    """Walk the stop ladder; return which rung ended it.

    Only SIGINT yields metadata.yaml; the lower rungs are for a wedged writer
    and leave a bag for `ros2 bag reindex`, so the result is reported, not
    raised.
    """
    process = active.process
    if process.poll() is not None:
        return "already_exited"

    process.send_signal(signal.SIGINT)
    if _wait_while_writing(logger, active):
        return "sigint"
    logger.warning(
        "[RecordingGateway] The recorder stopped flushing after SIGINT, "
        "sending SIGTERM",
        recording=active.name,
    )

    try:
        process.terminate()
        process.wait(timeout=_SIGTERM_TIMEOUT)
        return "sigterm"
    except subprocess.TimeoutExpired:
        logger.warning(
            "[RecordingGateway] The recorder ignored SIGTERM, sending SIGKILL",
            recording=active.name,
        )

    try:
        process.kill()
        process.wait(timeout=_SIGKILL_TIMEOUT)
    except (subprocess.TimeoutExpired, OSError) as exc:
        # Only a child stuck in uninterruptible IO. Reported, not raised: the
        # slot is already released.
        logger.error(
            "[RecordingGateway] Could not kill the recorder",
            recording=active.name,
            error=str(exc),
        )
        return "kill_failed"
    return "sigkill"


def _wait_while_writing(
    logger: structlog.stdlib.BoundLogger, active: _Active
) -> bool:
    """Wait for a SIGINTed recorder to exit; True if it did.

    Gives up after _SIGINT_STALL_TIMEOUT with no file size/mtime change in the
    bag directory, or after _SIGINT_MAX_TIMEOUT in total.
    """
    process = active.process
    started = time.monotonic()
    last_progress = started
    last_seen = _dir_progress(active.path)

    while True:
        try:
            process.wait(timeout=_SIGINT_POLL_INTERVAL)
            return True
        except subprocess.TimeoutExpired:
            pass

        now = time.monotonic()
        seen = _dir_progress(active.path)
        if seen != last_seen:
            last_seen = seen
            last_progress = now

        if now - last_progress >= _SIGINT_STALL_TIMEOUT:
            return False
        if now - started >= _SIGINT_MAX_TIMEOUT:
            logger.warning(
                "[RecordingGateway] The recorder was still writing at the "
                "SIGINT budget",
                recording=active.name,
                budget_seconds=_SIGINT_MAX_TIMEOUT,
            )
            return False


def _dir_progress(path: str) -> Tuple[Tuple[str, int, int], ...]:
    """A fingerprint of ``path``'s files that changes whenever one is written.

    Only the top level: rosbag2 writes no subdirectories.
    """
    entries = []
    try:
        with os.scandir(path) as it:
            for entry in it:
                try:
                    st = entry.stat()
                except FileNotFoundError:
                    continue
                entries.append((entry.name, st.st_size, st.st_mtime_ns))
    except FileNotFoundError:
        pass
    return tuple(sorted(entries))


def _dir_size(path: str) -> int:
    """Bytes under ``path``, or 0 if it is not there yet.

    Duplicated from the catalogue repo's _walk_stats: a gateway importing a
    repository would invert the layering.
    """
    total = 0
    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                total += os.stat(os.path.join(root, filename)).st_size
            except FileNotFoundError:
                continue
    return total


def _free_bytes(path: str) -> int:
    """Free space where recordings land, answered before record/ exists."""
    probe = path
    while not os.path.exists(probe):
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent
    return shutil.disk_usage(probe).free


def init_recording_gateway(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    records_dir: str,
) -> RecordingGateway:
    return RecordingGateway(logger=logger, robot_id=robot_id, records_dir=records_dir)
