import io
import json
import os
import subprocess
import tarfile
import tempfile
import time
from datetime import datetime, timezone
from typing import BinaryIO, Callable, Dict, List, Optional

import structlog


# What GET /api/v1/logs/export covers: the last day.
DEFAULT_WINDOW_S = 24 * 60 * 60

# The stack's logs are not in the journal: NodeManager (syncai_sys_manager)
# taps every byobu pane with `tmux pipe-pane` into multilog, one directory per
# pane under the workspace's log/stack/<robot_id>/. Absolute for the reason
# record/ is: entrypoints do not reliably run from the workspace root.
STACK_LOG_ROOT = "~/robot_ws/log/stack"

# The host's persistent journal, bind-mounted read-only (docker-compose.yml).
# Read with `journalctl -D`, so the container's own machine-id is irrelevant.
JOURNAL_DIR = "/var/log/journal"

# journalctl over a day of a busy host is seconds; past this it is wedged on
# something, and the stack logs are the more useful half to return anyway.
_JOURNAL_TIMEOUT_S = 120.0

# What multilog writes that is log: the live file and the finished
# rotations (`@<tai64n>.s`, gzipped by the `!gzip` processor). Its `lock`,
# `state`, and a half-processed `processed` / `previous` are bookkeeping.
_CURRENT = "current"
_ROTATED_PREFIX = "@"
_ROTATED_SUFFIX = ".s"


class LogBundleGateway:
    """The stack's pane logs and the host journal, as one tar.

    Holds no node and no state: each call walks the log tree and runs
    ``journalctl`` afresh. Either half missing (no mount, no journalctl, no
    permission) is recorded in the bundle's ``manifest.json`` rather than
    failing it -- whatever is readable is what an operator debugging the robot
    wants.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_id: str,
        stack_dir: str,
        journal_dir: str,
        scratch_dir: str,
        run: Callable[..., subprocess.CompletedProcess] = subprocess.run,
        clock: Callable[[], float] = time.time,
    ):
        self.logger = logger
        self.robot_id = robot_id
        self.stack_dir = stack_dir
        self.journal_dir = journal_dir
        self.scratch_dir = scratch_dir
        self._run = run
        self._clock = clock

    def write_bundle(self, dest: BinaryIO, window_s: int = DEFAULT_WINDOW_S) -> Dict:
        """Write the tar into ``dest``; return its manifest.

        Every member sits under one ``<robot_id>-logs-<stamp>/`` directory,
        the manifest's ``name``, which is also the download's file name. A
        pane's file is taken whole when its mtime is inside the window: a
        rotation's mtime is its last write, so that is exactly the files whose
        lines overlap the window, the oldest one reaching somewhat before it.
        Not compressed again: the rotations are gzip already.
        """
        now = self._clock()
        since = now - window_s
        root = f"{self.robot_id}-logs-{_stamp(now)}"

        manifest: Dict = {
            "name": root,
            "robot_id": self.robot_id,
            "generated_at": _iso(now),
            "since": _iso(since),
            "window_seconds": window_s,
        }
        with tarfile.open(fileobj=dest, mode="w") as tar:
            manifest["stack"] = self._add_stack(tar, root, since)
            manifest["journal"] = self._add_journal(tar, root, since)
            data = json.dumps(manifest, indent=2).encode()
            info = tarfile.TarInfo(f"{root}/manifest.json")
            info.size = len(data)
            info.mtime = int(now)
            tar.addfile(info, io.BytesIO(data))

        self.logger.info(
            "[LogBundleGateway] Wrote log bundle",
            panes=len(manifest["stack"]["panes"]),
            journal=manifest["journal"]["included"],
        )
        return manifest

    def _add_stack(self, tar: tarfile.TarFile, root: str, since: float) -> Dict:
        try:
            panes = [e for e in os.scandir(self.stack_dir) if e.is_dir()]
        except (FileNotFoundError, NotADirectoryError):
            self.logger.warning(
                "[LogBundleGateway] Stack log directory does not exist",
                path=self.stack_dir,
            )
            return {"included": False, "error": "stack log directory not found", "panes": {}}
        except OSError as exc:
            self.logger.warning(
                "[LogBundleGateway] Stack log directory unreadable",
                path=self.stack_dir,
                error=str(exc),
            )
            return {"included": False, "error": str(exc), "panes": {}}

        described: Dict[str, List[Dict]] = {}
        for pane in sorted(panes, key=lambda e: e.name):
            files = []
            for path in _pane_files(pane.path, since):
                member = f"{root}/stack/{pane.name}/{os.path.basename(path)}"
                size = _add_file(tar, path, member)
                if size is not None:
                    files.append({"name": os.path.basename(path), "bytes": size})
            described[pane.name] = files
        return {"included": True, "panes": described}

    def _add_journal(self, tar: tarfile.TarFile, root: str, since: float) -> Dict:
        if not os.path.isdir(self.journal_dir):
            return self._journal_missing(f"{self.journal_dir} is not mounted")

        command = [
            "journalctl",
            "-D",
            self.journal_dir,
            f"--since=@{int(since)}",
            "--no-pager",
            "-o",
            "short-iso-precise",
        ]
        # Spooled to the scratch disk, not a pipe into memory: tar needs the
        # size before the bytes, and a day of journal can be hundreds of MB.
        with tempfile.TemporaryFile(dir=self.scratch_dir) as out:
            try:
                result = self._run(
                    command,
                    stdout=out,
                    stderr=subprocess.PIPE,
                    timeout=_JOURNAL_TIMEOUT_S,
                    check=False,
                )
            except FileNotFoundError:
                return self._journal_missing("journalctl is not installed")
            except subprocess.TimeoutExpired:
                return self._journal_missing(
                    f"journalctl did not finish within {_JOURNAL_TIMEOUT_S:.0f} s"
                )
            except OSError as exc:
                return self._journal_missing(f"journalctl could not run: {exc}")

            size = out.seek(0, os.SEEK_END)
            if result.returncode != 0 and size == 0:
                stderr = (result.stderr or b"").decode(errors="replace").strip()
                return self._journal_missing(
                    f"journalctl exited {result.returncode}: {stderr or 'no output'}"
                )

            out.seek(0)
            info = tarfile.TarInfo(f"{root}/journal.log")
            info.size = size
            info.mtime = int(self._clock())
            tar.addfile(info, out)
        return {"included": True, "bytes": size}

    def _journal_missing(self, reason: str) -> Dict:
        self.logger.warning("[LogBundleGateway] Journal left out", reason=reason)
        return {"included": False, "error": reason}


def _pane_files(pane_dir: str, since: float) -> List[str]:
    """The pane's log files written to at or after ``since``, oldest first.

    Rotations are named by their TAI64N stamp, so name order is time order;
    ``current`` is always the newest.
    """
    try:
        entries = list(os.scandir(pane_dir))
    except OSError:
        return []
    rotated = []
    current = None
    for entry in entries:
        name = entry.name
        is_rotated = name.startswith(_ROTATED_PREFIX) and name.endswith(_ROTATED_SUFFIX)
        if not (is_rotated or name == _CURRENT):
            continue
        try:
            stat = entry.stat()
        except OSError:
            continue
        if not entry.is_file() or stat.st_mtime < since:
            continue
        if is_rotated:
            rotated.append(entry.path)
        else:
            current = entry.path
    rotated.sort()
    return rotated + ([current] if current else [])


def _add_file(tar: tarfile.TarFile, path: str, member: str) -> Optional[int]:
    """Add one log file as it stood when opened; None when it is gone.

    ``current`` is being appended to: the size is taken from the open
    descriptor and exactly that many bytes are copied, so a write meanwhile
    cannot make the member longer than its header. A rotation renaming it
    away does not matter either, the descriptor follows the file.
    """
    try:
        with open(path, "rb") as f:
            stat = os.fstat(f.fileno())
            info = tarfile.TarInfo(member)
            info.size = stat.st_size
            info.mtime = int(stat.st_mtime)
            tar.addfile(info, f)
            return stat.st_size
    except FileNotFoundError:
        # Rotated or deleted between the listing and the open.
        return None


def _stamp(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


def init_log_bundle_gateway(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
    scratch_dir: str,
) -> LogBundleGateway:
    return LogBundleGateway(
        logger=logger,
        robot_id=robot_id,
        stack_dir=os.path.join(os.path.expanduser(STACK_LOG_ROOT), robot_id),
        journal_dir=JOURNAL_DIR,
        scratch_dir=scratch_dir,
    )
