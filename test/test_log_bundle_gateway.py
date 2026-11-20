"""gateways/logs: the stack's pane logs and the host journal, as one tar.

No journalctl and no multilog: the pane directories are written into a tmp
dir with chosen mtimes, and ``journalctl`` is a stub ``run`` that writes into
the stdout file it is handed (or fails the way the real one can).
"""

import io
import json
import os
import subprocess
import tarfile

import pytest

from syncai_backend.gateways.logs.logs import DEFAULT_WINDOW_S, LogBundleGateway


# 2026-11-20T00:00:00Z.
NOW = 1_795_132_800.0
DAY = DEFAULT_WINDOW_S


def _write(path, data, mtime):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    os.utime(path, (mtime, mtime))


class _Journalctl:
    """A ``subprocess.run`` stand-in for journalctl."""

    def __init__(self, output=b"", returncode=0, stderr=b"", raises=None):
        self.output = output
        self.returncode = returncode
        self.stderr = stderr
        self.raises = raises
        self.commands = []

    def __call__(self, command, stdout, stderr, timeout, check):
        self.commands.append(command)
        if self.raises is not None:
            raise self.raises
        stdout.write(self.output)
        return subprocess.CompletedProcess(command, self.returncode, None, self.stderr)


@pytest.fixture
def dirs(tmp_path):
    stack = tmp_path / "stack" / "robot01"
    journal = tmp_path / "journal"
    scratch = tmp_path / "scratch"
    stack.mkdir(parents=True)
    journal.mkdir()
    scratch.mkdir()
    return stack, journal, scratch


def _gateway(logger, dirs, journalctl):
    stack, journal, scratch = dirs
    return LogBundleGateway(
        logger=logger,
        robot_id="robot01",
        stack_dir=str(stack),
        journal_dir=str(journal),
        scratch_dir=str(scratch),
        run=journalctl,
        clock=lambda: NOW,
    )


def _bundle(gateway):
    buf = io.BytesIO()
    manifest = gateway.write_bundle(buf)
    buf.seek(0)
    with tarfile.open(fileobj=buf) as tar:
        members = {m.name: tar.extractfile(m).read() for m in tar.getmembers()}
    return manifest, members


ROOT = "robot01-logs-20261120T000000Z"


def test_takes_the_files_written_to_in_the_last_day(logger, dirs):
    stack = dirs[0]
    _write(stack / "planner" / "current", b"live", NOW - 10)
    _write(stack / "planner" / "@400000006a000002.s", b"newer", NOW - 3_600)
    _write(stack / "planner" / "@400000006a000001.s", b"older", NOW - DAY + 60)
    _write(stack / "planner" / "@4000000069000000.s", b"stale", NOW - DAY - 60)
    _write(stack / "planner" / "lock", b"", NOW)
    _write(stack / "planner" / "state", b"\0" * 4, NOW)
    _write(stack / "controller" / "current", b"ctl", NOW - 5)

    manifest, members = _bundle(_gateway(logger, dirs, _Journalctl(b"kernel: hi\n")))

    assert members[f"{ROOT}/stack/planner/current"] == b"live"
    assert members[f"{ROOT}/stack/planner/@400000006a000002.s"] == b"newer"
    assert members[f"{ROOT}/stack/planner/@400000006a000001.s"] == b"older"
    assert members[f"{ROOT}/stack/controller/current"] == b"ctl"
    assert f"{ROOT}/stack/planner/@4000000069000000.s" not in members
    assert f"{ROOT}/stack/planner/lock" not in members
    assert f"{ROOT}/stack/planner/state" not in members
    assert members[f"{ROOT}/journal.log"] == b"kernel: hi\n"

    assert manifest["name"] == ROOT
    assert [f["name"] for f in manifest["stack"]["panes"]["planner"]] == [
        "@400000006a000001.s",
        "@400000006a000002.s",
        "current",
    ]
    assert json.loads(members[f"{ROOT}/manifest.json"]) == manifest


def test_a_pane_idle_all_day_is_listed_empty(logger, dirs):
    _write(dirs[0] / "sim" / "current", b"old", NOW - 2 * DAY)

    manifest, members = _bundle(_gateway(logger, dirs, _Journalctl()))

    assert manifest["stack"]["panes"] == {"sim": []}
    assert not [m for m in members if "/stack/" in m]


def test_journalctl_is_asked_for_the_window_from_the_mounted_directory(logger, dirs):
    journalctl = _Journalctl(b"x")
    _bundle(_gateway(logger, dirs, journalctl))

    (command,) = journalctl.commands
    assert command[:3] == ["journalctl", "-D", str(dirs[1])]
    assert f"--since=@{int(NOW - DAY)}" in command


def test_no_stack_directory_still_returns_the_journal(logger, dirs, tmp_path):
    gateway = _gateway(logger, dirs, _Journalctl(b"j"))
    gateway.stack_dir = str(tmp_path / "absent")

    manifest, members = _bundle(gateway)

    assert manifest["stack"]["included"] is False
    assert members[f"{ROOT}/journal.log"] == b"j"


@pytest.mark.parametrize(
    "journalctl, reason",
    [
        (_Journalctl(raises=FileNotFoundError()), "not installed"),
        (_Journalctl(raises=subprocess.TimeoutExpired("journalctl", 120)), "did not finish"),
        (_Journalctl(returncode=1, stderr=b"No journal files were found."), "No journal files"),
    ],
)
def test_a_failing_journalctl_leaves_the_journal_out(logger, dirs, journalctl, reason):
    _write(dirs[0] / "planner" / "current", b"live", NOW)

    manifest, members = _bundle(_gateway(logger, dirs, journalctl))

    assert manifest["journal"]["included"] is False
    assert reason in manifest["journal"]["error"]
    assert f"{ROOT}/journal.log" not in members
    assert members[f"{ROOT}/stack/planner/current"] == b"live"


def test_journalctl_output_is_kept_despite_a_nonzero_exit(logger, dirs):
    # journalctl exits 1 on one unreadable file among many; what it did print
    # is still the journal.
    manifest, members = _bundle(
        _gateway(logger, dirs, _Journalctl(b"partial\n", returncode=1))
    )

    assert manifest["journal"]["included"] is True
    assert members[f"{ROOT}/journal.log"] == b"partial\n"


def test_an_unmounted_journal_is_not_run(logger, dirs, tmp_path):
    journalctl = _Journalctl(b"x")
    gateway = _gateway(logger, dirs, journalctl)
    gateway.journal_dir = str(tmp_path / "absent")

    manifest, _ = _bundle(gateway)

    assert journalctl.commands == []
    assert "not mounted" in manifest["journal"]["error"]


def test_current_growing_while_tarred_keeps_the_size_it_was_opened_at(
    logger, dirs, monkeypatch
):
    path = dirs[0] / "planner" / "current"
    _write(path, b"12345", NOW)
    real_fstat = os.fstat

    def fstat_then_append(fd):
        stat = real_fstat(fd)
        with open(path, "ab") as f:
            f.write(b"more")
        return stat

    monkeypatch.setattr(os, "fstat", fstat_then_append)
    _, members = _bundle(_gateway(logger, dirs, _Journalctl()))

    assert members[f"{ROOT}/stack/planner/current"] == b"12345"
