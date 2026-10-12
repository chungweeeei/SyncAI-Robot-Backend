"""The recipe sidecar is written atomically, like the yaml beside it.

It used to be a plain ``open("w")`` -- the one non-atomic write in the map
directory. ``MapCatalogRepo`` reads the sidecar on every listing and tolerates
a torn file by returning None, at which point ``_grid_status`` falls through to
"the grid on disk decides" and reports ``ok`` for a re-conversion that had just
written ``failed``. The reader's tolerance is right; the writer should not be
producing the torn file in the first place.
"""

import json
import os

import pytest

pytest.importorskip("numpy")
pytest.importorskip("scipy")

from syncai_backend.repositories.map.catalog import (  # noqa: E402
    GRIDMAP_RECIPE_SIDECAR,
    MAP_CLEAN_SIDECAR,
)
from syncai_backend.services import gridmap_conversion as svc  # noqa: E402


def _sidecar(directory):
    return directory / GRIDMAP_RECIPE_SIDECAR


def test_the_sidecar_lands_whole_and_leaves_no_temp_file(logger, tmp_path):
    svc.write_recipe_sidecar(logger, str(tmp_path), {"status": "converting"})

    assert json.loads(_sidecar(tmp_path).read_text()) == {"status": "converting"}
    assert os.listdir(tmp_path) == [GRIDMAP_RECIPE_SIDECAR]


def test_a_failed_write_leaves_the_previous_record_untouched(logger, tmp_path, monkeypatch):
    """The atomic property: either the old file or the new one, never a torn
    one -- and no stray temp file for the catalogue to trip over."""
    svc.write_recipe_sidecar(logger, str(tmp_path), {"status": "ok", "recipe": "zband"})
    before = _sidecar(tmp_path).read_text()

    def _disk_full(src, dst):
        raise OSError("No space left on device")

    monkeypatch.setattr(svc.os, "replace", _disk_full)

    svc.write_recipe_sidecar(logger, str(tmp_path), {"status": "failed"})  # must not raise

    assert _sidecar(tmp_path).read_text() == before
    assert os.listdir(tmp_path) == [GRIDMAP_RECIPE_SIDECAR]


# --- wait_for_map_clean ------------------------------------------------------
#
# The robot's clean_map replaces map.pcd after save_maps has answered; a
# conversion that does not wait for it converts the raw cloud, people included.


class _Clock:
    """Monotonic + wall clock that only move when the wait sleeps."""

    def __init__(self, wall="2026-10-12T05:44:10+00:00"):
        from datetime import datetime

        self.t = 0.0
        self.wall0 = datetime.fromisoformat(wall)
        self.sleeps = []
        self.on_sleep = None

    def monotonic(self):
        return self.t

    def wall_now(self):
        from datetime import timedelta

        return self.wall0 + timedelta(seconds=self.t)

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.t += seconds
        if self.on_sleep is not None:
            self.on_sleep(self)


def _write_clean(directory, **record):
    (directory / MAP_CLEAN_SIDECAR).write_text(json.dumps(record))


def _wait(logger, directory, clock, **kwargs):
    return svc.wait_for_map_clean(
        logger,
        str(directory),
        sleep=clock.sleep,
        monotonic=clock.monotonic,
        wall_now=clock.wall_now,
        **kwargs,
    )


def test_no_clean_sidecar_converts_at_once(logger, tmp_path):
    clock = _Clock()
    assert _wait(logger, tmp_path, clock) == {"status": "absent"}
    assert clock.sleeps == []


def test_waits_until_the_clean_says_ok(logger, tmp_path):
    _write_clean(tmp_path, status="converting", started_at="2026-10-12T05:44:08Z")
    clock = _Clock()

    def _finish_after_three_polls(c):
        if len(c.sleeps) == 3:
            _write_clean(tmp_path, status="ok", started_at="2026-10-12T05:44:08Z")

    clock.on_sleep = _finish_after_three_polls

    result = _wait(logger, tmp_path, clock, poll_s=2.0)

    assert result == {"status": "ok", "waited_s": 6.0}
    assert clock.sleeps == [2.0, 2.0, 2.0]


def test_a_failed_clean_converts_the_raw_cloud_and_says_why(logger, tmp_path):
    _write_clean(tmp_path, status="failed", error="no patch under patches/ could be read")
    result = _wait(logger, tmp_path, _Clock())
    assert result["status"] == "failed"
    assert result["error"] == "no patch under patches/ could be read"


def test_a_clean_killed_long_ago_is_not_waited_for(logger, tmp_path):
    """A hard kill leaves `converting` forever; its started_at ages it out."""
    _write_clean(tmp_path, status="converting", started_at="2026-10-12T04:00:00Z")
    clock = _Clock()
    result = _wait(logger, tmp_path, clock, timeout_s=900.0)
    assert result["status"] == "stale"
    assert clock.sleeps == []


def test_gives_up_at_the_deadline(logger, tmp_path):
    """The deadline holds on its own, without a started_at to age the clean by
    (a sidecar that lacks one, or a wall clock that disagrees with the robot's)."""
    _write_clean(tmp_path, status="converting")
    clock = _Clock()
    result = _wait(logger, tmp_path, clock, timeout_s=10.0, poll_s=2.0)
    assert result == {"status": "timed_out", "waited_s": 10.0}


def test_an_unreadable_sidecar_is_retried_not_taken_as_absent(logger, tmp_path):
    """Read as absent, a torn sidecar would convert the raw cloud on a misread."""
    (tmp_path / MAP_CLEAN_SIDECAR).write_text('{"status": "conv')
    clock = _Clock()
    clock.on_sleep = lambda c: _write_clean(tmp_path, status="ok")
    assert _wait(logger, tmp_path, clock, poll_s=2.0) == {"status": "ok", "waited_s": 2.0}


def test_the_conversion_thread_reads_the_cloud_only_after_the_clean(
    logger, tmp_path, monkeypatch
):
    """End to end through the service: nothing reads map.pcd while the robot is
    still rewriting it, and the grid's record says which cloud it came from."""
    import threading

    (tmp_path / "map.pcd").write_bytes(b"")
    _write_clean(tmp_path, status="converting", started_at=svc.iso_now())

    measured = threading.Event()

    def _measure(_logger, _path):
        measured.set()
        raise ValueError("stop here; the recipe itself is not under test")

    monkeypatch.setattr(svc, "measure_cloud", _measure)
    service = svc.GridmapConversionService(logger, map_clean_poll_s=0.01)

    assert service.start("m", str(tmp_path)) is True
    assert not measured.wait(0.2), "converted before the clean finished"
    assert service.is_converting("m")
    assert json.loads(_sidecar(tmp_path).read_text())["status"] == "converting"

    _write_clean(tmp_path, status="ok")
    assert measured.wait(5.0)
    for _ in range(500):
        if not service.is_converting("m"):
            break
        threading.Event().wait(0.01)

    record = json.loads(_sidecar(tmp_path).read_text())
    assert record["status"] == "failed"
    assert record["map_clean"]["status"] == "ok"
