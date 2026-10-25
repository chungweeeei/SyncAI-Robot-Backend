"""helpers/disk.py: what a directory tree holds."""

import os

import pytest

from syncai_backend.helpers.disk import tree_size, walk_stats


def _touch(path, size, mtime):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"x" * size)
    os.utime(path, (mtime, mtime))


def test_walk_stats_sums_nested_files_and_takes_the_newest(tmp_path):
    _touch(tmp_path / "map.pcd", 10, 1_000)
    _touch(tmp_path / "patches" / "0001.pcd", 5, 3_000)
    os.utime(tmp_path, (2_000, 2_000))

    assert walk_stats(str(tmp_path)) == (15, 3_000)


def test_walk_stats_of_an_empty_directory_is_its_own_mtime(tmp_path):
    os.utime(tmp_path, (2_000, 2_000))
    assert walk_stats(str(tmp_path)) == (0, 2_000)


def test_walk_stats_raises_for_a_missing_directory(tmp_path):
    with pytest.raises(FileNotFoundError):
        walk_stats(str(tmp_path / "gone"))


def test_tree_size_is_zero_for_a_missing_directory(tmp_path):
    """The live bag's directory may not exist yet when it is first asked."""
    assert tree_size(str(tmp_path / "not-yet")) == 0


def test_a_file_removed_mid_walk_is_skipped(tmp_path, monkeypatch):
    _touch(tmp_path / "a.db3", 7, 1_000)
    _touch(tmp_path / "b.db3", 3, 1_000)
    real_stat = os.stat

    def flaky_stat(path, *args, **kwargs):
        if str(path).endswith("b.db3"):
            raise FileNotFoundError(path)
        return real_stat(path, *args, **kwargs)

    monkeypatch.setattr(os, "stat", flaky_stat)

    assert tree_size(str(tmp_path)) == 7
