"""How much a directory tree holds, for the map and recording catalogues and
the recording gateway.

In helpers, the bottom layer, so a gateway and a repository can share it: the
recording gateway kept its own copy because importing a repository would have
inverted the layering.

A real walk, not the directory's own size: a map's ``patches/`` holds hundreds
of small files and a long bag splits into several ``<name>_N.db3``.
"""

import os
from typing import Iterator, Tuple


def _file_stats(path: str) -> Iterator[os.stat_result]:
    """Every file under ``path``; one removed mid-walk (a bag split being
    rotated, a save in flight) is skipped. Nothing for a missing ``path``."""
    for root, _dirs, files in os.walk(path):
        for filename in files:
            try:
                yield os.stat(os.path.join(root, filename))
            except FileNotFoundError:
                continue


def walk_stats(path: str) -> Tuple[int, float]:
    """Total bytes and newest mtime under ``path``, the directory itself
    included (so an empty one has an mtime). Raises for a missing ``path``."""
    total = 0
    newest = os.stat(path).st_mtime
    for stats in _file_stats(path):
        total += stats.st_size
        newest = max(newest, stats.st_mtime)
    return total, newest


def tree_size(path: str) -> int:
    """Total bytes under ``path``, or 0 if it is not there yet."""
    return sum(stats.st_size for stats in _file_stats(path))
