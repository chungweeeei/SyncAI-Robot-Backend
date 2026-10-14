"""The storeys of a map, found from where the robot walked while mapping.

A map that spans storeys is one tree in one frame, and the console splits it
into floors to show one at a time. The split is read off the trajectory, not
off the building: the heights the robot stood at are floors by definition,
while a ceiling is a different height on every site and often half scanned.
A storey is a height the robot spent a real share of the run at; a stair or
a landing is passed through, and the share is what tells the two apart.
"""

import math
from typing import List, Sequence

# Height bins the trajectory is counted in, metres. Wider than a body's bob
# and a slab's slope over a floor, narrower than any storey.
FLOOR_BIN_M = 0.5
# Share of the run a bin needs to be a floor rather than a stair or landing.
# Keyframes are spaced by distance, so this is a share of the walk, not of time.
FLOOR_MIN_SHARE = 0.05
# Levels closer than this are one storey (a ramp, a split level); the one
# walked more keeps it.
FLOOR_MIN_GAP_M = 2.0
# Fewer keyframes than this is too short a run to tell a floor from noise.
FLOOR_MIN_POSES = 20


def read_pose_heights(path: str) -> List[float]:
    """The z of every keyframe in a pgo ``poses.txt`` (``<N>.pcd x y z qw qx qy qz``).

    Lines that do not parse are skipped rather than failing the read: the
    floors are a viewing aid, and a damaged line costs one sample.
    """
    heights: List[float] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                z = float(parts[3])
            except ValueError:
                continue
            if math.isfinite(z):
                heights.append(z)
    return heights


def detect_floor_levels(heights: Sequence[float]) -> List[float]:
    """The body heights of the storeys walked, lowest first; ``[]`` if too few.

    Bins the heights, keeps the bins holding at least FLOOR_MIN_SHARE of
    them, joins neighbouring kept bins into one storey (a level that sits on a
    bin edge splits across two) and reports each storey's median. Storeys
    closer than FLOOR_MIN_GAP_M collapse into the one walked more.
    """
    if len(heights) < FLOOR_MIN_POSES:
        return []
    lo = min(heights)
    bins: dict = {}
    for z in heights:
        bins.setdefault(int((z - lo) // FLOOR_BIN_M), []).append(z)

    need = FLOOR_MIN_SHARE * len(heights)
    kept = sorted(index for index, members in bins.items() if len(members) >= need)

    runs: List[List[float]] = []
    previous = None
    for index in kept:
        if previous is not None and index == previous + 1:
            runs[-1].extend(bins[index])
        else:
            runs.append(list(bins[index]))
        previous = index

    storeys = [(_median(run), len(run)) for run in runs]
    merged: List[tuple] = []
    for level, weight in storeys:
        if merged and level - merged[-1][0] < FLOOR_MIN_GAP_M:
            if weight > merged[-1][1]:
                merged[-1] = (level, weight)
            continue
        merged.append((level, weight))
    return [round(level, 2) for level, _ in merged]


def floor_name(index: int) -> str:
    """What a detected storey is called: ``1F`` for the lowest, and up."""
    return f"{index + 1}F"


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])
