"""The storeys of a map, found from where the robot walked while mapping.

A map that spans storeys is one tree in one frame, and the console splits it
into floors to show one at a time. The split is read off the trajectory, not
off the building: the heights the robot stood at are floors by definition,
while a ceiling is a different height on every site and often half scanned.
A storey is a height the robot spent a real share of the run at; a stair or
a landing is passed through, and the share is what tells the two apart.

A height is not yet a storey, though. The console splits a map so a floor
above stops hiding the one under it, and two heights walked in different
places hide nothing: the top and the foot of a long slope, a ramp down to a
loading bay. Those are one floor that is not flat, and splitting it cut the
slope in two (outdoor_1027, dp_1028, 2026-10-29). So two heights are two
storeys only where the robot walked one over the other.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Set, Tuple

# Height bins the trajectory is counted in, metres. Wider than a body's bob
# and a slab's slope over a floor, narrower than any storey.
FLOOR_BIN_M = 0.5
# Share of the run a bin needs to be a floor rather than a stair or landing.
# Keyframes are spaced by distance, so this is a share of the walk, not of time.
FLOOR_MIN_SHARE = 0.05
# Share a bin next to a storey needs to widen it: a part of the floor walked
# less, such as one end of a sloping deck. A flight of stairs is a share of a
# percent a bin, so it stays out and the floor ends where the stair starts.
FLOOR_EXTENT_SHARE = 0.02
# Levels closer than this are one storey (a split level); the one walked
# more keeps it.
FLOOR_MIN_GAP_M = 2.0
# Fewer keyframes than this is too short a run to tell a floor from noise.
FLOOR_MIN_POSES = 20
# Plan cells the stacking is checked in, metres: about a corridor's width,
# so a floor walked over another along a parallel line still meets it.
FLOOR_STACK_CELL_M = 2.0
# Cells two heights have to share to be one over the other. More than one,
# so the bend at the top of a slope, where the two ends' cells touch, does
# not stack them.
FLOOR_STACK_MIN_CELLS = 3

Pose = Tuple[float, float, float]


@dataclass(frozen=True)
class DetectedFloor:
    """One storey: the body height it is known by, and the heights walked on it."""

    level: float
    low: float
    high: float


def read_poses(path: str) -> List[Pose]:
    """The x, y, z of every keyframe in a pgo ``poses.txt`` (``<N>.pcd x y z qw qx qy qz``).

    Lines that do not parse are skipped rather than failing the read: the
    floors are a viewing aid, and a damaged line costs one sample.
    """
    poses: List[Pose] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                poses.append((x, y, z))
    return poses


@dataclass
class _Storey:
    # Where the robot walked at the storey's own heights, which is what the
    # stacking is checked on: the bins it widened over are the slope between
    # two candidates, which lies under both and would stack every ramp.
    core: List[Pose]
    poses: List[Pose]
    level: float
    weight: int

    def cells(self) -> Set[Tuple[int, int]]:
        return {
            (math.floor(x / FLOOR_STACK_CELL_M), math.floor(y / FLOOR_STACK_CELL_M))
            for x, y, _ in self.core
        }


def detect_floors(poses: Sequence[Pose]) -> List[DetectedFloor]:
    """The storeys walked, lowest first; ``[]`` if too few keyframes.

    Bins the heights, keeps the bins holding at least FLOOR_MIN_SHARE of
    them, and joins neighbouring kept bins into one candidate (a level that
    sits on a bin edge splits across two). Going up, a candidate joins the
    storey under it when the two are closer than FLOOR_MIN_GAP_M or were
    never walked one over the other; the storey keeps the level of the
    candidate walked more, and spans the heights of both, so a slope is
    one floor from its foot to its top.
    """
    if len(poses) < FLOOR_MIN_POSES:
        return []
    lo = min(z for _, _, z in poses)
    bins: Dict[int, List[Pose]] = {}
    for pose in poses:
        bins.setdefault(int((pose[2] - lo) // FLOOR_BIN_M), []).append(pose)

    need = FLOOR_MIN_SHARE * len(poses)
    kept = sorted(index for index, members in bins.items() if len(members) >= need)

    groups: List[List[int]] = []
    for index in kept:
        if groups and index == groups[-1][-1] + 1:
            groups[-1].append(index)
        else:
            groups.append([index])

    # Each run widens over the bins either side of it that were walked too
    # little to be a storey of their own but enough to be part of one. They
    # widen the heights the storey spans, not its level, which stays where
    # most of the walk was.
    widen = FLOOR_EXTENT_SHARE * len(poses)
    taken = set(kept)
    runs: List[Tuple[List[Pose], List[Pose]]] = []
    for group in groups:
        edges: List[Pose] = []
        for step, start in ((-1, group[0]), (1, group[-1])):
            index = start + step
            while index not in taken and len(bins.get(index, ())) >= widen:
                taken.add(index)
                edges.extend(bins[index])
                index += step
        core = [pose for index in group for pose in bins[index]]
        runs.append((core, core + edges))

    storeys: List[_Storey] = []
    for core, run in runs:
        level = _median([z for _, _, z in core])
        candidate = _Storey(core=list(core), poses=list(run), level=level, weight=len(core))
        below = storeys[-1] if storeys else None
        if below is not None and (
            level - below.level < FLOOR_MIN_GAP_M
            or len(below.cells() & candidate.cells()) < FLOOR_STACK_MIN_CELLS
        ):
            if candidate.weight > below.weight:
                below.level = candidate.level
                below.weight = candidate.weight
            below.core.extend(core)
            below.poses.extend(run)
            continue
        storeys.append(candidate)

    return [
        DetectedFloor(
            level=round(storey.level, 2),
            low=round(min(z for _, _, z in storey.poses), 2),
            high=round(max(z for _, _, z in storey.poses), 2),
        )
        for storey in storeys
    ]


def floor_name(index: int) -> str:
    """What a detected storey is called: ``1F`` for the lowest, and up."""
    return f"{index + 1}F"


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])
