"""The storeys of a map, found from where the robot walked while mapping.

A map that spans storeys is one tree in one frame, and the console splits it
into floors to show one at a time. The split is read off the trajectory, not
off the building: the heights the robot stood at are floors by definition,
while a ceiling is a different height on every site and often half scanned.

The console splits a map so a floor above stops hiding the one under it, so
where the split goes is decided by that alone: where the robot walked one
piece of ground over another, a cut has to fall between the two, and nowhere
else is one needed. Two heights walked in different places hide nothing --
the top and the foot of a long slope, a ramp down to a loading bay -- and
splitting them cut a slope in two (outdoor_1027, dp_1028, 2026-10-29).

The cuts used to come from the heights instead: the levels the robot spent a
real share of the run at, joined bottom up while they were close or not
stacked. That holds for a building whose storeys are joined by stairs, and
fails a parking structure whose decks are joined by ramps: a deck that slopes
all the way round has no level of its own, so a garage of three turns came
out one floor, nine metres tall, every turn over the one under it. Cutting
where ground stacks works for both: the stair of a building and the ramp
between two decks are each where the fewest keyframes are, and that is where
a cut is put (outdoor_1119, 2026-11-23).

Within one floor, its level is still the height walked the most, and its
heights are those walked a real share of the run, so the stair that leads to
it is not part of it.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

# Height bins a floor's level and heights are read in, metres. Wider than a
# body's bob and a slab's slope over a floor, narrower than any storey.
FLOOR_BIN_M = 0.5
# Share of the run a bin needs to be where a floor's level is, rather than a
# stair or a landing. Keyframes are spaced by distance, so this is a share of
# the walk, not of time.
FLOOR_MIN_SHARE = 0.05
# Share a bin next to a level needs to widen the heights a floor spans: a
# part of the floor walked less, such as one end of a sloping deck. A flight
# of stairs is a share of a percent a bin, so it stays out and the floor ends
# where the stair starts.
FLOOR_EXTENT_SHARE = 0.02
# Fewer keyframes than this is too short a run to tell a floor from noise.
FLOOR_MIN_POSES = 20
# Plan cells the stacking is checked in, metres: about a corridor's width,
# so a floor walked over another along a parallel line still meets it.
FLOOR_STACK_CELL_M = 2.0
# Heights walked in one cell further apart than this are two pieces of
# ground, one over the other; closer, one ground (a slope across the cell, a
# split level, a flight of a stair). About a storey's clearance.
FLOOR_STACK_GAP_M = 2.0
# Keyframes a piece of ground in one cell needs to count, so one keyframe of
# drift cannot stack a floor.
FLOOR_STACK_MIN_POSES = 2
# Cells that have to agree on a cut for it to be made. More than one, so the
# bend at the top of a slope, where the two ends' cells touch, does not cut
# it.
FLOOR_STACK_MIN_CELLS = 3
# Bins a cut is moved along to the one walked least, metres: a stair's step.
FLOOR_CUT_BIN_M = 0.1

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


def detect_floors(poses: Sequence[Pose]) -> List[DetectedFloor]:
    """The storeys walked, lowest first; ``[]`` if too few keyframes.

    Cuts the run's heights where it walked ground over ground
    (``_stack_cuts``), then reads each piece's level and heights
    (``_floor_of``). A run with nothing stacked is one floor.

    A piece holding less than FLOOR_MIN_SHARE of the run is no floor: it is
    a stair, whose flights lie over the floor it leaves and under the one it
    reaches, so it is cut off both. Like a stair inside a floor, it is on
    none.
    """
    if len(poses) < FLOOR_MIN_POSES:
        return []
    cuts = _stack_cuts(poses)
    bands: List[List[Pose]] = [[] for _ in range(len(cuts) + 1)]
    for pose in poses:
        bands[sum(1 for cut in cuts if pose[2] >= cut)].append(pose)
    lo = min(z for _, _, z in poses)
    need = FLOOR_MIN_SHARE * len(poses)
    return [_floor_of(band, lo, len(poses)) for band in bands if len(band) >= need]


def _stack_cuts(poses: Sequence[Pose]) -> List[float]:
    """The heights the run is cut at, lowest first, so no floor holds ground
    walked over other ground of its own.

    Every cell where two pieces of ground lie one over the other asks for a
    cut between them: over the lower one's highest keyframe, under the upper
    one's lowest. The fewest cuts that answer every ask are found greedily
    (the intervals by their top: one cut answers each that starts under the
    first one's top), and a cut is then moved, within what all the cells it
    answers allow, to the height walked least -- a stair, the middle of a
    ramp -- so the fewest keyframes land on the wrong side of it.
    """
    cells: Dict[Tuple[int, int], List[float]] = {}
    for x, y, z in poses:
        key = (math.floor(x / FLOOR_STACK_CELL_M), math.floor(y / FLOOR_STACK_CELL_M))
        cells.setdefault(key, []).append(z)

    asks: List[Tuple[float, float]] = []
    for heights in cells.values():
        grounds = _grounds(sorted(heights))
        for lower, upper in zip(grounds, grounds[1:]):
            asks.append((lower[-1], upper[0]))
    asks.sort(key=lambda ask: ask[1])

    counts: Dict[int, int] = {}
    for _, _, z in poses:
        index = math.floor(z / FLOOR_CUT_BIN_M)
        counts[index] = counts.get(index, 0) + 1

    cuts: List[float] = []
    i = 0
    while i < len(asks):
        bottom, top = asks[i]
        answered = 1
        i += 1
        while i < len(asks) and asks[i][0] < top:
            bottom = max(bottom, asks[i][0])
            answered += 1
            i += 1
        if answered >= FLOOR_STACK_MIN_CELLS:
            cuts.append(_least_walked(bottom, top, counts))
    return cuts


def _grounds(heights: Sequence[float]) -> List[List[float]]:
    """The pieces of ground in one cell: its sorted heights split where two
    are FLOOR_STACK_GAP_M apart, each piece walked FLOOR_STACK_MIN_POSES times."""
    pieces: List[List[float]] = []
    for z in heights:
        if pieces and z - pieces[-1][-1] < FLOOR_STACK_GAP_M:
            pieces[-1].append(z)
        else:
            pieces.append([z])
    return [piece for piece in pieces if len(piece) >= FLOOR_STACK_MIN_POSES]


def _least_walked(bottom: float, top: float, counts: Dict[int, int]) -> float:
    """A cut over ``bottom`` and no higher than ``top``: the bin edge with
    the fewest keyframes in the two bins either side of it, the one nearest
    the middle among equals; the middle itself when no edge falls between."""
    middle = 0.5 * (bottom + top)
    first = math.floor(bottom / FLOOR_CUT_BIN_M) + 1
    last = math.ceil(top / FLOOR_CUT_BIN_M) - 1
    best: Optional[Tuple[int, float, float]] = None
    for index in range(first, last + 1):
        # A bin's lower edge is the cut: everything in it goes up, so the
        # cut must clear the lower ground and stay under the upper one.
        cut = index * FLOOR_CUT_BIN_M
        if not bottom < cut <= top:
            continue
        rank = (counts.get(index - 1, 0) + counts.get(index, 0), abs(cut - middle), cut)
        if best is None or rank < best:
            best = rank
    return round(best[2], 2) if best is not None else round(middle, 3)


def _floor_of(band: Sequence[Pose], lo: float, total: int) -> DetectedFloor:
    """One floor from the keyframes between two cuts.

    Bins its heights, keeps the bins holding FLOOR_MIN_SHARE of the whole
    run, and joins neighbouring kept bins into one run (a level on a bin
    edge splits across two). Each run widens over the bins either side
    walked FLOOR_EXTENT_SHARE of it; the floor spans every run, at the
    level of the one walked most. A floor with no such bin -- a deck that
    slopes all the way between its cuts -- is the whole band, at its median.
    """
    bins: Dict[int, List[Pose]] = {}
    for pose in band:
        bins.setdefault(int((pose[2] - lo) // FLOOR_BIN_M), []).append(pose)

    need = FLOOR_MIN_SHARE * total
    kept = sorted(index for index, members in bins.items() if len(members) >= need)
    if not kept:
        heights = [z for _, _, z in band]
        return DetectedFloor(
            level=round(_median(heights), 2),
            low=round(min(heights), 2),
            high=round(max(heights), 2),
        )

    groups: List[List[int]] = []
    for index in kept:
        if groups and index == groups[-1][-1] + 1:
            groups[-1].append(index)
        else:
            groups.append([index])

    widen = FLOOR_EXTENT_SHARE * total
    taken = set(kept)
    spanned: List[float] = []
    heaviest: List[float] = []
    for group in groups:
        core = [z for index in group for _, _, z in bins[index]]
        if len(core) > len(heaviest):
            heaviest = core
        spanned.extend(core)
        for step, start in ((-1, group[0]), (1, group[-1])):
            index = start + step
            while index not in taken and len(bins.get(index, ())) >= widen:
                taken.add(index)
                spanned.extend(z for _, _, z in bins[index])
                index += step

    return DetectedFloor(
        level=round(_median(heaviest), 2),
        low=round(min(spanned), 2),
        high=round(max(spanned), 2),
    )


def floor_name(index: int) -> str:
    """What a detected storey is called: ``1F`` for the lowest, and up."""
    return f"{index + 1}F"


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])
