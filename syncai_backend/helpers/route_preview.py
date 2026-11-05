"""A route preview's answer, as ``POST /api/v1/route_preview`` sends it.

The planner's ``preview_path_to_pose`` (syncai_global_planner, 2026-11)
answers with ComputePathToPose's result: the path it would drive, or the
``error_code`` / ``error_msg`` of why there is none. Both are an answer to the
operator's question "what route would the robot take?", so a planning failure
is a 200 with ``reachable: false``, not an error status: the request was fine,
the goal is not.

Pure (no rclpy), so the rules are tested off the robot: the thinning the
console's route ribbon needs, and the failure's sentence, which is the task
runner's own (``pose_navigator.cpp``'s ``kPlanningSentenceByCode``) so a
preview that fails reads exactly like the job that would have failed -- and
the console's translation of that sentence (its ``lib/task/failure.ts``)
covers both.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


Point = Tuple[float, float, float]

# The same bounds as the live route's (subscribers/path_subscriber.py): the
# console draws both with one ribbon.
MAX_PATH_POINTS = 512
MIN_POINT_SPACING_M = 1e-3

# ComputePathToPose's 2xx (syncai_common), each with the wire code the console
# may branch on and the task runner's sentence. A code missing here (one the
# interface adds first) is answered as PLAN_FAILED, with the planner's own
# detail still in the parentheses, so the reason is not lost to a stale table.
PLAN_FAILED = 200
_FAILURES: dict[int, Tuple[str, str]] = {
    PLAN_FAILED: ("plan_failed", "Planning failed: no path to the goal"),
    201: ("invalid_planner", "Planning failed: the requested planner is not loaded"),
    202: ("tf_error", "Planning failed: no robot pose or transform to plan with"),
    203: ("start_outside_map", "Planning failed: the robot is outside the map"),
    204: ("goal_outside_map", "Planning failed: the goal is outside the map"),
    205: ("start_occupied", "Planning failed: no free ground where the robot stands"),
    206: (
        "goal_occupied",
        "Planning failed: no free ground at the goal (a wall, a ledge, another "
        "floor, or an obstacle on it)",
    ),
    207: ("timeout", "Planning failed: the search for a path ran out of time"),
    208: ("no_valid_path", "Planning failed: the way to the goal is closed"),
    209: ("no_viapoints_given", "Planning failed: the route has no waypoints"),
    220: ("map_not_loaded", "Planning failed: no map is loaded"),
    221: ("map_loading", "Planning failed: the map is still loading"),
    222: (
        "start_in_keepout",
        "Planning failed: the robot stands inside a keepout zone",
    ),
    223: ("goal_in_keepout", "Planning failed: the goal is inside a keepout zone"),
    224: (
        "disconnected",
        "Planning failed: the goal is on ground no stair connects to the robot's",
    ),
}


@dataclass(frozen=True)
class RoutePreview:
    reachable: bool
    # Thinned (x, y, z) in the map frame; z is the ground the route stands on,
    # so a route up a stair is on both storeys. Empty when not reachable.
    points: Tuple[Point, ...]
    # Along the whole path, before thinning, in 3D: a stair's rise counts.
    length_m: float
    # Machine-readable reason when not reachable, else None.
    code: Optional[str]
    # The operator's sentence when not reachable, else "".
    message: str


def thin_points(points: Sequence[Point]) -> Tuple[Point, ...]:
    """At most MAX_PATH_POINTS deduplicated triples, the goal always kept."""
    if not points:
        return ()
    stride = max(1, -(-len(points) // MAX_PATH_POINTS))  # ceil division

    def rounded(p: Point) -> Point:
        # Millimetres keep the JSON small; far finer than the planner's cells.
        return (round(p[0], 3), round(p[1], 3), round(p[2], 3))

    out: list = []
    for i in range(0, len(points), stride):
        xyz = rounded(points[i])
        if out and _too_close(out[-1], xyz):
            continue
        out.append(xyz)

    # An uneven stride would drop the goal.
    goal = rounded(points[-1])
    if not _too_close(out[-1], goal):
        out.append(goal)
    return tuple(out)


def _too_close(a: Point, b: Point) -> bool:
    # All three axes: a point straight over the last one is a step up a stair,
    # not a duplicate -- and a zero-length segment blanks the console's ribbon.
    return all(abs(p - q) < MIN_POINT_SPACING_M for p, q in zip(a, b))


def path_length(points: Sequence[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def planned(points: Sequence[Point]) -> RoutePreview:
    return RoutePreview(
        reachable=True,
        points=thin_points(points),
        length_m=round(path_length(points), 2),
        code=None,
        message="",
    )


def refused(error_code: int, error_msg: str) -> RoutePreview:
    """A plan the planner aborted, in the task runner's words: its sentence
    for the code, then the planner's own detail in parentheses."""
    code, sentence = _FAILURES.get(error_code, _FAILURES[PLAN_FAILED])
    detail = error_msg.strip()
    return RoutePreview(
        reachable=False,
        points=(),
        length_m=0.0,
        code=code,
        message=f"{sentence} ({detail})" if detail else sentence,
    )
