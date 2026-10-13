"""Whether a job may drive on the map that is loaded right now.

``POST /api/v1/tasks`` asks before a run starts (409), and
``RobotActivities.execute_move`` asks again before every goal (fails the run),
because a schedule skips the first check and a map can start rebuilding
mid-run. Both share these codes and sentences, which are operator prose: the
console and the history row show them verbatim. Pure, so the rule is tested
here without a catalogue or a worker.
"""

from dataclasses import dataclass
from typing import Optional


# Wire codes beside ConflictError's detail. ``conversion_running`` matches the
# code the map routes use for the same condition.
MAP_MISMATCH = "map_mismatch"
CONVERSION_RUNNING = "conversion_running"


@dataclass(frozen=True)
class MoveRefusal:
    code: str
    message: str


def move_refusal(
    expected_map: Optional[str],
    active_map: Optional[str],
    active_converting: bool,
) -> Optional[MoveRefusal]:
    """Why a job with MOVE steps must not drive now, or None if it may.

    ``expected_map`` is the map the job's coordinates are in (the dispatch's
    expectation, or a run's ``TaskMap``). None means unknown and is not
    refused: the MCP server, curl and old schedules carry none. A mismatch is
    checked first, since switching maps is then the fix that applies.
    """
    if expected_map and expected_map != active_map:
        loaded = (
            f"the robot is using '{active_map}'"
            if active_map
            else "the robot has no map loaded"
        )
        return MoveRefusal(
            code=MAP_MISMATCH,
            message=(
                f"This job's positions are on the map '{expected_map}', but "
                f"{loaded}. Switch the robot to '{expected_map}' first."
            ),
        )
    if active_converting:
        return MoveRefusal(
            code=CONVERSION_RUNNING,
            message=(
                "The floor plan of the map in use is being rebuilt. Wait for it "
                "to finish before sending the robot anywhere."
            ),
        )
    return None
