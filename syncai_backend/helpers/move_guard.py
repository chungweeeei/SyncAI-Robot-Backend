"""Whether a job may drive on the map that is loaded right now.

Two consumers ask the same question at two moments, and they must not
disagree about the answer: ``POST /api/v1/tasks`` asks it before a run is
started (and answers 409), and ``RobotActivities.execute_move`` asks it again
before every goal it sends (and fails the run). The second exists because a
schedule fires without passing through the first, and because a map can start
rebuilding between a dispatch and its third MOVE step. Both read the codes and
sentences from here, which is also why the sentences are operator prose: the
router hands them to the console verbatim, and a failed step's message is what
the history row shows.

Pure on purpose -- the active map and whether it is converting are handed in --
so the rule is tested once, here, without a catalogue or a worker.
"""

from dataclasses import dataclass
from typing import Optional


# The wire codes, beside ConflictError's detail. ``conversion_running`` is the
# code the map routes already answer for "this map's floor plan is being
# rebuilt"; the console reads one word for one condition, whichever route said it.
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

    ``expected_map`` is the map the job's coordinates are in, when anyone said:
    a caller's expectation at dispatch, a run's ``TaskMap`` in the activity.
    None means unknown, and an unknown map is not refused -- the MCP server and
    curl dispatch without one, and a schedule older than the attribute has none.

    The mismatch is checked first. A job planned on another map is wrong
    however the loaded one is doing, and the fix the sentence names (switch
    maps) is the one that applies.
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
