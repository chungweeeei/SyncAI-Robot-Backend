"""Tests for the Temporal activities, run under ``ActivityEnvironment``.

``temporalio.testing.ActivityEnvironment`` supplies the activity context that
``activity.heartbeat`` needs, so the real polling loops run unmodified; the
gateways are MagicMocks (the CoreManager seam-mocking pattern) and the module's
``time`` is patched where a test would otherwise sleep.

What is pinned here is the retryability contract, because Temporal acts on it:
a MOVE that aborts is retryable (the path may clear), a rejected motion key is
not (the driver said no and will keep saying no).

Cancellation is covered by raising ``CancelledError`` from the gateway seam
(where Temporal would throw it into the thread) and asserting the cleanup call;
the real thread injection and shielding need a threaded worker, which is
integration-test territory.
"""

from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip("temporalio")

from temporalio.exceptions import ApplicationError, CancelledError  # noqa: E402
from temporalio.testing import ActivityEnvironment  # noqa: E402

from syncai_backend.gateways.robot.robot import MotionKey  # noqa: E402
from syncai_backend.gateways.workflow.schema import MoveParams  # noqa: E402
from syncai_backend.temporal.activities import RobotActivities  # noqa: E402


SLEEP = "syncai_backend.temporal.activities.time.sleep"


@pytest.fixture
def robot_gw():
    return MagicMock()


@pytest.fixture
def tts_gw():
    return MagicMock()


@pytest.fixture
def activities(logger, robot_gw, tts_gw) -> RobotActivities:
    return RobotActivities(logger=logger, robot_gw=robot_gw, tts_gw=tts_gw)


@pytest.fixture
def env() -> ActivityEnvironment:
    return ActivityEnvironment()


class TestExecuteMove:
    def test_success_converts_degrees_and_polls_to_terminal(
        self, env, activities, robot_gw
    ):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "succeeded"}

        result = env.run(
            activities.execute_move, MoveParams(x=1.0, y=2.0, theta=90.0)
        )

        assert result.success is True
        assert (result.goal_id, result.state) == ("goal-1", "succeeded")
        # Degrees on the wire, radians at the gateway — this boundary converts.
        kwargs = robot_gw.move.call_args[1]
        assert kwargs["yaw"] == pytest.approx(1.5707963)

    def test_rejection_is_retryable(self, env, activities, robot_gw):
        robot_gw.move.return_value = (False, "server not available", None)

        with pytest.raises(ApplicationError, match="Move rejected") as exc_info:
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        # The task runner coming up a moment later must be given the chance.
        assert exc_info.value.non_retryable is False

    def test_an_aborted_goal_fails_the_attempt(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "aborted"}

        with pytest.raises(ApplicationError, match="move ended in aborted"):
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

    def test_polling_heartbeats_until_terminal(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.side_effect = [
            {"goal_id": "goal-1", "state": "executing"},
            {"goal_id": "goal-1", "state": "succeeded"},
        ]
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        with patch(SLEEP):  # the 1 s poll pause, not needed under test
            result = env.run(
                activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0)
            )

        assert result.success is True
        # One before the send, then one per poll: what keeps the 3 s
        # heartbeat_timeout fed from activity start onwards.
        assert beats == [("sending",), ("executing",), ("succeeded",)]

    def test_the_first_heartbeat_precedes_the_send(self, env, activities, robot_gw):
        """The heartbeat clock starts at activity start, and move() has no loop
        to heartbeat from; a slow nav2 used to fail the attempt before the poll
        loop -- and its first heartbeat -- was ever reached."""
        order = []

        def _move(**kwargs):
            order.append("move")
            return True, "", "goal-1"

        robot_gw.move.side_effect = _move
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "succeeded"}
        env.on_heartbeat = lambda *details: order.append(("beat", *details))

        env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        assert order[:2] == [("beat", "sending"), "move"]

    def test_the_heartbeat_window_covers_the_goal_send(self):
        """Tighten either side and the other has to follow -- see the comments
        on both constants."""
        from syncai_backend.gateways.robot.robot import NAV_GOAL_SEND_BUDGET_S
        from syncai_backend.temporal.workflows import MOVE_HEARTBEAT_TIMEOUT

        assert MOVE_HEARTBEAT_TIMEOUT.total_seconds() > NAV_GOAL_SEND_BUDGET_S

    def test_a_cancel_during_the_send_cancels_whatever_is_executing(
        self, env, activities, robot_gw
    ):
        """No goal id to remember yet, so the cleanup goes by goal state."""
        robot_gw.move.side_effect = CancelledError()

        with pytest.raises(CancelledError):
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        robot_gw.cancel_active_moves.assert_called_once()

    def test_a_cancel_while_polling_cancels_the_goal(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.side_effect = CancelledError()

        with pytest.raises(CancelledError):
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        robot_gw.cancel_active_moves.assert_called_once()


class TestPostureActivities:
    def test_stand_sends_its_motion_key(self, env, activities, robot_gw):
        robot_gw.set_motion_key.return_value = (True, "")

        result = env.run(activities.execute_stand)

        assert result.success is True
        assert robot_gw.set_motion_key.call_args[1]["key"] is MotionKey.STAND

    def test_a_rejected_key_is_non_retryable(self, env, activities, robot_gw):
        # A False from the driver means unknown key or safety lock — neither
        # goes away on its own, so retrying would just hammer the service.
        robot_gw.set_motion_key.return_value = (False, "LOCKED")

        with pytest.raises(ApplicationError, match="LieDown rejected") as exc_info:
            env.run(activities.execute_lie_down)

        assert exc_info.value.non_retryable is True
