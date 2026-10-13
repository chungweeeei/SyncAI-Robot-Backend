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
from syncai_backend.gateways.failure import Failure, fail  # noqa: E402
from syncai_backend.gateways.workflow.schema import MoveParams, SpeakParams  # noqa: E402
from syncai_backend.temporal.activities import RobotActivities  # noqa: E402


SLEEP = "syncai_backend.temporal.activities.time.sleep"


@pytest.fixture
def robot_gw():
    return MagicMock()


@pytest.fixture
def tts_gw():
    return MagicMock()


@pytest.fixture
def map_catalog_repo():
    repo = MagicMock()
    repo.active_name.return_value = "lab"
    return repo


@pytest.fixture
def conversion_svc():
    svc = MagicMock()
    svc.is_converting.return_value = False
    return svc


@pytest.fixture
def activities(
    logger, robot_gw, tts_gw, map_catalog_repo, conversion_svc
) -> RobotActivities:
    return RobotActivities(
        logger=logger,
        robot_gw=robot_gw,
        tts_gw=tts_gw,
        map_catalog_repo=map_catalog_repo,
        conversion_svc=conversion_svc,
    )


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

    def test_an_aborted_goal_carries_the_robots_reason(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {
            "goal_id": "goal-1",
            "state": "aborted",
            "error_code": 100,
            "error_msg": "Path following failed: the robot could not reach the goal",
        }

        with pytest.raises(ApplicationError) as exc_info:
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        assert str(exc_info.value) == (
            "move ended in aborted: "
            "Path following failed: the robot could not reach the goal"
        )

    def test_an_abort_with_no_reason_says_navigation_did_not_start(
        self, env, activities, robot_gw
    ):
        # NavigateToGoal's contract: ABORTED + NONE is superseded or never
        # started, not a navigation failure.
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {
            "goal_id": "goal-1",
            "state": "aborted",
            "error_code": 0,
            "error_msg": "",
        }

        with pytest.raises(ApplicationError, match="navigation did not start"):
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

    def test_polling_heartbeats_until_terminal(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.side_effect = [
            {"goal_id": "goal-1", "state": "executing"},
            {"goal_id": "goal-1", "state": "succeeded"},
        ]
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        with patch(SLEEP):  # the poll pause, not needed under test
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

    def test_a_cancel_reaches_the_poll_loop_quickly(self):
        """A cancel arrives on a heartbeat reply and lands after the current
        poll sleep. The throttle cap only shortens anything while it is below
        the SDK's own 0.8 x heartbeat_timeout, and the poll has to produce a
        heartbeat at least as often as one may be sent."""
        from syncai_backend.temporal.activities import NAV_POLL_INTERVAL_S
        from syncai_backend.temporal.worker import HEARTBEAT_THROTTLE_MAX
        from syncai_backend.temporal.workflows import MOVE_HEARTBEAT_TIMEOUT

        throttle = HEARTBEAT_THROTTLE_MAX.total_seconds()
        assert throttle < 0.8 * MOVE_HEARTBEAT_TIMEOUT.total_seconds()
        assert NAV_POLL_INTERVAL_S <= throttle

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


class TestMoveMapCheck:
    """The same rule POST /api/v1/tasks applies, asked again before driving --
    the only check a scheduled run ever meets."""

    def test_a_run_on_the_loaded_map_drives(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "succeeded"}

        result = env.run(
            activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0), "lab"
        )

        assert result.success is True

    def test_a_run_from_another_map_never_sends_a_goal(
        self, env, activities, robot_gw
    ):
        with pytest.raises(ApplicationError, match="'warehouse'") as exc_info:
            env.run(
                activities.execute_move,
                MoveParams(x=0.0, y=0.0, theta=0.0),
                "warehouse",
            )

        # Retrying cannot switch the map back; the run fails with the reason.
        assert exc_info.value.non_retryable is True
        assert exc_info.value.type == "map_mismatch"
        robot_gw.move.assert_not_called()

    def test_a_run_with_no_map_is_not_refused(self, env, activities, robot_gw):
        # A schedule older than TaskMap: it drives as it always did.
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "succeeded"}

        result = env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        assert result.success is True

    def test_a_rebuilding_floor_plan_stops_the_move(
        self, env, activities, robot_gw, conversion_svc
    ):
        conversion_svc.is_converting.return_value = True

        with pytest.raises(ApplicationError, match="being rebuilt") as exc_info:
            env.run(
                activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0), "lab"
            )

        assert exc_info.value.non_retryable is True
        assert exc_info.value.type == "conversion_running"
        conversion_svc.is_converting.assert_called_with("lab")
        robot_gw.move.assert_not_called()


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


def _job(status: str, **overrides) -> dict:
    return {"id": "job-1", "status": status, "duration": 1.25, "error": None,
            "code": None, **overrides}


class TestExecuteSpeak:
    """The speech job is driven from here so a cancel can stop it mid-sentence.

    ``speak_outcome`` is a staticmethod the activity calls on the class, so the
    real translation runs even though the gateway instance is a MagicMock.
    """

    def test_queues_then_polls_with_a_heartbeat_per_read(self, env, activities, tts_gw):
        tts_gw.start_speak.return_value = (True, "", _job("queued"))

        def _await(job, on_poll):
            on_poll("queued")
            on_poll("playing")
            return True, "", _job("done")

        tts_gw.await_speak.side_effect = _await
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        result = env.run(activities.execute_speak, SpeakParams(text="hello"))

        assert result.success is True
        assert tts_gw.start_speak.call_args[1]["text"] == "hello"
        assert beats == [("queued",), ("playing",)]
        tts_gw.cancel_speak.assert_not_called()

    def test_a_cancel_while_playing_stops_the_utterance(self, env, activities, tts_gw):
        tts_gw.start_speak.return_value = (True, "", _job("playing"))
        tts_gw.await_speak.side_effect = CancelledError()
        tts_gw.cancel_speak.return_value = (True, "")

        with pytest.raises(CancelledError):
            env.run(activities.execute_speak, SpeakParams(text="hello"))

        tts_gw.cancel_speak.assert_called_once_with("job-1")

    def test_a_failed_stop_is_logged_and_the_cancel_still_lands(
        self, env, activities, tts_gw
    ):
        tts_gw.start_speak.return_value = (True, "", _job("playing"))
        tts_gw.await_speak.side_effect = CancelledError()
        tts_gw.cancel_speak.return_value = (False, "could not reach the speech service")

        with pytest.raises(CancelledError):
            env.run(activities.execute_speak, SpeakParams(text="hello"))

    def test_a_cancel_before_the_job_exists_has_nothing_to_stop(
        self, env, activities, tts_gw
    ):
        tts_gw.start_speak.side_effect = CancelledError()

        with pytest.raises(CancelledError):
            env.run(activities.execute_speak, SpeakParams(text="hello"))

        tts_gw.await_speak.assert_not_called()
        tts_gw.cancel_speak.assert_not_called()

    def test_an_unknown_voice_is_non_retryable(self, env, activities, tts_gw):
        tts_gw.start_speak.return_value = (
            False, fail(Failure.UNKNOWN_VOICE, "unknown voice: 'xx'"), None
        )

        with pytest.raises(ApplicationError, match="Speak failed") as exc_info:
            env.run(activities.execute_speak, SpeakParams(text="hello", voice="xx"))

        assert exc_info.value.non_retryable is True
        tts_gw.await_speak.assert_not_called()

    def test_a_failed_job_is_retryable_with_the_services_reason(
        self, env, activities, tts_gw
    ):
        tts_gw.start_speak.return_value = (True, "", _job("queued"))
        tts_gw.await_speak.return_value = (
            True, "", _job("failed", error="aplay failed: audio open error")
        )

        with pytest.raises(ApplicationError, match="audio open error") as exc_info:
            env.run(activities.execute_speak, SpeakParams(text="hello"))

        assert exc_info.value.non_retryable is False

    def test_the_heartbeat_window_covers_the_post(self):
        """The POST that queues the job is the one stretch with no heartbeat."""
        from syncai_backend.gateways.tts.tts import SPEAK_START_TIMEOUT_S
        from syncai_backend.temporal.workflows import SPEAK_HEARTBEAT_TIMEOUT

        assert SPEAK_HEARTBEAT_TIMEOUT.total_seconds() > SPEAK_START_TIMEOUT_S

    def test_a_cancel_reaches_the_poll_loop_quickly(self):
        """Same relation as MOVE's: a heartbeat waiting whenever one may go out."""
        from syncai_backend.gateways.tts.tts import SPEAK_POLL_INTERVAL_S
        from syncai_backend.temporal.worker import HEARTBEAT_THROTTLE_MAX
        from syncai_backend.temporal.workflows import SPEAK_HEARTBEAT_TIMEOUT

        throttle = HEARTBEAT_THROTTLE_MAX.total_seconds()
        assert throttle < 0.8 * SPEAK_HEARTBEAT_TIMEOUT.total_seconds()
        assert SPEAK_POLL_INTERVAL_S <= throttle
