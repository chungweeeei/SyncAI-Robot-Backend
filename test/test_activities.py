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

import asyncio
import dataclasses
import math
import threading
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

pytest.importorskip("temporalio")

from temporalio.exceptions import ApplicationError, CancelledError  # noqa: E402
from temporalio.testing import ActivityEnvironment  # noqa: E402

from syncai_backend.gateways.robot.robot import MotionKey  # noqa: E402
from syncai_backend.gateways.failure import Failure, fail  # noqa: E402
from syncai_backend.tasks.schema import (  # noqa: E402
    MoveParams,
    PatrolParams,
    SpeakParams,
)
from syncai_backend.services.map_usage import MapUsagePolicy  # noqa: E402
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
def activities(logger, robot_gw, tts_gw, map_catalog_repo) -> RobotActivities:
    return RobotActivities(
        logger=logger,
        robot_gw=robot_gw,
        tts_gw=tts_gw,
        map_catalog_repo=map_catalog_repo,
        # Real, over the mocks: execute_move's check is the policy's rule.
        map_usage=MapUsagePolicy(
            map_catalog_repo=map_catalog_repo,
            task_template_repo=None,
        ),
        task_record_repo=MagicMock(),
        task_template_repo=MagicMock(),
        map_repo=MagicMock(),
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
        assert kwargs["z"] == 0.0

    def test_the_goal_keeps_its_storey(self, env, activities, robot_gw):
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {"goal_id": "goal-1", "state": "succeeded"}

        env.run(activities.execute_move, MoveParams(x=1.0, y=2.0, theta=0.0, z=5.02))

        assert robot_gw.move.call_args[1]["z"] == 5.02

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

    @pytest.mark.parametrize(
        "code", [201, 203, 204, 209, 220, 222, 223, 224], ids=lambda c: f"code_{c}"
    )
    def test_a_permanent_planning_failure_is_not_retried(
        self, env, activities, robot_gw, code
    ):
        # The same goal cannot succeed 5 s later (a goal inside a keepout zone,
        # 223, was the case: three tries, 10 s, for a failure known at once).
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {
            "goal_id": "goal-1",
            "state": "aborted",
            "error_code": code,
            "error_msg": "Planning failed: the goal is inside a keepout zone (...)",
        }

        with pytest.raises(ApplicationError, match="Planning failed") as exc_info:
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        assert exc_info.value.non_retryable is True

    @pytest.mark.parametrize(
        "code", [0, 1, 100, 200, 202, 205, 206, 207, 208, 221, 300],
        ids=lambda c: f"code_{c}",
    )
    def test_a_failure_that_may_clear_is_retried(self, env, activities, robot_gw, code):
        # A person on the goal (206) or across the way (208), a map still
        # loading (221), no finer reason (200), a controller failure,
        # an abort before the tree ran (0).
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.return_value = {
            "goal_id": "goal-1", "state": "aborted", "error_code": code, "error_msg": "x",
        }

        with pytest.raises(ApplicationError) as exc_info:
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        assert exc_info.value.non_retryable is False

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

    def test_a_cancelled_move_leaves_the_gait_alone(self, env, activities, robot_gw):
        # move.xml never leaves RL, so a cancel puts no gait back.
        robot_gw.move.return_value = (True, "", "goal-1")
        robot_gw.get_move_status.side_effect = CancelledError()

        with pytest.raises(CancelledError):
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        robot_gw.set_motion_key.assert_not_called()
        assert "behavior_tree" not in robot_gw.move.call_args[1]


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
        # The count said so far rides beside the status (see the retry test).
        assert beats == [("queued", 0), ("playing", 0), ("done", 1)]
        tts_gw.cancel_speak.assert_not_called()

    def test_says_the_text_as_many_times_as_asked(self, env, activities, tts_gw):
        tts_gw.start_speak.return_value = (True, "", _job("queued"))
        tts_gw.await_speak.return_value = (True, "", _job("done"))
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        result = env.run(activities.execute_speak, SpeakParams(text="hello", repeat=3))

        assert result.success is True
        assert tts_gw.start_speak.call_count == 3
        assert [said for _, said in beats] == [1, 2, 3]

    def test_a_retry_says_only_what_is_left(self, env, activities, tts_gw):
        # The third of five failed on the last attempt, after two were said:
        # the heartbeat that attempt left is what this one starts from.
        env.info = dataclasses.replace(env.info, heartbeat_details=["playing", 2])
        tts_gw.start_speak.return_value = (True, "", _job("queued"))
        tts_gw.await_speak.return_value = (True, "", _job("done"))

        env.run(activities.execute_speak, SpeakParams(text="hello", repeat=5))

        assert tts_gw.start_speak.call_count == 3

    def test_a_heartbeat_from_before_repeat_reads_as_none_said(
        self, env, activities, tts_gw
    ):
        env.info = dataclasses.replace(env.info, heartbeat_details=["playing"])
        tts_gw.start_speak.return_value = (True, "", _job("queued"))
        tts_gw.await_speak.return_value = (True, "", _job("done"))

        env.run(activities.execute_speak, SpeakParams(text="hello", repeat=2))

        assert tts_gw.start_speak.call_count == 2

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


@pytest.fixture
def worker_loop():
    """The worker's event loop, running in a thread as run_worker's does."""
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    yield loop
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=5)
    loop.close()


@pytest.fixture
def client(activities, worker_loop):
    client = MagicMock()
    client.get_workflow_handle.return_value.signal = AsyncMock()
    activities.bind_client(client, worker_loop)
    return client


def _as_group_member(env):
    # The id shape RobotWorkflow schedules a group member under.
    env.info = dataclasses.replace(env.info, activity_id="step-0#1")


class TestReportStarted:
    """A group member tells its run it has started, so the next one can go
    (RobotWorkflow._run_group); the workflow side is in test_workflows.py."""

    def test_a_moves_report_follows_the_accepted_goal(
        self, env, activities, robot_gw, client
    ):
        _as_group_member(env)
        order = []
        robot_gw.move.side_effect = lambda **kw: order.append("move") or (True, "", "g")
        robot_gw.get_move_status.return_value = {"goal_id": "g", "state": "succeeded"}
        client.get_workflow_handle.return_value.signal.side_effect = (
            lambda *args: order.append(("signal", *args))
        )
        env.on_heartbeat = lambda *details: order.append(("beat", *details))

        env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        from syncai_backend.tasks.config import STEP_STARTED_SIGNAL

        assert order[:4] == [
            ("beat", "sending"),
            "move",
            ("beat", "accepted"),
            ("signal", STEP_STARTED_SIGNAL, "step-0#1"),
        ]
        client.get_workflow_handle.assert_called_once_with(
            env.info.workflow_id, run_id=env.info.workflow_run_id
        )

    def test_a_lone_step_reports_nothing(self, env, activities, robot_gw, client):
        robot_gw.move.return_value = (True, "", "g")
        robot_gw.get_move_status.return_value = {"goal_id": "g", "state": "succeeded"}
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        client.get_workflow_handle.assert_not_called()
        assert ("accepted",) not in beats

    def test_a_rejected_move_reports_nothing(self, env, activities, robot_gw, client):
        _as_group_member(env)
        robot_gw.move.return_value = (False, "server not available", None)

        with pytest.raises(ApplicationError):
            env.run(activities.execute_move, MoveParams(x=0.0, y=0.0, theta=0.0))

        client.get_workflow_handle.assert_not_called()

    def test_a_speak_reports_once_queued_and_once_per_attempt(
        self, env, activities, tts_gw, client
    ):
        _as_group_member(env)
        order = []
        tts_gw.start_speak.side_effect = (
            lambda **kw: order.append("post") or (True, "", _job("queued"))
        )
        tts_gw.await_speak.return_value = (True, "", _job("done"))
        client.get_workflow_handle.return_value.signal.side_effect = (
            lambda *args: order.append("signal")
        )

        env.run(activities.execute_speak, SpeakParams(text="hello", repeat=3))

        assert order == ["post", "signal", "post", "post"]

    def test_a_report_that_cannot_be_sent_does_not_fail_the_step(
        self, env, activities, tts_gw, client
    ):
        _as_group_member(env)
        tts_gw.start_speak.return_value = (True, "", _job("queued"))
        tts_gw.await_speak.return_value = (True, "", _job("done"))
        client.get_workflow_handle.return_value.signal.side_effect = RuntimeError(
            "temporal unreachable"
        )

        result = env.run(activities.execute_speak, SpeakParams(text="hello"))

        assert result.success is True

    def test_an_unbound_worker_reports_nothing(self, env, activities, tts_gw):
        _as_group_member(env)
        tts_gw.start_speak.return_value = (True, "", _job("queued"))
        tts_gw.await_speak.return_value = (True, "", _job("done"))

        assert env.run(activities.execute_speak, SpeakParams(text="hello")).success

    def test_the_report_fits_inside_the_moves_heartbeat_window(self):
        """Beaten just before the report, next beaten after it and one poll."""
        from syncai_backend.temporal.activities import (
            NAV_POLL_INTERVAL_S,
            STARTED_REPORT_TIMEOUT_S,
        )
        from syncai_backend.temporal.workflows import MOVE_HEARTBEAT_TIMEOUT

        assert (
            STARTED_REPORT_TIMEOUT_S + NAV_POLL_INTERVAL_S
            < MOVE_HEARTBEAT_TIMEOUT.total_seconds()
        )


def _patrol_params(count=4):
    # Waypoint i at x = i, on alternate storeys, the last facing -90 deg.
    return PatrolParams(
        waypoints=[
            MoveParams(x=float(i), y=0.0, theta=-90.0 if i == count - 1 else 0.0,
                       z=3.1 * (i % 2))
            for i in range(count)
        ]
    )


def _patrol_status(state, remaining=None):
    status = {"goal_id": "p", "state": state}
    if remaining is not None:
        status["poses_remaining"] = remaining
    return status


class TestExecutePatrol:
    def test_sends_every_waypoint_with_its_height_and_the_last_heading(
        self, env, activities, robot_gw
    ):
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = _patrol_status("succeeded")

        result = env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        assert (result.success, result.goal_id) == (True, "p")
        waypoints = robot_gw.patrol.call_args.args[0]
        assert [(w["x"], w["z"]) for w in waypoints] == [
            (0.0, 0.0), (1.0, 3.1), (2.0, 0.0), (3.0, 3.1)
        ]
        # Degrees on the wire, radians at the gateway, for every waypoint.
        assert waypoints[-1]["yaw"] == pytest.approx(-math.pi / 2)
        assert robot_gw.patrol.call_args.kwargs["total"] == 4
        robot_gw.move.assert_not_called()

    def test_an_abort_carries_the_robots_reason(self, env, activities, robot_gw):
        # NavigateThroughGoals (2026-11): a patrol's failure reads as a MOVE's.
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = {
            **_patrol_status("aborted"),
            "error_code": 208,
            "error_msg": "Planning failed: the way to the goal is closed (...)",
        }

        with pytest.raises(ApplicationError) as raised:
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        assert str(raised.value) == (
            "patrol ended in aborted: Planning failed: the way to the goal is closed (...)"
        )
        # A person across the way may leave.
        assert raised.value.non_retryable is False

    def test_a_waypoint_in_a_keepout_zone_is_not_retried(self, env, activities, robot_gw):
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = {
            **_patrol_status("aborted"),
            "error_code": 223,
            "error_msg": "Planning failed: the goal is inside a keepout zone (...)",
        }

        with pytest.raises(ApplicationError, match="keepout zone") as raised:
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        assert raised.value.non_retryable is True

    def test_an_abort_with_no_code_says_navigation_did_not_start(
        self, env, activities, robot_gw
    ):
        # NavigateToGoal's contract, now a patrol's too: ABORTED + NONE is
        # superseded or never started.
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = {
            **_patrol_status("aborted"), "error_code": 0, "error_msg": ""
        }

        with pytest.raises(ApplicationError, match="navigation did not start") as raised:
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        assert raised.value.non_retryable is False

    def test_a_lost_patrol_keeps_this_sides_reason(self, env, activities, robot_gw):
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = {
            **_patrol_status("aborted"), "error_msg": "no answer to the cancel request"
        }

        with pytest.raises(ApplicationError, match="no answer to the cancel request"):
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

    def test_a_rejected_patrol_is_retryable(self, env, activities, robot_gw):
        robot_gw.patrol.return_value = (False, "Goal rejected by task runner node", None)

        with pytest.raises(ApplicationError, match="Patrol rejected") as raised:
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        assert raised.value.non_retryable is False

    def test_heartbeats_carry_the_waypoints_passed(self, env, activities, robot_gw):
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.side_effect = [
            _patrol_status("executing"),
            _patrol_status("executing", remaining=4),
            _patrol_status("executing", remaining=2),
            # The robot may say 0 at the end; the last waypoint is never "passed".
            _patrol_status("succeeded", remaining=0),
        ]
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        with patch(SLEEP):
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        assert beats == [
            ("sending", 0),
            ("executing", 0),
            ("executing", 0),
            ("executing", 2),
            ("succeeded", 3),
        ]

    def test_a_retry_resumes_from_its_heartbeat(self, env, activities, robot_gw):
        env.info = dataclasses.replace(env.info, heartbeat_details=["executing", 2])
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.side_effect = [
            _patrol_status("executing", remaining=2),
            _patrol_status("executing", remaining=1),
            _patrol_status("succeeded", remaining=1),
        ]
        beats = []
        env.on_heartbeat = lambda *details: beats.append(details)

        with patch(SLEEP):
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        waypoints = robot_gw.patrol.call_args.args[0]
        assert [w["x"] for w in waypoints] == [2.0, 3.0]
        # Still counted against the whole step, and from where it resumed.
        assert robot_gw.patrol.call_args.kwargs["total"] == 4
        assert beats[0] == ("sending", 2)
        assert beats[-1] == ("succeeded", 3)

    def test_a_redispatch_starts_past_what_the_run_was_told(
        self, env, activities, robot_gw
    ):
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = _patrol_status("succeeded")

        env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 1)

        assert [w["x"] for w in robot_gw.patrol.call_args.args[0]] == [1.0, 2.0, 3.0]

    def test_the_last_waypoint_is_always_sent(self, env, activities, robot_gw):
        # Its heading is the step's end, so even "all passed" drives to it.
        env.info = dataclasses.replace(env.info, heartbeat_details=["executing", 9])
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = _patrol_status("succeeded")

        env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 7)

        assert [w["x"] for w in robot_gw.patrol.call_args.args[0]] == [3.0]

    def test_an_old_heartbeat_without_a_count_starts_from_the_dispatch(
        self, env, activities, robot_gw
    ):
        env.info = dataclasses.replace(env.info, heartbeat_details=["executing"])
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.return_value = _patrol_status("succeeded")

        env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 1)

        assert len(robot_gw.patrol.call_args.args[0]) == 3

    def test_progress_is_reported_to_the_run_as_it_grows(
        self, env, activities, robot_gw, client
    ):
        from syncai_backend.tasks.config import PATROL_PROGRESS_SIGNAL

        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.side_effect = [
            _patrol_status("executing", remaining=4),
            _patrol_status("executing", remaining=3),
            _patrol_status("executing", remaining=3),
            _patrol_status("succeeded", remaining=1),
        ]
        signals = []
        client.get_workflow_handle.return_value.signal.side_effect = (
            lambda name, **kw: signals.append((name, kw["args"]))
        )

        with patch(SLEEP):
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        # Lone step or not: a pause interrupts both. Once per waypoint passed.
        assert signals == [
            (PATROL_PROGRESS_SIGNAL, ["s1", 1]),
            (PATROL_PROGRESS_SIGNAL, ["s1", 3]),
        ]

    def test_a_report_that_cannot_be_sent_does_not_fail_the_patrol(
        self, env, activities, robot_gw, client
    ):
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.side_effect = [
            _patrol_status("executing", remaining=2),
            _patrol_status("succeeded", remaining=1),
        ]
        client.get_workflow_handle.return_value.signal.side_effect = RuntimeError(
            "temporal unreachable"
        )

        with patch(SLEEP):
            result = env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        assert result.success is True

    def test_a_cancel_while_polling_cancels_the_goal(self, env, activities, robot_gw):
        robot_gw.patrol.return_value = (True, "", "p")
        robot_gw.get_move_status.side_effect = CancelledError()

        with pytest.raises(CancelledError):
            env.run(activities.execute_patrol, _patrol_params(), "lab", "s1", 0)

        robot_gw.cancel_active_moves.assert_called_once()
        # patrol.xml never leaves RL, so no gait to put back.
        robot_gw.set_motion_key.assert_not_called()

    def test_a_patrol_planned_on_another_map_is_refused(self, env, activities, robot_gw):
        with pytest.raises(ApplicationError) as raised:
            env.run(activities.execute_patrol, _patrol_params(), "warehouse", "s1", 0)

        assert raised.value.non_retryable is True
        robot_gw.patrol.assert_not_called()


class TestResolveTemplate:
    """A scheduled run's template, read as the run starts (the workflow's half is
    pinned in test_workflows.py)."""

    @pytest.fixture
    def resolver(self, logger, map_catalog_repo, task_template_repo, map_repo):
        return RobotActivities(
            logger=logger,
            robot_gw=MagicMock(),
            tts_gw=MagicMock(),
            map_catalog_repo=map_catalog_repo,
            map_usage=MapUsagePolicy(
                map_catalog_repo=map_catalog_repo,
                task_template_repo=None,
            ),
            task_record_repo=MagicMock(),
            task_template_repo=task_template_repo,
            map_repo=map_repo,
        )

    @staticmethod
    def _template(task_template_repo, map_repo, map_name="lab"):
        dock, hall = map_repo.create_vertices(
            map="lab",
            vertices=[
                {"name": "dock", "type": "MOVE", "x": 3.0, "y": -1.5, "theta": 90.0},
                {"name": "hall", "type": "MOVE", "x": 5.0, "y": 2.0, "theta": 0.0},
            ],
        )
        snapshot = {"x": 0.0, "y": 0.0, "theta": 0.0, "z": 0.0}
        row = task_template_repo.create_task_template(
            name="Morning round",
            description="",
            map_name=map_name,
            steps=[
                {"id": "1-move", "type": "MOVE", "params": snapshot,
                 "vertex_id": str(dock.id)},
                {"id": "2-patrol", "type": "PATROL",
                 "params": {"waypoints": [snapshot, snapshot]},
                 "vertex_ids": [str(hall.id), None]},
                {"id": "3-speak", "type": "SPEAK", "params": {"text": "Done"},
                 "group": "g"},
            ],
        )
        return row, dock, hall

    def test_a_template_is_resolved_against_its_vertices_as_they_are_now(
        self, resolver, task_template_repo, map_repo
    ):
        row, dock, _ = self._template(task_template_repo, map_repo)
        map_repo.update_vertex(vertex_id=dock.id, x=9.0)

        resolved = resolver.resolve_template(str(row.id))

        assert resolved.name == "Morning round"
        assert resolved.map_name == "lab"
        assert resolved.refusal is None
        move, patrol, speak = resolved.steps
        assert (move.params.x, move.params.y, move.params.theta) == (9.0, -1.5, 90.0)
        # A waypoint from a vertex goes where it is; a free point stays put.
        assert [(w.x, w.y) for w in patrol.params.waypoints] == [(5.0, 2.0), (0.0, 0.0)]
        assert (speak.params.text, speak.group) == ("Done", "g")

    def test_a_deleted_vertex_falls_back_to_the_snapshot(
        self, resolver, task_template_repo, map_repo
    ):
        row, dock, _ = self._template(task_template_repo, map_repo)
        map_repo.delete_vertex(vertex_id=dock.id)

        move = resolver.resolve_template(str(row.id)).steps[0]

        assert (move.params.x, move.params.y) == (0.0, 0.0)

    def test_another_map_loaded_is_answered_not_raised(
        self, resolver, task_template_repo, map_repo, map_catalog_repo
    ):
        row, _, _ = self._template(task_template_repo, map_repo)
        map_catalog_repo.active_name.return_value = "warehouse"

        resolved = resolver.resolve_template(str(row.id))

        assert resolved.name == "Morning round"
        assert "'lab'" in resolved.refusal and "'warehouse'" in resolved.refusal

    def test_a_template_that_is_gone_answers_none(self, resolver):
        assert resolver.resolve_template("0f2b8a34-6c11-4d0e-9f52-1a9b7c3d4e55") is None
