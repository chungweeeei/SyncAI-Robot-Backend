import asyncio
import math
import os
import time
from typing import Optional

import httpx
import structlog
from pydantic import BaseModel

from temporalio import activity
from temporalio.client import Client
from temporalio.exceptions import ApplicationError, CancelledError


from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.tasks.config import STEP_STARTED_SIGNAL, reports_start
from syncai_backend.tasks.schema import (
    INSPECTION_DESCRIPTIONS,
    DemoParams,
    MoveParams,
    SpeakParams,
)
from syncai_backend.gateways.robot.robot import (
    DOCK_BEHAVIOR_TREE,
    UNDOCK_BEHAVIOR_TREE,
    MotionKey,
    RobotGateway,
)
from syncai_backend.gateways.tts.tts import TtsGateway

from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.services.dock_state import DockStateService
from syncai_backend.services.map_usage import MapUsagePolicy


# How often _wait_for_nav_goal reads the goal and heartbeats. A cancel arrives on a
# heartbeat reply (at most every HEARTBEAT_THROTTLE_MAX, worker.py) and lands in this
# thread only after the current sleep, so keep this at or under the throttle
# (test_activities.py pins it). get_move_status is an in-memory read.
NAV_POLL_INTERVAL_S = 0.25

# Bounds _report_started against a wedged frontend; must fit inside
# MOVE_HEARTBEAT_TIMEOUT beside the heartbeat sent just before it (test_activities.py).
STARTED_REPORT_TIMEOUT_S = 1.0

# RobotState.low_level_mode.motion_state values the auto-undock reads (the
# console's table is routers/robot.py's _MOTION_STATE_TO_STR).
MOTION_STATE_STAND = 0
MOTION_STATE_LIE_DOWN = 2

# How long a MOVE's auto-undock waits for a robot lying on the dock to report
# STAND after the STANDUP key, before the attempt fails (retryably). Unmeasured:
# a stand-up has been seen to take a few seconds, and robot_state publishes at
# 1 Hz, so this leaves room for both with margin.
STAND_UP_TIMEOUT_S = 15.0


def _move_failure(status: dict, label: str = "move") -> str:
    """The step's error_msg for a MOVE / DOCK / UNDOCK that did not succeed.

    The robot's own sentence when it gave one (why the backend uses NavigateToGoal,
    not NavigateToPose; a dock tree's DOCK_FAILED names the node that failed). An
    abort with error_code NONE is a goal that was superseded or never started, not a
    navigation failure, and is said so.
    """
    state = status["state"]
    if state != "aborted":
        return f"{label} ended in {state}"
    if status.get("error_msg"):
        return f"{label} ended in aborted: {status['error_msg']}"
    if status.get("error_code") == 0:
        return (
            f"{label} ended in aborted: navigation did not start "
            "(another navigation is running, or the behavior tree failed to load)"
        )
    return f"{label} ended in aborted"


# DEMO (demo branch only): where execute_demo reports an inspection. Host networking,
# so 127.0.0.1 is the robot's own loopback.
DEMO_CAPTURE_URL = os.environ.get(
    "DEMO_CAPTURE_URL", "http://127.0.0.1:8090/internal/capture/inspection"
)
# The service answers 408 SYSTEM_REQUEST_TIMEOUT itself at 30 s; wait past that so
# its own answer, not ours, is what the step shows.
DEMO_CAPTURE_TIMEOUT_S = 35.0

# The capture service's error.code values, in the words a step's error_msg shows.
DEMO_CAPTURE_ERRORS = {
    "CAMERA_UNAVAILABLE": "Camera unavailable",
    "MEDIA_UNAVAILABLE": "Media store unavailable",
    "SYSTEM_REQUEST_TIMEOUT": "Capture timed out",
    "SYSTEM_VALIDATION_FAILED": "Capture request rejected",
    "RESOURCE_NOT_FOUND": "Capture endpoint not found",
    "SYSTEM_INTERNAL_ERROR": "Capture service error",
}


def _capture_failure(response: httpx.Response) -> ApplicationError:
    """A non-2xx capture answer as a non-retryable failure named by its error.code.

    The body is ``{"error": {"code", "message", "details"?}}``; anything else (a
    proxy's page, no body) still fails, under ``CAPTURE_HTTP_<status>``.
    """
    code = message = None
    try:
        error = response.json().get("error")
    except (ValueError, AttributeError):
        error = None
    if isinstance(error, dict):
        code = error.get("code") if isinstance(error.get("code"), str) else None
        message = error.get("message") if isinstance(error.get("message"), str) else None
        if error.get("details"):
            message = f"{message or ''} {error['details']}".strip()
    if message is None:
        message = response.text[:200]

    # ApplicationError's str() is "<type>: <message>", which is the step's error_msg,
    # so the code is said once, as the type.
    label = DEMO_CAPTURE_ERRORS.get(code, "Capture failed")
    text = f"{label} ({response.status_code})"
    if message:
        text = f"{text}: {message}"
    return ApplicationError(
        text,
        type=code or f"CAPTURE_HTTP_{response.status_code}",
        non_retryable=True,
    )


class ActivityResult(BaseModel):
    success: bool
    goal_id: str | None = None
    state: str | None = None


class RobotActivities:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_gw: RobotGateway,
        tts_gw: TtsGateway,
        map_catalog_repo: MapCatalogRepo,
        map_usage: MapUsagePolicy,
        robot_repo: RobotRepo,
        dock_state: DockStateService,
    ):
        self._logger = logger
        self._robot_gw = robot_gw
        self._tts_gw = tts_gw
        # Read-only: the loaded map, for the auto-undock.
        self._map_catalog_repo = map_catalog_repo
        # execute_move's check, the same rule POST /api/v1/tasks applies.
        self._map_usage = map_usage
        # The latest RobotState (pose, motion state), for the auto-undock's checks.
        self._robot_repo = robot_repo
        # Where the last DOCK ended; see services/dock_state.py. One per process,
        # built in main.py like every other collaborator.
        self._dock_state = dock_state
        # Bound by run_worker, for _report_started: a sync activity cannot use the
        # async-only activity.client().
        self._client: Optional[Client] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def bind_client(self, client: Client, loop: asyncio.AbstractEventLoop) -> None:
        self._client = client
        self._loop = loop

    def _report_started(self, beat: tuple = ()) -> None:
        """Tell the run this group member has started (STEP_STARTED_SIGNAL).

        Only ids from group_activity_id report. Sent on the worker's loop from this
        thread. Never fatal: a lost report makes the next member wait for this one
        to end, keeping the order and losing only the overlap. A cancel is let
        through for the caller to clean up.

        ``beat``, when given, is heartbeated first, for a caller whose heartbeat
        window the wait could otherwise run out.
        """
        info = activity.info()
        if not reports_start(info.activity_id) or self._client is None:
            return
        if beat:
            activity.heartbeat(*beat)
        handle = self._client.get_workflow_handle(
            info.workflow_id, run_id=info.workflow_run_id
        )
        try:
            asyncio.run_coroutine_threadsafe(
                handle.signal(STEP_STARTED_SIGNAL, info.activity_id), self._loop
            ).result(timeout=STARTED_REPORT_TIMEOUT_S)
        except CancelledError:
            raise
        except Exception as err:
            self._logger.warning(
                "[RobotActivity] Could not report the step started",
                activity_id=info.activity_id,
                error=str(err),
            )

    def _wait_for_nav_goal(self, goal_id: str) -> dict:
        """Poll a navigation goal to a terminal state, heartbeating each round.

        Returns the gateway's last status dict: ``state``, plus the robot's
        ``error_code`` / ``error_msg`` when the goal was aborted.
        """
        while True:
            status = self._robot_gw.get_move_status(goal_id=goal_id)
            state = status["state"] if status else None

            activity.heartbeat(state)

            if state in ["succeeded", "aborted", "canceled"]:
                return status

            time.sleep(NAV_POLL_INTERVAL_S)

    @activity.defn
    def execute_move(
        self, params: MoveParams, expected_map: Optional[str] = None
    ) -> ActivityResult:
        """Send a NavigateToGoal goal and supervise it to a terminal state.

        A robot still standing (or lying) on the dock the last DOCK ended on is
        taken off it first (_undock_if_docked): a MOVE straight off a dock starts
        with the footprint on its lethal cells and fails. The map check runs
        before that, so a MOVE refused for its map never backs the robot out.
        """
        self._refuse_wrong_map(expected_map)
        self._undock_if_docked()
        return self._navigate(params, expected_map, behavior_tree="", label="move")

    @activity.defn
    def execute_dock(
        self, params: MoveParams, expected_map: Optional[str] = None
    ) -> ActivityResult:
        """Dock at the pose in ``params``: a NavigateToGoal on dock.xml.

        The tree drives to the staging pose, switches to MPC and approaches with
        DockToPose; it ends standing on the dock in MPC (the task runner stopped
        lying it down on 2026-10-15 -- a LIEDOWN step after the DOCK does that).
        Success records the dock for the next MOVE's auto-undock, under the
        run's map, or the loaded one for a run that carries none.
        """
        result = self._navigate(
            params, expected_map, behavior_tree=DOCK_BEHAVIOR_TREE, label="dock"
        )
        self._dock_state.record(
            expected_map if expected_map is not None
            else self._map_catalog_repo.active_name(),
            params.x,
            params.y,
            params.theta,
        )
        return result

    @activity.defn
    def execute_undock(self) -> ActivityResult:
        """Back off the dock: a NavigateToGoal on undock.xml (MPC, back out 1 m
        from where the robot stands, RL). The tree reads no pose, so the goal's
        is left empty and there is no map to check it against.

        Kept for the API and for stored templates / schedules: a MOVE after a
        DOCK undocks by itself now, so the console no longer offers the step.
        Success forgets the dock, so the next MOVE does not back out again.
        """
        result = self._navigate(
            None, None, behavior_tree=UNDOCK_BEHAVIOR_TREE, label="undock"
        )
        self._dock_state.clear()
        return result

    def _undock_if_docked(self) -> None:
        """Get the robot off the recorded dock, if it is still on it.

        Lying (a LIEDOWN after the DOCK): STANDUP first, and wait until the
        controller reports STAND, because undock.xml starts from standing and
        what its MPC switch does from lying is unmeasured. Then undock.xml, then
        forget the dock.

        Runs inside the MOVE's attempt, so the MOVE's retry, pause and cancel
        cover it: a cancel or pause during the back-out lands in _navigate's
        cleanup (goal cancelled, RL restored), and the next attempt (or the
        resumed step) asks again. A robot stopped within the on-dock tolerance
        backs out again from there; one stopped outside it is not on the dock
        any more and just drives. In a group, the undock goal's acceptance is
        what reports the MOVE started, so a grouped SPEAK starts with the
        back-out rather than after it.
        """
        state = self._robot_repo.get_robot_state()
        if not self._dock_state.is_on_dock(state, self._map_catalog_repo.active_name()):
            return

        self._logger.info("[RobotActivity] Robot is on the dock; undocking before the move")
        if state.low_level_mode.motion_state == MOTION_STATE_LIE_DOWN:
            self._set_motion_key(key=MotionKey.STAND, label="Stand")
            self._wait_for_motion_state(MOTION_STATE_STAND, STAND_UP_TIMEOUT_S, "standing")

        self._navigate(None, None, behavior_tree=UNDOCK_BEHAVIOR_TREE, label="undock")
        self._dock_state.clear()

    def _wait_for_motion_state(self, motion_state: int, timeout_s: float, label: str) -> None:
        """Poll RobotState until the controller reports ``motion_state``,
        heartbeating each round. Retryable on timeout: the next attempt sends
        the key again from whatever the robot is doing by then."""
        deadline = time.monotonic() + timeout_s
        while True:
            state = self._robot_repo.get_robot_state()
            reported = None if state is None else state.low_level_mode.motion_state
            activity.heartbeat(label, reported)
            if reported == motion_state:
                return
            if time.monotonic() >= deadline:
                raise ApplicationError(
                    f"Auto-undock: the robot did not report {label} within "
                    f"{timeout_s:.0f} s (motion state {reported})",
                    non_retryable=False,
                )
            time.sleep(NAV_POLL_INTERVAL_S)

    def _refuse_wrong_map(self, expected_map: Optional[str]) -> None:
        """Raise move_guard's refusal (non-retryable) for a goal planned on
        ``expected_map`` when another map is loaded."""
        refusal = self._map_usage.move_refusal(
            expected_map, self._map_catalog_repo.active_name()
        )
        if refusal is not None:
            raise ApplicationError(refusal.message, type=refusal.code, non_retryable=True)

    def _navigate(
        self,
        params: Optional[MoveParams],
        expected_map: Optional[str],
        behavior_tree: str,
        label: str,
    ) -> ActivityResult:
        """Send a NavigateToGoal goal on ``behavior_tree`` and supervise it.

        ``expected_map`` is the run's ``TaskMap``. A goal with a pose is first
        checked against the loaded map (helpers/move_guard.py, as POST
        /api/v1/tasks does): another map's coordinates are refused,
        non-retryable; this is the check that catches a
        schedule firing after a map switch. ``params`` None (an UNDOCK) skips it and
        sends an empty pose: the tree works from the robot's own frame.

        Cancellation is *thrown* into this thread as CancelledError (in the poll
        loop or inside move()), so cleanup lives in the except clause. A blocking
        call like time.sleep is not interrupted, hence the short NAV_POLL_INTERVAL_S.
        """
        if params is not None:
            self._refuse_wrong_map(expected_map)
            move_kwargs = {
                "x": params.x,
                "y": params.y,
                "z": params.z,
                "yaw": math.radians(params.theta),
            }
        else:
            move_kwargs = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        if behavior_tree:
            move_kwargs["behavior_tree"] = behavior_tree

        # The heartbeat_timeout clock starts with the activity, and move() can block
        # up to NAV_GOAL_SEND_BUDGET_S; beat first so a slow nav2 does not fail it.
        activity.heartbeat("sending")

        try:
            accepted, msg, goal_id = self._robot_gw.move(**move_kwargs)
            if not accepted:
                raise ApplicationError(
                    f"{label.capitalize()} rejected: {msg}", non_retryable=False
                )

            self._logger.info(
                f"[RobotActivity] {label.capitalize()} accepted", goal_id=goal_id
            )
            # Beat first: move() may have used most of the heartbeat window.
            self._report_started(beat=("accepted",))

            status = self._wait_for_nav_goal(goal_id=goal_id)
            state = status["state"]

        except CancelledError:
            # Cancel whatever is executing, not a remembered goal_id: a cancel inside
            # move() has no id yet. Shielded so the cancel RPC finishes first.
            with activity.shield_thread_cancel_exception():
                self._robot_gw.cancel_active_moves()
                if behavior_tree:
                    self._restore_locomotion(label)

            self._logger.warning(
                f"[RobotActivity] {label.capitalize()} activity has been cancelled"
            )
            raise

        if state != "succeeded":
            raise ApplicationError(_move_failure(status, label), non_retryable=False)

        return ActivityResult(success=True, goal_id=goal_id, state=state)

    def _restore_locomotion(self, label: str) -> None:
        """Put the walking gait back after a dock tree was cut short.

        A halted dock.xml / undock.xml does not switch back from MPC itself (its
        fallback only runs on a failure), and the next MOVE needs RL. Best effort:
        a refusal (safety lock) is logged, never raised over the cancel.
        """
        try:
            accepted, msg = self._robot_gw.set_motion_key(key=MotionKey.LOCOMOTION)
        except Exception as err:
            accepted, msg = False, str(err)
        if not accepted:
            self._logger.warning(
                f"[RobotActivity] Could not restore locomotion after {label}",
                error=msg,
            )

    def _set_motion_key(self, key: MotionKey, label: str) -> ActivityResult:
        """Send a motion key. Fire-and-forget: this does NOT wait for the pose.

        MODE is a one-way UDP command, so the step completes while the legs are
        still moving and a following MOVE starts against a robot not yet standing.
        Deliberately no fixed sleep: the fix is to poll the `mode` topic's motion
        state (data[1]), pending its value mapping from the gait controller side.

        A rejected key (unknown, or safety lock engaged) is non_retryable.
        """
        accepted, msg = self._robot_gw.set_motion_key(key=key)
        if not accepted:
            raise ApplicationError(f"{label} rejected: {msg}", non_retryable=True)

        self._logger.info(f"[RobotActivity] {label} command sent", key=key.value)

        return ActivityResult(success=True, state="succeeded")

    @activity.defn
    def execute_stand(self) -> ActivityResult:
        return self._set_motion_key(key=MotionKey.STAND, label="Stand")

    @activity.defn
    def execute_lie_down(self) -> ActivityResult:
        return self._set_motion_key(key=MotionKey.LIE_DOWN, label="LieDown")

    @activity.defn
    def execute_speak(self, params: SpeakParams) -> ActivityResult:
        """Speak on the robot speaker, heartbeating until playback finishes.

        Playback is a job (POST queues, GET polls, DELETE stops): this queues, then
        polls and heartbeats like MOVE does, so a run cancel (task cancel, or a
        failed group sibling) lands in ``except CancelledError`` and stops it
        mid-sentence. A pause lets it finish (RobotWorkflow._run_step).

        The POST waits for synthesis with no heartbeat, so SPEAK_HEARTBEAT_TIMEOUT
        outlasts the gateway's timeout on it (pinned in test_activities.py).

        ``params.repeat`` utterances play back to back; the count said rides in the
        heartbeat, so a retry says only the rest (an old heartbeat without it reads
        as none said). The speech service caches rendered audio, so only the first
        one synthesises.

        Only Failure.UNKNOWN_VOICE is non-retryable; everything else, including a
        full speech queue, may be transient and is bounded by maximum_attempts=3.
        """
        details = activity.info().heartbeat_details
        start = details[1] if len(details) > 1 and isinstance(details[1], int) else 0
        duration = 0.0
        for said in range(start, params.repeat):
            duration += self._speak_once(params, said, first=said == start)
            # Counted when it ends, or a retry after a failed POST says it again.
            activity.heartbeat("done", said + 1)

        self._logger.info(
            "[RobotActivity] Speak finished", duration=duration, repeat=params.repeat
        )

        return ActivityResult(success=True, state="succeeded")

    def _speak_once(self, params: SpeakParams, said: int, first: bool = False) -> float:
        """One utterance of ``params``, heartbeating ``said`` beside its status.

        The ``first`` of an attempt reports the step started once queued: the
        service plays in acceptance order, so a grouped SPEAK queues behind it.

        Returns how long it played; raises as execute_speak documents.
        """
        job_id = None
        try:
            # Shielded so a cancel during the POST is raised only once the job id is
            # known: the utterance is queued as soon as the service answers, and the
            # except below needs the id to stop it.
            with activity.shield_thread_cancel_exception():
                success, message, job = self._tts_gw.start_speak(
                    text=params.text, voice=params.voice, speed=params.speed
                )
                if success:
                    job_id = job.get("id")

            if success and first:
                self._report_started()

            if success:
                success, message, job = self._tts_gw.await_speak(
                    job, on_poll=lambda status: activity.heartbeat(status, said)
                )

        except CancelledError:
            if job_id is not None:
                # Shielded: the DELETE has to finish before the cancel does.
                with activity.shield_thread_cancel_exception():
                    stopped, why = self._tts_gw.cancel_speak(job_id)
                if not stopped:
                    self._logger.warning(
                        "[RobotActivity] Could not stop the utterance",
                        job_id=job_id,
                        error=why,
                    )

            self._logger.warning("[RobotActivity] Speak activity has been cancelled")
            raise

        if success:
            success, message, duration = TtsGateway.speak_outcome(job)

        if not success:
            raise ApplicationError(
                f"Speak failed: {message}",
                # The code, not the prose: the tts router decides 400 vs 502 from it.
                non_retryable=failure_code(message) is Failure.UNKNOWN_VOICE,
            )

        return duration

    @activity.defn
    def execute_demo(self, params: DemoParams) -> ActivityResult:
        """Report an inspection result to the capture service (demo branch only).

        One POST, no heartbeat. Every failure is non-retryable: the step fails at
        once with the service's error.code in its error_msg, and a retry could
        capture twice when only the answer was lost.
        """
        description = INSPECTION_DESCRIPTIONS[params.inspection]
        try:
            response = httpx.post(
                DEMO_CAPTURE_URL,
                json={"description": description},
                timeout=DEMO_CAPTURE_TIMEOUT_S,
            )
        except httpx.TimeoutException as err:
            raise ApplicationError(
                f"Capture service did not answer within {DEMO_CAPTURE_TIMEOUT_S:g} s",
                type="CAPTURE_UNREACHABLE",
                non_retryable=True,
            ) from err
        except httpx.HTTPError as err:
            raise ApplicationError(
                f"Capture service unreachable: {err}",
                type="CAPTURE_UNREACHABLE",
                non_retryable=True,
            ) from err

        if not response.is_success:
            raise _capture_failure(response)

        self._logger.info(
            "[RobotActivity] Inspection reported",
            inspection=params.inspection.value,
            description=description,
        )
        return ActivityResult(success=True, state="succeeded")
