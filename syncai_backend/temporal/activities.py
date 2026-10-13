import asyncio
import math
import time
from typing import Optional

import structlog
from pydantic import BaseModel

from temporalio import activity
from temporalio.client import Client
from temporalio.exceptions import ApplicationError, CancelledError


from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.gateways.workflow.config import STEP_STARTED_SIGNAL, reports_start
from syncai_backend.gateways.workflow.schema import MoveParams, SpeakParams
from syncai_backend.gateways.robot.robot import MotionKey, RobotGateway
from syncai_backend.gateways.tts.tts import TtsGateway

from syncai_backend.helpers.move_guard import move_refusal
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.services.gridmap_conversion import GridmapConversionService


# How often _wait_for_nav_goal reads the goal and heartbeats. A cancel arrives on a
# heartbeat reply (at most every HEARTBEAT_THROTTLE_MAX, worker.py) and lands in this
# thread only after the current sleep, so keep this at or under the throttle
# (test_activities.py pins it). get_move_status is an in-memory read.
NAV_POLL_INTERVAL_S = 0.25

# Bounds _report_started against a wedged frontend; must fit inside
# MOVE_HEARTBEAT_TIMEOUT beside the heartbeat sent just before it (test_activities.py).
STARTED_REPORT_TIMEOUT_S = 1.0


def _move_failure(status: dict) -> str:
    """The step's error_msg for a MOVE that did not succeed.

    The robot's own sentence when it gave one (why the backend uses NavigateToGoal,
    not NavigateToPose). An abort with error_code NONE is a goal that was superseded
    or never started, not a navigation failure, and is said so.
    """
    state = status["state"]
    if state != "aborted":
        return f"move ended in {state}"
    if status.get("error_msg"):
        return f"move ended in aborted: {status['error_msg']}"
    if status.get("error_code") == 0:
        return (
            "move ended in aborted: navigation did not start "
            "(another navigation is running, or the behavior tree failed to load)"
        )
    return "move ended in aborted"


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
        conversion_svc: GridmapConversionService,
    ):
        self._logger = logger
        self._robot_gw = robot_gw
        self._tts_gw = tts_gw
        # Read-only, for execute_move's check; the same conversion registry the map
        # routes write (shared process).
        self._map_catalog_repo = map_catalog_repo
        self._conversion_svc = conversion_svc
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

        ``expected_map`` is the run's ``TaskMap``. The goal is first checked against
        the loaded map (helpers/move_guard.py, as POST /api/v1/tasks does): another
        map's coordinates, or a floor plan being rebuilt. Both refusals are
        non-retryable; this is the check that catches a schedule firing after a map
        switch.

        Cancellation is *thrown* into this thread as CancelledError (in the poll
        loop or inside move()), so cleanup lives in the except clause. A blocking
        call like time.sleep is not interrupted, hence the short NAV_POLL_INTERVAL_S.
        """
        active_map = self._map_catalog_repo.active_name()
        refusal = move_refusal(
            expected_map,
            active_map,
            active_map is not None and self._conversion_svc.is_converting(active_map),
        )
        if refusal is not None:
            raise ApplicationError(
                refusal.message, type=refusal.code, non_retryable=True
            )

        yaw = math.radians(params.theta)

        # The heartbeat_timeout clock starts with the activity, and move() can block
        # up to NAV_GOAL_SEND_BUDGET_S; beat first so a slow nav2 does not fail it.
        activity.heartbeat("sending")

        try:
            accepted, msg, goal_id = self._robot_gw.move(x=params.x, y=params.y, yaw=yaw)
            if not accepted:
                raise ApplicationError(f"Move rejected: {msg}", non_retryable=False)

            self._logger.info("[RobotActivity] Move accepted", goal_id=goal_id)
            # Beat first: move() may have used most of the heartbeat window.
            self._report_started(beat=("accepted",))

            status = self._wait_for_nav_goal(goal_id=goal_id)
            state = status["state"]

        except CancelledError:
            # Cancel whatever is executing, not a remembered goal_id: a cancel inside
            # move() has no id yet. Shielded so the cancel RPC finishes first.
            with activity.shield_thread_cancel_exception():
                self._robot_gw.cancel_active_moves()

            self._logger.warning("[RobotActivity] Move activity has been cancelled")
            raise

        if state != "succeeded":
            raise ApplicationError(_move_failure(status), non_retryable=False)

        return ActivityResult(success=True, goal_id=goal_id, state=state)

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
        as none said).

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
