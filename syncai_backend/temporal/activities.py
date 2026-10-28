import asyncio
import math
import os
import time
import uuid
from typing import Optional

import httpx
import structlog
from pydantic import BaseModel

from temporalio import activity
from temporalio.client import Client
from temporalio.exceptions import ApplicationError, CancelledError


from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.tasks.config import (
    PATROL_PROGRESS_SIGNAL,
    STEP_STARTED_SIGNAL,
    reports_start,
)
from syncai_backend.tasks.schema import (
    INSPECTION_DESCRIPTIONS,
    DemoParams,
    MoveParams,
    PatrolParams,
    SpeakParams,
    TaskRecord,
)
from syncai_backend.tasks.template_resolution import (
    ResolvedTemplate,
    resolve_stored_steps,
    stored_vertex_ids,
)
from syncai_backend.gateways.robot.robot import (
    CHARGE_BEHAVIOR_TREE,
    MotionKey,
    RobotGateway,
)
from syncai_backend.gateways.tts.tts import TtsGateway

from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.task.task_record import TaskRecordRepo
from syncai_backend.repositories.task.task_template import TaskTemplateRepo
from syncai_backend.services.map_usage import MapUsagePolicy


# How often _wait_for_nav_goal reads the goal and heartbeats. A cancel arrives on a
# heartbeat reply (at most every HEARTBEAT_THROTTLE_MAX, worker.py) and lands in this
# thread only after the current sleep, so keep this at or under the throttle
# (test_activities.py pins it). get_move_status is an in-memory read.
NAV_POLL_INTERVAL_S = 0.25

# Bounds _report_started against a wedged frontend; must fit inside
# MOVE_HEARTBEAT_TIMEOUT beside the heartbeat sent just before it (test_activities.py).
STARTED_REPORT_TIMEOUT_S = 1.0


def _move_failure(status: dict, label: str = "move") -> str:
    """The step's error_msg for a MOVE / PATROL that did not succeed
    (NavigateThroughGoals carries NavigateToGoal's error fields).

    The robot's own sentence when it gave one (why the backend uses NavigateToGoal,
    not NavigateToPose). An
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


# NavigateToGoal / NavigateThroughGoals error_codes for which the same goal sent
# again cannot succeed, so the step fails at once instead of spending the retry
# policy's two retries, 5 s apart, on it. SyncAI-Robot-Interface's planning table
# (ComputePathToPose.action documents each code and whether a retry helps):
# INVALID_PLANNER 201, START_OUTSIDE_MAP 203, GOAL_OUTSIDE_MAP 204,
# NO_VIAPOINTS_GIVEN 209, MAP_NOT_LOADED 220 (until a map switch),
# START_IN_KEEPOUT 222 (until the robot is moved), GOAL_IN_KEEPOUT 223 (until the
# zone is edited), DISCONNECTED 224. Literal numbers, not the generated constants:
# the codes are the interface's contract and never renumbered, and this module
# imports without syncai_common under test. Everything else stays retryable: 200
# (no finer reason), a person on the goal (206) or across the way (208), a map
# still loading (221), a controller failure, an abort with no code.
NON_RETRYABLE_NAV_CODES = frozenset({201, 203, 204, 209, 220, 222, 223, 224})


def _nav_failure(status: dict, label: str) -> ApplicationError:
    """The error a MOVE / PATROL that did not succeed raises.

    Non-retryable when the robot's error_code says the same goal cannot succeed
    (NON_RETRYABLE_NAV_CODES). Before 2026-11 every abort was retried, and a
    goal inside a keepout zone failed three times, 10 s apart in all."""
    return ApplicationError(
        _move_failure(status, label),
        non_retryable=status.get("error_code") in NON_RETRYABLE_NAV_CODES,
    )


def _pose_kwargs(params: MoveParams) -> dict:
    """RobotGateway.move() / patrol()'s pose keywords for one pose step's
    params: theta (degrees) as yaw (radians)."""
    return {
        "x": params.x,
        "y": params.y,
        "z": params.z,
        "yaw": math.radians(params.theta),
    }


def _patrol_passed(details) -> int:
    """How many waypoints the previous attempt left behind it, off its heartbeat
    details (``(state, passed)``); 0 when it beat none or something older."""
    if len(details) > 1 and isinstance(details[1], int) and details[1] >= 0:
        return details[1]
    return 0


# DEMO (demo branch only): where execute_demo reports an inspection. Host networking,
# so 127.0.0.1 is the robot's own loopback.
DEMO_CAPTURE_URL = os.environ.get(
    "DEMO_CAPTURE_URL", "http://127.0.0.1:8090/internal/capture/inspection"
)
DEMO_CAPTURE_TIMEOUT_S = 10.0


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
        task_record_repo: TaskRecordRepo,
        task_template_repo: TaskTemplateRepo,
        map_repo: MapRepo,
    ):
        self._logger = logger
        self._robot_gw = robot_gw
        self._tts_gw = tts_gw
        # Read-only: the loaded map, for the map checks.
        self._map_catalog_repo = map_catalog_repo
        # execute_move's check, the same rule POST /api/v1/tasks applies.
        self._map_usage = map_usage
        # Where record_task leaves each finished run (the report export's source).
        self._task_record_repo = task_record_repo
        # What resolve_template reads: a scheduled run's template and its vertices.
        self._task_template_repo = task_template_repo
        self._map_repo = map_repo
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

    def _report_patrol_progress(self, step_id: str, passed: int) -> None:
        """Tell the run how many of ``step_id``'s waypoints are behind the robot
        (PATROL_PROGRESS_SIGNAL), for a resume to start past them.

        Every patrol reports, lone or grouped: a pause interrupts both. Same
        loop hand-off and bound as _report_started, and never fatal: a lost
        report only makes a resume drive through waypoints it already passed.
        """
        if self._client is None:
            return
        info = activity.info()
        handle = self._client.get_workflow_handle(
            info.workflow_id, run_id=info.workflow_run_id
        )
        try:
            asyncio.run_coroutine_threadsafe(
                handle.signal(PATROL_PROGRESS_SIGNAL, args=[step_id, passed]),
                self._loop,
            ).result(timeout=STARTED_REPORT_TIMEOUT_S)
        except CancelledError:
            raise
        except Exception as err:
            self._logger.warning(
                "[RobotActivity] Could not report patrol progress",
                step_id=step_id,
                passed=passed,
                error=str(err),
            )

    def _wait_for_patrol(
        self, goal_id: str, step_id: str, passed: int, sent: int, total: int
    ) -> dict:
        """_wait_for_nav_goal for a patrol, also tracking the waypoints passed.

        ``passed`` waypoints were behind the robot when ``sent`` were sent;
        number_of_poses_remaining counts the goal's still ahead, so passed +
        (sent - remaining) are behind it now. Capped below ``total``: the last
        waypoint is never skipped, since its heading is the step's end. Beaten
        in the details on every poll, so a retry starts from it, and reported
        to the run when it grows, so a resume does.
        """
        # Counted from what this goal skipped, never from the running count: the
        # same remaining count arrives on poll after poll, and each would add again.
        skipped = passed
        while True:
            status = self._robot_gw.get_move_status(goal_id=goal_id)
            state = status["state"] if status else None

            remaining = status.get("poses_remaining") if status else None
            grew = False
            if remaining is not None:
                now = min(total - 1, skipped + max(0, sent - remaining))
                grew = now > passed
                passed = max(passed, now)

            # Beat before the report: it can take up to STARTED_REPORT_TIMEOUT_S.
            activity.heartbeat(state, passed)
            if grew:
                self._report_patrol_progress(step_id, passed)

            if state in ["succeeded", "aborted", "canceled"]:
                return status

            time.sleep(NAV_POLL_INTERVAL_S)

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

        Drives from wherever the robot stands. Until 2026-11-12 a MOVE after a
        DOCK first backed the robot off the dock on undock.xml (the auto-undock);
        it went with the DOCK / UNDOCK steps and the robot stack's dock trees.
        """
        return self._navigate(params, expected_map, label="move")

    @activity.defn
    def execute_charge(
        self, params: MoveParams, expected_map: Optional[str] = None
    ) -> ActivityResult:
        """Charge at the charger waypoint in ``params``: a NavigateToGoal on
        charge.xml, sent as stored -- the console stores a charger's heading
        out of the marker's face, which is what the tree takes.

        The last metre follows the marker as the camera sees it, so it ends on
        the charger and not where the map says the charger is. It ends standing
        on the charger in MPC; a LIEDOWN step after it lies the robot down.
        """
        return self._navigate(
            params, expected_map, label="charge", behavior_tree=CHARGE_BEHAVIOR_TREE
        )

    @activity.defn
    def execute_patrol(
        self,
        params: PatrolParams,
        expected_map: Optional[str] = None,
        step_id: str = "",
        start: int = 0,
    ) -> ActivityResult:
        """Drive through ``params.waypoints`` in order: one NavigateThroughGoals.

        ``start`` is how many waypoints a dispatch before this one had passed
        (RobotWorkflow keeps the count from PATROL_PROGRESS_SIGNAL), and a retry
        of this dispatch reads its own from the last attempt's heartbeat
        details; the goal carries only the waypoints past the larger of the two,
        the last one always. ``step_id`` names the step in those reports.

        The map check is a MOVE's (_navigate's). The cleanup on a cancel is too:
        whatever is executing is cancelled.
        """
        self._refuse_wrong_map(expected_map)

        total = len(params.waypoints)
        passed = max(start, _patrol_passed(activity.info().heartbeat_details))
        passed = min(passed, total - 1)
        rest = params.waypoints[passed:]

        # patrol() can block up to NAV_GOAL_SEND_BUDGET_S; beat first, as _navigate.
        activity.heartbeat("sending", passed)

        try:
            accepted, msg, goal_id = self._robot_gw.patrol(
                [_pose_kwargs(waypoint) for waypoint in rest], total=total
            )
            if not accepted:
                raise ApplicationError(f"Patrol rejected: {msg}", non_retryable=False)

            self._logger.info(
                "[RobotActivity] Patrol accepted",
                goal_id=goal_id,
                waypoints=len(rest),
                skipped=passed,
            )
            self._report_started(beat=("accepted", passed))

            status = self._wait_for_patrol(
                goal_id, step_id, passed=passed, sent=len(rest), total=total
            )
            state = status["state"]

        except CancelledError:
            with activity.shield_thread_cancel_exception():
                self._robot_gw.cancel_active_moves()
            self._logger.warning("[RobotActivity] Patrol activity has been cancelled")
            raise

        if state != "succeeded":
            # NavigateThroughGoals (2026-11) carries NavigateToGoal's error_code /
            # error_msg, so a patrol's failure reads as a MOVE's, "did not start"
            # for code 0 included.
            raise _nav_failure(status, "patrol")

        return ActivityResult(success=True, goal_id=goal_id, state=state)

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
        params: MoveParams,
        expected_map: Optional[str],
        label: str,
        behavior_tree: str = "",
    ) -> ActivityResult:
        """Send a NavigateToGoal goal on ``behavior_tree`` ("" is the task
        runner's default, move.xml) and supervise it.

        ``expected_map`` is the run's ``TaskMap``. The goal is first checked
        against the loaded map (helpers/move_guard.py, as POST /api/v1/tasks
        does): another map's coordinates are refused, non-retryable; this is the
        check that catches a schedule firing after a map switch.

        Cancellation is *thrown* into this thread as CancelledError (in the poll
        loop or inside move()), so cleanup lives in the except clause. A blocking
        call like time.sleep is not interrupted, hence the short NAV_POLL_INTERVAL_S.
        """
        self._refuse_wrong_map(expected_map)
        move_kwargs = _pose_kwargs(params)
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

            self._logger.warning(
                f"[RobotActivity] {label.capitalize()} activity has been cancelled"
            )
            raise

        if state != "succeeded":
            raise _nav_failure(status, label)

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
    def record_task(self, record: TaskRecord) -> None:
        """Write a finished run's ``task_record`` row (RobotWorkflow._record).

        On the IO queue. Idempotent, so a retry after a lost completion is safe. A
        database error raises and is retried; the workflow drops it after that.
        """
        self._task_record_repo.insert(record)
        self._logger.info(
            "[RobotActivity] Task recorded",
            task_id=record.task_id,
            status=record.status,
        )

    @activity.defn
    def resolve_template(self, task_template_id: str) -> Optional[ResolvedTemplate]:
        """A scheduled run's template as it is now (RobotWorkflow._resolve_template).

        On the IO queue: database reads only. None when the template is gone,
        which leaves the run on the snapshot it was registered with. Another map
        loaded is not raised but answered, in move_guard's sentence, so the run
        is still named after the template it failed to run.
        """
        row = self._task_template_repo.get_task_template(uuid.UUID(task_template_id))
        if row is None:
            return None
        vertices = {}
        if row.map_name is not None:
            ids = stored_vertex_ids(row.steps)
            vertices = {
                v.id: v for v in self._map_repo.get_vertices(map=row.map_name, vertex_ids=ids)
            }
        refusal = self._map_usage.move_refusal(
            row.map_name, self._map_catalog_repo.active_name()
        )
        return ResolvedTemplate(
            name=row.name,
            map_name=row.map_name,
            steps=resolve_stored_steps(row.steps, vertices),
            refusal=refusal.message if refusal is not None else None,
        )

    @activity.defn
    def execute_demo(self, params: DemoParams) -> ActivityResult:
        """Report an inspection result to the capture service (demo branch only).

        One POST, no heartbeat. Any failure is retryable, bounded by the run's
        maximum_attempts.
        """
        description = INSPECTION_DESCRIPTIONS[params.inspection]
        try:
            response = httpx.post(
                DEMO_CAPTURE_URL,
                json={"description": description},
                timeout=DEMO_CAPTURE_TIMEOUT_S,
            )
        except httpx.HTTPError as err:
            raise ApplicationError(f"Capture request failed: {err!r}") from err

        if not response.is_success:
            raise ApplicationError(
                f"Capture service answered {response.status_code}: "
                f"{response.text[:200]}"
            )

        self._logger.info(
            "[RobotActivity] Inspection reported",
            inspection=params.inspection.value,
            description=description,
        )
        return ActivityResult(success=True, state="succeeded")
