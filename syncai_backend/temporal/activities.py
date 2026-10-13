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


# How often _wait_for_nav_goal reads the goal and heartbeats. A cancel reaches
# this thread in two hops, and this is the second: the worker learns of it from
# the server's reply to a heartbeat (at most every HEARTBEAT_THROTTLE_MAX, see
# worker.py), then throws CancelledError into the thread -- which only takes
# effect once the thread is back in Python, i.e. after the current sleep. Kept
# at or below the throttle so a heartbeat is always waiting when the next one
# may go out. get_move_status is an in-memory read, so polling this often is
# free.
NAV_POLL_INTERVAL_S = 0.25

# How long _report_started waits for its signal to be accepted. Temporal is
# local and answers a signal in milliseconds; this only bounds a wedged
# frontend, and it has to fit inside MOVE_HEARTBEAT_TIMEOUT beside the
# heartbeat execute_move sends just before it (test_activities.py checks).
STARTED_REPORT_TIMEOUT_S = 1.0


def _move_failure(status: dict) -> str:
    """The step's error_msg for a MOVE that did not succeed.

    The robot's own sentence when it gave one: NavigateToGoal's error_msg says
    whether planning or path following failed, which is the whole reason the
    backend moved off nav2's NavigateToPose (whose Humble result is empty). An
    abort with error_code NONE is, by the action's contract, a goal that was
    superseded or never started -- another navigation held the robot, or the
    behavior tree failed to load -- not a navigation failure, and it is said
    so rather than left as a bare "aborted".
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
        # Read-only here: which map is loaded and whether its floor plan is
        # being rebuilt, for the check at the top of execute_move. The worker
        # shares this process with the REST server, so it is the same
        # conversion registry the map routes write -- one answer, not two.
        self._map_catalog_repo = map_catalog_repo
        self._conversion_svc = conversion_svc
        # The worker's client and the loop it runs on, for _report_started.
        # Bound by run_worker once it has connected; a sync activity cannot
        # reach either through activity.client(), which is async-only.
        self._client: Optional[Client] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def bind_client(self, client: Client, loop: asyncio.AbstractEventLoop) -> None:
        self._client = client
        self._loop = loop

    def _report_started(self, beat: tuple = ()) -> None:
        """Tell the run this group member has started (STEP_STARTED_SIGNAL).

        Only a group member reports -- its run starts the next member on it
        (RobotWorkflow._run_group) -- and it knows it is one by the id it was
        scheduled under. The signal goes out on the worker's loop, from this
        thread. Never fatal: a report that cannot be sent leaves the next
        member waiting for this one to end rather than to start, which keeps
        the order and only loses the overlap. A cancel thrown in while it is
        out is the caller's to clean up after, so it is let through.

        ``beat``, when given, is heartbeated before the signal goes out, for
        a caller whose heartbeat window the wait could otherwise run out.
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

        ``expected_map`` is the run's ``TaskMap``, handed over by the workflow
        when the run carries one. Before anything is sent the goal is checked
        against the loaded map (helpers/move_guard.py, the same rule
        POST /api/v1/tasks applies): coordinates from another map are a real
        place on this one, just the wrong place, and a floor plan being
        rebuilt is about to be replaced under the planner. Both refusals are
        non-retryable, because three attempts five seconds apart fix neither
        -- and a schedule that fired after a map switch is exactly the run
        no dispatch-time check ever saw.

        This runs in a synchronous (threaded) activity. On cancellation Temporal
        *throws* CancelledError into this thread wherever it happens to be --
        in the poll loop, or inside move() while nav2 is still deciding -- so
        cleanup lives in the except clause below, not in an is_cancelled() poll.
        (A blocking C call such as time.sleep is not interrupted: the exception
        lands when it returns, which is why NAV_POLL_INTERVAL_S is short.)
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

        # The heartbeat_timeout clock starts when the activity starts, not at
        # the first heartbeat, and move() blocks with no loop to heartbeat from
        # for up to NAV_GOAL_SEND_BUDGET_S. Beating first is what keeps a slow
        # nav2 from failing the attempt before the poll loop below ever runs.
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
            # Cancel whatever is executing rather than a remembered goal_id:
            # if the cancel landed inside move() there is no id yet, and the
            # gateway disowns a goal nav2 accepts after its wait was
            # interrupted. Between the two, nothing is left driving. Shielded
            # so the cancel RPC finishes before the CancelledError propagates
            # and marks the activity cancelled.
            with activity.shield_thread_cancel_exception():
                self._robot_gw.cancel_active_moves()

            self._logger.warning("[RobotActivity] Move activity has been cancelled")
            raise

        if state != "succeeded":
            raise ApplicationError(_move_failure(status), non_retryable=False)

        return ActivityResult(success=True, goal_id=goal_id, state=state)

    def _set_motion_key(self, key: MotionKey, label: str) -> ActivityResult:
        """Send a motion key. Fire-and-forget: this does NOT wait for the pose.

        MODE is a one-way UDP command, so a successful service call only means
        the datagram was sent -- the step completes while the robot is still
        moving its legs. A step queued right behind this one (e.g. a MOVE) will
        therefore start against a robot that has not finished standing up.

        We deliberately do not paper over that with a fixed sleep: the driver
        manager already republishes the controller's MODE_STATE telemetry on
        the `mode` topic (data[0] = policy state, data[1] = motion state), so
        the real fix is to subscribe to it and poll the actual motion state
        here. That is pending the value mapping for data[1], which is defined
        on the gait controller side, not in this workspace.

        A False from the service means the driver manager rejected the key
        (unknown key, or the safety lock is engaged) -- retrying will not fix
        either on its own, hence non_retryable.
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

        The speech service's playback is a job: POST queues the utterance and
        answers with an id, GET reports its state, DELETE stops it. So this
        queues, then polls the job and heartbeats on every read -- the shape
        `_wait_for_nav_goal` has for MOVE -- and a cancel of the run (a task
        cancel, or a sibling in its group failing) reaches it on a heartbeat's
        reply and lands in ``except CancelledError``, which stops the job: the
        speaker goes quiet mid-sentence instead of finishing it.

        A pause never cancels a SPEAK; the workflow lets it finish (see
        RobotWorkflow._run_step).

        The one stretch with no heartbeat is the POST, which waits for
        synthesis (and, on a cold service, the model load). SPEAK's
        heartbeat_timeout outlasts the gateway's timeout on that request
        (SPEAK_HEARTBEAT_TIMEOUT, pinned in test_activities.py).

        ``params.repeat`` utterances are said back to back in one attempt. The
        count already said rides in the heartbeat beside the job's status, so
        a retry after the third of five fails says the remaining two rather
        than starting over; a heartbeat from before the field carries no count
        and reads as none said.

        Only Failure.UNKNOWN_VOICE is the request's fault and non-retryable;
        everything else (the service unreachable, its model missing, device
        trouble) is treated as possibly transient, same philosophy as the move
        rejections — the workflow's maximum_attempts=3 bounds the ones that are
        not. Note that a full speech queue is retryable on purpose: three
        attempts five seconds apart is exactly the right response to a backlog.
        """
        details = activity.info().heartbeat_details
        start = details[1] if len(details) > 1 and isinstance(details[1], int) else 0
        duration = 0.0
        for said in range(start, params.repeat):
            duration += self._speak_once(params, said, first=said == start)
            # Counted the moment it ends, not on the next utterance's first
            # poll: a retry after a failed POST would otherwise say this one
            # again.
            activity.heartbeat("done", said + 1)

        self._logger.info(
            "[RobotActivity] Speak finished", duration=duration, repeat=params.repeat
        )

        return ActivityResult(success=True, state="succeeded")

    def _speak_once(self, params: SpeakParams, said: int, first: bool = False) -> float:
        """One utterance of ``params``, heartbeating ``said`` beside its status.

        The ``first`` of an attempt reports the step started once it is
        queued: the service plays in the order it accepts, so a SPEAK grouped
        behind this one is queued behind it too.

        Returns how long it played; raises as execute_speak documents.
        """
        job_id = None
        try:
            # Shielded so a cancel that lands while the POST is out is raised
            # only once the job's id is known: the utterance is queued the
            # moment the service answers, whether or not this thread has read
            # the answer, and the except below needs the id to stop it.
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
                # Shielded for the same reason execute_move shields its
                # cleanup: the DELETE has to finish before the cancel does.
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
                # The gateway's code, not its prose. The tts router decides 400
                # vs 502 from the same one, so the two answers to "whose fault
                # is this?" cannot drift apart when the sentence is reworded.
                non_retryable=failure_code(message) is Failure.UNKNOWN_VOICE,
            )

        return duration
