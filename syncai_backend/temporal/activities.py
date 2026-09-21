import math
import time
import structlog
from pydantic import BaseModel

from temporalio import activity
from temporalio.exceptions import ApplicationError, CancelledError


from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.gateways.workflow.schema import MoveParams, SpeakParams
from syncai_backend.gateways.robot.robot import MotionKey, RobotGateway
from syncai_backend.gateways.tts.tts import TtsGateway


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
    ):
        self._logger = logger
        self._robot_gw = robot_gw
        self._tts_gw = tts_gw

    def _wait_for_nav_goal(self, goal_id: str) -> str:
        """Poll a navigation goal to a terminal state, heartbeating each round."""
        while True:
            status = self._robot_gw.get_move_status(goal_id=goal_id)
            state = status["state"] if status else None

            activity.heartbeat(state)

            if state in ["succeeded", "aborted", "canceled"]:
                return state

            time.sleep(1.0)

    @activity.defn
    def execute_move(self, params: MoveParams) -> ActivityResult:
        """Send a NavigateToPose goal and supervise it to a terminal state.

        This runs in a synchronous (threaded) activity. On cancellation Temporal
        *throws* CancelledError into this thread wherever it happens to be --
        inside time.sleep, or inside move() while nav2 is still deciding -- so
        cleanup lives in the except clause below, not in an is_cancelled() poll.
        """
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

            state = self._wait_for_nav_goal(goal_id=goal_id)

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
            raise ApplicationError(f"move ended in {state}", non_retryable=False)

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
        """Speak on the robot speaker, blocking until playback finishes.

        This never heartbeats: TtsGateway.speak() sits in one blocking call —
        now a single HTTP request to the syncai_tts service, held open for the
        utterance by `wait=true` — so there is no loop to heartbeat from. The
        workflow therefore drops the heartbeat_timeout for SPEAK steps and
        relies on a short start_to_close instead; see the per-step options in
        workflows.py. Same reason it is effectively not cancellable
        mid-utterance: without heartbeats the worker never learns of a cancel,
        so a canceled task finishes the sentence it is on before the workflow's
        CancelledError lands. An utterance is bounded (text is capped at 1000
        chars, and the gateway gives up at 240 s), so that is a few seconds of
        latency, not a hang.

        **That is now a choice rather than a constraint.** The speech service's
        playback is a job: POST returns an id, GET reports its state and DELETE
        stops it. Rewriting this to enqueue and then poll once a second would
        make the heartbeat real and let `except CancelledError` cut the
        utterance, the way `_wait_for_nav_goal` already does for MOVE. It is
        left blocking here so that moving speech out of this process changed
        nothing about the task path; do that next, not at the same time.

        Only Failure.UNKNOWN_VOICE is the request's fault and non-retryable;
        everything else (the service unreachable, its model missing, device
        trouble) is treated as possibly transient, same philosophy as the move
        rejections — the workflow's maximum_attempts=3 bounds the ones that are
        not. Note that a full speech queue is retryable on purpose: three
        attempts five seconds apart is exactly the right response to a backlog.
        """
        success, message, duration = self._tts_gw.speak(
            text=params.text, voice=params.voice, speed=params.speed
        )
        if not success:
            raise ApplicationError(
                f"Speak failed: {message}",
                # The gateway's code, not its prose. The tts router decides 400
                # vs 502 from the same one, so the two answers to "whose fault
                # is this?" cannot drift apart when the sentence is reworded.
                non_retryable=failure_code(message) is Failure.UNKNOWN_VOICE,
            )

        self._logger.info("[RobotActivity] Speak finished", duration=duration)

        return ActivityResult(success=True, state="succeeded")
