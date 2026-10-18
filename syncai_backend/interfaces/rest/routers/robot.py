import json
import math
import structlog
from datetime import datetime
from enum import Enum
from typing import Optional
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_common.msg import RobotMode, RobotState as RobotStateMsg

from syncai_backend.exceptions import ConflictError, UpstreamError, NotFoundError
from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.gateways.robot.robot import MotionKey, RobotGateway
from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.services.mode_restart import ModeRestartService, RestartStatus
from syncai_backend.services.safety_lock import SafetyLockService


# Reverse lookup: RobotMode uint8 constant -> human-readable name.
_ROBOT_MODE_TO_STR = {
    RobotMode.MAINTENANCE: "MAINTENANCE",
    RobotMode.MANUAL: "MANUAL",
    RobotMode.AUTO: "AUTO",
}


def _mode_to_str(mode: int) -> str:
    return _ROBOT_MODE_TO_STR.get(mode, "UNKNOWN")


# Reverse lookups for the gait controller's state (RobotState.low_level_mode).
#
# Plain dicts with an UNKNOWN fallback, NOT the PolicyMode enum: that is the
# command vocabulary (0 and 1 only), and a robot reporting any other index
# would fail validation and 500 the state endpoint. The vocabularies are
# documented, unverified, in syncai_common/msg/RobotLowLevelMode.msg.
_POLICY_STATE_TO_STR = {0: "PPO", 1: "HIMLOCO", 2: "CHAMP", 3: "ISSAC"}

# 8 is IDLE (no controller driving the motors); labelled because the console
# must tell a return to IDLE from MPC, whose code is unknown and so reads
# "UNKNOWN". The raw integer is only on the `robot_state` topic.
_MOTION_STATE_TO_STR = {
    0: "STAND",
    1: "LOCOMOTION",
    2: "LIE_DOWN",
    3: "DAMPING",
    4: "ESTOP",
    8: "IDLE",
}


def _policy_state_to_str(policy_state: int) -> str:
    return _POLICY_STATE_TO_STR.get(policy_state, "UNKNOWN")


def _motion_state_to_str(motion_state: int) -> str:
    return _MOTION_STATE_TO_STR.get(motion_state, "UNKNOWN")


def _parse_wifi_info(wifi_info: str) -> "RobotNetworkStatus":
    # wifi_info is a JSON string published by syncai_robot_state; it is
    # "null" until the first wifi_status message arrives.
    try:
        data = json.loads(wifi_info)
    except (json.JSONDecodeError, TypeError):
        data = None

    if not isinstance(data, dict):
        data = {}

    return RobotNetworkStatus(**data)


class RobotPose(BaseModel):
    x: float = Field(..., description="The x-coordinate of the robot's position.")
    y: float = Field(..., description="The y-coordinate of the robot's position.")
    z: float = Field(
        ...,
        description=(
            "Height of base_link in the map frame, metres: which storey the "
            "robot is on (the ground is about 0.43 m below it, standing)."
        ),
    )
    theta: float = Field(..., description="The yaw angle of the robot, in degrees.")


class RobotLocalizationStatus(BaseModel):
    position: RobotPose = Field(..., description="The position of the robot.")
    velocity: float = Field(..., description="The velocity of the robot.")


class RobotNetworkStatus(BaseModel):
    ssid: str = Field("", description="The SSID of the connected WiFi network.")
    bssid: str = Field("", description="The BSSID of the connected WiFi access point.")
    rssi: int = Field(0, description="The WiFi signal strength of the robot, in dBm.")
    ip_address: str = Field(
        "", description="The IP address of the robot's WiFi interface."
    )
    mac_address: str = Field(
        "", description="The MAC address of the robot's WiFi interface."
    )


class RobotBatteryStatus(BaseModel):
    battery_percentage: int = Field(
        ..., description="The battery percentage of the robot."
    )


class RobotMotorStatus(BaseModel):
    """One joint's health, as exposed outward.

    A deliberate subset of MotorState: no kinematics, because this is an
    unordered 10 Hz diagnostic snapshot. Joint motion is on the telemetry
    WebSocket.
    """

    name: str = Field(..., description="The URDF joint name of the motor.")
    temperature: int = Field(..., description="The motor temperature, in Celsius.")
    error: int = Field(..., description="The motor error code; 0 when healthy.")


class RobotLowLevelMode(BaseModel):
    """What the gait controller says it is doing, as opposed to what we asked.

    The only controller-reported view: the motion-key and policy commands are
    unacknowledged UDP. **Not** `RobotState.mode` (the byobu session); see
    syncai_common/msg/RobotLowLevelMode.msg for the three "modes".

    Labels only: unmapped integers (MPC's included) collapse to "UNKNOWN"; the
    raw values are on `ros2 topic echo /<robot_id>/robot_state --field
    low_level_mode`. No freshness either: "PPO" / "STAND" is also what a
    never-heard-from controller reads as.
    """

    policy: str = Field(
        ...,
        description=(
            "The controller's RL policy: PPO / HIMLOCO, or "
            "UNKNOWN for an index this backend has no name for."
        ),
    )
    motion: str = Field(
        ...,
        description=(
            "The controller's motion state: STAND / LOCOMOTION / LIE_DOWN / "
            "DAMPING / ESTOP / IDLE (8, motors not driven by any controller), "
            "or UNKNOWN for an index this backend has no name for. MPC has no "
            "known code, so it lands on UNKNOWN."
        ),
    )
    safety_locked: bool = Field(
        ...,
        description=(
            "Whether syncai_driver_manager's software safety lock is engaged: "
            "cmd_vel and every motion key but ESTOP are "
            "dropped until POST /api/v1/robot/estop releases it. Every running "
            "task is cancelled when it engages. False before the driver has "
            "published, indistinguishable from a genuine release."
        ),
    )


class RobotState(BaseModel):
    timestamp: int = Field(..., description="The timestamp of the robot state.")
    robot_id: str = Field(..., description="The unique identifier of the robot.")
    map: str = Field(..., description="The name of the map the robot is on.")
    mode: str = Field(
        ...,
        description=(
            "Which byobu session is up (MAINTENANCE/MANUAL/AUTO). NOT the gait "
            "controller's state machine — that is `low_level_mode`."
        ),
    )
    low_level_mode: RobotLowLevelMode = Field(
        ...,
        description=(
            "What the gait controller reports it is doing, as opposed to what "
            "was commanded. See the model for why it carries raw integers too."
        ),
    )
    localization_valid: bool = Field(
        ...,
        description=(
            "Whether localization_status carries a real pose. False before the "
            "localizer has been relocalized, and for the whole of a mapping "
            "(MANUAL) run — the pose fields are then a zeroed placeholder, not "
            "the robot standing on the map origin."
        ),
    )
    localization_status: RobotLocalizationStatus = Field(
        ..., description="The localization status of the robot."
    )
    network_status: RobotNetworkStatus = Field(
        ..., description="The network status of the robot."
    )
    battery_status: RobotBatteryStatus = Field(
        ..., description="The battery status of the robot."
    )
    motor_status: list[RobotMotorStatus] = Field(
        default_factory=list,
        description=(
            "Per-joint motor health (name/temperature/error). Empty while "
            "syncai_driver_manager is not publishing motor_states."
        ),
    )


class OperatingMode(str, Enum):
    """The switchable operating modes — the byobu session families.

    No MAINTENANCE: it means "no session" and sys_manager refuses it, so the
    schema 422s it. It still appears in RobotState.mode as a reported value.
    """

    MANUAL = "MANUAL"
    AUTO = "AUTO"


# The REST vocabulary is the name; the srv field is the RobotMode uint8.
_OPERATING_MODE_TO_INT = {
    OperatingMode.MANUAL: RobotMode.MANUAL,
    OperatingMode.AUTO: RobotMode.AUTO,
}


class SwitchModeRequest(BaseModel):
    mode: OperatingMode = Field(
        ...,
        description=(
            "Target operating mode: MANUAL (mapping session) or AUTO "
            "(navigation session). MAINTENANCE is not a switchable target."
        ),
    )


class SwitchModeResponse(BaseModel):
    mode: OperatingMode = Field(..., description="The mode the request asked for.")
    switching: bool = Field(
        ...,
        description=(
            "True when the switch was dispatched and the stack is being torn "
            "down and rebuilt — INCLUDING the process serving this API, so "
            "expect this very response to be the last one for ~10-30 s (or to "
            "never arrive: a dropped connection right after this POST is the "
            "switch working, not failing). Poll GET /api/v1/robot/state until "
            "its `mode` reports the target. False means sys_manager answered "
            "without a rebuild — the robot was already in the requested mode."
        ),
    )
    message: str = Field(..., description="Human-readable result of the request.")


class RestartModeResponse(BaseModel):
    restarting: bool = Field(
        ...,
        description=(
            "True when the restart was dispatched and the live mode's stack is "
            "being torn down and rebuilt. This process is not part of that "
            "stack and keeps answering, so poll GET /api/v1/robot/restart for "
            "how it ends — GET /api/v1/robot/state keeps serving the last "
            "frame through the outage and cannot say. False means sys_manager "
            "answered inside the ack window that the rebuild is already done."
        ),
    )
    message: str = Field(..., description="Human-readable result of the request.")


class RestartStatusResponse(BaseModel):
    status: RestartStatus = Field(
        ...,
        description=(
            "The latest restart this process dispatched: `idle` if none has "
            "been since it started, `restarting` while sys_manager has not "
            "answered, then `succeeded` or `failed`. In memory, so a backend "
            "that was itself restarted says `idle` rather than guess."
        ),
    )
    message: str = Field(
        ...,
        description=(
            "sys_manager's answer once there is one (empty before): the "
            "reason, on `failed`."
        ),
    )
    started_at: Optional[datetime] = Field(
        None, description="When the restart was dispatched (UTC); null when idle."
    )
    finished_at: Optional[datetime] = Field(
        None, description="When it ended (UTC); null until then."
    )


class SetInitialPoseRequest(BaseModel):
    x: float = Field(..., description="The x-coordinate of the pose, in the map frame.")
    y: float = Field(..., description="The y-coordinate of the pose, in the map frame.")
    theta: float = Field(
        ..., description="The yaw angle of the pose, in degrees (map frame)."
    )
    z: Optional[float] = Field(
        None,
        allow_inf_nan=False,
        description=(
            "Height of the ground the pose stands on (metres, map frame), as a "
            "vertex's `z`: which storey to look for the robot on. Omitted, the "
            "localizer keeps the storey it is on, so a pose on another storey "
            "needs it."
        ),
    )


class SetInitialPoseResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the request.")


class SetMotionKeyRequest(BaseModel):
    """A gait-controller motion key, as the driver's numeric string.

    Typed with the gateway's ``MotionKey``, not a REST copy or a ``str``: the
    gateway takes the enum, and a ``str`` would 500 inside it.
    """

    key: MotionKey = Field(
        ...,
        description=(
            "'0' stand, '1' locomotion, '2' lie down, '3' damping, "
            "'4' emergency stop (accepted but NOT forwarded — see `sent`), "
            "'5' MPC."
        ),
    )


class SetMotionKeyResponse(BaseModel):
    key: MotionKey = Field(..., description="The key the request asked for.")
    sent: bool = Field(
        ...,
        description=(
            "Whether the key was actually forwarded to the driver. False only "
            "for '4' (emergency stop), which this endpoint accepts but does not "
            "forward — no ESTOP datagram was produced. True means the driver "
            "reported writing the datagram, NOT that the robot moved: the "
            "command is one-way UDP with no acknowledgement."
        ),
    )
    message: str = Field(..., description="Human-readable result of the request.")


class EstopRequest(BaseModel):
    locked: bool = Field(
        ...,
        description=(
            "True engages the driver's safety lock (cmd_vel and every motion "
            "key but ESTOP are dropped; every running task is cancelled), "
            "false releases it."
        ),
    )


class EstopResponse(BaseModel):
    locked: bool = Field(..., description="The lock state that was requested.")
    cancel_requested: bool = Field(
        ...,
        description=(
            "Whether this request started cancelling every running task: true "
            "only when it engaged a lock that was not already engaged. Started, "
            "not finished -- the cancel runs after this response."
        ),
    )
    message: str = Field(..., description="Human-readable result of the request.")
    # No `changed` flag: the driver only says so in prose. "Is it locked now"
    # is low_level_mode.safety_locked on GET /api/v1/robot/state.


class PolicyMode(int, Enum):
    """Gait-controller policy indices this REST surface accepts.

    From the udp_ros_bridge Readme ("0=PPO, 1=HIMLOCO, 2=CHAMP, 3=ISSAC"),
    unverified against current firmware: trust the index, not the label. The
    driver forwards any number unvalidated, so this enum is the only gate.

    Only 0 and 1: CHAMP and ISSAC are untested on this robot and must not be
    reachable from an unauthenticated POST; widening is a reviewed code change.
    REST vocabulary only: the srv and ``RobotGateway.set_policy_mode()`` stay
    a plain int. Not ``RobotMode``, and not the reported policy, which is
    decoded through ``_POLICY_STATE_TO_STR`` so it cannot 500.
    """

    PPO = 0
    HIMLOCO = 1


class SetPolicyModeRequest(BaseModel):
    mode: PolicyMode = Field(
        ...,
        description=(
            "The gait controller's policy index: 0 PPO, 1 HIMLOCO. Not the "
            "robot mode reported by GET /api/v1/robot/state."
        ),
    )


class SetPolicyModeResponse(BaseModel):
    mode: PolicyMode = Field(..., description="The policy mode that was sent.")
    message: str = Field(..., description="Human-readable result of the request.")
    # No `sent` flag: every admitted value is forwarded, so it would always be
    # True.


def init_robot_router(
    logger: structlog.stdlib.BoundLogger,
    robot_repo: RobotRepo,
    robot_gw: RobotGateway,
    restart_svc: ModeRestartService,
    safety_lock_svc: SafetyLockService,
) -> APIRouter:
    robot_router = APIRouter(prefix="", tags=["Robot"])

    @robot_router.get("/api/v1/robot/state", response_model=RobotState)
    async def get_robot_state():
        # A frozen third-party contract: fields are copied one by one, a
        # WHITELIST, not a mirror, so a field added to the message (or to
        # MotorState) stays internal until someone deliberately exposes it.
        #
        # None means the robot_state node has never published (404); "not
        # localized yet" is localization_valid in the payload.
        state: RobotStateMsg = robot_repo.get_robot_state()
        if state is None:
            raise NotFoundError("Robot state is not available yet.")

        return RobotState(
            timestamp=state.timestamp,
            robot_id=state.robot_id,
            map=state.map,
            mode=_mode_to_str(state.mode),
            # Through the fallback dicts, never PolicyMode (it would 500).
            low_level_mode=RobotLowLevelMode(
                policy=_policy_state_to_str(state.low_level_mode.policy_state),
                motion=_motion_state_to_str(state.low_level_mode.motion_state),
                # The driver's lock, not the controller's (RobotLowLevelMode.msg).
                safety_locked=state.low_level_mode.safety_state,
            ),
            # Tells the zeroed placeholder pose from a robot at the origin.
            localization_valid=state.localization_valid,
            localization_status=RobotLocalizationStatus(
                position=RobotPose(
                    x=state.localization_status.position.x,
                    y=state.localization_status.position.y,
                    z=state.localization_status.position.z,
                    theta=math.degrees(state.localization_status.position.yaw),
                ),
                velocity=state.localization_status.velocity,
            ),
            network_status=_parse_wifi_info(state.network_status.wifi_info),
            battery_status=RobotBatteryStatus(
                battery_percentage=int(state.battery_status.battery_percentage),
            ),
            motor_status=[
                RobotMotorStatus(
                    name=motor.name,
                    # int() keeps rclpy's numpy scalars out of the JSON encoder.
                    temperature=int(motor.temperature),
                    error=int(motor.error),
                )
                for motor in state.motor_status.states
            ],
        )

    @robot_router.post("/api/v1/robot/mode", response_model=SwitchModeResponse)
    def switch_mode(request: SwitchModeRequest):
        """Switch the operating mode — the one request that outlives its server.

        A real switch kills the byobu session this backend runs in, so the
        client usually sees a dropped connection, which means success in
        progress; only the no-op and a refusal reliably answer. 502 on failure.

        Plain ``def``: the gateway blocks on wait_for_service plus the ack
        window. No GET twin: the live mode is on GET /api/v1/robot/state.
        """
        success, message = robot_gw.switch_mode(
            mode=int(_OPERATING_MODE_TO_INT[request.mode])
        )

        if success is False:
            logger.error(
                "Failed to switch mode", mode=request.mode.value, message=message
            )
            raise UpstreamError(message)

        if success is True:
            # Only the no-op answers this fast.
            return SwitchModeResponse(
                mode=request.mode,
                switching=False,
                message=f"Already in {request.mode.value}; nothing to do.",
            )

        # success is None: dispatched, teardown underway. A 200, not a 202.
        return SwitchModeResponse(
            mode=request.mode,
            switching=True,
            message=(
                f"Switching to {request.mode.value}. The console will lose "
                "this API while the stack rebuilds; poll robot state."
            ),
        )

    @robot_router.post("/api/v1/robot/restart", response_model=RestartModeResponse)
    def restart_mode():
        """Rebuild the live mode's byobu session — ``restart_mode`` on sys_manager.

        For a wedged stack (POST /api/v1/robot/mode treats the live mode as a
        no-op). No body: sys_manager restarts whatever mode is live.

        Answers within the ack window; the outcome is kept for GET
        /api/v1/robot/restart. sys_manager refuses in MAINTENANCE, with both
        sessions up, and in MANUAL (an unsaved map may be in RAM): 409
        ``restart_refused``. 409 ``restart_running`` while one runs; 502 when
        sys_manager is unreachable.
        """
        success, message = restart_svc.start()

        if success is False:
            logger.error("Failed to restart mode", message=message)
            if failure_code(message) is Failure.RESTART_REFUSED:
                raise ConflictError(message, code=Failure.RESTART_REFUSED.value)
            raise UpstreamError(message)

        if success is True:
            return RestartModeResponse(restarting=False, message=message)

        return RestartModeResponse(
            restarting=True,
            message=(
                "Restarting the live mode; poll GET /api/v1/robot/restart for "
                "the outcome."
            ),
        )

    @robot_router.get("/api/v1/robot/restart", response_model=RestartStatusResponse)
    async def get_restart_status():
        """How the latest restart this process dispatched is going, or went.

        `async` because it only reads a slot under a lock — no ROS, no I/O.
        """
        record = restart_svc.snapshot()
        return RestartStatusResponse(
            status=record.status,
            message=record.message,
            started_at=record.started_at,
            finished_at=record.finished_at,
        )

    @robot_router.post(
        "/api/v1/robot/set_initial_pose", response_model=SetInitialPoseResponse
    )
    def set_initial_pose(request: SetInitialPoseRequest):
        # Plain `def`: the gateway call blocks on rclpy.
        #
        # REST is degrees everywhere, ROS radians; this is the conversion.
        success, message = robot_gw.set_initial_pose(
            x=request.x, y=request.y, yaw=math.radians(request.theta), z=request.z
        )
        if not success:
            logger.error(
                "Failed to set initial pose",
                x=request.x,
                y=request.y,
                z=request.z,
                theta=request.theta,
                message=message,
            )
            raise UpstreamError(message)

        height = "" if request.z is None else f", z={request.z}"
        return SetInitialPoseResponse(
            message=(
                f"Published initial pose x={request.x}, y={request.y}{height}, "
                f"theta={request.theta}"
            )
        )

    # The two command endpoints below are plain `def`: the gateway blocks on
    # rclpy (wait_for_service alone is up to 5 s).
    #
    # No rate limiting, deliberately: a 200 only means a UDP datagram was
    # written, and the Temporal posture steps and `ros2 service call` bypass
    # this router, so a debounce here would imply a regulation that does not
    # exist. Both send `MODE ...` on the same socket with no ordering beyond
    # the driver's callback group. Any real interlock belongs on the driver or
    # the controller, where every caller passes.

    @robot_router.post(
        "/api/v1/robot/set_motion_key", response_model=SetMotionKeyResponse
    )
    def set_motion_key(request: SetMotionKeyRequest):
        # '4' (ESTOP) is accepted (a 422 would call a documented key invalid)
        # but NOT forwarded: an unacknowledged UDP emergency stop over
        # unauthenticated HTTP is a safety claim this layer cannot honour.
        # `sent=False` is the load-bearing answer; a 200, not an error, so
        # nothing retries against ESTOP. The lock is POST /api/v1/robot/estop.
        if request.key is MotionKey.ESTOP:
            logger.warning(
                "Refused to forward ESTOP motion key; no datagram sent",
                key=request.key.value,
            )
            return SetMotionKeyResponse(
                key=request.key,
                sent=False,
                message=(
                    "Emergency stop is not available over this API: no ESTOP "
                    "was sent to the robot."
                ),
            )

        success, message = robot_gw.set_motion_key(key=request.key)
        if not success:
            # Uniform 502, message verbatim: never match on the driver's
            # unpinned C++ strings.
            logger.error(
                "Failed to set motion key", key=request.key.value, message=message
            )
            raise UpstreamError(message)

        # No success log (the gateway logs it). The sentence is this API's own,
        # not the driver's.
        return SetMotionKeyResponse(
            key=request.key,
            sent=True,
            message=f"Sent motion key {request.key.value} ({request.key.name})",
        )

    @robot_router.post("/api/v1/robot/estop", response_model=EstopResponse)
    def estop(request: EstopRequest):
        """Engage or release the driver's safety lock (`set_safety_lock`).

        Engaging cancels every running task in the background, only on the
        rising edge; SafetyLockService shares that edge with robot_state, so
        one engagement is one cancel. 502 when the driver fails.

        Not the ESTOP motion key (a datagram to the controller); this is the
        driver's gate in front of it. Plain ``def``: a ROS round trip.
        """
        success, message = robot_gw.set_safety_lock(locked=request.locked)
        if not success:
            # Nothing was locked, so nothing is cancelled.
            logger.error(
                "Failed to set safety lock", locked=request.locked, message=message
            )
            raise UpstreamError(message)

        cancel_requested = request.locked and safety_lock_svc.engage_requested()
        if request.locked:
            message = (
                "Safety lock engaged; cancelling every running task."
                if cancel_requested
                else "Safety lock was already engaged."
            )
        else:
            message = "Safety lock released."
        return EstopResponse(
            locked=request.locked, cancel_requested=cancel_requested, message=message
        )

    @robot_router.post(
        "/api/v1/robot/set_policy_mode", response_model=SetPolicyModeResponse
    )
    def set_policy_mode(request: SetPolicyModeRequest):
        # PolicyMode 422s an unexposed index; nothing downstream validates it.
        # .value: the gateway's contract is a plain int.
        #
        # NOT gated by the driver's safety lock, unlike set_motion_key (a known
        # gap, see syncai_driver_manager/README.md).
        success, message = robot_gw.set_policy_mode(mode=request.mode.value)
        if not success:
            # Uniform 502; in practice only the gateway's own "not available" /
            # "Timeout", since the driver callback cannot fail.
            logger.error(
                "Failed to set policy mode", mode=request.mode.value, message=message
            )
            raise UpstreamError(message)

        # 200 means `MODE <n>` was written, not that the policy switched.
        return SetPolicyModeResponse(
            mode=request.mode,
            message=f"Sent policy mode {request.mode.value} ({request.mode.name})",
        )

    return robot_router
