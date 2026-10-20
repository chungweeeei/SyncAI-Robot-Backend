import math
import uuid
import threading
import structlog
from enum import Enum
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Tuple
from rclpy import qos
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from action_msgs.msg import GoalStatus

from syncai_common.action import NavigateToGoal
from syncai_common.msg import WifiConnection, WifiNetwork
from syncai_common.srv import (
    ActivateWifiConnection,
    ConnectWifiNetwork,
    DeleteWifiConnection,
    ListWifiConnections,
    RestartMode,
    ScanWifiNetworks,
    SetMotionKey,
    SetPolicyMode,
    SwitchMode,
)

from std_msgs.msg import Header
from std_srvs.srv import SetBool
from geometry_msgs.msg import (
    Point,
    Quaternion,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)

from syncai_backend.gateways.failure import Failure, fail


class MotionKey(str, Enum):
    """Keys accepted by the driver manager's `set_motion_key` service.

    Each maps to a gait-controller MODE character (see
    DriverManagerNode::setMotionKeyCallback), except ESTOP, which goes out as
    its own datagram. Strings because the srv field is forwarded verbatim.
    """

    STAND = "0"  # MODE Z
    LOCOMOTION = "1"  # MODE C
    LIE_DOWN = "2"  # MODE X
    DAMPING = "3"  # MODE R
    ESTOP = "4"  # ESTOP datagram, not a MODE character
    MPC = "5"  # MODE M


class MoveState(str, Enum):
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    REJECTED = "rejected"


# One NavigateToGoal goal. `feedback` is the latest Feedback (status_msg shows
# beside a running MOVE step); `result` the Result (error_code / error_msg).
@dataclass
class MoveGoal:
    goal_id: str
    goal_handle: ClientGoalHandle
    state: MoveState = MoveState.EXECUTING
    feedback: Optional[Any] = None
    result: Optional[Any] = None


# How many finished MOVE goals _goals keeps (otherwise it grows without bound).
# The MOVE activity polls only its own newest goal, so five is plenty. Eviction
# happens on insert and never touches an EXECUTING goal.
MAX_TRACKED_GOALS = 5

# How long switch_mode() / restart_mode() wait for sys_manager before reporting
# "dispatched". NOT sized to a switch: a real switch kills the byobu session
# this process runs in before any answer. It only catches the fast no-op and
# refusals. Module-level so tests can shrink it.
SWITCH_MODE_ACK_TIMEOUT = 2.0

# How long move() waits for the NavigateToGoal server, then for acceptance.
# The whole send must fit under the MOVE activity's 3 s heartbeat_timeout
# (temporal/workflows.py), since it cannot heartbeat inside this call; a kill
# mid-send can leave an accepted goal unsupervised. A slow server gets its
# chance from the workflow's retry policy. test_activities.py pins the budget.
NAV_SERVER_WAIT_S = 1.0
NAV_ACCEPT_WAIT_S = 1.5
NAV_GOAL_SEND_BUDGET_S = NAV_SERVER_WAIT_S + NAV_ACCEPT_WAIT_S

# NavigateToGoal.behavior_tree for the DOCK / UNDOCK steps: bare file names the task
# runner resolves in its own behavior_trees/ (syncai_common >= bbcc958). dock.xml takes
# the dock pose and works out the staging pose from it; undock.xml ignores the pose
# (task runner e027dbe) and backs out 1 m from where the robot stands. Both drive
# DockToPose themselves, so this process never calls that action. "" is the task
# runner's default, move.xml.
DOCK_BEHAVIOR_TREE = "dock.xml"
UNDOCK_BEHAVIOR_TREE = "undock.xml"

# What tells syncai_localizer that an initialpose's z is meant: the z variance
# (cell 14 of the row-major 6x6 covariance) above zero. RViz's 2D Pose Estimate
# and every sender from before leave it 0, and the localizer then keeps the
# height of its own estimate, so z = 0 from them never pulls the guess to 1F.
# The value itself is not read; this one says "a few centimetres".
INITIAL_POSE_Z_VARIANCE_INDEX = 14
INITIAL_POSE_Z_VARIANCE = 0.01


def _wait_for_future(future, timeout: Optional[float] = None) -> bool:
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout=timeout)


def _goal_id_str(goal_handle: ClientGoalHandle) -> str:
    return str(uuid.UUID(bytes=bytes(goal_handle.goal_id.uuid)))


def _clamp_normalized(value: float) -> float:
    """[-1, 1] with NaN/inf collapsed to 0 — the last gate before the wire."""
    if not math.isfinite(value):
        return 0.0
    return max(-1.0, min(1.0, value))


class RobotGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):

        self._logger = logger

        self._node = node

        self._action_clients: dict[str, ActionClient] = {}
        self.register_action_clients()

        self._service_clients: dict[str, Client] = {}
        self.register_service_clients()

        self._publishers: dict[str, Publisher] = {}
        self.register_publishers()

        self._lock = threading.Lock()
        self._goals: dict[str, MoveGoal] = {}

    def register_action_clients(self):

        move_client = ActionClient(
            node=self._node,
            # NavigateToGoal, not nav2's NavigateToPose: its result says why a
            # MOVE failed and its feedback what the robot is doing (Humble's
            # NavigateToPose result is Empty).
            action_type=NavigateToGoal,
            action_name="navigate_to_goal",
        )

        self._action_clients.update({"move": move_client})

    def register_service_clients(self):

        scan_wifi_client = self._node.create_client(
            srv_type=ScanWifiNetworks,
            srv_name="scan_wifi",
        )

        connect_wifi_client = self._node.create_client(
            srv_type=ConnectWifiNetwork,
            srv_name="connect_wifi",
        )

        # Saved networks (NetworkManager profiles). Same server as the above.
        list_wifi_connections_client = self._node.create_client(
            srv_type=ListWifiConnections,
            srv_name="list_wifi_connections",
        )

        activate_wifi_connection_client = self._node.create_client(
            srv_type=ActivateWifiConnection,
            srv_name="activate_wifi_connection",
        )

        delete_wifi_connection_client = self._node.create_client(
            srv_type=DeleteWifiConnection,
            srv_name="delete_wifi_connection",
        )

        set_motion_key_client = self._node.create_client(
            srv_type=SetMotionKey,
            srv_name="set_motion_key",
        )

        # Relative, like every client here, so it reaches this robot's
        # driver_manager only; an absolute name has been a bug before.
        set_policy_mode_client = self._node.create_client(
            srv_type=SetPolicyMode,
            srv_name="set_policy_mode",
        )

        # The driver's software safety lock, both ways. Same server as above.
        safety_lock_client = self._node.create_client(
            srv_type=SetBool,
            srv_name="set_safety_lock",
        )

        # sys_manager's operating-mode switch (which byobu session is up). Here,
        # not in MapGateway: it is a handle that can command the robot.
        switch_mode_client = self._node.create_client(
            srv_type=SwitchMode,
            srv_name="switch_mode",
        )

        # Rebuild the live mode's session, which switch_mode refuses to do.
        restart_mode_client = self._node.create_client(
            srv_type=RestartMode,
            srv_name="restart_mode",
        )

        self._service_clients.update(
            {
                "scan_wifi": scan_wifi_client,
                "connect_wifi": connect_wifi_client,
                "list_wifi_connections": list_wifi_connections_client,
                "activate_wifi_connection": activate_wifi_connection_client,
                "delete_wifi_connection": delete_wifi_connection_client,
                "set_motion_key": set_motion_key_client,
                "set_policy_mode": set_policy_mode_client,
                "set_safety_lock": safety_lock_client,
                "switch_mode": switch_mode_client,
                "restart_mode": restart_mode_client,
            }
        )

    def register_publishers(self):
        initial_pose_pub = self._node.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic="initialpose",
            qos_profile=qos.QoSProfile(depth=5),
        )

        cmd_vel_pub = self._node.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=qos.QoSProfile(depth=5),
        )

        self._publishers.update(
            {
                "initial_pose": initial_pose_pub,
                "cmd_vel": cmd_vel_pub
            }
        )

    def scan_wifi_networks(self) -> Tuple[bool, str, List[WifiNetwork]]:
        scan_client = self._service_clients.get("scan_wifi")
        if not scan_client.wait_for_service(timeout_sec=5.0):
            return False, "scan_wifi service is not available", []

        self._logger.info("[RobotGateway] Scanning WiFi networks")

        future = scan_client.call_async(ScanWifiNetworks.Request())
        # The service rescans (10s) then lists (30s); leave headroom on top.
        if not _wait_for_future(future, timeout=45.0):
            return False, "Timeout waiting for scan_wifi response", []

        response = future.result()
        if not response.success:
            return False, response.message, []

        return True, "", list(response.networks)

    def connect_wifi(self, ssid: str, password: str) -> Tuple[bool, str]:
        connect_client = self._service_clients.get("connect_wifi")
        if not connect_client.wait_for_service(timeout_sec=5.0):
            return False, "connect_wifi service is not available"

        self._logger.info("[RobotGateway] Connecting to WiFi network", ssid=ssid)

        future = connect_client.call_async(
            ConnectWifiNetwork.Request(ssid=ssid, password=password)
        )
        # The service itself waits up to 60s for nmcli; leave headroom on top.
        if not _wait_for_future(future, timeout=70.0):
            return False, "Timeout waiting for connect_wifi response"

        response = future.result()
        return response.success, response.message

    def list_wifi_connections(self) -> Tuple[bool, str, List[WifiConnection]]:
        list_client = self._service_clients.get("list_wifi_connections")
        if not list_client.wait_for_service(timeout_sec=5.0):
            return False, "list_wifi_connections service is not available", []

        future = list_client.call_async(ListWifiConnections.Request())
        # One 10 s list plus a short SSID read per profile.
        if not _wait_for_future(future, timeout=15.0):
            return False, "Timeout waiting for list_wifi_connections response", []

        response = future.result()
        if not response.success:
            return False, response.message, []

        return True, "", list(response.connections)

    def activate_wifi_connection(self, uuid: str) -> Tuple[bool, str, str]:
        """Rejoin a saved network; answers (success, message, ssid)."""
        activate_client = self._service_clients.get("activate_wifi_connection")
        if not activate_client.wait_for_service(timeout_sec=5.0):
            return False, "activate_wifi_connection service is not available", ""

        self._logger.info("[RobotGateway] Activating saved WiFi network", uuid=uuid)

        future = activate_client.call_async(ActivateWifiConnection.Request(uuid=uuid))
        # The service waits up to 60s for nmcli, as connect_wifi does.
        if not _wait_for_future(future, timeout=70.0):
            return False, "Timeout waiting for activate_wifi_connection response", ""

        response = future.result()
        return response.success, response.message, response.ssid

    def delete_wifi_connection(self, uuid: str) -> Tuple[bool, str]:
        delete_client = self._service_clients.get("delete_wifi_connection")
        if not delete_client.wait_for_service(timeout_sec=5.0):
            return False, "delete_wifi_connection service is not available"

        self._logger.info("[RobotGateway] Deleting saved WiFi network", uuid=uuid)

        future = delete_client.call_async(DeleteWifiConnection.Request(uuid=uuid))
        # A list to find the profile, then a 15 s delete.
        if not _wait_for_future(future, timeout=30.0):
            return False, "Timeout waiting for delete_wifi_connection response"

        response = future.result()
        return response.success, response.message

    def set_motion_key(self, key: MotionKey) -> Tuple[bool, str]:
        motion_key_client = self._service_clients.get("set_motion_key")
        if not motion_key_client.wait_for_service(timeout_sec=5.0):
            return False, "set_motion_key service is not available"

        self._logger.info("[RobotGateway] Setting motion key", key=key.value)

        # .value: the srv field is a plain string and rosidl stores whatever
        # it is handed.
        future = motion_key_client.call_async(SetMotionKey.Request(key=key.value))
        if not _wait_for_future(future, timeout=10.0):
            return False, "Timeout waiting for set_motion_key response"

        response = future.result()
        return response.success, response.message

    def set_policy_mode(self, mode: int) -> Tuple[bool, str]:
        """Switch the gait controller's policy (`MODE <uint>` over UDP).

        A plain int: the legal set is the gait controller's policy index, which
        this workspace does not own (see syncai_common/README.md); REST narrows
        it to the exposed indices.

        Fire-and-forget: the driver ignores sendto()'s return, validates
        nothing and is not gated by the safety lock. True means the datagram
        was written, not that the controller switched policy.
        """
        policy_mode_client = self._service_clients.get("set_policy_mode")
        if not policy_mode_client.wait_for_service(timeout_sec=5.0):
            return False, "set_policy_mode service is not available"

        self._logger.info("[RobotGateway] Setting policy mode", mode=mode)

        future = policy_mode_client.call_async(SetPolicyMode.Request(mode=mode))
        # Same 10.0 as set_motion_key: the callback cannot block, but ~1 s would
        # turn a discovery hiccup or a starved executor into a spurious 502.
        if not _wait_for_future(future, timeout=10.0):
            return False, "Timeout waiting for set_policy_mode response"

        response = future.result()
        return response.success, response.message

    def set_safety_lock(self, locked: bool) -> Tuple[bool, str]:
        """Engage (True) or release (False) the driver's software safety lock.

        Engaging only drops motion keys other than ESTOP and cmd_vel; it sends
        no lie-down. The driver answers success whether or not the state
        changed (never match its prose); the held state is
        RobotState.low_level_mode.safety_state.
        """
        safety_lock_client = self._service_clients.get("set_safety_lock")
        if not safety_lock_client.wait_for_service(timeout_sec=5.0):
            return False, "set_safety_lock service is not available"

        self._logger.info("[RobotGateway] Setting safety lock", locked=locked)

        future = safety_lock_client.call_async(SetBool.Request(data=locked))
        # Same 10.0 as its siblings: the callback cannot block.
        if not _wait_for_future(future, timeout=10.0):
            return False, "Timeout waiting for set_safety_lock response"

        response = future.result()
        return response.success, response.message

    def switch_mode(self, mode: int) -> Tuple[Optional[bool], str]:
        """Ask sys_manager to switch the operating mode. Three-valued on purpose.

        ``(True|False, msg)`` when sys_manager answered inside
        ``SWITCH_MODE_ACK_TIMEOUT``; ``(None, msg)`` otherwise, which is the
        EXPECTED outcome of a real switch: sys_manager kills the byobu session
        this backend runs in first. The console treats the dropped connection
        as the signal and polls robot_state. The short wait catches the fast
        answers ("Already in X; nothing to do", refusals).

        Takes the RobotMode uint8 (MANUAL=1 / AUTO=2), not a REST enum.
        """
        switch_mode_client = self._service_clients.get("switch_mode")
        if not switch_mode_client.wait_for_service(timeout_sec=5.0):
            return False, "switch_mode service is not available"

        self._logger.info("[RobotGateway] Switching operating mode", mode=mode)

        future = switch_mode_client.call_async(SwitchMode.Request(mode=mode))
        if not _wait_for_future(future, timeout=SWITCH_MODE_ACK_TIMEOUT):
            return None, "Mode switch dispatched; the stack is rebuilding"

        response = future.result()
        return response.success, response.message

    def restart_mode(
        self, on_done: Optional[Callable[[bool, str], None]] = None
    ) -> Tuple[Optional[bool], str]:
        """Ask sys_manager to rebuild the live mode's session. Three-valued.

        sys_manager answers only once the rebuild is over, so ``(None, msg)``
        is the EXPECTED outcome of one that goes ahead. Fast answers are
        refusals with nothing touched (MAINTENANCE, both sessions up, MANUAL
        with a possibly unsaved map — so in practice AUTO only), tagged
        ``Failure.RESTART_REFUSED``. An unreachable service stays untagged.

        ``on_done(success, message)`` gets the late outcome, once, on the
        executor thread, only when this returned ``None``. A late failure is
        untagged (a rebuild that went wrong, not a refusal). As a byobu pane
        the process dies first and it never runs.
        """
        restart_mode_client = self._service_clients.get("restart_mode")
        if not restart_mode_client.wait_for_service(timeout_sec=5.0):
            return False, "restart_mode service is not available"

        self._logger.info("[RobotGateway] Restarting the live operating mode")

        future = restart_mode_client.call_async(RestartMode.Request())
        if not _wait_for_future(future, timeout=SWITCH_MODE_ACK_TIMEOUT):
            if on_done is not None:
                future.add_done_callback(
                    lambda done: on_done(*self._late_restart_outcome(done))
                )
            return None, "Mode restart dispatched; the stack is rebuilding"

        response = future.result()
        if not response.success:
            return False, fail(Failure.RESTART_REFUSED, response.message)
        return True, response.message

    def _late_restart_outcome(self, future) -> Tuple[bool, str]:
        # Never raise out of a done callback: it runs on the executor thread,
        # and an exception there would end spin() and the process.
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 -- see above
            self._logger.error("[RobotGateway] restart_mode failed", error=str(exc))
            return False, "The restart did not report back."
        if response is None:
            return False, "The restart did not report back."
        return bool(response.success), response.message

    def _make_pose_stamped(
        self, x: float, y: float, yaw: float, z: float = 0.0
    ) -> PoseStamped:
        return PoseStamped(
            header=Header(frame_id="map", stamp=self._node.get_clock().now().to_msg()),
            pose=Pose(
                position=Point(x=x, y=y, z=z),
                orientation=Quaternion(
                    x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
                ),
            ),
        )

    def set_initial_pose(
        self, x: float, y: float, yaw: float, z: Optional[float] = None
    ) -> Tuple[bool, str]:
        """Seed localization with an operator-supplied map-frame pose.

        ``z`` is the height of the ground the pose stands on, as a vertex's z,
        which picks the storey; the localizer adds its lidar height. None keeps
        the storey the localizer is already on: the guess then takes its height
        from the current estimate, as RViz's does.

        Fire-and-forget on a volatile topic: no ack, and only already-discovered
        subscribers see it. Hence the warning when nobody is subscribed — an
        unreceived initialpose looks the same as an ignored one.
        """
        initial_pose_pub = self._publishers.get("initial_pose")

        if initial_pose_pub.get_subscription_count() == 0:
            self._logger.warning(
                "[RobotGateway] Publishing initialpose with no subscriber; "
                "is the localizer running?"
            )

        self._logger.info(
            "[RobotGateway] Publishing initial pose", x=x, y=y, z=z, yaw=yaw
        )

        pose_stamped = self._make_pose_stamped(
            x=x, y=y, yaw=yaw, z=0.0 if z is None else z
        )
        msg = PoseWithCovarianceStamped(header=pose_stamped.header)
        msg.pose.pose = pose_stamped.pose
        # The rest of the covariance stays zero: the localizer reads x / y /
        # yaw, and z only when marked (see applyPlanarGuess).
        if z is not None:
            msg.pose.covariance[INITIAL_POSE_Z_VARIANCE_INDEX] = INITIAL_POSE_Z_VARIANCE
        initial_pose_pub.publish(msg)

        return True, ""

    def teleop_cmd_vel(self, vx: float, vy: float, wz: float) -> Tuple[bool, str]:
        """Publish one teleop velocity command, clamped to [-1, 1] per axis.

        The clamped value goes on the wire as m/s and rad/s; a lower ceiling,
        if ever wanted, goes here.

        Refused while a MOVE goal is EXECUTING: the controller owns cmd_vel and
        there is no mux, so teleop would make the robot judder. No per-call
        logging: this runs at ~10 Hz.
        """
        if self._autonomous_move_active():
            return False, "autonomous move in progress"

        twist = Twist()
        twist.linear.x = _clamp_normalized(vx)
        twist.linear.y = _clamp_normalized(vy)
        twist.angular.z = _clamp_normalized(wz)

        self._publishers.get("cmd_vel").publish(twist)

        return True, ""

    def teleop_stop(self) -> None:
        """Publish a zero-velocity Twist — the teleop watchdog/disconnect stop.

        The driver manager has NO cmd_vel watchdog, so the backend owns
        stop-on-disconnect / stale input. Gated like teleop_cmd_vel, so zeros
        never fight the controller during a MOVE.
        """
        if self._autonomous_move_active():
            return

        self._publishers.get("cmd_vel").publish(Twist())

    def _autonomous_move_active(self) -> bool:
        with self._lock:
            return any(
                goal.state is MoveState.EXECUTING for goal in self._goals.values()
            )

    def navigation_active(self) -> bool:
        """Whether a navigation this process sent (a MOVE, DOCK or UNDOCK goal)
        is still executing.

        The telemetry stream's gate for the local costmap and the controller's
        local plan, which mean something only while the robot drives. Asked of
        the goals rather than read off the route on ``plan``: the planner keeps
        re-sending a route after a success (trimmed to a stub) and after a
        failure (on purpose, for the operator), and clears it only on a cancel.
        A goal sent from outside this process (rviz, the CLI) is not seen.
        """
        return self._autonomous_move_active()

    def _send_nav_goal(
        self, client_name: str, goal_msg
    ) -> Tuple[bool, str, Optional[str]]:
        client = self._action_clients.get(client_name)
        if not client.wait_for_server(timeout_sec=NAV_SERVER_WAIT_S):
            return False, "Action server is not available", None

        # The response callback (executor thread) registers the goal, so a goal
        # nav2 accepts after this thread stopped waiting (timeout, or a Temporal
        # cancel) is still known and can be cancelled. `pending` is the
        # handshake: whichever side is second learns what the first did.
        pending: dict = {"abandoned": False, "goal_id": None}
        future = client.send_goal_async(
            goal=goal_msg, feedback_callback=self._move_feedback_cb
        )
        future.add_done_callback(lambda fut: self._goal_response_cb(fut, pending))

        try:
            answered = _wait_for_future(future, timeout=NAV_ACCEPT_WAIT_S)
        except BaseException:
            # Typically Temporal's CancelledError: the goal must not keep running.
            self._disown_pending(pending)
            raise
        if not answered:
            self._disown_pending(pending)
            return False, "Timeout waiting for goal acceptance", None

        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            return False, "Goal rejected by task runner node", None

        return True, "", _goal_id_str(goal_handle)

    def _goal_response_cb(self, future, pending: dict) -> None:
        """Register an accepted goal. Runs on the executor thread; see _send_nav_goal."""
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._logger.error("[RobotGateway] Goal request failed", error=str(exc))
            return
        if goal_handle is None or not goal_handle.accepted:
            return

        goal_id = _goal_id_str(goal_handle)
        with self._lock:
            self._goals[goal_id] = MoveGoal(goal_id=goal_id, goal_handle=goal_handle)
            self._evict_finished_goals()
            pending["goal_id"] = goal_id
            abandoned = pending["abandoned"]

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, gid=goal_id: self._move_result_cb(gid, fut)
        )

        if abandoned:
            # The sender gave up before nav2 answered; nobody will poll this
            # goal. Fire-and-forget: the result callback records CANCELED.
            self._logger.warning(
                "[RobotGateway] Goal accepted after its sender gave up; cancelling",
                goal_id=goal_id,
            )
            goal_handle.cancel_goal_async()

    def _disown_pending(self, pending: dict) -> None:
        """The sender stopped waiting: cancel the goal if it already arrived,
        or arrange for the response callback to cancel it when it does."""
        with self._lock:
            pending["abandoned"] = True
            goal_id = pending["goal_id"]
            goal = self._goals.get(goal_id) if goal_id else None
        if goal is not None and goal.state is MoveState.EXECUTING:
            self._logger.warning(
                "[RobotGateway] Sender stopped waiting on an accepted goal; cancelling",
                goal_id=goal_id,
            )
            goal.goal_handle.cancel_goal_async()

    def _evict_finished_goals(self) -> None:
        """Trim _goals to MAX_TRACKED_GOALS, oldest finished first.

        Caller holds self._lock. Insertion order is age; EXECUTING goals are
        never evicted, even if that leaves the dict over the cap.
        """
        excess = len(self._goals) - MAX_TRACKED_GOALS
        if excess <= 0:
            return

        for goal_id in list(self._goals):
            if excess <= 0:
                break
            if self._goals[goal_id].state is MoveState.EXECUTING:
                continue
            del self._goals[goal_id]
            excess -= 1

    def move(
        self,
        x: float,
        y: float,
        yaw: float,
        z: float = 0.0,
        behavior_tree: str = "",
    ) -> Tuple[bool, str, Optional[str]]:
        """Send a navigation goal; ``z`` is the ground height of its storey."""
        self._logger.info(
            "[RobotGateway] Sending navigation goal ",
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            behavior_tree=behavior_tree,
        )

        goal_msg = NavigateToGoal.Goal(
            pose=self._make_pose_stamped(x=x, y=y, yaw=yaw, z=z),
            behavior_tree=behavior_tree,
        )

        return self._send_nav_goal("move", goal_msg)

    def _move_feedback_cb(self, feedback_msg):
        # rclpy passes the FeedbackMessage wrapper ({goal_id, feedback}).
        goal_id = _goal_id_str(feedback_msg)

        with self._lock:
            goal = self._goals.get(goal_id)
            if not goal:
                return

            goal.feedback = feedback_msg.feedback

    def _move_result_cb(self, goal_id: str, future):
        result = future.result()
        with self._lock:
            goal = self._goals.get(goal_id)
            if not goal:
                return

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                goal.state = MoveState.SUCCEEDED
            elif result.status == GoalStatus.STATUS_CANCELED:
                goal.state = MoveState.CANCELED
            elif result.status == GoalStatus.STATUS_ABORTED:
                goal.state = MoveState.ABORTED

            goal.result = result.result

    def get_move_status(self, goal_id: str) -> Optional[dict[str, Any]]:

        with self._lock:
            goal = self._goals.get(goal_id)
            if not goal:
                return

            status: dict[str, Any] = {
                "goal_id": goal.goal_id,
                "state": goal.state.value,
            }
            # The reason, on an abort only. NONE on an abort ("superseded or
            # never started") is meaningful, so it is passed on.
            if goal.state is MoveState.ABORTED and goal.result is not None:
                status["error_code"] = goal.result.error_code
                status["error_msg"] = goal.result.error_msg
            return status

    def get_move_message(self) -> str:
        """What the robot's navigator says it is doing, for a running MOVE.

        The executing goal's latest feedback status_msg, or "". Not keyed by
        goal id: the task-state read does not know it, and only one navigation
        runs at a time.
        """
        with self._lock:
            for goal in self._goals.values():
                if goal.state is MoveState.EXECUTING and goal.feedback is not None:
                    return goal.feedback.status_msg
            return ""

    def cancel_move(self, goal_id: str) -> Tuple[bool, str]:
        with self._lock:
            goal = self._goals.get(goal_id)
            if goal is None:
                return False, "Unknown goal id"
            if goal.state != MoveState.EXECUTING:
                return False, f"Goal already finished: {goal.state.value}"
            goal_handle = goal.goal_handle

        self._logger.info("[RobotGateway] Cancelling goal", goal_id=goal_id)

        cancel_future = goal_handle.cancel_goal_async()
        if not _wait_for_future(cancel_future, timeout=10.0):
            return False, "Timeout waiting for cancel response"

        response = cancel_future.result()
        if len(response.goals_canceling) == 0:
            return False, "Cancel rejected by server"

        return True, ""

    def cancel_active_moves(self) -> Tuple[bool, str]:
        """Cancel every goal still executing -- the cancelled-MOVE cleanup path.

        By state, not id: a Temporal cancel can land before move() returned an
        id. Only one goal runs at a time, so this is ours. A goal accepted
        after this ran is handled by _disown_pending.
        """
        with self._lock:
            active = [
                goal_id
                for goal_id, goal in self._goals.items()
                if goal.state is MoveState.EXECUTING
            ]
        if not active:
            return True, "no goal was executing"

        failures: List[str] = []
        for goal_id in active:
            success, message = self.cancel_move(goal_id=goal_id)
            if not success:
                failures.append(f"{goal_id}: {message}")
        return (not failures), "; ".join(failures)


def init_robot_gateway(
    logger: structlog.stdlib.BoundLogger, node: Node
) -> RobotGateway:
    robot_gw = RobotGateway(logger=logger, node=node)
    return robot_gw
