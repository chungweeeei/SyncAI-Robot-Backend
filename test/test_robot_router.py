"""Unit tests for the robot REST router: the pure helpers, GET state, the POSTs.

The router is mounted on a bare FastAPI app with the production exception
handlers registered, so the domain-exception -> status-code mapping under test is
the real one (UpstreamError -> 502). RobotRepo is real — it is a lock and a
slot — and only the gateway is stubbed, because a real one needs a live DDS graph
with syncai_driver_manager on it.

``GET /api/v1/robot/state`` had no coverage at all until a `RobotState` field was
restructured underneath it and nothing failed. The most valuable test here is
therefore not the happy path but ``test_state_payload_keys_are_exactly_the_whitelist``:
the router's field list is a whitelist, five documents say so, and until that test
existed nothing mechanical enforced it.
"""

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("syncai_common")
# The router imports RobotGateway, which pulls in rclpy, nav2_msgs and
# action_msgs; without these the module import would be an ImportError rather
# than a skip. httpx is TestClient's transport.
pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")
pytest.importorskip("httpx")

from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from syncai_common.msg import RobotMode, RobotStatus  # noqa: E402

from syncai_backend.gateways.failure import Failure, fail  # noqa: E402
from syncai_backend.gateways.robot.robot import MotionKey  # noqa: E402
from syncai_backend.exceptions import UpstreamError  # noqa: E402
from syncai_backend.interfaces.rest.routers.robot import (  # noqa: E402
    RobotNetworkStatus,
    init_robot_router,
    _mode_to_str,
    _parse_wifi_info,
)
from syncai_backend.interfaces.rest.server import (  # noqa: E402
    register_exception_handlers,
)
from syncai_backend.repositories.robot.robot import (  # noqa: E402
    ROBOT_STATE_MAX_AGE_S,
    init_robot_repo,
    status_name,
)
from syncai_backend.services.map_usage import JobGate  # noqa: E402
from syncai_backend.services.mode_restart import (  # noqa: E402
    init_mode_restart_service,
)


class _StubRobotGateway:
    """Records service calls instead of making them.

    The one thing this suite cannot make real: every method here needs a live
    driver_manager on a DDS graph. ``motion_keys`` / ``policy_modes`` are what
    the "did not reach the gateway" assertions read — an empty list is the claim.
    """

    def __init__(self):
        self.motion_keys = []
        self.policy_modes = []
        self.switched_modes = []
        self.result = (True, "Motion key sent")
        # Its own default rather than sharing `result`: the gateway's
        # switch_mode is three-valued, and None (dispatched, no answer inside
        # the ack window) is the outcome of every real switch.
        self.switch_result = (None, "Mode switch dispatched")
        self.restarts = 0
        self.restart_result = (None, "Mode restart dispatched")
        # The late-answer callback of the last dispatched restart: a test calls
        # it to play sys_manager answering once the rebuild is over.
        self.restart_done = None
        self.safety_locks = []
        self.safety_lock_result = (True, "Safety lock engaged. Remote control blocked.")
        self.initial_poses = []
        # The operator's switch: the RobotStatus uint8s asked for, and the
        # gateway's (success, message, resulting state) answer.
        self.state_changes = []
        self.state_result = (True, "Position confirmed.", RobotStatus.IDLE)

    def set_robot_state(self, state):
        self.state_changes.append(state)
        return self.state_result

    def set_motion_key(self, key):
        self.motion_keys.append(key)
        return self.result

    def set_policy_mode(self, mode):
        self.policy_modes.append(mode)
        return self.result

    def switch_mode(self, mode):
        self.switched_modes.append(mode)
        return self.switch_result

    def set_safety_lock(self, locked):
        self.safety_locks.append(locked)
        return self.safety_lock_result

    def set_initial_pose(self, x, y, yaw, z=None):
        self.initial_poses.append({"x": x, "y": y, "yaw": yaw, "z": z})
        return True, ""

    def restart_mode(self, on_done=None):
        self.restarts += 1
        if self.restart_result[0] is None:
            self.restart_done = on_done
        return self.restart_result


class _StubSafetyLockService:
    """Answers engage_requested as told; counts the calls."""

    def __init__(self):
        self.engage_requests = 0
        self.edge = True

    def engage_requested(self):
        self.engage_requests += 1
        return self.edge


class _StubWorkflowGateway:
    """What is running, for the job gate in front of the state switch and the
    initial pose. Two reads, counted apart, because which one a route uses is
    the point: ``list_active_tasks`` is the gateway's 1.5 s cache,
    ``running_task_ids`` a fresh read of Temporal. ``error`` stands for an
    outage, which the gate must not read as "nothing is running".
    """

    def __init__(self):
        self.running = []
        self.error = None
        self.cached_reads = 0
        self.fresh_reads = 0

    async def list_active_tasks(self):
        self.cached_reads += 1
        if self.error is not None:
            raise self.error
        return [SimpleNamespace(id=task_id) for task_id in self.running], [], None

    async def running_task_ids(self):
        self.fresh_reads += 1
        if self.error is not None:
            raise self.error
        return list(self.running)


@pytest.fixture
def robot_gw():
    return _StubRobotGateway()


@pytest.fixture
def workflow_gw():
    return _StubWorkflowGateway()


@pytest.fixture
def safety_lock_svc():
    return _StubSafetyLockService()


class _Clock:
    """RobotRepo's clock, moved by hand: a sample ages when a test says so."""

    def __init__(self):
        self.now = 500.0

    def __call__(self):
        return self.now


@pytest.fixture
def clock():
    return _Clock()


@pytest.fixture
def robot_repo(logger, clock):
    """A real, empty RobotRepo. Populate it with `robot_repo.update_robot_state`.

    Separate from the `client` fixture so a test can seed a state sample before
    the request; the POST tests leave it empty, which is also the 404 case. On
    the `clock` fixture, which stands still unless a test moves it: a seeded
    sample stays fresh however slow the test machine is.
    """
    return init_robot_repo(logger=logger, now=clock)


@pytest.fixture
def client(logger, robot_repo, robot_gw, safety_lock_svc, workflow_gw):
    app = FastAPI()
    register_exception_handlers(app)
    app.include_router(
        init_robot_router(
            logger=logger,
            robot_repo=robot_repo,
            robot_gw=robot_gw,
            restart_svc=init_mode_restart_service(logger=logger, robot_gw=robot_gw),
            safety_lock_svc=safety_lock_svc,
            # Real, over the stub: the 409s under test are the gate's own. The
            # catalogue is only read by the per-map gate, which no route here uses.
            job_gate=JobGate(
                logger=logger, map_catalog_repo=MagicMock(), workflow_gw=workflow_gw
            ),
        )
    )
    return TestClient(app)


def _get_state(client):
    return client.get("/api/v1/robot/state")


def _post_motion_key(client, payload):
    return client.post("/api/v1/robot/set_motion_key", json=payload)


def _post_policy_mode(client, payload):
    return client.post("/api/v1/robot/set_policy_mode", json=payload)


# --- helpers ----------------------------------------------------------------


def test_mode_to_str_known_modes():
    assert _mode_to_str(RobotMode.AUTO) == "AUTO"
    assert _mode_to_str(RobotMode.MANUAL) == "MANUAL"
    assert _mode_to_str(RobotMode.MAINTENANCE) == "MAINTENANCE"


def test_mode_to_str_unknown_mode():
    assert _mode_to_str(255) == "UNKNOWN"


@pytest.mark.parametrize(
    "status,label",
    [
        (RobotStatus.UNINITIALIZED, "UNINITIALIZED"),
        (RobotStatus.IDLE, "IDLE"),
        (RobotStatus.RUNNING, "RUNNING"),
        (RobotStatus.WARNING, "WARNING"),
        (RobotStatus.ERROR, "ERROR"),
        (RobotStatus.CHARGING, "CHARGING"),
        (255, "UNKNOWN"),
    ],
)
def test_status_name(status, label):
    # The table lives beside RobotRepo since the task guard reads the same
    # names; this route is one of its three readers.
    assert status_name(status) == label


def test_parse_wifi_info_valid_json():
    info = _parse_wifi_info('{"ssid": "net", "rssi": -40, "ip_address": "10.0.0.2"}')

    assert isinstance(info, RobotNetworkStatus)
    assert info.ssid == "net"
    assert info.rssi == -40
    assert info.ip_address == "10.0.0.2"


@pytest.mark.parametrize("payload", ["null", "not-json", "", "[1, 2, 3]"])
def test_parse_wifi_info_falls_back_to_defaults(payload):
    info = _parse_wifi_info(payload)

    assert isinstance(info, RobotNetworkStatus)
    assert info.ssid == ""
    assert info.rssi == 0


# --- GET /api/v1/robot/state -------------------------------------------------


def test_state_404_before_any_sample(client):
    """The frontend gates its whole dashboard on this 404."""
    response = _get_state(client)

    assert response.status_code == 404
    assert "not available" in response.json()["detail"]
    # The same sentence as ever, with a code beside it since the route grew a
    # second 404 a client has to tell from this one.
    assert response.json() == {
        "detail": "Robot state is not available yet.",
        "code": "robot_state_unavailable",
    }


def test_state_404_once_the_robot_has_stopped_reporting(
    client, robot_repo, clock, make_robot_state
):
    """A sample that aged out is not served as the robot's state: the console
    draws a state chip from this route, and IDLE from a robot whose state
    node died an hour ago would read as live. Same body shape as the 404
    above, told apart by `code`, with how long it has been."""
    robot_repo.update_robot_state(state=make_robot_state(state=RobotStatus.IDLE))
    clock.now += 7.4

    response = _get_state(client)

    assert response.status_code == 404
    assert response.json() == {
        "detail": (
            "The robot has not reported its state for 7 seconds. Check that "
            "the robot's software is running."
        ),
        "code": "robot_state_stale",
    }


def test_state_is_served_up_to_the_age_limit(client, robot_repo, clock, make_robot_state):
    robot_repo.update_robot_state(state=make_robot_state())
    clock.now += ROBOT_STATE_MAX_AGE_S

    assert _get_state(client).status_code == 200


def test_state_comes_back_with_the_next_sample(client, robot_repo, clock, make_robot_state):
    robot_repo.update_robot_state(state=make_robot_state())
    clock.now += 60.0
    assert _get_state(client).status_code == 404

    robot_repo.update_robot_state(state=make_robot_state(state=RobotStatus.UNINITIALIZED))

    response = _get_state(client)
    assert response.status_code == 200
    assert response.json()["state"] == "UNINITIALIZED"


def test_state_projects_the_message(client, robot_repo, make_robot_state):
    robot_repo.update_robot_state(state=make_robot_state())

    body = _get_state(client).json()

    assert body["robot_id"] == "robot01"
    assert body["map"] == "dp2f"
    assert body["timestamp"] == 1754000000
    assert body["mode"] == "AUTO"
    assert body["state"] == "IDLE"
    assert body["localization_status"]["position"]["x"] == pytest.approx(1.5)
    assert body["localization_status"]["velocity"] == pytest.approx(0.25)
    assert body["network_status"]["ssid"] == "net"
    # 0-100 float on the wire, int in the payload.
    assert body["battery_status"]["battery_percentage"] == 87


def test_state_converts_yaw_to_degrees(client, robot_repo, make_robot_state):
    """Radians in the message, degrees in the REST vocabulary."""
    robot_repo.update_robot_state(
        state=make_robot_state(position=(0.0, 0.0, 0.0, math.pi / 2))
    )

    theta = _get_state(client).json()["localization_status"]["position"]["theta"]

    assert theta == pytest.approx(90.0)


def test_state_trims_each_motor_to_health_fields(client, robot_repo, make_robot_state):
    """q / dq are set to non-zero in the factory precisely so a leak would show."""
    robot_repo.update_robot_state(
        state=make_robot_state(motors=(("FL_HipX_joint", 41, 0), ("FR_Knee_joint", 55, 7)))
    )

    motors = _get_state(client).json()["motor_status"]

    assert [m["name"] for m in motors] == ["FL_HipX_joint", "FR_Knee_joint"]
    assert motors[1]["temperature"] == 55
    assert motors[1]["error"] == 7
    assert set(motors[0]) == {"name", "temperature", "error"}


def test_state_reports_charging(client, robot_repo, make_robot_state):
    robot_repo.update_robot_state(state=make_robot_state(state=RobotStatus.CHARGING))

    assert _get_state(client).json()["state"] == "CHARGING"


def test_state_degrades_an_unknown_status_code(client, robot_repo, make_robot_state):
    """A code outside RobotStatus is a label, not a 500."""
    robot_repo.update_robot_state(state=make_robot_state(state=99))

    response = _get_state(client)

    assert response.status_code == 200
    assert response.json()["state"] == "UNKNOWN"


def test_state_reports_the_low_level_mode(client, robot_repo, make_robot_state):
    robot_repo.update_robot_state(
        state=make_robot_state(policy_state=1, motion_state=1)
    )

    low = _get_state(client).json()["low_level_mode"]

    # Labels only: the controller's raw integers stay on the ROS topic.
    assert low == {"policy": "HIMLOCO", "motion": "LOCOMOTION", "safety_locked": False}


def test_state_reports_an_engaged_safety_lock(client, robot_repo, make_robot_state):
    robot_repo.update_robot_state(state=make_robot_state(safety_state=True))

    assert _get_state(client).json()["low_level_mode"]["safety_locked"] is True


@pytest.mark.parametrize(
    "policy_state,motion_state,policy,motion",
    [
        # CHAMP and ISSAC are real controller policies the command surface
        # deliberately refuses. Decoding them through the PolicyMode enum would
        # raise a validation error and 500 the whole endpoint -- this is the test
        # that keeps the reverse-map fallback in place.
        (2, 0, "CHAMP", "STAND"),
        (3, 4, "ISSAC", "ESTOP"),
        # 8 is IDLE and has to stay its own label: the console ends a commanded
        # MPC on a return to IDLE, which only works if IDLE is not the same
        # string as the UNKNOWN that MPC's out-of-table code degrades to.
        (0, 8, "PPO", "IDLE"),
        (0, 6, "PPO", "UNKNOWN"),
        (99, 0, "UNKNOWN", "STAND"),
    ],
)
def test_state_degrades_unknown_low_level_codes(
    client, robot_repo, make_robot_state, policy_state, motion_state, policy, motion
):
    robot_repo.update_robot_state(
        state=make_robot_state(policy_state=policy_state, motion_state=motion_state)
    )

    response = _get_state(client)

    # A 200 is the whole point: decoding through the PolicyMode enum instead of
    # the fallback map would raise a validation error here and take the endpoint
    # down for a robot that is running perfectly well.
    assert response.status_code == 200
    low = response.json()["low_level_mode"]
    assert (low["policy"], low["motion"]) == (policy, motion)


def test_state_payload_keys_are_exactly_the_whitelist(
    client, robot_repo, make_robot_state
):
    """The response is a whitelist, and this is the only thing enforcing it.

    Five documents say a field added to RobotState must not appear here until
    somebody decides it should. Before this test, nothing but those comments stood
    between an operator-facing field and a frozen third-party contract.

    If this fails because you *meant* to expose something, widen the set here in
    the same commit — that is the deliberate act the rule asks for.
    """
    robot_repo.update_robot_state(state=make_robot_state())

    body = _get_state(client).json()

    assert set(body) == {
        "timestamp",
        "robot_id",
        "map",
        "mode",
        "state",
        "state_message",
        "low_level_mode",
        "localization_valid",
        "localization_converged",
        "localization_status",
        "network_status",
        "battery_status",
        "motor_status",
    }
    # Named individually as well, because this is what the message carries and
    # the payload must keep holding back. (localization_valid and state used to
    # be held back too; each was widened in deliberately — localization_valid
    # when the subscriber stopped dropping unlocalized samples, state when
    # syncai_robot_state began deriving CHARGING, and state_message /
    # localization_converged when it took the robot's state machine: the
    # console's Confirm control cannot work without them.)
    assert "timestamp" not in body["motor_status"][0]


def test_state_reports_an_unlocalized_sample(client, robot_repo, make_robot_state):
    """The mapping-mode shape: sample stored, pose labelled as a placeholder.

    The subscriber used to drop localization_valid=false samples, which made
    the whole endpoint 404 for an entire mapping run — mode, battery and the
    gait state included. The pose-honesty half of that trade now rides on the
    flag instead.
    """
    robot_repo.update_robot_state(
        state=make_robot_state(
            mode=1,  # RobotMode.MANUAL
            localization_valid=False,
            position=(0.0, 0.0, 0.0, 0.0),
        )
    )

    body = _get_state(client).json()

    assert body["mode"] == "MANUAL"
    assert body["localization_valid"] is False
    assert body["localization_status"]["position"]["x"] == 0.0


def test_state_falls_back_when_wifi_info_is_null(client, robot_repo, make_robot_state):
    """syncai_robot_state dumps an empty json object as the literal "null"."""
    robot_repo.update_robot_state(state=make_robot_state(wifi_info="null"))

    network = _get_state(client).json()["network_status"]

    assert network["ssid"] == ""
    assert network["rssi"] == 0


# --- POST /api/v1/robot/set_motion_key --------------------------------------


@pytest.mark.parametrize("key", ["0", "1", "2", "3", "5"])
def test_each_forwardable_key_reaches_the_gateway(client, robot_gw, key):
    response = _post_motion_key(client, {"key": key})

    assert response.status_code == 200
    assert response.json()["sent"] is True
    assert robot_gw.motion_keys == [MotionKey(key)]
    # A MotionKey member, not the bare string: the gateway reads `key.value`, so
    # handing it a str would raise inside it. This is what pins that contract.
    assert isinstance(robot_gw.motion_keys[0], MotionKey)


def test_estop_key_is_accepted_but_not_forwarded(client, robot_gw):
    response = _post_motion_key(client, {"key": "4"})

    assert response.status_code == 200
    body = response.json()
    assert body["key"] == "4"
    assert body["sent"] is False
    assert robot_gw.motion_keys == []


def test_estop_response_says_no_estop_was_sent(client):
    """The point is that this 200 cannot be misread as "stop engaged"."""
    message = _post_motion_key(client, {"key": "4"}).json()["message"]

    assert "no ESTOP" in message


@pytest.mark.parametrize("payload", [{"key": "9"}, {"key": "z"}, {"key": ""}, {}])
def test_invalid_motion_key_is_422_before_the_gateway(client, robot_gw, payload):
    assert _post_motion_key(client, payload).status_code == 422
    assert robot_gw.motion_keys == []


def test_motion_key_gateway_failure_is_502_with_the_message_verbatim(client, robot_gw):
    """LOCKED is the driver's own string; the operator has to see it.

    Unreachable on hardware today — nothing calls triggerSafeShutdown(), so
    safe_lock_ is never set. This pins the router's mapping, not an observed
    driver behaviour.
    """
    robot_gw.result = (False, "LOCKED")

    response = _post_motion_key(client, {"key": "0"})

    assert response.status_code == 502
    assert response.json()["detail"] == "LOCKED"


# --- POST /api/v1/robot/estop -----------------------------------------------


def _post_estop(client, locked):
    return client.post("/api/v1/robot/estop", json={"locked": locked})


def test_estop_engaging_cancels_on_the_edge(client, robot_gw, safety_lock_svc):
    response = _post_estop(client, True)

    assert response.status_code == 200
    assert robot_gw.safety_locks == [True]
    assert safety_lock_svc.engage_requests == 1
    assert response.json()["locked"] is True
    assert response.json()["cancel_requested"] is True


def test_estop_engaging_an_engaged_lock_cancels_nothing(
    client, robot_gw, safety_lock_svc
):
    safety_lock_svc.edge = False

    response = _post_estop(client, True)

    assert response.status_code == 200
    assert response.json()["cancel_requested"] is False


def test_estop_release_never_cancels(client, robot_gw, safety_lock_svc):
    robot_gw.safety_lock_result = (True, "Safety lock released. Remote control restored.")

    response = _post_estop(client, False)

    assert response.status_code == 200
    assert robot_gw.safety_locks == [False]
    assert safety_lock_svc.engage_requests == 0
    assert response.json()["cancel_requested"] is False


def test_estop_gateway_failure_is_502_and_cancels_nothing(
    client, robot_gw, safety_lock_svc
):
    robot_gw.safety_lock_result = (False, "set_safety_lock service is not available")

    response = _post_estop(client, True)

    assert response.status_code == 502
    assert response.json()["detail"] == "set_safety_lock service is not available"
    # A lock that never engaged must not stop anyone's task.
    assert safety_lock_svc.engage_requests == 0


def test_estop_requires_the_lock_state(client, robot_gw):
    response = client.post("/api/v1/robot/estop", json={})

    assert response.status_code == 422
    assert robot_gw.safety_locks == []


# --- POST /api/v1/robot/set_policy_mode -------------------------------------


@pytest.mark.parametrize("mode", [0, 1])
def test_exposed_policy_modes_reach_the_gateway(client, robot_gw, mode):
    response = _post_policy_mode(client, {"mode": mode})

    assert response.status_code == 200
    assert response.json()["mode"] == mode
    # A plain int, which is the gateway's contract and what keeps the srv field a
    # bare uint8.
    assert robot_gw.policy_modes == [mode]


@pytest.mark.parametrize("mode", [2, 3, 300, -1, "PPO"])
def test_unexposed_or_illegal_policy_mode_is_422(client, robot_gw, mode):
    """2 (CHAMP) and 3 (ISSAC) are legal on the controller but not exposed."""
    assert _post_policy_mode(client, {"mode": mode}).status_code == 422
    assert robot_gw.policy_modes == []


def test_policy_mode_gateway_failure_is_502(client, robot_gw):
    robot_gw.result = (False, "set_policy_mode service is not available")

    response = _post_policy_mode(client, {"mode": 1})

    assert response.status_code == 502
    assert response.json()["detail"] == "set_policy_mode service is not available"


# --- POST /api/v1/robot/mode -------------------------------------------------


def _post_mode(client, payload):
    return client.post("/api/v1/robot/mode", json=payload)


def test_dispatched_switch_reports_switching(client, robot_gw):
    """The gateway's None — every real switch — is a 200 with switching=true.

    In production the client rarely reads this body (the switch kills the
    process serving it); the test pins the contract for the cases where it
    does arrive.
    """
    response = _post_mode(client, {"mode": "MANUAL"})

    assert response.status_code == 200
    body = response.json()
    assert body["mode"] == "MANUAL"
    assert body["switching"] is True
    # The wire value is the RobotMode uint8, not the REST string.
    assert robot_gw.switched_modes == [RobotMode.MANUAL]


def test_noop_switch_reports_not_switching(client, robot_gw):
    robot_gw.switch_result = (True, "Already in AUTO; nothing to do")

    response = _post_mode(client, {"mode": "AUTO"})

    assert response.status_code == 200
    assert response.json()["switching"] is False
    assert robot_gw.switched_modes == [RobotMode.AUTO]


def test_switch_refusal_is_502_with_the_message_verbatim(client, robot_gw):
    robot_gw.switch_result = (False, "switch_mode service is not available")

    response = _post_mode(client, {"mode": "AUTO"})

    assert response.status_code == 502
    assert response.json()["detail"] == "switch_mode service is not available"


@pytest.mark.parametrize("mode", ["MAINTENANCE", "auto", 2, None])
def test_unswitchable_mode_is_422_before_the_gateway(client, robot_gw, mode):
    """MAINTENANCE is derivable but not a switch target; ints are not the API."""
    assert _post_mode(client, {"mode": mode}).status_code == 422
    assert robot_gw.switched_modes == []


# --- POST /api/v1/robot/restart ----------------------------------------------


def _post_restart(client):
    return client.post("/api/v1/robot/restart")


def test_dispatched_restart_reports_restarting(client, robot_gw):
    response = _post_restart(client)

    assert response.status_code == 200
    assert response.json()["restarting"] is True
    assert robot_gw.restarts == 1


def test_restart_answered_in_the_window_reports_not_restarting(client, robot_gw):
    robot_gw.restart_result = (True, "Restarted AUTO (session auto)")

    response = _post_restart(client)

    assert response.status_code == 200
    assert response.json() == {
        "restarting": False,
        "message": "Restarted AUTO (session auto)",
    }


def test_restart_refusal_is_409_with_its_code(client, robot_gw):
    """sys_manager's refusals are the robot's state, not a broken upstream."""
    refusal = "restart_mode is not available in MANUAL: ..."
    robot_gw.restart_result = (False, fail(Failure.RESTART_REFUSED, refusal))

    response = _post_restart(client)

    assert response.status_code == 409
    assert response.json()["detail"] == refusal
    assert response.json()["code"] == "restart_refused"


def test_restart_unreachable_is_502(client, robot_gw):
    robot_gw.restart_result = (False, "restart_mode service is not available")

    response = _post_restart(client)

    assert response.status_code == 502
    assert response.json()["detail"] == "restart_mode service is not available"


def _get_restart(client):
    return client.get("/api/v1/robot/restart")


def test_restart_status_is_idle_before_any_restart(client):
    response = _get_restart(client)

    assert response.status_code == 200
    assert response.json() == {
        "status": "idle",
        "message": "",
        "started_at": None,
        "finished_at": None,
    }


def test_dispatched_restart_reads_back_as_restarting_until_it_answers(
    client, robot_gw
):
    """The POST cannot wait for the rebuild; GET is where its end is reported."""
    _post_restart(client)

    running = _get_restart(client).json()
    assert running["status"] == "restarting"
    assert running["started_at"] is not None
    assert running["finished_at"] is None

    robot_gw.restart_done(True, "Restarted AUTO (session auto)")

    done = _get_restart(client).json()
    assert done["status"] == "succeeded"
    assert done["message"] == "Restarted AUTO (session auto)"
    assert done["finished_at"] is not None


def test_a_rebuild_that_went_wrong_reads_back_as_failed(client, robot_gw):
    _post_restart(client)
    robot_gw.restart_done(False, "Tried to restart AUTO but ended up in MAINTENANCE")

    done = _get_restart(client).json()
    assert done["status"] == "failed"
    assert "MAINTENANCE" in done["message"]


def test_a_second_press_while_restarting_is_409(client, robot_gw):
    _post_restart(client)

    response = _post_restart(client)

    assert response.status_code == 409
    assert response.json()["code"] == "restart_running"
    assert robot_gw.restarts == 1


def test_a_refused_restart_leaves_no_record(client, robot_gw):
    """Nothing was touched, so there is nothing to report as under way."""
    robot_gw.restart_result = (False, fail(Failure.RESTART_REFUSED, "MANUAL"))

    _post_restart(client)

    assert _get_restart(client).json()["status"] == "idle"


def test_a_restart_answered_in_the_window_reads_back_as_succeeded(client, robot_gw):
    robot_gw.restart_result = (True, "Restarted AUTO (session auto)")

    _post_restart(client)

    assert _get_restart(client).json()["status"] == "succeeded"


# --- POST /api/v1/robot/set_initial_pose -------------------------------------


def _post_initial_pose(client, payload):
    return client.post("/api/v1/robot/set_initial_pose", json=payload)


def test_an_initial_pose_without_z_keeps_the_localizers_storey(client, robot_gw):
    """No z reaches the gateway as None, never as 0.

    0 is a height (the ground floor); None is "keep the storey you are on",
    which is all a client from before z could have meant.
    """
    response = _post_initial_pose(client, {"x": 1.0, "y": -2.0, "theta": 90.0})

    assert response.status_code == 200
    assert robot_gw.initial_poses == [
        {"x": 1.0, "y": -2.0, "yaw": pytest.approx(math.pi / 2), "z": None}
    ]


def test_an_initial_pose_on_another_storey_carries_its_z(client, robot_gw):
    response = _post_initial_pose(
        client, {"x": 1.0, "y": -2.0, "theta": 0.0, "z": 3.2}
    )

    assert response.status_code == 200
    assert robot_gw.initial_poses[0]["z"] == 3.2
    assert "z=3.2" in response.json()["message"]


# --- GET /api/v1/robot/state: the state machine's fields ----------------------


@pytest.mark.parametrize(
    "status,label",
    [(RobotStatus.RUNNING, "RUNNING"), (RobotStatus.ERROR, "ERROR")],
)
def test_state_reports_the_states_the_robot_now_derives(
    client, robot_repo, make_robot_state, status, label
):
    robot_repo.update_robot_state(state=make_robot_state(state=status))

    assert _get_state(client).json()["state"] == label


def test_state_carries_the_robots_sentence_and_the_localizers_verdict(
    client, robot_repo, make_robot_state
):
    # A boot guess that converged: the precondition for confirming holds, and
    # the robot is still UNINITIALIZED until an operator has looked.
    robot_repo.update_robot_state(
        state=make_robot_state(
            state=RobotStatus.UNINITIALIZED,
            state_message="The position has not been confirmed.",
            localization_converged=True,
        )
    )

    body = _get_state(client).json()

    assert body["state"] == "UNINITIALIZED"
    assert body["state_message"] == "The position has not been confirmed."
    assert body["localization_converged"] is True


def test_state_reports_a_localizer_that_has_not_converged(
    client, robot_repo, make_robot_state
):
    robot_repo.update_robot_state(state=make_robot_state(localization_converged=False))

    assert _get_state(client).json()["localization_converged"] is False


# --- RobotRepo: what the task guard reads off the same sample ------------------


def test_the_repo_answers_no_status_before_any_sample(robot_repo):
    # None, not UNINITIALIZED: "nobody is saying" is its own answer, which the
    # dispatch's advisory check lets through (begin_task decides) and the
    # check before a navigation goal holds back retryably -- where
    # UNINITIALIZED would be a non-retryable refusal on a backend that merely
    # came up before the robot's state node. The ageing is test_robot_repo.py's.
    assert robot_repo.status() is None
    assert robot_repo.state_message() == ""


def test_the_repo_names_the_latest_samples_state(robot_repo, make_robot_state):
    robot_repo.update_robot_state(
        state=make_robot_state(
            state=RobotStatus.ERROR, state_message="Task morning-round failed."
        )
    )

    assert robot_repo.status() == "ERROR"
    assert robot_repo.state_message() == "Task morning-round failed."


def test_the_repo_degrades_an_unknown_code_to_a_name(robot_repo, make_robot_state):
    # A code a newer interface adds must not fail a task on a lookup; UNKNOWN
    # is neither ERROR nor UNINITIALIZED, so the guard lets it through and the
    # robot's own begin_task answers.
    robot_repo.update_robot_state(state=make_robot_state(state=99))

    assert robot_repo.status() == "UNKNOWN"


# --- POST /api/v1/robot/state --------------------------------------------------


def _post_state(client, payload):
    return client.post("/api/v1/robot/state", json=payload)


@pytest.mark.parametrize(
    "target,sent",
    [("IDLE", RobotStatus.IDLE), ("UNINITIALIZED", RobotStatus.UNINITIALIZED)],
)
def test_the_operators_switch_sends_the_status_constant(client, robot_gw, target, sent):
    response = _post_state(client, {"state": target})

    assert response.status_code == 200
    assert robot_gw.state_changes == [sent]


def test_the_answer_is_the_state_the_robot_reports_and_its_sentence(client, robot_gw):
    # The rollup, not the state asked for: confirmed on the charger.
    robot_gw.state_result = (True, "Position confirmed.", RobotStatus.CHARGING)

    response = _post_state(client, {"state": "IDLE"})

    assert response.json() == {"state": "CHARGING", "message": "Position confirmed."}


@pytest.mark.parametrize(
    "payload",
    [
        {"state": "RUNNING"},
        {"state": "ERROR"},
        {"state": "CHARGING"},
        {"state": "WARNING"},
        {"state": "idle"},
        {"state": 1},
        {},
    ],
)
def test_only_the_two_commandable_states_are_accepted(client, robot_gw, payload):
    # RUNNING, ERROR, WARNING and CHARGING are derived by the robot, never
    # commanded; the schema stops them before anything is asked.
    response = _post_state(client, payload)

    assert response.status_code == 422
    assert robot_gw.state_changes == []


def test_the_switch_is_refused_while_a_job_runs(client, robot_gw, workflow_gw):
    # The robot would not refuse it: either target clears the task it counts
    # as running, whose end_task then matches nothing.
    workflow_gw.running = ["task-7"]

    response = _post_state(client, {"state": "UNINITIALIZED"})

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "task-7" in response.json()["detail"]
    assert robot_gw.state_changes == []


def test_the_switch_asks_temporal_now_not_the_cache(client, workflow_gw):
    # Nothing stands behind this gate, so a 1.5 s old "idle" is not good enough.
    _post_state(client, {"state": "IDLE"})

    assert (workflow_gw.fresh_reads, workflow_gw.cached_reads) == (1, 0)


def test_the_switch_is_refused_when_temporal_cannot_answer(client, robot_gw, workflow_gw):
    workflow_gw.error = UpstreamError("List active tasks failed")

    response = _post_state(client, {"state": "IDLE"})

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert robot_gw.state_changes == []


def test_a_robot_that_has_not_converged_is_409_with_its_own_sentence(client, robot_gw):
    robot_gw.state_result = (
        False,
        fail(Failure.NOT_CONVERGED, "The localizer has not converged on the map yet."),
        RobotStatus.UNINITIALIZED,
    )

    response = _post_state(client, {"state": "IDLE"})

    assert response.status_code == 409
    assert response.json() == {
        "detail": "The localizer has not converged on the map yet.",
        "code": "not_converged",
    }


def test_any_other_refusal_is_409_robot_state_refused(client, robot_gw):
    robot_gw.state_result = (
        False,
        fail(Failure.ROBOT_STATE_REFUSED, "That state cannot be commanded."),
        RobotStatus.ERROR,
    )

    response = _post_state(client, {"state": "IDLE"})

    assert response.status_code == 409
    assert response.json()["code"] == "robot_state_refused"
    assert response.json()["detail"] == "That state cannot be commanded."


def test_an_unreachable_state_service_is_502(client, robot_gw):
    robot_gw.state_result = (False, "The robot's state service is not available.", None)

    response = _post_state(client, {"state": "IDLE"})

    assert response.status_code == 502
    assert "code" not in response.json()


# --- POST /api/v1/robot/set_initial_pose: the job gate -------------------------


def test_an_initial_pose_is_refused_while_a_job_runs(client, robot_gw, workflow_gw):
    # A new pose withdraws the operator's confirmation, so the running task's
    # next navigation step would fail and latch ERROR.
    workflow_gw.running = ["task-7"]

    response = _post_initial_pose(client, {"x": 1.0, "y": -2.0, "theta": 90.0})

    assert response.status_code == 409
    assert response.json()["code"] == "task_running"
    assert "setting the initial pose" in response.json()["detail"]
    assert robot_gw.initial_poses == []


def test_an_initial_pose_is_refused_when_temporal_cannot_answer(
    client, robot_gw, workflow_gw
):
    workflow_gw.error = UpstreamError("List active tasks failed")

    response = _post_initial_pose(client, {"x": 1.0, "y": -2.0, "theta": 90.0})

    assert response.status_code == 409
    assert response.json()["code"] == "tasks_unknown"
    assert robot_gw.initial_poses == []


def test_an_initial_pose_reads_the_same_cache_as_the_map_gates(client, workflow_gw):
    # The navigation step's own position check stands behind this one, as
    # execute_move's map check stands behind the map routes' gate.
    _post_initial_pose(client, {"x": 1.0, "y": -2.0, "theta": 90.0})

    assert (workflow_gw.cached_reads, workflow_gw.fresh_reads) == (1, 0)
