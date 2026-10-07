import structlog
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from syncai_common.msg import RobotState as RobotStateMsg

from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.services.safety_lock import SafetyLockService


class RobotStateSubscriber:
    """Feeds RobotRepo, and therefore GET /api/v1/robot/state, from the
    ``robot_state`` topic.

    ``RobotState`` carries more than that REST payload exposes — ``motor_status``
    reaches it flattened to its ``states`` array and trimmed to
    name/temperature/error, while that field's own ``timestamp`` and the
    message-level ``state`` do not reach it at all. The router's field list is a
    whitelist and must stay one; this subscriber hands the whole message through
    unfiltered.

    Every sample is stored, including the ones with ``localization_valid=false``
    and a zeroed pose. This subscriber used to drop those so the endpoint kept
    404-ing instead of reporting the robot parked on the map origin — which also
    made battery, the gait controller's state and above all ``mode`` unreadable
    exactly when they matter: before relocalization in nav, and for the ENTIRE
    mapping run, whose TF chain never reaches base_link at all (the mapping
    console was blind because of this). The pose-honesty half of that trade is
    kept by exposing ``localization_valid`` in the payload instead, so a zeroed
    pose arrives labelled rather than not arriving.

    Every sample's ``low_level_mode.safety_state`` also goes to
    SafetyLockService, which cancels every task on the lock's rising edge.
    Best-effort QoS is fine for that: the edge is judged against the last
    value seen, so a dropped sample only delays it to the next one.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        robot_repo: RobotRepo,
        safety_lock_svc: SafetyLockService,
    ):
        self._logger = logger
        self._robot_repo = robot_repo
        self._safety_lock_svc = safety_lock_svc

    def register(self, node: Node):

        self._robot_state_sub = node.create_subscription(
            msg_type=RobotStateMsg,
            topic="robot_state",
            callback=self._robot_state_cb,
            qos_profile=QoSProfile(
                depth=3,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
        )

    def _robot_state_cb(self, msg: RobotStateMsg):
        self._robot_repo.update_robot_state(state=msg)
        self._safety_lock_svc.observe(engaged=msg.low_level_mode.safety_state)


def init_robot_state_subscriber(
    logger: structlog.stdlib.BoundLogger,
    node: Node,
    robot_repo: RobotRepo,
    safety_lock_svc: SafetyLockService,
) -> RobotStateSubscriber:
    robot_state_subscriber = RobotStateSubscriber(
        logger=logger, robot_repo=robot_repo, safety_lock_svc=safety_lock_svc
    )
    robot_state_subscriber.register(node=node)
