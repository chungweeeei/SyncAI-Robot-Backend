import structlog
from rclpy.node import Node

from syncai_common.msg import RobotState as RobotStateMsg

from syncai_backend.subscribers import qos
from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.services.safety_lock import SafetyLockService


class RobotStateSubscriber:
    """Feeds RobotRepo, and therefore GET /api/v1/robot/state, from the
    ``robot_state`` topic.

    The whole message is stored unfiltered; the router's field list is the
    whitelist and must stay one.

    Every sample is stored, including ``localization_valid=false`` with a
    zeroed pose. Do not drop those: battery and ``mode`` would go unreadable
    before relocalization and for the whole mapping run. The payload carries
    ``localization_valid`` instead.

    ``low_level_mode.safety_state`` also goes to SafetyLockService (cancel on
    the rising edge). Best-effort QoS only delays the edge to the next sample.
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
            qos_profile=qos.best_effort(depth=3),
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
