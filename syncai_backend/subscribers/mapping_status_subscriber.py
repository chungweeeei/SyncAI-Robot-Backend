"""pgo's run state (``pgo/mapping_status``) -> ``MappingStatusRepo``.

A ~20 B typed message, RELIABLE + TRANSIENT_LOCAL depth 1, for the same
reason the map-cloud notice is latched: a backend that (re)starts mid-session
must learn whether a Start is needed without waiting for the next transition.
The durability has to match pgo's publisher exactly -- a VOLATILE reader
connects and gets nothing replayed. Relative topic, so the node's namespace
(robot_id) scopes it. No dedicated callback group: it is as cheap as
``robot_state``.

The callback translates the message's ``uint8`` constants into the repo's
enum so nothing above the subscriber imports a ROS type, and it never raises:
an exception out of a callback would end the executor and the process, and an
unknown state code (a pgo newer than this backend) is a warning that leaves
the slot as it was.
"""

import structlog
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from syncai_common.msg import MappingStatus as MappingStatusMsg

from syncai_backend.repositories.mapping.mapping import (
    MappingState,
    MappingStatusRepo,
)


_STATE_BY_CODE = {
    MappingStatusMsg.IDLE: MappingState.IDLE,
    MappingStatusMsg.MAPPING: MappingState.MAPPING,
    MappingStatusMsg.RESETTING: MappingState.RESETTING,
}


class MappingStatusSubscriber:
    def __init__(
        self, logger: structlog.stdlib.BoundLogger, mapping_status_repo: MappingStatusRepo
    ):
        self._logger = logger
        self._repo = mapping_status_repo

    def register(self, node: Node):
        self._sub = node.create_subscription(
            msg_type=MappingStatusMsg,
            topic="pgo/mapping_status",
            callback=self._status_cb,
            qos_profile=QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
        )

    def _status_cb(self, msg: MappingStatusMsg):
        state = _STATE_BY_CODE.get(msg.state)
        if state is None:
            self._logger.warning(
                "[MappingStatusSubscriber] unknown mapping state code; ignoring",
                state=int(msg.state),
            )
            return
        self._repo.update(
            state=state,
            key_poses=int(msg.key_poses),
            loop_closures=int(msg.loop_closures),
            stamp=msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        )


def init_mapping_status_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, mapping_status_repo: MappingStatusRepo
) -> MappingStatusSubscriber:
    subscriber = MappingStatusSubscriber(logger=logger, mapping_status_repo=mapping_status_repo)
    subscriber.register(node=node)
    return subscriber
