"""pgo's run state (``pgo/mapping_status``) -> ``MappingStatusRepo``.

RELIABLE + TRANSIENT_LOCAL depth 1, so a (re)started backend learns the state
at once; durability must match pgo's publisher exactly (a VOLATILE reader gets
nothing replayed). Cheap, so no dedicated callback group.

The callback maps the ``uint8`` codes to the repo's enum (no ROS types above
here) and never raises, since that would end the executor and the process; an
unknown code is a warning that leaves the slot as it was.
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
