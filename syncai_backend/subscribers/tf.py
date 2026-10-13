import structlog

from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfListener:
    """The backend's single /tf + /tf_static subscription.

    Built once in ``SyncAIBackend.__init__`` and injected into the subscribers
    that need transforms; one per subscriber would only duplicate the TF tree.

    Consumers get the raw ``buffer``, NOT a wrapped ``lookup()``: their lookups
    differ (map->odom at latest vs. a split ``lookup_transform_full``) and each
    keeps its own edge-triggered "TF went away" logging.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):
        self.logger = logger

        self.buffer = Buffer()
        # spin_thread=False: serviced by the node's executor (no extra thread),
        # in the default callback group, so PointCloudSubscriber can block on a
        # lookup in its own group without starving the transforms it awaits.
        # Held on self to make ownership visible.
        self._listener = TransformListener(self.buffer, node, spin_thread=False)


def init_tf_listener(logger: structlog.stdlib.BoundLogger, node: Node) -> TfListener:
    return TfListener(logger=logger, node=node)
