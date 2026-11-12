from typing import Callable, Tuple

import structlog

from rclpy.node import Node

from nav_msgs.msg import Path

from syncai_backend.helpers.route_preview import thin_points
from syncai_backend.subscribers import qos
from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# The global costmap's frame. Deliberately un-namespaced (see
# syncai_global_planner/launch/planner_server.launch.py).
_MAP_FRAME = "map"

# What a subscriber hands its repo slot: thinned (x, y, z) triples and the stamp.
PathSink = Callable[[Tuple[Tuple[float, float, float], ...], float], None]


class PathSubscriber:
    """Feeds TelemetryRepo's route slot, and therefore the telemetry
    WebSocket, from a map-frame ``nav_msgs/Path`` topic: the planner's
    ``plan`` (``init_path_subscriber``). The topic and the slot are arguments
    because a second map-frame path, the docking controller's
    ``docking_plan``, shared the frame gate, the thinning and the
    empty-means-clear rule until it went with the robot stack's dock trees
    (2026-11).

    Apart from TelemetrySubscriber: a route arrives once a second in the map
    frame (no TF) and wants different QoS. Not the controller's
    ``received_global_plan``, which is a pruned slice in base_link.

    The publisher (syncai_global_planner/src/planner_server.cpp) is VOLATILE
    but re-sends its last plan at plan_republish_rate (1 Hz), trimmed to start
    at the robot, so a restarted backend shows the route within a second. A
    cancelled navigation is cleared by an **empty** Path in the ``map`` frame
    (task_runner calls the planner's clear_plan; sent three times, one per
    republish tick, because this subscription is BEST_EFFORT). The repo's TTL
    is only the fallback for a planner that stopped re-sending.

    Each point keeps its z: the ground the route stands on (OctomapPlanner3D's
    floor surface), the same height a waypoint's z is. A
    route that climbs a stair is on two storeys, and the console draws each
    part on its own.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        topic: str,
        sink: PathSink,
    ):
        self._logger = logger.bind(topic=topic)
        self._topic = topic
        self._sink = sink

        # Edge-triggered logging for the frame check (tri-state, as in
        # TelemetrySubscriber._tf_available).
        self._frame_ok = None

    def register(self, node: Node):
        # BEST_EFFORT still matches the RELIABLE publisher. A dropped plan costs
        # one republish period (1 s) of stale route; a dropped clear is covered
        # by the planner sending it three times.
        node.create_subscription(
            msg_type=Path,
            topic=self._topic,  # relative, so it inherits the robot_id namespace
            callback=self._plan_cb,
            qos_profile=qos.best_effort(depth=1),
        )

    def _plan_cb(self, msg: Path):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # An empty path is a clear (the planner's clear_plan), and it is
        # honoured in any frame: it
        # carries no coordinates to misplace, and dropping one would leave the
        # old route up until the TTL.
        if not msg.poses:
            self._sink((), stamp)
            return

        frame = msg.header.frame_id
        if frame != _MAP_FRAME:
            # Dropped, not reprojected: everything downstream assumes map metres.
            if self._frame_ok is not False:
                self._logger.warning(
                    "planned path dropping: unexpected frame (expected the "
                    "global costmap's map frame)",
                    frame=frame,
                    expected=_MAP_FRAME,
                )
                self._frame_ok = False
            return

        if self._frame_ok is not True:
            self._logger.info("planned path streaming", frame=frame)
            self._frame_ok = True

        self._sink(self._thin(msg), stamp)

    def _thin(self, msg: Path) -> tuple:
        """Reduce a plan to at most MAX_PATH_POINTS deduplicated (x, y, z)
        triples: the rule a route preview's answer shares
        (helpers/route_preview.py), since the console draws both alike."""
        return thin_points(
            [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        )


def init_path_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, telemetry_repo: TelemetryRepo
) -> PathSubscriber:
    subscriber = PathSubscriber(
        logger=logger,
        topic="plan",
        sink=lambda points, stamp: telemetry_repo.update_path(points=points, stamp=stamp),
    )
    subscriber.register(node=node)
    return subscriber
