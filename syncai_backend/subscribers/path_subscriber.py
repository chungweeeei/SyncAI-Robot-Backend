import structlog

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Path

from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# The global costmap's frame. Deliberately un-namespaced (see
# syncai_global_planner/launch/planner_server.launch.py).
_MAP_FRAME = "map"

# Upper bound on points forwarded to the viewport (NavFn emits one per 0.05 m
# cell); 512 already looks smooth and keeps the JSON ~8 kB.
_MAX_PATH_POINTS = 512

# Consecutive poses closer than this (metres) are one point: a zero-length
# segment makes the frontend ribbon's normal NaN and blanks the whole mesh.
_MIN_POINT_SPACING_M = 1e-3


class PathSubscriber:
    """Feeds TelemetryRepo's path slot, and therefore the telemetry WebSocket,
    from the planner's ``plan`` topic.

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
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger, telemetry_repo: TelemetryRepo):
        self._logger = logger
        self._repo = telemetry_repo

        # Edge-triggered logging for the frame check (tri-state, as in
        # TelemetrySubscriber._tf_available).
        self._frame_ok = None

    def register(self, node: Node):
        # BEST_EFFORT still matches the RELIABLE publisher. A dropped plan costs
        # one republish period (1 s) of stale route; a dropped clear is covered
        # by the planner sending it three times.
        node.create_subscription(
            msg_type=Path,
            topic="plan",  # relative, so it inherits the robot_id namespace
            callback=self._plan_cb,
            qos_profile=QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
        )

    def _plan_cb(self, msg: Path):
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

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # An empty plan is the planner's clear on cancel (clear_plan). Its frame
        # is `map`, so it passes the check above; one in any other frame would
        # be dropped there and leave the old route up.
        if not msg.poses:
            self._repo.update_path(points=(), stamp=stamp)
            return

        self._repo.update_path(points=self._thin(msg), stamp=stamp)

    def _thin(self, msg: Path) -> tuple:
        """Reduce a plan to at most _MAX_PATH_POINTS deduplicated (x, y) pairs."""
        # Stride sampling suffices: NavFn's output is uniformly spaced.
        stride = max(1, -(-len(msg.poses) // _MAX_PATH_POINTS))  # ceil division

        points = []
        for i in range(0, len(msg.poses), stride):
            p = msg.poses[i].pose.position
            # Millimetres keep the JSON small; far finer than the costmap.
            xy = (round(p.x, 3), round(p.y, 3))
            if points and self._too_close(points[-1], xy):
                continue
            points.append(xy)

        # Always keep the goal, which an uneven stride would drop.
        last = msg.poses[-1].pose.position
        goal = (round(last.x, 3), round(last.y, 3))
        if not points or not self._too_close(points[-1], goal):
            points.append(goal)

        return tuple(points)

    @staticmethod
    def _too_close(a: tuple, b: tuple) -> bool:
        return (
            abs(a[0] - b[0]) < _MIN_POINT_SPACING_M
            and abs(a[1] - b[1]) < _MIN_POINT_SPACING_M
        )


def init_path_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, telemetry_repo: TelemetryRepo
) -> PathSubscriber:
    subscriber = PathSubscriber(logger=logger, telemetry_repo=telemetry_repo)
    subscriber.register(node=node)
    return subscriber
