from typing import Callable, Tuple

import structlog

from rclpy.node import Node

from nav_msgs.msg import Path

from syncai_backend.subscribers import qos
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


# What a subscriber hands its repo slot: thinned (x, y, z) triples and the stamp.
PathSink = Callable[[Tuple[Tuple[float, float, float], ...], float], None]


class PathSubscriber:
    """Feeds one of TelemetryRepo's path slots, and therefore the telemetry
    WebSocket, from a map-frame ``nav_msgs/Path`` topic: the planner's
    ``plan`` (``init_path_subscriber``) or the docking controller's
    ``docking_plan`` (``init_docking_plan_subscriber``). The two share the
    frame gate, the thinning and the empty-means-clear rule, which is why
    they are one class with the topic and the slot as arguments.

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
    floor surface; 0 from a 2D planner), the same height a waypoint's z is. A
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

        # An empty path is a clear (the planner's clear_plan, controller_server
        # at the end of every dock goal), and it is honoured in any frame: it
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
        """Reduce a plan to at most _MAX_PATH_POINTS deduplicated (x, y, z) triples."""
        # Stride sampling suffices: NavFn's output is uniformly spaced.
        stride = max(1, -(-len(msg.poses) // _MAX_PATH_POINTS))  # ceil division

        points = []
        for i in range(0, len(msg.poses), stride):
            p = msg.poses[i].pose.position
            # Millimetres keep the JSON small; far finer than the costmap.
            xyz = (round(p.x, 3), round(p.y, 3), round(p.z, 3))
            if points and self._too_close(points[-1], xyz):
                continue
            points.append(xyz)

        # Always keep the goal, which an uneven stride would drop.
        last = msg.poses[-1].pose.position
        goal = (round(last.x, 3), round(last.y, 3), round(last.z, 3))
        if not points or not self._too_close(points[-1], goal):
            points.append(goal)

        return tuple(points)

    @staticmethod
    def _too_close(a: tuple, b: tuple) -> bool:
        # All three axes: a point straight over the last one is a step up a
        # stair, not a duplicate.
        return all(abs(p - q) < _MIN_POINT_SPACING_M for p, q in zip(a, b))


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


def init_docking_plan_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, telemetry_repo: TelemetryRepo
) -> PathSubscriber:
    """The DockingController's predicted approach onto the dock (2 Hz, map
    frame), which the planner's ``plan`` does not cover: the final metre is
    driven by a control law with no planner involved. The staging leg before
    it is an ordinary plan and arrives on ``plan``.

    Its own slot rather than the route's, so the console can draw the two
    differently and a dock's clear never erases a route (or the reverse).
    controller_server publishes the empty Path when the dock goal ends,
    however it ends; the repo's TTL covers a controller that died mid-dock.
    """
    subscriber = PathSubscriber(
        logger=logger,
        topic="docking_plan",
        sink=lambda points, stamp: telemetry_repo.update_docking_plan(
            points=points, stamp=stamp
        ),
    )
    subscriber.register(node=node)
    return subscriber
