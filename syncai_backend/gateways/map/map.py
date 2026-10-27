"""The map router's ROS surface: map_server reloads, localizer swaps, pgo saves.

Separate from ``RobotGateway`` so the map router holds no handle that can
command the robot to move. Its ``initialpose`` publisher only re-seeds where
the robot thinks it is; a second publisher on RobotGateway's topic is fine.

The clients split by session: ``map_server/load_map``,
``filter_mask_server/load_map``, ``relocalize``, ``relocalize_check`` and
``reset_odometry`` exist only in the nav (AUTO) session; ``pgo/start_mapping``,
``pgo/save_maps`` and ``pgo/reset_mapping`` only in the mapping (MANUAL) one.
"Service is not available" usually means "wrong mode", which activate turns
into ``stack_not_ready``. The nav launch writes a blank keepout mask for a map
without one, so ``filter_mask_server`` is up on every map.

``planner_server/Octo3D/load_map`` (the 3D planner's map) exists only on a
workspace that carries OctomapPlanner3D (``feature/octomap-planner-3d``); its
absence is a capability answer, not a mode one — see ``planner_3d_available``.
"""

import math
import os
import time
import structlog
from typing import Optional
from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Header
from nav2_msgs.srv import LoadMap
from syncai_common.srv import (
    IsValid, Relocalize, ResetMapping, ResetOdometry, SaveMaps, StartMapping,
)

from syncai_backend.gateways import ros


def _load_map_failure(result: int, server: str, image: str, yaml: str) -> str:
    """The operator's sentence for a failed ``LoadMap`` call.

    LoadMap.srv has no `message` field, so the prose is written here,
    parameterised because two servers answer it (``map_server`` for
    ``gridmap.*``, ``filter_mask_server`` for ``keepout.*``).
    RESULT_UNDEFINED_FAILURE is kept for a stock nav2 map_server.
    """
    messages = {
        LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST: f"{server} could not find the map yaml",
        LoadMap.Response.RESULT_INVALID_MAP_DATA: f"{server} could not read {image}",
        LoadMap.Response.RESULT_INVALID_MAP_METADATA: f"{server} rejected {yaml}",
        LoadMap.Response.RESULT_UNDEFINED_FAILURE: f"{server} reported an undefined failure",
    }
    return messages.get(result, f"{server} returned result {result}")


class MapGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):
        self._logger = logger
        self._node = node

        self._service_clients: dict[str, Client] = {}
        self.register_service_clients()

        # Same topic and QoS as RobotGateway's: volatile, depth 5.
        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, "initialpose", QoSProfile(depth=5)
        )

    def register_service_clients(self):

        load_map_client = self._node.create_client(
            srv_type=LoadMap,
            srv_name="map_server/load_map",
        )

        # The keepout mask's map_server (costmap_filter_info.launch.py); its
        # service carries the node name, like load_map's.
        load_keepout_client = self._node.create_client(
            srv_type=LoadMap,
            srv_name="filter_mask_server/load_map",
        )

        # OctomapPlanner3D's tree (2026-10). Under planner_server's node name
        # plus the plugin id, the way map_server's carries its node name. Same
        # LoadMap type: map_url is the octomap.bt, and an empty one unloads.
        load_octomap_client = self._node.create_client(
            srv_type=LoadMap,
            srv_name="planner_server/Octo3D/load_map",
        )

        # Octo3D's forbidden zones (2026-10-23): the map's keepout.json, read
        # without rebuilding the tree. Same LoadMap type; an empty map_url
        # clears them. load_map reads the file itself, so a switch needs no call.
        load_keepout_3d_client = self._node.create_client(
            srv_type=LoadMap,
            srv_name="planner_server/Octo3D/load_keepout",
        )

        save_maps_client = self._node.create_client(
            srv_type=SaveMaps,
            srv_name="pgo/save_maps",
        )

        # Like save_maps, under the /<robot_id>/pgo namespace.
        reset_mapping_client = self._node.create_client(
            srv_type=ResetMapping,
            srv_name="pgo/reset_mapping",
        )

        # Moves pgo from IDLE to MAPPING; a mapping session starts idle.
        start_mapping_client = self._node.create_client(
            srv_type=StartMapping,
            srv_name="pgo/start_mapping",
        )

        # Bare names, unlike load_map's: the localizer's plain
        # `create_service("relocalize", ...)` lands at /<robot_id>/relocalize.
        # Do not write `localizer/relocalize`; it caused false stack_not_ready.
        # Check with `ros2 node info /<robot_id>/localizer_node`, not
        # `ros2 service list`, which can show stale entries.
        relocalize_client = self._node.create_client(
            srv_type=Relocalize,
            srv_name="relocalize",
        )

        relocalize_check_client = self._node.create_client(
            srv_type=IsValid,
            srv_name="relocalize_check",
        )

        # Bare like relocalize and in its mutually exclusive callback group, so
        # calls queue behind a running search. Finding the robot goes through
        # reset_odometry (skip_seed), not auto_localize: see auto_localize().
        reset_odometry_client = self._node.create_client(
            srv_type=ResetOdometry,
            srv_name="reset_odometry",
        )

        self._service_clients.update(
            {
                "load_map": load_map_client,
                "load_keepout": load_keepout_client,
                "load_octomap": load_octomap_client,
                "load_keepout_3d": load_keepout_3d_client,
                "save_maps": save_maps_client,
                "reset_mapping": reset_mapping_client,
                "start_mapping": start_mapping_client,
                "relocalize": relocalize_client,
                "relocalize_check": relocalize_check_client,
                "reset_odometry": reset_odometry_client,
            }
        )

    def localizer_ready(self, timeout_sec: float = 2.0) -> bool:
        """Is the AUTO-session localizer (`relocalize`) discoverable right now?

        The precondition for every map switch, and how mapping mode is told
        apart from a running nav stack. Not read off the cached RobotState
        mode, which can be stale and names the session, not whether its
        services are up.
        """
        return self._service_clients["relocalize"].wait_for_service(
            timeout_sec=timeout_sec
        )

    def map_server_ready(self, timeout_sec: float = 2.0) -> bool:
        """Is map_server (`load_map`) discoverable right now?

        Asked separately from the localizer: a 3D nav session runs without
        map_server (Octo3D plans on the octree), so only a switch to a 2D map
        requires it. Discoverability, as ``localizer_ready``.
        """
        return self._service_clients["load_map"].wait_for_service(
            timeout_sec=timeout_sec
        )

    def swap_localizer_map(
        self, pcd_path: str, x: float, y: float, yaw: float
    ) -> tuple[bool, str]:
        """Point the running localizer at another map's cloud and re-seed it.

        Two calls, and the second is not optional.

        `relocalize` swaps the map (a failed load keeps the old one serving).
        Its `success` does not mean registered: that runs later on a 5 Hz timer
        and only `relocalize_check` reports it.

        `relocalize` takes the raw 6-DOF guess, bypassing `applyPlanarGuess`.
        The lidar is tilted (~15 degrees of pitch), so the flat zeros below would
        freeze the localizer; the `initialpose` publish overwrites them with a
        tilt-correct guess. Dropping it reintroduces the freeze.
        """
        relocalize_client = self._service_clients.get("relocalize")
        if not relocalize_client.wait_for_service(timeout_sec=5.0):
            return False, "relocalize service is not available."

        map_path = os.path.abspath(os.path.expanduser(pcd_path))

        self._logger.info("[MapGateway] Relocalizing onto map cloud", pcd=map_path)

        # Reads and downsamples a ~100-200 MB PCD; bounded since this holds a
        # FastAPI worker thread.
        response = ros.call_service(
            relocalize_client,
            Relocalize.Request(
                pcd_path=map_path, x=float(x), y=float(y), z=0.0,
                yaw=float(yaw), pitch=0.0, roll=0.0,
            ),
            timeout_s=60.0,
        )
        if response is None:
            return False, "Timeout waiting for relocalize response"

        if not response.success:
            return False, f"localizer refused the map: {response.message}"

        self._seed_initial_pose(x=x, y=y, yaw=yaw)
        return True, ""

    def _seed_initial_pose(self, x: float, y: float, yaw: float) -> None:
        """Publish `initialpose` so the guess goes through `applyPlanarGuess`.

        Covariance stays zero (the localizer reads x/y/yaw only). A missing
        subscriber is logged, as in RobotGateway.
        """
        if self._initial_pose_pub.get_subscription_count() == 0:
            self._logger.warning(
                "[MapGateway] Publishing initialpose with no subscriber; "
                "is the localizer running?"
            )

        self._logger.info("[MapGateway] Seeding initial pose", x=x, y=y, yaw=yaw)

        msg = PoseWithCovarianceStamped(
            header=Header(
                frame_id="map", stamp=self._node.get_clock().now().to_msg()
            )
        )
        msg.pose.pose = Pose(
            position=Point(x=float(x), y=float(y), z=0.0),
            orientation=Quaternion(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
            ),
        )
        self._initial_pose_pub.publish(msg)

    def localization_converged(self, timeout_s: float = 3.0) -> Optional[bool]:
        """Has the localizer's ICP actually converged? None if it cannot be asked.

        Polls `relocalize_check` (`code=0`, the real `localize_success`). The
        only surface that answers this: `RobotState.localization_valid` is
        TF-presence only and reads true before any convergence.

        Best effort: false means "not yet" (registration retries forever). The
        budget is short because this runs inside a REST handler.
        """
        check_client = self._service_clients.get("relocalize_check")
        if not check_client.wait_for_service(timeout_sec=1.0):
            return None

        deadline = time.monotonic() + timeout_s
        while True:
            remaining = max(0.5, deadline - time.monotonic())
            response = ros.call_service(check_client, IsValid.Request(code=0), timeout_s=remaining)
            if response is None:
                return None
            if response.valid:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.25)

    def auto_localize(
        self,
    ) -> tuple[bool, str, Optional[ResetOdometry.Response]]:
        """Find the robot on the loaded map with no hint: kidnap recovery.

        Calls the localizer's ``reset_odometry`` with ``skip_seed``, not
        ``auto_localize``: Point-LIO is reset first, then the same hint-free
        search runs on scans of the new run. A carry often diverges Point-LIO
        too, and ``auto_localize`` alone cannot recover from that (its scan
        window is posed by the diverged odometry); a bare ``pointlio/reset``
        would leave the localizer's offset on the old origin. ``skip_seed``
        tries no seed, because after a kidnap the last good pose is wrong.

        Unlike ``relocalize``, ``success`` is a result: true means localized.
        The odometry has been reset either way — that step is not undone — and
        on failure the map pose stays where it was before the call; the
        response carries the candidate that did not converge, for diagnosis.
        The robot must be standing still (nothing here can check it).

        The 40 s deadline outlasts the localizer's own (2 s LIO reset + 10 s
        new-run scans + 10 s search + 5 s convergence), so a slow answer is
        its verdict, not ours.
        """
        client = self._service_clients["reset_odometry"]
        if not client.wait_for_service(timeout_sec=5.0):
            return False, (
                "reset_odometry is not available — finding the robot needs it "
                "in AUTO (navigation) mode."
            ), None

        self._logger.info("[MapGateway] Auto-localizing (reset odometry + search)")

        response = ros.call_service(client, ResetOdometry.Request(skip_seed=True), timeout_s=40.0)
        if response is None:
            return False, "Timeout waiting for reset_odometry response", None

        self._logger.info(
            "[MapGateway] Auto-localize answered",
            success=response.success,
            message=response.message,
            method=response.method,
            fitness=response.fitness,
            inlier_ratio=response.inlier_ratio,
            candidates=response.candidates,
        )
        return response.success, response.message, response

    def reload_map(self, yaml_path: str) -> tuple[bool, str]:
        """Make the running map_server re-read a map and re-publish it.

        Re-reads yaml + pgm (no caching) and republishes on the transient_local
        topic, so late joiners get the edit. A failed reload keeps the old grid.

        map_server holds one grid: this call *is* the running map. Passing a
        non-active map is only valid paired with `swap_localizer_map` and the
        INI write, as activate does; alone it swaps the grid out from under the
        localizer.
        """
        load_map_client = self._service_clients.get("load_map")
        if not load_map_client.wait_for_service(timeout_sec=5.0):
            return False, "load_map service is not available."

        map_url = os.path.abspath(os.path.expanduser(yaml_path))

        self._logger.info("[MapGateway] Reloading map", map_url=map_url)

        # Under a second in practice; 20 s is headroom yet short enough that a
        # wedged map_server does not hold a worker thread for long.
        response = ros.call_service(
            load_map_client, LoadMap.Request(map_url=map_url), timeout_s=20.0
        )
        if response is None:
            return False, "Timeout waiting for map_server/load_map response"

        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return False, _load_map_failure(
                response.result, "map_server", "gridmap.pgm", "gridmap.yaml"
            )

        return True, ""

    def reload_keepout(self, yaml_path: str) -> tuple[bool, str]:
        """Make ``filter_mask_server`` re-read a keepout mask and re-publish it.

        The same ``LoadMap`` call as ``reload_map``, against the KeepoutFilter's
        map_server. The transient_local topic reaches the running filter and
        any later one.

        - **A map switch does not move the mask along**: the old map's zones
          stay in force, so activate calls this too.
        - **Clearing zones is a reload, not a delete**: the caller writes an
          all-unknown mask and reloads it.

        "Not available" means mapping mode or a down keepout pane.
        """
        client = self._service_clients.get("load_keepout")
        if not client.wait_for_service(timeout_sec=5.0):
            return False, (
                "filter_mask_server/load_map is not available — the keepout filter "
                "only runs in AUTO mode; the forbidden zones apply when the nav stack "
                "next starts on this map."
            )

        map_url = os.path.abspath(os.path.expanduser(yaml_path))

        self._logger.info("[MapGateway] Reloading keepout mask", map_url=map_url)

        # Same as reload_map.
        response = ros.call_service(client, LoadMap.Request(map_url=map_url), timeout_s=20.0)
        if response is None:
            return False, "Timeout waiting for filter_mask_server/load_map response"

        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return False, _load_map_failure(
                response.result, "filter_mask_server", "keepout.pgm", "keepout.yaml"
            )

        return True, ""

    def reload_keepout_3d(self, json_path: str) -> tuple[bool, str]:
        """Make the 3D planner (Octo3D) re-read a map's ``keepout.json``.

        The 3D twin of ``reload_keepout``: Octo3D does not read the mask, it
        rasterises the json's polygons (with their floor bands) onto its own
        surface map, and ``planner_server/Octo3D/load_keepout`` swaps them in
        without rebuilding the tree. A refused file keeps the zones in force;
        the robot's running path is re-judged against the new ones within a
        second. ``json_path=""`` clears them.

        The bound covers the planner finishing a boot build or a plan first.
        """
        client = self._service_clients["load_keepout_3d"]
        if not client.wait_for_service(timeout_sec=5.0):
            return False, (
                "planner_server/Octo3D/load_keepout is not available — the nav stack "
                "is not in AUTO mode or predates 3D forbidden zones; the zones apply "
                "when the nav stack next loads this map."
            )

        map_url = os.path.abspath(os.path.expanduser(json_path)) if json_path else ""

        self._logger.info(
            "[MapGateway] Reloading the 3D planner's forbidden zones",
            map_url=map_url or "(clear)",
        )

        response = ros.call_service(client, LoadMap.Request(map_url=map_url), timeout_s=120.0)
        if response is None:
            return False, "Timeout waiting for planner_server/Octo3D/load_keepout response"

        if response.result == LoadMap.Response.RESULT_UNDEFINED_FAILURE:
            # What the planner answers when it has no 3D map to draw zones on.
            return False, "the 3D planner has no 3D map loaded"
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return False, _load_map_failure(
                response.result, "the 3D planner", "keepout.json", "keepout.json"
            )

        return True, ""

    def planner_3d_available(self, timeout_sec: float = 2.0) -> bool:
        """Does the running nav stack have the 3D planner (Octo3D) at all?

        Asked before a switch touches anything: a stack built without
        OctomapPlanner3D cannot navigate a 3D map, and has no 3D map to unload
        when switching to a 2D one. Discoverability, as ``nav_services_ready``.
        """
        return self._service_clients["load_octomap"].wait_for_service(
            timeout_sec=timeout_sec
        )

    def reload_octomap(self, bt_path: str) -> tuple[bool, str]:
        """Point Octo3D at a map's ``octomap.bt``, or unload it with ``""``.

        The 3D counterpart of ``reload_map``, and what makes the robot's
        task_runner pick the 3D trees: Octo3D latches whether it has a map,
        and every goal runs move_3d.xml / dock_3d.xml while it does. So a
        switch to a 2D map must unload (``bt_path=""``) — otherwise the 3D
        trees keep planning on the previous site's octree.

        Unlike ``relocalize``, success is a result: the call answers once the
        surface map is built (seconds; the 120 s bound covers a large site).
        A missing file is refused with the current map kept; any other failure
        leaves Octo3D with no map at all, never the old one.
        """
        client = self._service_clients["load_octomap"]
        if not client.wait_for_service(timeout_sec=5.0):
            return False, (
                "planner_server/Octo3D/load_map is not available — the nav stack "
                "has no 3D planner, or it is not in AUTO mode."
            )

        map_url = os.path.abspath(os.path.expanduser(bt_path)) if bt_path else ""

        self._logger.info(
            "[MapGateway] Reloading the 3D planner's map",
            map_url=map_url or "(unload)",
        )

        response = ros.call_service(client, LoadMap.Request(map_url=map_url), timeout_s=120.0)
        if response is None:
            return False, "Timeout waiting for planner_server/Octo3D/load_map response"

        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return False, _load_map_failure(
                response.result, "the 3D planner", "octomap.bt", "the octomap path"
            )

        return True, ""

    def save_map(self, directory: str, octomap: Optional[bool] = None) -> tuple[bool, str]:
        """Have pgo write the run into ``directory`` and end it.

        ``octomap`` is the save's 3D map choice; None sends OCTOMAP_DEFAULT,
        which leaves it to pgo's map_clean_octomap.
        """
        save_maps_client = self._service_clients.get("save_maps")
        if not save_maps_client.wait_for_service(timeout_sec=5.0):
            # Name the mode: pgo runs only in the MANUAL session.
            return False, (
                "pgo/save_maps is not available — saving a map needs the robot "
                "in MANUAL (mapping) mode."
            )

        file_path = os.path.abspath(os.path.expanduser(directory))
        self._logger.info("[MapGateway] Saving map", file_path=file_path)

        response = ros.call_service(
            save_maps_client,
            SaveMaps.Request(
                file_path=file_path,
                save_patches=True,
                octomap=(
                    SaveMaps.Request.OCTOMAP_DEFAULT
                    if octomap is None
                    else SaveMaps.Request.OCTOMAP_ON
                    if octomap
                    else SaveMaps.Request.OCTOMAP_OFF
                ),
            ),
            timeout_s=180.0,
        )
        if response is None:
            return False, "Timeout waiting for pgo/save_maps response"

        return response.success, response.message

    def reset_mapping(self, reset_lio: bool = True) -> tuple[bool, str]:
        """Discard the map built so far and start a new one, restarting nothing.

        Irreversible: pgo's unsaved keyframes are discarded. Saving first is
        the caller's decision, kept separate on purpose.

        ``reset_lio`` is True for every operator call; False (graph only) is
        for bag replay, since the map's origin follows the odometry origin.

        pgo owns the sequencing; a failure leaves the pose graph untouched.
        30 s covers an in-flight merge, pgo's 5 s pointlio cap and freeing
        the keyframe clouds.
        """
        reset_client = self._service_clients.get("reset_mapping")
        if not reset_client.wait_for_service(timeout_sec=5.0):
            # Name the mode, as save_map does.
            return False, (
                "pgo/reset_mapping is not available — starting a new map needs "
                "the robot in MANUAL (mapping) mode."
            )

        self._logger.info("[MapGateway] Resetting the mapping run", reset_lio=reset_lio)

        response = ros.call_service(
            reset_client, ResetMapping.Request(reset_lio=reset_lio), timeout_s=30.0
        )
        if response is None:
            return False, "Timeout waiting for pgo/reset_mapping response"

        if response.success:
            # Zero means the run had banked no keyframes yet.
            self._logger.info(
                "[MapGateway] Mapping run reset",
                dropped_key_poses=response.dropped_key_poses,
                lio_last_odom_time=response.lio_last_odom_time,
            )
        return response.success, response.message

    def start_mapping(self, reset_lio: bool = True) -> tuple[bool, str]:
        """Begin a mapping run: pgo from IDLE to MAPPING.

        The opening bracket of a run; a successful ``save_map`` closes it.
        pgo refuses while already mapping, so a stale tab cannot discard a run.

        ``reset_lio`` as for reset_mapping: only a LIO reset starts the map
        where the robot stands, which must be stationary meanwhile (pgo's
        ``message`` says so). No ``Failure`` code: the router pre-empts
        "already mapping" from the latched status.
        """
        start_client = self._service_clients.get("start_mapping")
        if not start_client.wait_for_service(timeout_sec=5.0):
            return False, (
                "pgo/start_mapping is not available — starting mapping needs "
                "the robot in MANUAL (mapping) mode."
            )

        self._logger.info("[MapGateway] Starting the mapping run", reset_lio=reset_lio)

        response = ros.call_service(
            start_client, StartMapping.Request(reset_lio=reset_lio), timeout_s=30.0
        )
        if response is None:
            return False, "Timeout waiting for pgo/start_mapping response"

        if response.success:
            self._logger.info(
                "[MapGateway] Mapping run started",
                lio_last_odom_time=response.lio_last_odom_time,
            )
        return response.success, response.message


def init_map_gateway(logger: structlog.stdlib.BoundLogger, node: Node) -> MapGateway:
    return MapGateway(logger=logger, node=node)
