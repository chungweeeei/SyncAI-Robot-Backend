"""Where a known landmark is, as the robot's camera sees it right now.

One client, ``detect_landmark`` on landmark_detector (the workspace's
syncai_landmark_detection). Its own gateway for the reason MapGateway is one:
the router that holds it can look, and can command nothing.

The service is a query, not a trigger (DetectLandmark.srv): the first call
wakes a detector in standby and answers SENSOR_STARTING, every later one
returns the newest single-frame sighting at once, and after 5 s without a call
the detector sleeps again. So one "detect" here is a short poll: wake it, then
ask until a sighting is good enough to keep or the budget runs out. What "good
enough" means is the caller's by the srv's own contract, and for a pose an
operator saves once it is ``MIN_CONSECUTIVE`` sightings in a row with the
mirror ambiguity ruled out.

The pose is asked for in ``map``: the detector transforms it at the image
stamp, so this process needs no TF of its own for it. The srv prefers odom for
a docking target that is tracked; a waypoint is stored in the map, once.
"""

import time
import structlog
from typing import Optional
from rclpy.node import Node
from syncai_common.srv import DetectLandmark

from syncai_backend.gateways import ros
from syncai_backend.gateways.failure import Failure, fail

MAP_FRAME = "map"

# One detect, end to end: the detector's wake (camera stream up, first frames)
# plus MIN_CONSECUTIVE frames, with room for a mirror ambiguity to resolve.
DETECT_BUDGET_S = 6.0
# Under the detector's frame period would only re-read the same sighting.
DETECT_POLL_INTERVAL_S = 0.2
# The detector answers from memory; a second without one is a dead server.
DETECT_CALL_TIMEOUT_S = 1.0
SERVICE_WAIT_S = 2.0
# Sightings in a row before a pose is kept: one frame's PnP is not a waypoint.
MIN_CONSECUTIVE = 3

# What a budget that ran out means, by the last answer. Not seen (or still
# waking) is the operator's to fix by facing the landmark; the rest is the
# robot's.
_NOT_SEEN = (
    DetectLandmark.Response.NOT_DETECTED,
    DetectLandmark.Response.SENSOR_STARTING,
)


class LandmarkGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):
        self._logger = logger
        self._client = node.create_client(
            srv_type=DetectLandmark,
            srv_name="detect_landmark",
        )

    def detect(
        self,
        landmark_id: str,
        budget_s: float = DETECT_BUDGET_S,
        poll_interval_s: float = DETECT_POLL_INTERVAL_S,
    ) -> tuple[bool, str, Optional[DetectLandmark.Response]]:
        """Poll for a steady sighting of ``landmark_id`` in the map frame.

        The response on success: its ``pose`` is the landmark (origin at its
        centre, +x out of its face, +z up), 6-DOF. On failure the message is
        the detector's last sentence, tagged ``LANDMARK_NOT_DETECTED`` when
        the landmark was simply not (steadily) in view and
        ``LANDMARK_UNKNOWN`` for an id the registry does not carry.
        """
        if not self._client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return False, (
                "detect_landmark is not available — the landmark detector "
                "is not running."
            ), None

        request = DetectLandmark.Request(landmark_id=landmark_id, target_frame=MAP_FRAME)
        deadline = time.monotonic() + budget_s
        last: Optional[DetectLandmark.Response] = None
        unsteady = False

        while True:
            response = ros.call_service(self._client, request, timeout_s=DETECT_CALL_TIMEOUT_S)
            if response is not None:
                last = response
                if response.error_code == DetectLandmark.Response.UNKNOWN_LANDMARK:
                    return False, fail(Failure.LANDMARK_UNKNOWN, response.error_msg), None
                if response.error_code == DetectLandmark.Response.NONE:
                    if response.consecutive >= MIN_CONSECUTIVE and not response.ambiguous:
                        self._logger.info(
                            "[LandmarkGateway] Landmark detected",
                            landmark_id=landmark_id,
                            consecutive=response.consecutive,
                            age=response.age,
                            reprojection_error_px=response.reprojection_error_px,
                        )
                        return True, "", response
                    unsteady = True
                else:
                    unsteady = False

            if time.monotonic() >= deadline:
                break
            time.sleep(poll_interval_s)

        if last is None:
            return False, "Timeout waiting for detect_landmark response", None
        if unsteady:
            return False, fail(
                Failure.LANDMARK_NOT_DETECTED,
                f"'{landmark_id}' was seen but not steadily enough to keep "
                f"({last.consecutive} sighting(s) in a row"
                f"{', still ambiguous' if last.ambiguous else ''}).",
            ), None
        message = last.error_msg or f"detect_landmark failed with code {last.error_code}"
        self._logger.warning(
            "[LandmarkGateway] Landmark not detected",
            landmark_id=landmark_id,
            error_code=last.error_code,
            error_msg=last.error_msg,
        )
        if last.error_code in _NOT_SEEN:
            return False, fail(Failure.LANDMARK_NOT_DETECTED, message), None
        return False, message, None


def init_landmark_gateway(logger: structlog.stdlib.BoundLogger, node: Node) -> LandmarkGateway:
    return LandmarkGateway(logger=logger, node=node)
