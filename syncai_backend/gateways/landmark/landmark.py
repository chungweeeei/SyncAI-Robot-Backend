"""Where a known landmark is, as the robot's camera sees it right now.

One client, ``detect_landmark`` on landmark_detector (the workspace's
syncai_landmark_detection). Its own gateway for the reason MapGateway is one:
the router that holds it can look, and can command nothing.

The service is a query, not a trigger (DetectLandmark.srv): the first call
wakes a detector in standby and answers SENSOR_STARTING, every later one
returns the newest single-frame sighting at once, and after 5 s without a call
the detector sleeps again. So one "detect" here is a short poll: wake it, then
collect ``SIGHTINGS`` answers that are ``MIN_CONSECUTIVE`` frames into a streak,
or as many as the budget allows. What is good enough is the caller's by the
srv's own contract: an ambiguous sighting is kept (its position is right; which
way it faces is the waypoint's to judge, ``landmark_waypoint``), where until 2026-11-17 it
was refused, and a robot at the 1.0-1.3 m an operator stands it at never saw
anything else.

The pose is asked for in the camera's own frame (an empty ``target_frame``) and
carried into ``map`` here, with the process's one TF buffer at the image stamp:
the heading needs where the camera stood, and the landmark and the camera must
come through the same transform to agree. Until 2026-11-17 the detector was
asked for ``map`` and this process held no TF for it.
"""

import time
import structlog
from typing import Optional
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from syncai_common.srv import DetectLandmark

from syncai_backend.gateways import ros
from syncai_backend.gateways.failure import Failure, fail
from syncai_backend.helpers.landmark_heading import LandmarkSighting, sighting_in_map

MAP_FRAME = "map"
# Empty asks for the sensor's own frame; its name and the image stamp ride on the header.
SENSOR_FRAME = ""

# One detect, end to end: the detector's wake (stream, exposure, first frames) plus the streak.
DETECT_BUDGET_S = 6.0
# Under the detector's frame period would only re-read the same sighting.
DETECT_POLL_INTERVAL_S = 0.2
# The detector answers from memory; a second without one is a dead server.
DETECT_CALL_TIMEOUT_S = 1.0
SERVICE_WAIT_S = 2.0
# Sightings in a row before a pose is kept: one frame's PnP is not a waypoint.
MIN_CONSECUTIVE = 3
# Answers kept per detect, one poll apart (1 s): the best fit among them judges the face.
SIGHTINGS = 5
# An image trails the newest TF by tens of ms; no transform after this is a broken chain.
TF_LOOKUP_TIMEOUT_S = 0.2

# A budget run out on 'not seen' or 'still waking' is the operator's to fix; the rest the robot's.
_NOT_SEEN = (
    DetectLandmark.Response.NOT_DETECTED,
    DetectLandmark.Response.SENSOR_STARTING,
)


class LandmarkGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node, tf_buffer: Buffer):
        """Hold the detect_landmark client and the process's one TF buffer."""
        self._logger = logger
        self._tf_buffer = tf_buffer
        self._client = node.create_client(
            srv_type=DetectLandmark,
            srv_name="detect_landmark",
        )

    def detect(
        self,
        landmark_id: str,
        budget_s: float = DETECT_BUDGET_S,
        poll_interval_s: float = DETECT_POLL_INTERVAL_S,
    ) -> tuple[bool, str, list[LandmarkSighting]]:
        """Collect steady sightings, ambiguous or not, in the map frame.

        Success carries one to ``SIGHTINGS`` of them, oldest first. On failure the
        message is the detector's last sentence (tagged ``LANDMARK_NOT_DETECTED``
        or ``LANDMARK_UNKNOWN`` where the caller answers differently) or the
        transform's, and the list is empty.
        """
        started = time.monotonic()
        self._logger.info(
            "[LandmarkGateway] Detect started", landmark_id=landmark_id, budget_s=budget_s
        )
        if not self._client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            self._logger.error(
                "[LandmarkGateway] detect_landmark not available",
                landmark_id=landmark_id,
                waited_s=SERVICE_WAIT_S,
            )
            return False, (
                "detect_landmark is not available — the landmark detector "
                "is not running."
            ), []

        request = DetectLandmark.Request(landmark_id=landmark_id, target_frame=SENSOR_FRAME)
        deadline = time.monotonic() + budget_s
        last: Optional[DetectLandmark.Response] = None
        sightings: list[LandmarkSighting] = []
        answers = 0

        while len(sightings) < SIGHTINGS:
            response = ros.call_service(self._client, request, timeout_s=DETECT_CALL_TIMEOUT_S)
            if response is not None:
                answers += 1
                last = response
                self._logger.debug(
                    "[LandmarkGateway] Answer",
                    landmark_id=landmark_id,
                    error_code=response.error_code,
                    consecutive=response.consecutive,
                    ambiguous=response.ambiguous,
                    reprojection_error_px=response.reprojection_error_px,
                )
                if response.error_code == DetectLandmark.Response.UNKNOWN_LANDMARK:
                    self._logger.warning(
                        "[LandmarkGateway] Unknown landmark",
                        landmark_id=landmark_id,
                        error_msg=response.error_msg,
                    )
                    return False, fail(Failure.LANDMARK_UNKNOWN, response.error_msg), []
                if (
                    response.error_code == DetectLandmark.Response.NONE
                    and response.consecutive >= MIN_CONSECUTIVE
                ):
                    sighting, error = self._sighting_in_map(response)
                    if sighting is None:
                        self._logger.error(
                            "[LandmarkGateway] Transform into map failed",
                            landmark_id=landmark_id,
                            frame=response.pose.header.frame_id,
                            error=error,
                            duration_s=time.monotonic() - started,
                        )
                        return False, error, []
                    sightings.append(sighting)

            if time.monotonic() >= deadline:
                break
            if len(sightings) < SIGHTINGS:
                time.sleep(poll_interval_s)

        duration_s = time.monotonic() - started
        if sightings:
            self._logger.info(
                "[LandmarkGateway] Landmark detected",
                landmark_id=landmark_id,
                sightings=len(sightings),
                ambiguous=sum(sighting.ambiguous for sighting in sightings),
                reprojection_error_px=[
                    round(sighting.reprojection_error_px, 3) for sighting in sightings
                ],
                answers=answers,
                duration_s=duration_s,
            )
            return True, "", sightings
        if last is None:
            self._logger.error(
                "[LandmarkGateway] No answer from detect_landmark",
                landmark_id=landmark_id,
                duration_s=duration_s,
            )
            return False, "Timeout waiting for detect_landmark response", []
        if last.error_code == DetectLandmark.Response.NONE:
            self._logger.warning(
                "[LandmarkGateway] Landmark seen but not steady",
                landmark_id=landmark_id,
                consecutive=last.consecutive,
                answers=answers,
                duration_s=duration_s,
            )
            return False, fail(
                Failure.LANDMARK_NOT_DETECTED,
                f"'{landmark_id}' was seen but not steadily enough to keep "
                f"({last.consecutive} sighting(s) in a row).",
            ), []
        message = last.error_msg or f"detect_landmark failed with code {last.error_code}"
        self._logger.warning(
            "[LandmarkGateway] Landmark not detected",
            landmark_id=landmark_id,
            error_code=last.error_code,
            error_msg=last.error_msg,
            answers=answers,
            duration_s=duration_s,
        )
        if last.error_code in _NOT_SEEN:
            return False, fail(Failure.LANDMARK_NOT_DETECTED, message), []
        return False, message, []

    def _sighting_in_map(
        self, response: DetectLandmark.Response
    ) -> tuple[Optional[LandmarkSighting], str]:
        """The answer in the map at the image stamp, or None and why the transform failed."""
        header = response.pose.header
        try:
            transform = self._tf_buffer.lookup_transform(
                MAP_FRAME,
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_S),
            )
        except TransformException as exc:
            return None, f"{header.frame_id} -> {MAP_FRAME} at the image's stamp failed: {exc}"
        pose = response.pose.pose
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return sighting_in_map(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
            ambiguous=response.ambiguous,
            consecutive=response.consecutive,
            age=response.age,
            reprojection_error_px=response.reprojection_error_px,
        ), ""


def init_landmark_gateway(
    logger: structlog.stdlib.BoundLogger, node: Node, tf_buffer: Buffer
) -> LandmarkGateway:
    """Build the gateway main.py injects into the landmark router."""
    return LandmarkGateway(logger=logger, node=node, tf_buffer=tf_buffer)
