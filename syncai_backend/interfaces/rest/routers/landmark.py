import math
import structlog
import threading
from typing import Optional
from fastapi import APIRouter
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.gateways.landmark.landmark import LandmarkGateway
from syncai_backend.helpers.planar_pose import planar_yaw
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.robot.robot import RobotRepo

# The detector's registry name for the charger's marker (its params file).
DEFAULT_LANDMARK_ID = "charger_a"


class LandmarkDetectRequest(BaseModel):
    landmark_id: str = Field(
        DEFAULT_LANDMARK_ID,
        min_length=1,
        description="The landmark's name in the detector's registry.",
    )


class LandmarkDetectResponse(BaseModel):
    map: str = Field(..., description="The loaded map, which the pose is on.")
    x: float = Field(..., description="The landmark's centre, metres.")
    y: float = Field(..., description="The landmark's centre, metres.")
    z: float = Field(
        ...,
        description=(
            "The ground height the robot stands on, metres: the floor the "
            "landmark is on, and a waypoint's z. Not the landmark's own height."
        ),
    )
    theta: float = Field(
        ...,
        description=(
            "Where the landmark faces, degrees: out of its face, towards a "
            "robot in front of it."
        ),
    )
    marker_z: float = Field(..., description="The landmark's centre height, metres.")
    consecutive: int = Field(..., description="Sightings in a row behind this pose.")
    age: float = Field(..., description="Seconds from the image to the answer.")
    reprojection_error_px: float = Field(
        ..., description="The fit's residual in pixels; -1 when it has none."
    )


def init_landmark_router(
    logger: structlog.stdlib.BoundLogger,
    landmark_gw: LandmarkGateway,
    robot_repo: RobotRepo,
    map_catalog_repo: MapCatalogRepo,
) -> APIRouter:
    landmark_router = APIRouter(prefix="", tags=["Landmarks"])

    # One detect at a time: each is a poll of several seconds on the one
    # detector, and a second caller's answer would be the same sighting.
    detecting = threading.Lock()

    def _localized_z() -> float:
        state = robot_repo.get_robot_state()
        if state is None or not state.localization_valid:
            raise ConflictError(
                "The robot is not localized, so what it sees has no place on the map.",
                code="not_localized",
            )
        return state.localization_status.position.z

    # Plain `def`: the detect blocks for seconds on ROS.
    @landmark_router.post(
        "/api/v1/landmarks/detect",
        response_model=LandmarkDetectResponse,
        summary="Find a landmark with the robot's camera, on the loaded map",
    )
    def detect_landmark(req: Optional[LandmarkDetectRequest] = None):
        landmark_id = req.landmark_id if req is not None else DEFAULT_LANDMARK_ID

        active = map_catalog_repo.active_name()
        if not active:
            raise ConflictError("No map is loaded.", code="no_map_loaded")
        _localized_z()

        if not detecting.acquire(blocking=False):
            raise ConflictError("A detection is already running.", code="detect_running")
        try:
            ok, message, sighting = landmark_gw.detect(landmark_id)
        finally:
            detecting.release()

        if not ok:
            code = failure_code(message)
            if code is Failure.LANDMARK_NOT_DETECTED:
                raise ConflictError(message, code=code.value)
            if code is Failure.LANDMARK_UNKNOWN:
                raise BadRequestError(message, code=code.value)
            raise UpstreamError(message)

        pose = sighting.pose.pose
        yaw = planar_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        if yaw is None:
            raise ConflictError(
                f"'{landmark_id}' was seen facing the floor or the ceiling, "
                "so it has no heading on the map.",
                code=Failure.LANDMARK_NOT_DETECTED.value,
            )

        # Read after the detect, not before: the floor is the one the robot
        # stood on when it saw the landmark.
        ground_z = _localized_z()

        logger.info(
            "[LandmarkRouter] Landmark detected",
            landmark_id=landmark_id,
            map=active,
            x=pose.position.x,
            y=pose.position.y,
        )
        return LandmarkDetectResponse(
            map=active,
            x=pose.position.x,
            y=pose.position.y,
            z=ground_z,
            theta=math.degrees(yaw),
            marker_z=pose.position.z,
            consecutive=sighting.consecutive,
            age=sighting.age,
            reprojection_error_px=sighting.reprojection_error_px,
        )

    return landmark_router
