from datetime import datetime, timezone
from enum import Enum
from typing import List, Optional

import structlog
from fastapi import APIRouter, Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.failure import Failure, failure_code
from syncai_backend.gateways.recording.recording import (
    DEFAULT_TOPICS,
    RESERVED_NAMES,
    RecordingGateway,
)
from syncai_backend.repositories.recording.catalog import (
    RecordingCatalogRepo,
    StoredRecording,
)


# A boundary guard, not a resource limit: a malformed body must not become a
# huge argv.
_MAX_TOPICS = 64


class RecordingStatus(str, Enum):
    """How a recording on disk ended.

    ``recording`` is the live one, ``ok`` a bag with a metadata.yaml, and
    ``interrupted`` (derived, not stored) a directory with neither: the recorder
    died without flushing. Its messages survive; `ros2 bag reindex <dir>`
    rebuilds the metadata.
    """

    RECORDING = "recording"
    OK = "ok"
    INTERRUPTED = "interrupted"


class StartRecordingRequest(BaseModel):
    name: Optional[str] = Field(
        None,
        description="Directory name under record/. Defaults to a UTC timestamp "
        "(rec_YYYYmmdd_HHMMSS). Letters, digits, dot, dash and underscore only — "
        "rosbag2 names its split files after it.",
    )
    topics: Optional[List[str]] = Field(
        None,
        description="Topics to record. A name without a leading slash is resolved "
        "under this robot's namespace (`livox/lidar` → `/<robot_id>/livox/lidar`); "
        "an absolute name is passed through, which is how `/tf` and `/tf_static` "
        "are recorded. Defaults to the LIO inputs "
        f"({', '.join(DEFAULT_TOPICS)}), the pair a lost mapping run is replayed "
        "from.",
    )
    compression: bool = Field(
        False,
        description="Compress each closed 2 GB split with zstd. Off by default: "
        "the compression runs on the recorder's own thread in a burst, and the "
        "run worth recording is usually the one where the Tegra is already busy "
        "with LIO.",
    )


class ActiveRecordingResponse(BaseModel):
    name: str
    path: str = Field(..., description="Absolute path of the bag directory.")
    topics: List[str] = Field(..., description="Fully resolved topic names.")
    started_at: datetime
    elapsed_seconds: float
    compression: bool
    size_bytes: int = Field(
        ..., description="Bytes written so far. Walks the bag directory on each call."
    )


class StopRecordingResponse(BaseModel):
    name: str
    path: str
    topics: List[str]
    started_at: datetime
    elapsed_seconds: float
    size_bytes: int
    stopped_by: str = Field(
        ...,
        description="Which rung of the stop ladder ended it: `sigint` (the normal "
        "one), `sigterm`, `sigkill`, `already_exited` or `kill_failed`.",
    )
    complete: bool = Field(
        ...,
        description="Whether metadata.yaml was written. False means the bag needs "
        "`ros2 bag reindex` before it can be played — the messages are still there.",
    )


class RecordingSummary(BaseModel):
    name: str
    status: RecordingStatus
    size_bytes: int
    modified_at: datetime
    duration_seconds: Optional[float] = Field(
        None, description="From the bag metadata; null until the recording finishes."
    )
    message_count: Optional[int] = Field(
        None,
        description="From the bag metadata. A zero here on a finished bag means the "
        "topics were named but never published — the usual sign of a typo, since "
        "nothing refuses a topic that does not exist yet.",
    )
    topics: List[str] = Field(
        default_factory=list,
        description="The topics actually recorded, from the bag metadata. Empty "
        "while the recording is live: rosbag2 writes the list on shutdown.",
    )
    compression: Optional[str] = Field(
        None, description="Compression format, or null for an uncompressed bag."
    )


class ListRecordingsResponse(BaseModel):
    recordings: List[RecordingSummary]


def _summary(stored: StoredRecording, active_name: Optional[str]) -> RecordingSummary:
    """Project one directory, reconciling disk state with the live slot.

    Neither alone tells a recording in progress from one whose process is gone.
    """
    if stored.name == active_name:
        status = RecordingStatus.RECORDING
    elif stored.bag is None:
        status = RecordingStatus.INTERRUPTED
    else:
        status = RecordingStatus.OK

    bag = stored.bag
    return RecordingSummary(
        name=stored.name,
        status=status,
        size_bytes=stored.size_bytes,
        modified_at=stored.modified_at,
        duration_seconds=bag.duration_seconds if bag else None,
        message_count=bag.message_count if bag else None,
        topics=bag.topics if bag else [],
        compression=bag.compression if bag else None,
    )


def init_recording_router(
    logger: structlog.stdlib.BoundLogger,
    recording_gw: RecordingGateway,
    recording_catalog_repo: RecordingCatalogRepo,
) -> APIRouter:
    recording_router = APIRouter(prefix="", tags=["Recording"])

    # Plain `def`: start blocks for the liveness probe, stop until the child
    # flushes (up to ~5 min), and listing walks every bag directory.

    @recording_router.post(
        "/api/v1/recordings", status_code=201, response_model=ActiveRecordingResponse
    )
    def start_recording(request: StartRecordingRequest):
        """Start recording ROS topics into record/<name>/.

        A 201 means the child survived its liveness probe. Every refusal comes
        before anything is spawned: 409 ``recording_running`` (stop the current
        one first) or ``name_taken`` (rosbag2 will not write into an existing
        directory).
        """
        name = (request.name or "").strip() or _default_name()
        if name in RESERVED_NAMES:
            raise BadRequestError(
                f"'{name}' is a reserved name — GET /api/v1/recordings/{name} is "
                "this API's own route and a directory of that name could never "
                "be read back. Pick another."
            )

        topics = request.topics if request.topics is not None else list(DEFAULT_TOPICS)
        topics = [topic for topic in topics if topic.strip()]
        if not topics:
            raise BadRequestError(
                "No topics to record. Omit `topics` for the default "
                f"({', '.join(DEFAULT_TOPICS)}) rather than sending an empty list."
            )
        if len(topics) > _MAX_TOPICS:
            raise BadRequestError(f"Too many topics (> {_MAX_TOPICS}).")

        # Before the name check only for the more actionable sentence; the
        # authoritative one-at-a-time check is the gateway's slot under its lock.
        live = recording_gw.active_name()
        if live is not None:
            raise ConflictError(
                f"This robot is already recording '{live}'. Stop it before "
                "starting another (POST /api/v1/recordings/stop).",
                code="recording_running",
            )

        # exists() also validates the name (via resolve_dir).
        if recording_catalog_repo.exists(name):
            raise ConflictError(
                f"A recording named '{name}' already exists.", code="name_taken"
            )

        success, message, payload = recording_gw.start(
            name=name, topics=topics, compression=request.compression
        )
        if not success:
            logger.error("Failed to start recording", recording=name, message=message)
            _raise_for_start(message, payload)

        logger.info("Recording started", recording=name, topics=payload["topics"])
        return ActiveRecordingResponse(**payload)

    @recording_router.get(
        "/api/v1/recordings/active", response_model=Optional[ActiveRecordingResponse]
    )
    def active_recording():
        """What is being recorded right now, or `null`.

        Its own route so a per-second poll does not walk every bag directory.
        The slot is reaped on every read, so a crashed recorder stops reading
        as active without a stop.
        """
        status = recording_gw.status()
        return ActiveRecordingResponse(**status) if status is not None else None

    @recording_router.post(
        "/api/v1/recordings/stop", response_model=StopRecordingResponse
    )
    def stop_recording():
        """Stop the live recorder and wait for the bag to be closed.

        ``complete`` false means the recorder was killed and the bag needs
        `ros2 bag reindex`, not that data is lost. 409 ``not_recording``.
        """
        success, message, payload = recording_gw.stop()
        if not success:
            raise ConflictError(
                "Nothing is being recorded on this robot.", code="not_recording"
            )

        logger.info(
            "Recording stopped",
            recording=payload["name"],
            stopped_by=payload["stopped_by"],
            complete=payload["complete"],
        )
        return StopRecordingResponse(**payload)

    @recording_router.get("/api/v1/recordings", response_model=ListRecordingsResponse)
    def list_recordings():
        """Every bag under record/, newest first.

        The live slot is sampled once, so a stop mid-scan cannot show two states.
        """
        active_name = recording_gw.active_name()
        return ListRecordingsResponse(
            recordings=[
                _summary(stored, active_name)
                for stored in recording_catalog_repo.list_recordings()
            ]
        )

    @recording_router.delete(
        "/api/v1/recordings/{name}", status_code=204, response_class=Response
    )
    def delete_recording(name: str):
        """Delete record/<name>/ and everything under it. There is no undo.

        409 ``recording_active`` for the live one: the recorder would keep
        writing into an unlinked file.
        """
        if recording_gw.active_name() == name:
            raise ConflictError(
                f"'{name}' is being recorded right now. Stop it first "
                "(POST /api/v1/recordings/stop).",
                code="recording_active",
            )

        recording_catalog_repo.delete_dir(name)
        logger.info("Recording deleted", recording=name)
        return Response(status_code=204)

    return recording_router


def _default_name() -> str:
    """A UTC timestamp, like every other timestamp this backend emits."""
    return datetime.now(timezone.utc).strftime("rec_%Y%m%d_%H%M%S")


def _raise_for_start(message: str, payload: dict) -> None:
    """Map the gateway's failure onto a status code.

    The two tagged failures are 409s; everything else is 502 with the gateway's
    sentence.
    """
    code = failure_code(message)
    if code is Failure.RECORDING_RUNNING:
        raise ConflictError(
            f"This robot is {message}. Stop it before starting another "
            "(POST /api/v1/recordings/stop).",
            code=Failure.RECORDING_RUNNING.value,
        )
    if code is Failure.DISK_LOW:
        raise ConflictError(message, code=Failure.DISK_LOW.value)
    raise UpstreamError(message)
