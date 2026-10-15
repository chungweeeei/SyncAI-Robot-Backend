import asyncio
import hashlib
import math
import os
import struct
import threading
import time
import uuid
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

import structlog
from fastapi import APIRouter, Body, Depends, Request
from fastapi.responses import Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    NotFoundError,
    UpstreamError,
)
from syncai_backend.database.models import MapPoint
from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.gateways.workflow.schema import ActiveTask
from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.helpers.pgm import render_png, render_thumbnail
from syncai_backend.helpers.system_config import (
    set_active_map,
    system_ini_path,
)
from syncai_backend.helpers.pointcloud import (
    cap_points,
    pack_xyz_f32,
    read_pcd_xyz,
    voxel_downsample,
)
from syncai_backend.helpers.map_archive import (
    ArchiveFormat,
    MapArchiveError,
    build_archive,
    extract_archive,
    inspect_archive,
)
from syncai_backend.repositories.map.catalog import (
    POINTCLOUD_PCD,
    GridInfo,
    GridRecordStatus,
    KeepoutZone,
    MapCatalogRepo,
    StoredMap,
)
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.mapping.mapping import (
    MappingState,
    MappingStatusRepo,
)
from syncai_backend.repositories.task.task_template import TaskTemplateRepo

# The conversion lives in services/; this module validates requests against it
# and maps its refusals to status codes.
from syncai_backend.services.gridmap_conversion import (
    FLOOR_REFERENCE_DEFAULT,
    GRIDMAP_BANDS_ABOVE_FLOOR,
    TRAVERSABLE_DEBUG_SUBDIR_NAME,
    GridmapConversionService,
    iso_now,
)


# Decimation for a stored map.pcd. Duplicates PointCloudSubscriber's numbers so
# the router does not depend on a subscriber; keep the two in step.
MAP_CLOUD_VOXEL_SIZE = 0.3
MAP_CLOUD_MAX_POINTS = 300000


# --- Schemas ----------------------------------------------------------------


class VertexType(str, Enum):
    """Semantic role of a map vertex: what the robot does when it visits.

    Stored as a plain string in ``map_vertices``; validated at REST only.

    Narrowed to two in 2026-10 (from GENERAL / ARTIFACT / CHARGER / HOME /
    WAITING). There are no migrations, and a stored row with a retired value
    fails ``MapVertexResponse``, so clear or rewrite those rows before deploying.
    """

    # A plain navigation stop.
    MOVE = "MOVE"
    # A charging dock.
    CHARGE = "CHARGE"


class MapVertexRequest(BaseModel):
    """A vertex to create. The owning map comes from the URL, not the body.

    No ``map_name`` field, so a body cannot contradict the path's map.
    """

    name: str = Field(..., min_length=1, description="Human-readable vertex name.")
    type: VertexType = Field(..., description="Semantic role of the vertex.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")


class MapVertexUpdateRequest(BaseModel):
    """Fields to change on an existing vertex. All optional; omitted ones stay.

    No ``map_name``: moving a vertex between maps is a delete and a create.
    """

    name: Optional[str] = Field(None, min_length=1, description="New vertex name.")
    type: Optional[VertexType] = Field(None, description="New vertex role.")
    x: Optional[float] = Field(None, description="New world x-coordinate (metres).")
    y: Optional[float] = Field(None, description="New world y-coordinate (metres).")
    theta: Optional[float] = Field(None, description="New yaw angle in degrees.")


class MapVertexResponse(BaseModel):
    id: uuid.UUID = Field(..., description="Unique identifier of the vertex.")
    name: str = Field(..., description="Human-readable vertex name.")
    type: VertexType = Field(..., description="Semantic role of the vertex.")
    map_name: str = Field(..., description="Name of the map this vertex belongs to.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")


class DeleteResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the deletion.")


class GridOrigin(BaseModel):
    x: float = Field(..., description="World x of the grid's lower-left corner (m).")
    y: float = Field(..., description="World y of the grid's lower-left corner (m).")
    yaw: float = Field(..., description="Grid rotation in the map frame (radians).")


class GridInfoResponse(BaseModel):
    """Geometry of a stored gridmap.

    ``origin`` is ``gridmap.yaml``'s, so its third component is a **yaw**, not
    the z of an OccupancyGrid origin.
    """

    resolution: float = Field(..., description="Metres per cell.")
    origin: GridOrigin = Field(..., description="Pose of the grid's lower-left cell.")
    width: int = Field(..., description="Grid width in cells.")
    height: int = Field(..., description="Grid height in cells.")


class GridStatus(str, Enum):
    """How this map's 2D gridmap stands: the conversion-status surface.

    The sidecar holds three of these (``GridRecordStatus``); ``interrupted``
    (sidecar says converting, no thread behind it) and ``none`` (no grid, no
    record) are derived. Kept separate from ``GridRecordStatus``: this is the
    client contract, that is the file vocabulary; ``_grid_status`` maps one to
    the other.

    ``none`` (never converted), ``failed`` (reason in ``grid_error``) and
    ``interrupted`` (retry will likely work) need different operator actions,
    so do not collapse them into "no grid".
    """

    NONE = "none"
    CONVERTING = "converting"
    OK = "ok"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


class MapSummaryResponse(BaseModel):
    name: str = Field(..., description="Directory name under the maps root.")
    active: bool = Field(..., description="Whether this is the map the stack was launched with.")
    grid: Optional[GridInfoResponse] = Field(
        None,
        description=(
            "Gridmap geometry, or null when the map has been saved from LIO but "
            "not yet converted to a 2D gridmap."
        ),
    )
    thumbnail: Optional[str] = Field(
        None,
        description=("Path of this map's thumbnail endpoint, or null when it has no grid."),
    )
    has_pointcloud: bool = Field(
        ..., description="Whether map.pcd is present (the 3D localizer's source)."
    )
    grid_status: GridStatus = Field(
        ...,
        description=(
            "How this map's gridmap stands. This is the conversion-status "
            "surface: there is no separate status endpoint and no job resource, "
            "so a client that starts a conversion polls the catalogue until "
            "this leaves `converting`. Unlike the `grid_converting` flag it "
            "replaces, the terminal states are distinguishable — `failed` "
            "carries its reason in `grid_error`, and `interrupted` means the "
            "backend was restarted mid-conversion."
        ),
    )
    grid_error: Optional[str] = Field(
        None,
        description=(
            "Why the last conversion failed, as the pipeline diagnosed it "
            "(an empty cloud, an intensity window that selected no ground, an "
            "oversized grid, a missing open3d). Null for every status other "
            "than `failed`."
        ),
    )
    grid_converting: bool = Field(
        ...,
        description=(
            "Deprecated: true exactly when `grid_status` is `converting`. Kept "
            "for the curl/MCP callers written against it; new clients read "
            "`grid_status`, which also tells a failed conversion apart from a "
            "map nobody has converted yet."
        ),
    )
    size_bytes: int = Field(..., description="Total size of the map directory.")
    modified_at: str = Field(
        ..., description="ISO 8601 timestamp of the newest file in the directory."
    )
    vertex_count: int = Field(..., description="Number of stored vertices belonging to this map.")


class CreateMapRequest(BaseModel):
    name: str = Field(
        ...,
        min_length=1,
        max_length=64,
        description=(
            "Directory name for the new map (letters, digits, dot, dash, "
            "underscore). Becomes map/<name>/ on the robot."
        ),
    )


class CreateMapResponse(BaseModel):
    name: str = Field(..., description="The map that was created.")
    has_pointcloud: bool = Field(..., description="Whether pgo wrote map.pcd (true on any 200).")
    grid_pending: bool = Field(
        ...,
        description=(
            "Whether the pcd -> gridmap conversion was started in the "
            "background. The map lists with grid: null until it finishes, so "
            "watch the catalogue's grid_status for the outcome rather than "
            "treating this as one — POST /api/v1/maps/{name}/grid/convert "
            "(re)runs a conversion that failed or never started."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class ResetMappingResponse(BaseModel):
    reset: bool = Field(
        ...,
        description=(
            "True on any 200 — pgo dropped its keyframes and the LIO front end "
            "is re-initialising. A failure is a 502, never a 200 with false."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class StartMappingResponse(BaseModel):
    started: bool = Field(
        ...,
        description=(
            "True on any 200 — pgo is building a map from where the robot "
            "stands and the LIO front end is re-initialising. A failure is a "
            "502 (or a 409 while already mapping), never a 200 with false."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class MappingRunState(str, Enum):
    """The run state as REST reports it.

    The repo's three states plus ``unknown``: no pgo sample yet (every
    navigating robot) or the last one is older than its TTL.
    """

    IDLE = "idle"
    MAPPING = "mapping"
    RESETTING = "resetting"
    UNKNOWN = "unknown"


class MappingStatusResponse(BaseModel):
    state: MappingRunState = Field(
        ...,
        description=(
            "idle: in mapping mode with nothing banked — press Start. "
            "mapping: keyframes accumulating. resetting: a start or reset is "
            "in flight (seconds). unknown: pgo has not reported, which is the "
            "answer on every navigating robot."
        ),
    )
    key_poses: int = Field(..., description="Keyframes in pgo's graph; 0 unless mapping.")
    loop_closures: int = Field(..., description="Loop closures the graph has accepted.")


class RenameMapRequest(BaseModel):
    name: str = Field(
        ...,
        min_length=1,
        max_length=64,
        description=(
            "The new directory name (letters, digits, dot, dash, underscore). "
            "map/<old>/ becomes map/<name>/ on the robot."
        ),
    )


class RenameMapResponse(BaseModel):
    old_name: str = Field(..., description="The name the map had before the rename.")
    name: str = Field(..., description="The name it has now.")
    vertices_moved: int = Field(
        ..., description="Stored vertices re-keyed from old_name to name."
    )
    templates_moved: int = Field(
        ..., description="Task templates whose map binding was re-keyed."
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class DeleteMapResponse(BaseModel):
    """DELETE /api/v1/maps/{name} — what the delete took with it.

    Not the bare ``DeleteResponse``: the card unmounts on success, so the count
    travels inside ``message``.
    """

    name: str = Field(..., description="The map that was deleted.")
    vertices_deleted: int = Field(
        ..., description="Stored vertices removed along with the map directory."
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class ImportMapResponse(BaseModel):
    """POST /api/v1/maps/import — what landed on this robot."""

    name: str = Field(..., description="The map directory the archive became.")
    replaced: bool = Field(
        ..., description="True when a map of that name existed and was replaced."
    )
    files: int = Field(
        ..., description="Files written under map/<name>/, every one md5-verified."
    )
    bytes: int = Field(..., description="Total bytes written.")
    vertices_created: int = Field(
        ..., description="Vertices recreated from the archive's manifest."
    )
    vertices_deleted: int = Field(
        ...,
        description=(
            "Vertices that named this map before the import and were removed — "
            "the replaced map's, or rows orphaned by an earlier hand-deleted directory."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class AutoLocalizeResponse(BaseModel):
    """POST /api/v1/localization/auto — where the localizer found the robot."""

    localized: bool = Field(
        ...,
        description=(
            "True on any 200: the search found a pose and the localizer has "
            "registered against a fresh scan from it, so there is nothing left "
            "to poll. A failure is a 502 (or a 409), never a 200 with false, "
            "and leaves localization as it was."
        ),
    )
    message: str = Field(..., description="The localizer's summary of the match.")
    x: float = Field(..., description="Map frame, metres.")
    y: float = Field(..., description="Map frame, metres.")
    z: float = Field(..., description="Map frame, metres.")
    theta: float = Field(..., description="Heading in degrees, map frame.")
    fitness: float = Field(
        ...,
        description="Mean squared nearest-neighbour distance of the match, m².",
    )
    inlier_ratio: float = Field(
        ...,
        description=(
            "Fraction of the scan within the inlier distance of the map after "
            "alignment (0-1) — the search's acceptance gate."
        ),
    )
    candidates: int = Field(
        ..., description="How many (position, heading) candidates were scored."
    )


class ActivateMapResponse(BaseModel):
    """POST /api/v1/maps/{name}/activate — the outcome of a live map switch."""

    name: str = Field(..., description="The map the robot is on now.")
    previous: Optional[str] = Field(
        None, description="The map it was on before, if there was one."
    )
    switched: bool = Field(
        ...,
        description=(
            "False when the map was already active and nothing was touched — "
            "the no-op, not a failure."
        ),
    )
    localized: Optional[bool] = Field(
        ...,
        description=(
            "Whether the localizer's registration converged inside the short "
            "wait after the swap. False means 'not yet' rather than 'never' — it "
            "retries indefinitely — and null means it could not be asked. Do not "
            "read RobotState.localization_valid for this: that is TF-presence "
            "only and reads true against a map the robot was never localized in."
        ),
    )
    keepout_reloaded: Optional[bool] = Field(
        None,
        description=(
            "Whether filter_mask_server now serves this map's forbidden zones "
            "(its keepout.yaml, written blank first if the map had none). A map "
            "switch does not move the mask on its own, so the switch does this "
            "too, best-effort: false means the switch succeeded but the old map's "
            "zones may still be in force until the nav stack restarts; null on "
            "the switched: false no-op."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class GridRecipe(str, Enum):
    """The two pcd -> gridmap recipes an operator can convert with.

    No "auto" member: the automatic pick was removed 2026-09 (see GRIDMAP_RECIPE
    in services/gridmap_conversion.py).
    """

    Z_BAND = "z-band"
    TRAVERSABILITY = "traversability"


class FloorReference(str, Enum):
    """Which floor the z-band recipe's bands are offsets from.

    ``local`` bands each point against the floor near its keyframe, so a map
    whose LIO trajectory drifted in z converts correctly. ``global`` is one
    level for the whole cloud: the escape hatch, and ``local``'s fallback
    without a poses.txt. Mirrors FLOOR_REFERENCES in
    services/gridmap_conversion.py.
    """

    LOCAL = "local"
    GLOBAL = "global"


class ZBandOffsets(BaseModel):
    """Per-request overrides for the z-band recipe's bands.

    Offsets from the floor (per ``floor_reference``), as in
    GRIDMAP_BANDS_ABOVE_FLOOR; never absolute z. Omitted fields keep the
    recipe's values. The ranges are sanity rails against typos, not tuning
    advice.
    """

    floor_zmin: Optional[float] = Field(None, ge=-2.0, le=2.0)
    floor_zmax: Optional[float] = Field(None, ge=-2.0, le=2.0)
    zmin: Optional[float] = Field(None, ge=-2.0, le=5.0)
    zmax: Optional[float] = Field(None, ge=-2.0, le=8.0)


class ConvertGridRequest(BaseModel):
    """POST /api/v1/maps/{name}/grid/convert — (re)build the 2D gridmap.

    Only ``recipe`` is surfaced in the frontend; the rest are tuning
    parameters for curl / MCP.
    """

    recipe: GridRecipe = Field(
        GridRecipe.Z_BAND,
        description=(
            "Which recipe converts. z-band (the default everywhere) is trinary "
            "and recoverable; traversability is the opt-in for a site too large "
            "to hand-edit, and its output walls off everything it did not "
            "observe, permanently."
        ),
    )
    gap_fill_size: Optional[float] = Field(
        None,
        gt=0.0,
        le=3.0,
        description=(
            "Traversability only: widest hole (m) bridged in the traversable "
            "cloud. The one parameter a site normally needs tuned."
        ),
    )
    z_band_offsets: Optional[ZBandOffsets] = Field(
        None, description="z-band only: band offsets from the floor."
    )
    floor_reference: Optional[FloorReference] = Field(
        None,
        description=(
            "z-band only: which floor the bands are offsets from. Omitted means "
            f"'{FLOOR_REFERENCE_DEFAULT}' (per-keyframe, follows a z-drifted "
            "map); 'global' is one level for the whole cloud, the older "
            "behaviour. Recorded in gridmap.recipe.json either way."
        ),
    )
    debug: bool = Field(
        False,
        description=(
            "Traversability: also write the segmentation's intermediate clouds "
            "into <map>/traversable_debug/ for tuning. They inflate the size "
            "the catalogue reports; delete the directory when done."
        ),
    )
    overwrite_edits: bool = Field(
        False,
        description=(
            "Confirm discarding hand edits. A map whose gridmap was edited "
            "refuses to re-convert without this; even with it, the edited grid "
            "survives as gridmap_prev.pgm."
        ),
    )
    reason: Optional[str] = Field(
        None,
        max_length=500,
        description="Why this recipe/override, recorded in gridmap.recipe.json.",
    )


class ConvertGridResponse(BaseModel):
    name: str = Field(..., description="The map being converted.")
    started: bool = Field(
        ...,
        description=(
            "The conversion thread started — same contract as grid_pending: the "
            "grid appears later or never. Poll the catalogue's grid_status for "
            "the outcome; a failure lands there as `failed` plus a reason."
        ),
    )
    recipe: GridRecipe = Field(..., description="The recipe that is converting.")
    message: str = Field(..., description="What happened, for the operator to read.")


class SaveGridmapResponse(BaseModel):
    name: str = Field(..., description="The map that was written.")
    etag: str = Field(
        ...,
        description=(
            "Strong ETag of the gridmap now on disk — the same value a "
            "subsequent GET of /image or /thumbnail answers with."
        ),
    )
    active: bool = Field(..., description="Whether this is the map the stack was launched with.")
    reloaded: bool = Field(
        ...,
        description=(
            "Whether the running map_server re-read the map and re-published it. "
            "False for any map that is not the active one, and for an active map "
            "whose reload failed — the save itself succeeded either way."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


class ZonePoint(BaseModel):
    """A corner of a forbidden zone, in the map frame (metres) like a vertex."""

    x: float
    y: float


class KeepoutZoneRequest(BaseModel):
    id: Optional[str] = Field(
        None,
        max_length=64,
        description=(
            "The console's handle for this zone. Kept as given when present, "
            "generated when absent; ids must be unique within the request."
        ),
    )
    points: List[ZonePoint] = Field(
        ...,
        description=(
            "Corners in drawing order, not closed (do not repeat the first point). "
            "At least three, all finite."
        ),
    )


class KeepoutZoneResponse(BaseModel):
    id: str
    points: List[ZonePoint]


class SaveKeepoutRequest(BaseModel):
    """PUT /api/v1/maps/{name}/keepout — the whole zone list, replacing the old one."""

    zones: List[KeepoutZoneRequest] = Field(
        ...,
        description=(
            "Every forbidden zone of the map. An empty list clears them: the "
            "mask is rewritten all-unknown and reloaded, the files stay."
        ),
    )


class KeepoutResponse(BaseModel):
    """GET /api/v1/maps/{name}/keepout — what keepout.json records."""

    name: str = Field(..., description="The map the zones belong to.")
    zones: List[KeepoutZoneResponse] = Field(
        ...,
        description=(
            "The zones as last saved through this API. Empty for a map whose "
            "mask was never drawn here — including one carrying only the blank "
            "mask the nav session writes at boot, or a hand-authored one."
        ),
    )
    active: bool = Field(..., description="Whether this is the map the stack is running on.")


class SaveKeepoutResponse(BaseModel):
    name: str = Field(..., description="The map that was written.")
    zones: List[KeepoutZoneResponse] = Field(
        ..., description="The zones as saved, ids filled in."
    )
    active: bool = Field(..., description="Whether this is the map the stack is running on.")
    reloaded: bool = Field(
        ...,
        description=(
            "Whether the running filter_mask_server re-read the mask, so the "
            "planner enforces the new zones now. False for any map that is not "
            "the active one, and for an active map whose reload failed — the "
            "save itself succeeded either way and the zones apply when the nav "
            "stack next starts on this map."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")


# --- Helpers ----------------------------------------------------------------


def _validate_zones(zones: List[KeepoutZoneRequest]) -> List[KeepoutZone]:
    """Turn the request's zones into repo zones, or raise a 400 with a sentence.

    The pydantic models are structural only, so these rules answer 400 with a
    sentence rather than pydantic's 422 list.
    """
    seen: set = set()
    result: List[KeepoutZone] = []
    for index, zone in enumerate(zones, start=1):
        if len(zone.points) < 3:
            raise BadRequestError(
                f"Zone {index} has {len(zone.points)} point"
                f"{'' if len(zone.points) == 1 else 's'}; a forbidden zone needs at least 3."
            )
        for point in zone.points:
            if not (math.isfinite(point.x) and math.isfinite(point.y)):
                raise BadRequestError(
                    f"Zone {index} has a corner that is not a finite coordinate."
                )
        zone_id = zone.id if zone.id is not None else uuid.uuid4().hex
        if not zone_id:
            raise BadRequestError(f"Zone {index} has an empty id.")
        if zone_id in seen:
            raise BadRequestError(f"Zone id {zone_id!r} appears more than once.")
        seen.add(zone_id)
        result.append(
            KeepoutZone(
                id=zone_id,
                points=tuple((point.x, point.y) for point in zone.points),
            )
        )
    return result


def _zone_responses(zones: List[KeepoutZone]) -> List[KeepoutZoneResponse]:
    return [
        KeepoutZoneResponse(id=zone.id, points=[ZonePoint(x=x, y=y) for x, y in zone.points])
        for zone in zones
    ]


def _vertex_response(vertex: MapPoint) -> MapVertexResponse:
    return MapVertexResponse(
        id=vertex.id,
        name=vertex.name,
        type=vertex.type,
        map_name=vertex.map,
        x=vertex.x,
        y=vertex.y,
        theta=vertex.theta,
    )


def _grid_status(stored: StoredMap, converting: bool) -> Tuple[GridStatus, Optional[str]]:
    """Reconcile the on-disk conversion record with the live thread registry.

    Lives in the router because it needs both disk state and this process's
    thread registry. The check order is the logic:

    - A live thread wins: the sidecar write is best-effort, so the registry is
      the authority while the process is up.
    - A sidecar saying ``converting`` with no thread is ``interrupted`` (backend
      restart, killed pane, ``switch_mode``); nothing will clear it.
    - ``failed`` outranks a present grid: a failed *re*-conversion leaves the
      previous grid on disk, and ``ok`` would report it as the requested one.
    - Otherwise a grid is ``ok`` (including pre-status sidecars), else ``none``.
    """
    if converting:
        return GridStatus.CONVERTING, None

    record = stored.grid_record
    if record is not None:
        if record.status is GridRecordStatus.CONVERTING:
            return GridStatus.INTERRUPTED, None
        if record.status is GridRecordStatus.FAILED:
            return GridStatus.FAILED, record.error

    return (GridStatus.OK if stored.grid is not None else GridStatus.NONE), None


def _summary(
    stored: StoredMap,
    active_name: Optional[str],
    vertex_count: int,
    conversion_svc: GridmapConversionService,
) -> MapSummaryResponse:
    grid = (
        GridInfoResponse(
            resolution=stored.grid.resolution,
            origin=GridOrigin(
                x=stored.grid.origin[0],
                y=stored.grid.origin[1],
                yaw=stored.grid.origin[2],
            ),
            width=stored.grid.width,
            height=stored.grid.height,
        )
        if stored.grid is not None
        else None
    )

    # Read the registry once so grid_status and its deprecated alias can never
    # disagree.
    converting = conversion_svc.is_converting(stored.name)
    status, error = _grid_status(stored, converting)

    return MapSummaryResponse(
        name=stored.name,
        active=stored.name == active_name,
        grid=grid,
        thumbnail=(f"/api/v1/maps/{stored.name}/thumbnail" if stored.grid is not None else None),
        has_pointcloud=stored.has_pointcloud,
        grid_status=status,
        grid_error=error,
        grid_converting=converting,
        size_bytes=stored.size_bytes,
        modified_at=stored.modified_at.isoformat().replace("+00:00", "Z"),
        vertex_count=vertex_count,
    )


def _content_tag(data: bytes) -> str:
    """Build a strong ETag over the gridmap's own bytes.

    Not ``(mtime, size)``: an edited gridmap keeps its size and this filesystem's
    mtime is coarse, so the pair survives a save and the pre-edit image would be
    served. One blake2b pass costs a few ms.
    """
    return f'"{hashlib.blake2b(data, digest_size=16).hexdigest()}"'


def _not_modified(request: Request, tag: str) -> bool:
    """Report whether If-None-Match already names this content (for a 304)."""
    header = request.headers.get("if-none-match")
    return bool(header) and tag in [value.strip() for value in header.split(",")]


# --- Router -----------------------------------------------------------------


def init_map_router(
    logger: structlog.stdlib.BoundLogger,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
    task_template_repo: TaskTemplateRepo,
    workflow_gw: WorkflowGateway,
    conversion_svc: GridmapConversionService,
    mapping_status_repo: MappingStatusRepo,
) -> APIRouter:

    map_router = APIRouter(prefix="", tags=["Map"])

    # pgo's latched run state, or None if it has not reported. The run routes
    # turn pgo's refusals into a coded 409 instead of a 502. Check-then-act,
    # not a lock: pgo's own precondition still holds; None lets pgo answer.
    def _run_state() -> Optional[MappingState]:
        status = mapping_status_repo.get()
        return None if status is None else status.state

    def _refuse_while_resetting():
        if _run_state() is MappingState.RESETTING:
            raise ConflictError(
                "The robot is starting or resetting a map right now; try again "
                "in a moment.",
                code="mapping_busy",
            )

    def _refuse_while_idle(nothing_to: str):
        state = _run_state()
        if state is MappingState.IDLE:
            raise ConflictError(
                f"There is no map to {nothing_to} — mapping has not been started.",
                code="mapping_idle",
            )
        _refuse_while_resetting()

    # --- The loaded map -----------------------------------------------------

    # Plain (non-async) handlers throughout: they block on disk, PNG encoding
    # and psycopg2, so FastAPI must run them in its thread pool.

    # Renderings are re-encoded only when their source changes. Keyed by map
    # name, one entry per map.
    thumbnail_cache: Dict[str, Tuple[str, bytes]] = {}
    image_cache: Dict[str, Tuple[str, bytes]] = {}
    cloud_cache: Dict[str, Tuple[Tuple[int, int], bytes]] = {}

    def _vertex_count(name: str) -> int:
        # count_vertices, not len(list_vertices(...)): called once per map on
        # every listing, so avoid hydrating every vertex.

        return map_repo.count_vertices(map=name)

    def _require(name: str) -> StoredMap:
        stored = map_catalog_repo.get_map(name)
        if stored is None:
            raise NotFoundError(f"No map named '{name}' on this robot.")
        return stored

    async def _running_tasks(change: str) -> List[ActiveTask]:
        """What is running now, or 409 ``tasks_unknown`` if that cannot be known.

        Refuse rather than assume idle: the gateway caches failures too, so an
        error here is a real outage. ``change`` names what was refused.
        """
        try:
            tasks, _ = await workflow_gw.list_active_tasks()
        except Exception as exc:
            logger.warning(
                "Could not confirm the robot is idle before changing a map",
                change=change,
                error=str(exc),
            )
            raise ConflictError(
                "Could not confirm that no job is running, and "
                f"{change} under a moving robot is not safe to guess at. Check "
                "the Tasks page and try again.",
                code="tasks_unknown",
            )
        return tasks

    async def _refuse_while_a_job_holds(name: str) -> None:
        """409 ``task_running`` while a running job's positions are on ``name``.

        Guards every route that edits what the planner, costmaps or a job's
        waypoints read (vertices, grid, rebuild, keepout). Async so the Temporal
        call runs on the event loop while the plain ``def`` route still runs in
        the threadpool.

        A map other than the loaded one passes without asking Temporal: a run
        can only hold the loaded map, and editing an idle map should not need
        Temporal up. Check-then-act, not a lock; execute_move's own check stops
        a run that slipped through the window.
        """
        active = map_catalog_repo.active_name()
        if name != active:
            return
        tasks = await _running_tasks("changing the map in use")
        if any(task.map_in_use(active) == name for task in tasks):
            raise ConflictError(
                "The robot is running a job on this map. Cancel it or wait for "
                "it to finish before changing the map.",
                code="task_running",
            )

    job_lock = [Depends(_refuse_while_a_job_holds)]

    def _refuse_if_in_use(name: str, *, verb: str, template_consequence: str) -> None:
        """The three 409s that guard destroying a map's contents, in order.

        Shared by delete and by an import that replaces a map:

        - ``map_active``: map_server and the localizer hold ``map/<name>/``.
        - ``conversion_running``: the thread would write its sidecar into a
          directory that is gone (or into the *new* one).
        - ``template_bound``: a template's MOVE steps name this map's vertex ids,
          which would dangle. The operator is told which templates to unbind;
          they are never deleted silently. Map-independent templates are
          ignored.
        """
        if name == map_catalog_repo.active_name():
            raise ConflictError(
                f"'{name}' is the map the stack is running on and cannot be "
                f"{verb} while it is in use. Switch the robot to another map "
                "first.",
                code="map_active",
            )
        if conversion_svc.is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; try again "
                "once the conversion has finished.",
                code="conversion_running",
            )

        bound = task_template_repo.list_task_templates(
            map_name=name, include_map_independent=False
        )
        if bound:
            names = ", ".join(sorted(f"'{row.name}'" for row in bound))
            raise ConflictError(
                f"{len(bound)} task "
                f"{'template' if len(bound) == 1 else 'templates'} still "
                f"{'targets' if len(bound) == 1 else 'target'} '{name}' "
                f"({names}). Point {'it' if len(bound) == 1 else 'them'} at "
                "another map or delete "
                f"{'it' if len(bound) == 1 else 'them'} first — "
                f"{template_consequence}",
                code="template_bound",
            )

    @map_router.get("/api/v1/maps", response_model=List[MapSummaryResponse])
    def list_maps():
        active_name = map_catalog_repo.active_name()
        return [
            _summary(stored, active_name, _vertex_count(stored.name), conversion_svc)
            for stored in map_catalog_repo.list_maps()
        ]

    @map_router.get("/api/v1/maps/{name}", response_model=MapSummaryResponse)
    def get_map(name: str):
        stored = _require(name)
        return _summary(
            stored,
            map_catalog_repo.active_name(),
            _vertex_count(name),
            conversion_svc,
        )

    @map_router.post("/api/v1/maps", response_model=CreateMapResponse)
    def create_map(request: CreateMapRequest):
        """Save the run as a new map -- and end it.

        A successful ``pgo/save_maps`` returns pgo to IDLE (it clears
        ``/dev/shm`` itself); the next run needs ``POST /api/v1/mapping/start``.
        409 ``mapping_idle`` while pgo is IDLE, before any directory is made,
        instead of pgo's ``NO POSES!`` as a 502.

        The conversion first waits for pgo to clean people out of map.pcd
        (``gridmap_conversion.wait_for_map_clean``), so ``grid_status`` stays
        ``converting`` for seconds to minutes.
        """
        _refuse_while_idle("save")

        directory = map_catalog_repo.create_map_dir(request.name)

        saved, detail = map_gw.save_map(directory)
        if not saved:
            map_catalog_repo.discard_empty_map_dir(request.name)
            logger.error("Failed to save map", map=request.name, error=detail)
            raise UpstreamError(detail)

        grid_pending = conversion_svc.start(request.name, directory)

        return CreateMapResponse(
            name=request.name,
            has_pointcloud=True,
            grid_pending=grid_pending,
            message=(
                f"Saved '{request.name}'. Mapping has stopped — start it again "
                "for another map."
                + (
                    " Converting to a 2D gridmap in the background, once the "
                    "robot has removed people from the point cloud."
                    if grid_pending
                    else (
                        " Re-run the conversion through "
                        f"POST /api/v1/maps/{request.name}/grid/convert."
                    )
                )
            ),
        )

    @map_router.get("/api/v1/maps/{name}/export")
    def export_map(name: str, format: ArchiveFormat = ArchiveFormat.ZIP):
        """The map directory as one archive, with its vertices and an md5 manifest.

        Every file under ``map/<name>/`` except ``traversable_debug/``, plus
        ``syncai_map.json`` (name, vertices without ids, md5 per file); layout
        in ``helpers/map_archive``. 409 ``conversion_running``, since the grid
        is about to be replaced. Exporting the active map is fine.
        """
        _require(name)
        if conversion_svc.is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; export it once "
                "the conversion has finished.",
                code="conversion_running",
            )

        vertices = [
            {"name": v.name, "type": v.type, "x": v.x, "y": v.y, "theta": v.theta}
            for v in map_repo.list_vertices(map=name)
        ]
        started = time.perf_counter()
        data = build_archive(
            map_catalog_repo.resolve_dir(name),
            name,
            vertices,
            format,
            exported_at=iso_now(),
            exclude_dirs=(TRAVERSABLE_DEBUG_SUBDIR_NAME,),
        )

        logger.info(
            "Exported map",
            map=name,
            format=format.value,
            bytes=len(data),
            vertices=len(vertices),
            build_ms=round((time.perf_counter() - started) * 1000),
        )
        return Response(
            content=data,
            media_type=format.media_type,
            headers={
                # No quoting needed: _NAME_RE admits only [A-Za-z0-9._-].
                "Content-Disposition": f'attachment; filename="{name}.{format.extension}"',
                "Cache-Control": "no-store",
            },
        )

    @map_router.post(
        "/api/v1/maps/import", status_code=201, response_model=ImportMapResponse
    )
    def import_map(
        name: Optional[str] = None,
        payload: bytes = Body(..., media_type="application/octet-stream"),
    ):
        """Create — or replace — a map from an archive ``export_map`` produced.

        The raw body is the archive (zip or tar.gz by magic bytes); the target
        is ``?name=`` or the manifest's name.

        Nothing is written until the archive is judged: a missing or mismatched
        manifest, a link or escaping member is 400, as is no ``map.pcd``, an
        unknown vertex type or a bad name. Replacing an existing map has
        delete's 409s (``map_active`` / ``conversion_running`` /
        ``template_bound``); 409 ``disk_low`` when the unpacked size exceeds
        free space (the replaced map's space does not count).

        Order: unpack and verify into ``map/.import-<token>/``, publish with one
        ``os.rename`` (parking the old map as ``.import-old-<token>``), then one
        transaction deletes every row naming the map (orphans too) and inserts
        the manifest's vertices; a DB failure swaps the directories back, and
        the parked ``rmtree`` runs last. No conversion is started.
        """
        # Phase timings on the success log decide whether this should go async.
        started = time.perf_counter()
        try:
            inspected = inspect_archive(payload)
        except MapArchiveError as exc:
            raise BadRequestError(str(exc))
        inspect_ms = round((time.perf_counter() - started) * 1000)
        manifest = inspected.manifest

        if POINTCLOUD_PCD not in manifest.files:
            raise BadRequestError(
                f"The archive has no {POINTCLOUD_PCD}; it is not a map this robot "
                "can localise on."
            )
        known_types = {member.value for member in VertexType}
        unknown = sorted(
            {str(v["type"]) for v in manifest.vertices if v["type"] not in known_types}
        )
        if unknown:
            raise BadRequestError(
                f"Unknown vertex type(s) in the manifest: {', '.join(unknown)}."
            )

        target = name if name is not None else manifest.name
        map_catalog_repo.resolve_dir(target)
        replacing = map_catalog_repo.get_map(target) is not None
        if replacing:
            _refuse_if_in_use(
                target,
                verb="replaced",
                template_consequence=(
                    "its MOVE steps point at this map's current vertices, which the "
                    "import replaces."
                ),
            )

        if inspected.total_bytes > map_catalog_repo.free_bytes():
            raise ConflictError(
                f"Not enough disk for '{target}': the archive unpacks to "
                f"{inspected.total_bytes} bytes.",
                code="disk_low",
            )

        new_vertices = [
            {
                "name": v["name"],
                "type": v["type"],
                "x": float(v["x"]),
                "y": float(v["y"]),
                "theta": float(v["theta"]),
            }
            for v in manifest.vertices
        ]

        staging = map_catalog_repo.begin_import(target)
        started = time.perf_counter()
        try:
            written = extract_archive(payload, inspected, staging)
            _, displaced = map_catalog_repo.commit_import(target, staging)
        except MapArchiveError as exc:
            map_catalog_repo.abort_import(staging)
            raise BadRequestError(str(exc))
        except BaseException:
            map_catalog_repo.abort_import(staging)
            raise
        extract_ms = round((time.perf_counter() - started) * 1000)

        started = time.perf_counter()
        try:
            with map_repo.transaction(op="import_map") as session:
                vertices_deleted = map_repo.delete_vertices(target, session=session)
                created = (
                    map_repo.create_vertices(target, new_vertices, session=session)
                    if new_vertices
                    else []
                )
        except Exception as exc:
            map_catalog_repo.undo_import(target, displaced)
            logger.error(
                "Map import failed while writing its vertices",
                map=target,
                replaced=replacing,
                error=str(exc),
            )
            raise UpstreamError(
                f"Could not store the vertices for '{target}': {exc}. "
                + (
                    "The previous map was put back."
                    if replacing
                    else "The map directory was removed again; nothing was imported."
                )
            )
        db_ms = round((time.perf_counter() - started) * 1000)

        # The irreversible step last.
        if displaced is not None:
            map_catalog_repo.abort_import(displaced)

        # Renderings of an earlier map of this name are stale.

        thumbnail_cache.pop(target, None)
        image_cache.pop(target, None)
        cloud_cache.pop(target, None)

        logger.info(
            "Imported map",
            map=target,
            replaced=replacing,
            format=inspected.format.value,
            files=len(manifest.files),
            bytes=written,
            vertices_created=len(created),
            vertices_deleted=vertices_deleted,
            inspect_ms=inspect_ms,
            extract_ms=extract_ms,
            db_ms=db_ms,
        )
        return ImportMapResponse(
            name=target,
            replaced=replacing,
            files=len(manifest.files),
            bytes=written,
            vertices_created=len(created),
            vertices_deleted=vertices_deleted,
            message=(
                f"{'Replaced' if replacing else 'Imported'} '{target}': "
                f"{len(manifest.files)} files and {len(created)} "
                f"{'vertex' if len(created) == 1 else 'vertices'}"
                + (
                    f" ({vertices_deleted} previous "
                    f"{'vertex' if vertices_deleted == 1 else 'vertices'} removed)."
                    if vertices_deleted
                    else "."
                )
                # Operator copy shown as-is: name the page, not the endpoint.
                + " Its floor plan, if any, came with it; rebuild one from the "
                "Maps page if it needs it."
            ),
        )

    @map_router.post("/api/v1/mapping/reset", response_model=ResetMappingResponse)
    def reset_mapping_run():
        """Discard the run in the robot's memory and start a new map.

        Touches no file. No body and no "save first" flag: saving is
        POST /api/v1/maps, a separate act. 409 ``mapping_idle`` while pgo is
        IDLE (nothing to discard); other failures are 502 with pgo's sentence
        (typically: not in a mapping session).
        """
        _refuse_while_idle("discard")

        ok, detail = map_gw.reset_mapping()
        if not ok:
            logger.error("Failed to reset the mapping run", error=detail)
            raise UpstreamError(detail)

        return ResetMappingResponse(reset=True, message=detail)

    @map_router.post("/api/v1/mapping/start", response_model=StartMappingResponse)
    def start_mapping():
        """Begin a mapping run: ``pgo/start_mapping``, IDLE -> MAPPING.

        A mapping session starts idle, so the drive to the starting point is
        not mapped; a save closes the run. Always resets LIO (the map origin is
        the odometry origin), so the robot must stay **stationary** until the
        live scan returns.

        409 ``mapping_running`` while a run is on, 409 ``mapping_busy`` while a
        start or reset is in flight, else 502 on pgo's refusal. The status slot
        is left to the latched topic so every console agrees.
        """
        if _run_state() is MappingState.MAPPING:
            raise ConflictError(
                "The robot is already building a map. Save it, or start a new "
                "one with a reset.",
                code="mapping_running",
            )
        _refuse_while_resetting()

        ok, detail = map_gw.start_mapping()
        if not ok:
            logger.error("Failed to start the mapping run", error=detail)
            raise UpstreamError(detail)

        return StartMappingResponse(started=True, message=detail)

    @map_router.get("/api/v1/mapping", response_model=MappingStatusResponse)
    async def get_mapping_status():
        """pgo's run state, as it last reported it.

        ``async``: it only reads a slot, no I/O. ``unknown`` is a normal answer
        (every navigating robot, or pgo quiet past the repo's TTL).
        """
        status = mapping_status_repo.get()
        if status is None:
            return MappingStatusResponse(
                state=MappingRunState.UNKNOWN, key_poses=0, loop_closures=0
            )
        return MappingStatusResponse(
            state=MappingRunState(status.state.value),
            key_poses=status.key_poses,
            loop_closures=status.loop_closures,
        )

    @map_router.patch("/api/v1/maps/{name}", response_model=RenameMapResponse)
    def rename_map(name: str, request: RenameMapRequest):
        """Rename a map: move its directory and re-key the rows that name it.

        Refusals, before any mutation: 404; 409 ``map_active`` (map_server and
        the localizer hold the path — switch away with ``…/activate`` first);
        409 ``conversion_running`` (the thread holds the old path); 400 / 409
        ``name_taken`` from ``rename_map_dir``.

        Filesystem first, database second: the move is the likeliest failure and
        needs no compensation. Vertices and template bindings are re-keyed in
        one transaction; if it fails the directory is moved back.
        """
        _require(name)

        active_name = map_catalog_repo.active_name()
        if name == active_name:
            raise ConflictError(
                f"'{name}' is the map the stack is running on and cannot be "
                "renamed while it is in use. Switch the robot to another map "
                "first.",
                code="map_active",
            )
        if conversion_svc.is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; rename it "
                "once the conversion has finished.",
                code="conversion_running",
            )

        new_dir = map_catalog_repo.rename_map_dir(name, request.name)

        try:
            with map_repo.transaction(op="rename_map") as session:
                vertices_moved = map_repo.move_vertices(
                    name, request.name, session=session
                )
                templates_moved = task_template_repo.rebind_map(
                    name, request.name, session=session
                )
        except Exception as exc:
            # Best-effort compensation; a second failure is logged, not raised
            # over the first.
            old_dir = os.path.join(os.path.dirname(new_dir), name)
            try:
                os.rename(new_dir, old_dir)
                restored = True
            except OSError as undo_exc:
                restored = False
                logger.error(
                    "Could not move the map directory back after a failed rename",
                    map=name,
                    new_name=request.name,
                    error=str(undo_exc),
                )
            logger.error(
                "Map rename failed while re-keying database rows",
                map=name,
                new_name=request.name,
                directory_restored=restored,
                error=str(exc),
            )
            raise UpstreamError(
                f"Could not re-key the rows that name '{name}': {exc}. "
                + (
                    "The map directory was left under its old name."
                    if restored
                    else f"The directory is now map/{request.name}/ but the "
                    f"database still says '{name}' — fix by hand."
                )
            )

        # Never served stale (the cache is content-tagged), but would leak.
        thumbnail_cache.pop(name, None)
        image_cache.pop(name, None)
        cloud_cache.pop(name, None)

        logger.info(
            "Renamed map",
            map=name,
            new_name=request.name,
            vertices_moved=vertices_moved,
            templates_moved=templates_moved,
        )
        message = (
            f"Renamed '{name}' to '{request.name}'. Moved {vertices_moved} "
            f"{'vertex' if vertices_moved == 1 else 'vertices'} and "
            f"{templates_moved} task {'template' if templates_moved == 1 else 'templates'}."
        )
        if templates_moved:
            # A schedule's memo is frozen at registration (routers/schedule.py).
            message += " Schedules already registered keep the old map label."
        return RenameMapResponse(
            old_name=name,
            name=request.name,
            vertices_moved=vertices_moved,
            templates_moved=templates_moved,
            message=message,
        )

    @map_router.delete("/api/v1/maps/{name}", response_model=DeleteMapResponse)
    def delete_map(name: str):
        """Delete a map: remove its directory and the vertices that name it.

        404, then ``_refuse_if_in_use``'s 409s (``map_active``,
        ``conversion_running``, ``template_bound``) before any mutation.

        **Database first, filesystem last** (the inverse of rename): ``rmtree``
        cannot be undone, so it goes last. A failed ``rmtree`` after the rows
        are gone converges on retry; the reverse would leave orphaned rows that
        a future map of the same name silently inherits.

        Registered schedules keep firing their frozen steps; the response says
        so.
        """
        _require(name)
        _refuse_if_in_use(
            name,
            verb="deleted",
            template_consequence=(
                "a template bound to a map that is gone can neither run nor be edited."
            ),
        )

        try:
            vertices_deleted = map_repo.delete_vertices(name)
        except Exception as exc:
            # Nothing to compensate, but say the map is still there.
            logger.error(
                "Map delete failed while removing its vertices",
                map=name,
                error=str(exc),
            )
            raise UpstreamError(
                f"Could not remove the vertices that name '{name}': {exc}. "
                "The map was left in place; try again."
            )

        map_catalog_repo.delete_map_dir(name)

        # Drop the renderings or they sit in memory until restart.
        thumbnail_cache.pop(name, None)
        image_cache.pop(name, None)
        cloud_cache.pop(name, None)

        logger.info("Deleted map", map=name, vertices_deleted=vertices_deleted)
        return DeleteMapResponse(
            name=name,
            vertices_deleted=vertices_deleted,
            message=(
                f"Deleted '{name}' and {vertices_deleted} "
                f"{'vertex' if vertices_deleted == 1 else 'vertices'}. "
                "Schedules already registered against this map keep running "
                "their frozen steps."
            ),
        )

    def _carry_keepout(name: str, grid: GridInfo) -> Tuple[bool, str]:
        """Point filter_mask_server at ``name``'s keepout mask; never raises.

        A map the nav session never booted into has no ``keepout.*``, so the
        pair is written first (from ``keepout.json`` if any, else blank) —
        the previous map's zones must be replaced by something.

        Returns ``(reloaded, note)``; the note is an operator sentence, empty
        when there is nothing to say.
        """
        zones = map_catalog_repo.read_keepout_zones(name)
        yaml_path = map_catalog_repo.keepout_yaml_path(name)
        if yaml_path is None:
            try:
                map_catalog_repo.write_keepout(name, zones, grid)
            except OSError as exc:
                logger.error(
                    "Could not write the keepout mask for the new map",
                    map=name,
                    error=str(exc),
                )
                return False, f" The keepout mask could not be written: {exc}."
            yaml_path = map_catalog_repo.keepout_yaml_path(name)
            if yaml_path is None:
                return False, " The keepout mask could not be written."

        reloaded, detail = map_gw.reload_keepout(yaml_path)
        if not reloaded:
            logger.error(
                "Switched maps but the keepout mask did not reload", map=name, error=detail
            )
            return False, f" The keepout mask could not be reloaded: {detail}."
        if zones:
            count = len(zones)
            return True, f" Its {count} forbidden zone{'s' if count != 1 else ''} are active."
        return True, ""

    @map_router.post(
        "/api/v1/maps/{name}/activate", response_model=ActivateMapResponse
    )
    async def activate_map(name: str):
        """Switch the robot onto another map, live, without restarting the stack.

        Three parts must agree: the localizer's cloud (`relocalize`),
        map_server's grid (`load_map`) and `[map] name` in the INI (what
        `active_name()` reports and a restart reads). The localizer goes first
        because a refusal there mutates nothing; each later step compensates the
        earlier ones on failure. The INI is last and preflighted for
        writability, since an unrecorded swap would leave REST naming the wrong
        map.

        All refusals come first. `stack_not_ready` (e.g. mapping mode) is
        detected by service discoverability, not the cached RobotState, which
        is absent on a robot that lost localization.

        `[initial_pose]` is zeroed: an old-frame pose is meaningless here. The
        operator sets the pose; `localized` says whether registration converged
        shortly after the swap.
        """
        stored = _require(name)

        previous = map_catalog_repo.active_name()
        if name == previous:
            # The no-op: reloading would only throw away a good registration.
            return ActivateMapResponse(
                name=name,
                previous=previous,
                switched=False,
                localized=None,
                message=f"'{name}' is already the map the robot is running on.",
            )

        if conversion_svc.is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; switch to it "
                "once the conversion has finished.",
                code="conversion_running",
            )

        status, grid_error = _grid_status(stored, False)
        if status != GridStatus.OK:
            # Refuse up front, before the localizer is swapped (and an
            # unloadable yaml would crash map_server at the next launch).
            detail = f" ({grid_error})" if grid_error else ""
            raise ConflictError(
                f"'{name}' has no usable gridmap{detail}. Build one from the "
                "map's Rebuild-grid control before switching to it.",
                code="grid_missing",
            )

        pcd_path = map_catalog_repo.pointcloud_path(name)
        if pcd_path is None:
            raise ConflictError(
                f"'{name}' has no map.pcd, so the localizer has nothing to "
                "match against. Only a map saved from a mapping run can be "
                "switched to.",
                code="pointcloud_missing",
            )

        yaml_path = map_catalog_repo.gridmap_yaml_path(name)
        if yaml_path is None:
            raise ConflictError(
                f"'{name}' has no gridmap.yaml to hand map_server.",
                code="grid_missing",
            )

        ini_path = system_ini_path()
        if not os.access(ini_path, os.W_OK):
            # Preflighted so the swap's last step cannot fail after the move.
            raise ConflictError(
                f"The system INI ({ini_path}) is not writable, so a switch "
                "could not be recorded and would not survive a restart.",
                code="ini_not_writable",
            )

        # Any job, not only one holding a map (stricter than
        # _refuse_while_a_job_holds): a switch re-points the localizer.
        active_tasks = await _running_tasks("a map switch")

        if active_tasks:
            running = ", ".join(task.id for task in active_tasks)
            raise ConflictError(
                f"The robot is running {len(active_tasks)} "
                f"{'task' if len(active_tasks) == 1 else 'tasks'} ({running}). "
                "Cancel or wait for it before switching maps.",
                code="task_running",
            )

        # The route is `async def` for the Temporal await (the client belongs to
        # the uvicorn loop); everything below blocks for up to a minute, so it
        # runs in a worker thread. On the loop it froze every WebSocket,
        # including the teleop watchdog.

        def _switch() -> ActivateMapResponse:
            if not map_gw.nav_services_ready():
                raise ConflictError(
                    "map_server and the localizer are not reachable, so there is no "
                    "running map to switch. The robot is in mapping mode, or the "
                    "nav stack is down.",
                    code="stack_not_ready",
                )

            previous_pcd = (
                map_catalog_repo.pointcloud_path(previous) if previous else None
            )

            def _restore_localizer(reason: str) -> str:
                """Put the localizer back on the old cloud. Returns a sentence."""
                if previous_pcd is None:
                    # The INI named no map (the likely case when its write fails),
                    # or that map lost its map.pcd.
                    where = (
                        f"'{previous}' has no map.pcd"
                        if previous
                        else "the INI named no previous map"
                    )
                    return (
                        f" The localizer is now on '{name}' and could not be put "
                        f"back — {where}."
                    )
                restored, detail = map_gw.swap_localizer_map(previous_pcd, 0.0, 0.0, 0.0)
                if restored:
                    return f" The localizer was put back on '{previous}'."
                logger.error(
                    "Could not restore the localizer after a failed map switch",
                    map=name,
                    previous=previous,
                    reason=reason,
                    error=detail,
                )
                return (
                    f" The localizer is still on '{name}' and could not be put back "
                    f"({detail}) — set the map by hand and restart the stack."
                )

            swapped, detail = map_gw.swap_localizer_map(pcd_path, 0.0, 0.0, 0.0)
            if not swapped:
                # Nothing has changed yet, so nothing to compensate.
                logger.error("Map switch failed at the localizer", map=name, error=detail)
                raise UpstreamError(
                    f"Could not point the localizer at '{name}': {detail}. The robot "
                    f"is still on '{previous}'."
                )

            reloaded, detail = map_gw.reload_map(yaml_path)
            if not reloaded:
                logger.error("Map switch failed at map_server", map=name, error=detail)
                raise UpstreamError(
                    f"The localizer moved to '{name}' but map_server would not load "
                    f"its gridmap: {detail}." + _restore_localizer("load_map failed")
                )

            try:
                set_active_map(name, logger)
            except (OSError, ValueError) as exc:
                logger.error(
                    "Map switch failed at the INI write", map=name, error=str(exc)
                )
                # Undone in the order done, as statements: inside the message
                # concatenation the order would follow the sentences.
                restored_cloud = _restore_localizer("INI write failed")

                previous_yaml = (
                    map_catalog_repo.gridmap_yaml_path(previous) if previous else None
                )
                if previous_yaml is None:
                    # Say so rather than leave map_server quietly serving a grid
                    # that does not match the restored cloud.
                    where = (
                        f"'{previous}' has no gridmap.yaml"
                        if previous
                        else "the INI named no previous map"
                    )
                    restored_grid = (
                        f" map_server is still serving '{name}' — {where} to put back."
                    )
                else:
                    back, back_detail = map_gw.reload_map(previous_yaml)
                    restored_grid = (
                        ""
                        if back
                        else f" map_server is still serving '{name}' ({back_detail})."
                    )

                raise UpstreamError(
                    f"The robot moved to '{name}' but the switch could not be "
                    f"recorded in {ini_path}: {exc}."
                    + restored_cloud
                    + restored_grid
                )

            # The switch is recorded; nothing can fail it now. The keepout mask
            # (still the previous map's) is carried over best-effort after this
            # commit point: a mask problem goes in the message, not a failure.
            keepout_reloaded, keepout_note = _carry_keepout(name, stored.grid)

            # Renderings are keyed by map name, so nothing cached is stale.
            localized = map_gw.localization_converged()

            logger.info(
                "Switched the active map",
                map=name,
                previous=previous,
                localized=localized,
                keepout_reloaded=keepout_reloaded,
            )

            message = f"The robot is now on '{name}'"
            message += f" (was '{previous}')." if previous else "."
            if localized:
                message += " The localizer converged on the new map."
            else:
                message += (
                    " The pose was reset to the map origin; set an initial pose on "
                    "the dashboard, because the localizer has not converged"
                )
                message += (
                    " yet." if localized is False else " and could not be asked."
                )
            message += keepout_note
            return ActivateMapResponse(
                name=name,
                previous=previous,
                switched=True,
                localized=localized,
                keepout_reloaded=keepout_reloaded,
                message=message,
            )

        return await asyncio.to_thread(_switch)

    # --- Localization -------------------------------------------------------

    # Held across one auto_localize call: a second press would queue behind the
    # first in the localizer and time out or search again. It is a 409 instead.
    auto_localize_lock = threading.Lock()

    @map_router.post(
        "/api/v1/localization/auto", response_model=AutoLocalizeResponse
    )
    async def auto_localize():
        """Find the robot on the loaded map with no hint — the localizer's
        ``auto_localize``, the recovery for a kidnapped robot.

        No body. Blocks up to ~15 s; a 200 means localized, nothing to poll.
        Writes nothing (not the INI's ``[initial_pose]``).

        409 ``task_running`` while any job runs (it moves the map frame under
        the robot), 409 ``tasks_unknown``, 409 ``localize_running``. Every
        localizer refusal is the uniform 502 with its sentence. ``async`` for
        the Temporal await, blocking half on a worker thread, as activate.
        """
        active_tasks = await _running_tasks("finding the robot")
        if active_tasks:
            running = ", ".join(task.id for task in active_tasks)
            raise ConflictError(
                f"The robot is running {len(active_tasks)} "
                f"{'task' if len(active_tasks) == 1 else 'tasks'} ({running}). "
                "Cancel or wait for it before finding the robot.",
                code="task_running",
            )

        if not auto_localize_lock.acquire(blocking=False):
            raise ConflictError(
                "The robot is already searching for where it stands; wait for "
                "that answer.",
                code="localize_running",
            )

        def _localize() -> AutoLocalizeResponse:
            try:
                ok, detail, found = map_gw.auto_localize()
            finally:
                auto_localize_lock.release()

            if not ok:
                logger.error("Auto-localization failed", error=detail)
                raise UpstreamError(detail)

            return AutoLocalizeResponse(
                localized=True,
                message=detail,
                x=found.x,
                y=found.y,
                z=found.z,
                # REST speaks degrees.
                theta=math.degrees(found.yaw),
                fitness=found.fitness,
                inlier_ratio=found.inlier_ratio,
                candidates=found.candidates,
            )

        # No await between the acquire and this submit, so _localize always
        # runs and releases the lock even if the client goes away.
        return await asyncio.to_thread(_localize)

    @map_router.post(
        "/api/v1/maps/{name}/grid/convert",
        response_model=ConvertGridResponse,
        dependencies=job_lock,
    )
    def convert_map_grid(name: str, request: ConvertGridRequest):
        """(Re)build a map's 2D gridmap from its map.pcd, in the background.

        Picks the recipe, applies parameter overrides, and can dump the
        traversability intermediates for tuning. 409 `conversion_running`:
        try later; 409 `gridmap_hand_edited`: confirm with `overwrite_edits`
        (the edited grid survives as gridmap_prev.pgm).
        """
        stored = _require(name)
        if not stored.has_pointcloud:
            raise BadRequestError(
                f"Map '{name}' has no map.pcd — there is nothing to convert."
            )

        # Cross-field validation as readable 400s, not schema 422s.
        if request.recipe is GridRecipe.Z_BAND and request.gap_fill_size is not None:
            raise BadRequestError(
                "gap_fill_size tunes the traversability recipe; the z-band "
                "recipe takes z_band_offsets."
            )
        if (
            request.recipe is GridRecipe.TRAVERSABILITY
            and request.z_band_offsets is not None
        ):
            raise BadRequestError(
                "z_band_offsets tune the z-band recipe; the traversability "
                "recipe takes gap_fill_size."
            )
        if (
            request.recipe is GridRecipe.TRAVERSABILITY
            and request.floor_reference is not None
        ):
            raise BadRequestError(
                "floor_reference belongs to the z-band recipe; the traversability "
                "recipe segments the floor itself."
            )

        band_overrides: Optional[Dict[str, float]] = None
        if request.z_band_offsets is not None:
            band_overrides = request.z_band_offsets.model_dump(exclude_none=True)
            merged = {**GRIDMAP_BANDS_ABOVE_FLOOR, **band_overrides}
            # Check the *merged* bands: overriding one end can invert a band,
            # which would otherwise fail later in the thread, unseen.
            if merged["floor_zmin"] >= merged["floor_zmax"]:
                raise BadRequestError(
                    f"floor band is inverted: floor_zmin {merged['floor_zmin']} "
                    f">= floor_zmax {merged['floor_zmax']}."
                )
            if merged["zmin"] >= merged["zmax"]:
                raise BadRequestError(
                    f"obstacle band is inverted: zmin {merged['zmin']} >= "
                    f"zmax {merged['zmax']}."
                )

        if map_catalog_repo.gridmap_edited(name) and not request.overwrite_edits:
            raise ConflictError(
                f"'{name}' has a hand-edited gridmap. Re-converting replaces it "
                "(the edited grid is kept as gridmap_prev.pgm) — pass "
                "overwrite_edits to proceed.",
                code="gridmap_hand_edited",
            )

        grid_overrides: Optional[Dict[str, object]] = (
            {"gap_fill_size": request.gap_fill_size}
            if request.gap_fill_size is not None
            else None
        )

        floor_reference = (
            request.floor_reference.value
            if request.floor_reference is not None
            else FLOOR_REFERENCE_DEFAULT
        )

        param_overrides: Dict[str, object] = {}
        if band_overrides:
            param_overrides["z_band_offsets"] = band_overrides
        if request.floor_reference is not None:
            param_overrides["floor_reference"] = floor_reference
        if grid_overrides:
            param_overrides.update(grid_overrides)
        override: Dict[str, object] = {
            "requested": request.recipe.value,
            "picked_by": f"POST /api/v1/maps/{name}/grid/convert",
            "reason": request.reason,
            "at": iso_now(),
            "param_overrides": param_overrides,
        }

        # An active map must reload map_server once the new grid exists, so
        # the reload runs in the conversion thread.
        on_success: Optional[Callable[[], None]] = None
        if name == map_catalog_repo.active_name():

            def _reload() -> None:
                yaml_path = map_catalog_repo.gridmap_yaml_path(name)
                if yaml_path is None:
                    logger.error(
                        "Converted the active map but gridmap.yaml is missing",
                        map=name,
                    )
                    return
                reloaded, detail = map_gw.reload_map(yaml_path)
                if not reloaded:
                    logger.error(
                        "Converted the active map but map_server did not reload",
                        map=name,
                        error=detail,
                    )

            on_success = _reload

        started = conversion_svc.start(
            name,
            map_catalog_repo.resolve_dir(name),
            recipe_request=request.recipe.value,
            floor_reference=floor_reference,
            band_offset_overrides=band_overrides,
            grid_overrides=grid_overrides,
            debug=request.debug,
            override=override,
            archive=lambda: map_catalog_repo.archive_gridmap(name),
            on_success=on_success,
        )
        # started=False needs a race deleting map.pcd; report it, do not assert.
        return ConvertGridResponse(
            name=name,
            started=started,
            recipe=request.recipe,
            message=(
                (
                    f"Converting '{name}' with the {request.recipe.value} recipe "
                    "in the background."
                    + (
                        " The previous gridmap is kept as gridmap_prev.pgm."
                        if stored.grid is not None
                        else ""
                    )
                )
                if started
                else f"map.pcd for '{name}' disappeared before the conversion started."
            ),
        )

    @map_router.get("/api/v1/maps/{name}/vertices", response_model=List[MapVertexResponse])
    def list_map_vertices(name: str, type: Optional[VertexType] = None):
        # _require first, so an unknown map is a 404 rather than [].
        _require(name)
        vertices = map_repo.list_vertices(map=name, type=type.value if type else None)
        return [_vertex_response(vertex) for vertex in vertices]

    @map_router.post(
        "/api/v1/maps/{name}/vertices",
        response_model=List[MapVertexResponse],
        dependencies=job_lock,
    )
    def create_map_vertices(name: str, reqs: List[MapVertexRequest] = Body(..., min_length=1)):
        _require(name)
        vertices = map_repo.create_vertices(
            map=name,
            vertices=[
                {
                    "name": req.name,
                    "type": req.type.value,
                    "x": req.x,
                    "y": req.y,
                    "theta": req.theta,
                }
                for req in reqs
            ],
        )
        return [_vertex_response(vertex) for vertex in vertices]

    def _require_vertex(name: str, vertex_id: uuid.UUID) -> MapPoint:
        """Fetch a vertex, but only as a member of ``name``.

        The id alone finds the row; the URL's map is checked so no vertex is
        reachable through another map's URL. A row on another map is a 404.
        """
        _require(name)
        vertex = map_repo.get_vertex(vertex_id=vertex_id)
        if vertex is None or vertex.map != name:
            raise NotFoundError(f"Map vertex {vertex_id} was not found in '{name}'.")
        return vertex

    @map_router.get("/api/v1/maps/{name}/vertices/{id}", response_model=MapVertexResponse)
    def get_map_vertex(name: str, id: uuid.UUID):
        return _vertex_response(_require_vertex(name, id))

    @map_router.put(
        "/api/v1/maps/{name}/vertices/{id}",
        response_model=MapVertexResponse,
        dependencies=job_lock,
    )
    def update_map_vertex(name: str, id: uuid.UUID, req: MapVertexUpdateRequest):
        _require_vertex(name, id)

        changes = req.model_dump(exclude_unset=True)
        if "type" in changes and changes["type"] is not None:
            changes["type"] = changes["type"].value

        vertex = map_repo.update_vertex(id, **changes)
        if vertex is None:
            raise NotFoundError(f"Map vertex {id} was not found in '{name}'.")
        return _vertex_response(vertex)

    @map_router.delete(
        "/api/v1/maps/{name}/vertices/{id}",
        response_model=DeleteResponse,
        dependencies=job_lock,
    )
    def delete_map_vertex(name: str, id: uuid.UUID):
        _require_vertex(name, id)
        map_repo.delete_vertex(vertex_id=id)
        return DeleteResponse(message=f"Map vertex {id} has been deleted.")

    def _read_gridmap(name: str) -> bytes:
        """Read the map's gridmap.pgm bytes, 404ing with the reason if absent.

        Never served raw: /image's lossless PNG is the grid's wire format.
        """
        _require(name)
        path = map_catalog_repo.gridmap_path(name)
        if path is None:
            raise NotFoundError(
                f"Map '{name}' has no gridmap. Convert its map.pcd first "
                f"(POST /api/v1/maps/{name}/grid/convert)."
            )
        try:
            with open(path, "rb") as handle:
                return handle.read()
        except OSError as exc:
            logger.error("Failed to read gridmap", map=name, error=str(exc))
            raise NotFoundError(f"Map '{name}' has no readable gridmap.")

    def _png_response(
        name: str,
        request: Request,
        cache: Dict[str, Tuple[str, bytes]],
        render: Callable[[bytes], bytes],
        what: str,
    ) -> Response:
        """Serve a PNG rendered from the map's gridmap, cached and revalidated.

        Cache and ETag key on the *source* .pgm bytes, not the encoded PNG.
        """
        payload = _read_gridmap(name)
        tag = _content_tag(payload)
        headers = {"ETag": tag, "Cache-Control": "no-cache"}

        # Before rendering, so a 304 costs no PNG encode.
        if _not_modified(request, tag):
            return Response(status_code=304, headers=headers)

        cached = cache.get(name)
        if cached is None or cached[0] != tag:
            try:
                png = render(payload)
            except ValueError as exc:
                logger.error(f"Failed to render map {what}", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable gridmap.")
            cached = (tag, png)
            cache[name] = cached

        return Response(content=cached[1], media_type="image/png", headers=headers)

    @map_router.get("/api/v1/maps/{name}/image")
    def get_map_image_by_name(name: str, request: Request):
        """The map's gridmap as a full-size PNG (raw bytes, not base64 JSON)."""
        return _png_response(name, request, image_cache, render_png, "image")

    @map_router.get("/api/v1/maps/{name}/thumbnail")
    def get_map_thumbnail(name: str, request: Request):
        return _png_response(name, request, thumbnail_cache, render_thumbnail, "thumbnail")

    @map_router.put(
        "/api/v1/maps/{name}/grid",
        response_model=SaveGridmapResponse,
        dependencies=job_lock,
    )
    def save_map_grid(
        name: str,
        response: Response,
        payload: bytes = Body(..., media_type="application/octet-stream"),
    ):
        """Write an edited gridmap back, and reload it if it is the live one.

        The body is exactly ``width * height`` raw cells in .pgm row order (row
        0 is max y): no encode/decode, so no colour-managed round trip can shift
        205 to 204.

        Plain ``def`` (load-bearing): it fsyncs and parks on ``reload_map`` for
        up to 25 s; ``bytes = Body(...)`` reads the body on the loop and hands
        it to the threadpool.

        Cell values are not validated: map_server classifies by range and real
        maps carry hand-edited 255s; only the length can make a misread file.
        No size cap: that is middleware's job.
        """
        stored = _require(name)

        if conversion_svc.is_converting(name):
            # The conversion would overwrite the save, and the once-only
            # gridmap_raw.pgm snapshot could capture a half-finished grid.
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; save the edit "
                "once it has finished, or the conversion will overwrite it.",
                code="conversion_running",
            )

        # stored.grid parsed both the .pgm header and gridmap.yaml, so one None
        # test covers missing or torn files before map_server re-reads both.
        if stored.grid is None:
            raise NotFoundError(
                f"Map '{name}' has no gridmap. Convert its map.pcd first "
                f"(POST /api/v1/maps/{name}/grid/convert)."
            )

        expected = stored.grid.width * stored.grid.height
        if len(payload) != expected:
            raise BadRequestError(
                f"Gridmap body is {len(payload)} bytes; '{name}' is "
                f"{stored.grid.width}x{stored.grid.height} = {expected} cells."
            )

        written = map_catalog_repo.write_gridmap(name, payload)

        # No cache eviction by design: _png_response re-hashes the file before
        # consulting the cache, so a stale entry is never served.
        tag = _content_tag(written)
        response.headers["ETag"] = tag

        active = name == map_catalog_repo.active_name()
        if not active:
            return SaveGridmapResponse(
                name=name,
                etag=tag,
                active=False,
                reloaded=False,
                message=(f"Saved {name}"),
            )

        # None only on a race; handled as a failed reload.
        yaml_path = map_catalog_repo.gridmap_yaml_path(name)
        if yaml_path is None:
            reloaded, detail = False, "gridmap.yaml is missing"
        else:
            reloaded, detail = map_gw.reload_map(yaml_path)

        if not reloaded:
            # Still a 200: the save worked, so a 5xx would mislead the operator.
            # `reloaded` says the reload did not.
            logger.error("Saved gridmap but map_server did not reload", map=name, error=detail)
            # `detail` goes to the operator too: it tells absent, rejected and
            # timed out apart.
            return SaveGridmapResponse(
                name=name,
                etag=tag,
                active=True,
                reloaded=False,
                message=(f"Saved '{name}', but map_server did not reload: {detail}"),
            )

        return SaveGridmapResponse(
            name=name,
            etag=tag,
            active=True,
            reloaded=True,
            message=f"Saved '{name}' and reloaded.",
        )

    @map_router.get("/api/v1/maps/{name}/keepout", response_model=KeepoutResponse)
    def get_map_keepout(name: str):
        """The map's forbidden zones, as polygons in map-frame metres.

        Read from ``keepout.json`` (the source of truth), not the mask, so a
        mask with no json answers ``zones: []``.
        """
        _require(name)
        zones = map_catalog_repo.read_keepout_zones(name)
        return KeepoutResponse(
            name=name,
            zones=_zone_responses(zones),
            active=name == map_catalog_repo.active_name(),
        )

    @map_router.put(
        "/api/v1/maps/{name}/keepout",
        response_model=SaveKeepoutResponse,
        dependencies=job_lock,
    )
    def save_map_keepout(name: str, request: SaveKeepoutRequest):
        """Replace the map's forbidden zones, and reload them if it is the live map.

        Shaped like ``save_map_grid``: plain ``def``, 409 while converting (the
        geometry may change), 404 with no gridmap, and 200 ``reloaded: false``
        when the save worked but ``filter_mask_server`` did not reload.

        The body is the *whole* list; ``[]`` clears the zones by rewriting the
        mask all-unknown (deleting files would not clear the running filter).
        Rasterised here to match ``gridmap.pgm`` cell for cell; see
        ``helpers/keepout.py``.
        """
        stored = _require(name)

        if conversion_svc.is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; save the forbidden "
                "zones once it has finished, so the mask matches the new grid.",
                code="conversion_running",
            )

        if stored.grid is None:
            raise NotFoundError(
                f"Map '{name}' has no gridmap to draw forbidden zones on. Convert "
                f"its map.pcd first (POST /api/v1/maps/{name}/grid/convert)."
            )

        zones = _validate_zones(request.zones)
        map_catalog_repo.write_keepout(name, zones, stored.grid)

        count = len(zones)
        saved = f"Saved {count} forbidden zone{'s' if count != 1 else ''} for '{name}'"
        active = name == map_catalog_repo.active_name()
        if not active:
            return SaveKeepoutResponse(
                name=name,
                zones=_zone_responses(zones),
                active=False,
                reloaded=False,
                message=f"{saved}.",
            )

        # None only on a race with a delete; handled as a failed reload.
        yaml_path = map_catalog_repo.keepout_yaml_path(name)
        if yaml_path is None:
            reloaded, detail = False, "keepout.yaml is missing"
        else:
            reloaded, detail = map_gw.reload_keepout(yaml_path)

        if not reloaded:
            logger.error(
                "Saved forbidden zones but filter_mask_server did not reload",
                map=name,
                error=detail,
            )
            return SaveKeepoutResponse(
                name=name,
                zones=_zone_responses(zones),
                active=True,
                reloaded=False,
                message=f"{saved}, but the keepout filter did not reload: {detail}",
            )

        return SaveKeepoutResponse(
            name=name,
            zones=_zone_responses(zones),
            active=True,
            reloaded=True,
            message=f"{saved} and reloaded the keepout filter.",
        )

    @map_router.get("/api/v1/maps/{name}/pointcloud")
    def get_map_pointcloud_by_name(name: str):
        """The map's saved map.pcd, packed for the viewer.

        Same wire format and decimation as the live stream: LE uint32 count,
        then ``3 * count`` LE float32 xyz.

        Cached on (size, mtime), not a content hash: map.pcd is only ever
        replaced wholesale, never edited in place.
        """
        _require(name)
        path = map_catalog_repo.pointcloud_path(name)
        if path is None:
            raise NotFoundError(f"Map '{name}' has no map.pcd.")

        try:
            stats = os.stat(path)
        except OSError as exc:
            logger.error("Failed to stat map cloud", map=name, error=str(exc))
            raise NotFoundError(f"Map '{name}' has no readable map.pcd.")

        stamp = (stats.st_size, stats.st_mtime_ns)
        cached = cloud_cache.get(name)
        if cached is None or cached[0] != stamp:
            try:
                points = read_pcd_xyz(path)
            except (OSError, ValueError) as exc:
                logger.error("Failed to read map cloud", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable map.pcd.")

            points = voxel_downsample(points=points, voxel_size=MAP_CLOUD_VOXEL_SIZE)
            points = cap_points(points=points, max_points=MAP_CLOUD_MAX_POINTS)
            payload = struct.pack("<I", points.shape[0]) + pack_xyz_f32(points)
            cached = (stamp, payload)
            cloud_cache[name] = cached
            logger.info("packed stored map cloud", map=name, num_points=int(points.shape[0]))

        return Response(
            content=cached[1],
            media_type="application/octet-stream",
            headers={"Cache-Control": "no-store"},
        )

    return map_router
