import asyncio
import math
import os
import struct
import threading
import time
import uuid
from collections import OrderedDict
from datetime import datetime, timezone
from enum import Enum
from typing import AsyncIterator, BinaryIO, Callable, Dict, Hashable, List, Optional, Tuple

import structlog
from fastapi import APIRouter, Body, Depends, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel, Field

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    NotFoundError,
    PayloadTooLargeError,
    UpstreamError,
)
from syncai_backend.database.models import MapPoint
from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.helpers.floors import detect_floor_levels, floor_name
from syncai_backend.helpers.pgm import render_png, render_thumbnail
from syncai_backend.helpers.pointcloud import (
    cap_points,
    pack_xyz_f32,
    read_pcd_xyz,
    voxel_downsample,
)
from syncai_backend.helpers.map_archive import (
    ArchiveFormat,
    write_archive,
)
from syncai_backend.repositories.map.catalog import (
    OCTOMAP_KEY_MAX,
    GridRecordStatus,
    KeepoutZone,
    MapCatalogRepo,
    MapFloor,
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
    parse_iso_utc,
)
from syncai_backend.services.map_import import MapImportService
from syncai_backend.services.map_switch import MapSwitchService
from syncai_backend.services.map_usage import JobGate, MapUsagePolicy


# Decimation for a stored map.pcd. Duplicates PointCloudSubscriber's numbers so
# the router does not depend on a subscriber; keep the two in step.
MAP_CLOUD_VOXEL_SIZE = 0.3
MAP_CLOUD_MAX_POINTS = 300000

# The largest archive POST /api/v1/maps/import reads. A map is a few hundred MB
# (map.pcd dominates; the octree and grids are smaller); a body past this is the
# wrong file, refused before it fills the disk. Checked on Content-Length first,
# and again while the body streams in, since the header is the client's word.
MAP_IMPORT_MAX_BYTES = 4 << 30

# Spooling granularity, both directions: an upload is written to disk (off the
# event loop) in pieces this big, and an export is sent in pieces this big.
_SPOOL_CHUNK = 1 << 20

# What POST /api/v1/maps/import documents as its body. Spelled out because the
# route reads the request stream itself (no Body parameter for FastAPI to
# describe); identical to what Body(..., media_type=octet-stream) produced.
_IMPORT_REQUEST_BODY = {
    "content": {
        "application/octet-stream": {
            "schema": {
                "contentMediaType": "application/octet-stream",
                "title": "Payload",
                "type": "string",
            }
        }
    },
    "required": True,
}

# A cleaning sidecar saying ``converting`` this long after its ``started_at``
# belongs to a clean that died hard (the longest measured is ~5 min; cleans
# queue per host behind a flock). Twice the conversion's own wait, since the
# tree is built after map.pcd is rewritten.
OCTOMAP_STALE_AFTER_S = 30 * 60.0

# Ground patches a save may carry. A whole floor at 0.1 m is a few hundred
# thousand columns; this is a bound on the body, not a budget.
OCTOMAP_EDITS_MAX = 1_000_000

# Floors a save may name, the longest name, and how close two levels may sit.
# The console draws one band per floor between neighbouring levels, so two
# levels closer than a body height would leave a band with nothing in it.
FLOORS_MAX = 16
FLOOR_NAME_MAX = 16
FLOOR_MIN_SEPARATION_M = 1.0


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
    z: float = Field(
        0.0,
        allow_inf_nan=False,
        description="Height of the ground the vertex stands on (metres, map frame): "
        "which storey it is on. 0 is the ground floor.",
    )


class MapVertexUpdateRequest(BaseModel):
    """Fields to change on an existing vertex. All optional; omitted ones stay.

    No ``map_name``: moving a vertex between maps is a delete and a create.
    """

    name: Optional[str] = Field(None, min_length=1, description="New vertex name.")
    type: Optional[VertexType] = Field(None, description="New vertex role.")
    x: Optional[float] = Field(None, description="New world x-coordinate (metres).")
    y: Optional[float] = Field(None, description="New world y-coordinate (metres).")
    theta: Optional[float] = Field(None, description="New yaw angle in degrees.")
    z: Optional[float] = Field(
        None,
        allow_inf_nan=False,
        description="New ground height (metres): a move to another storey.",
    )


class MapVertexResponse(BaseModel):
    id: uuid.UUID = Field(..., description="Unique identifier of the vertex.")
    name: str = Field(..., description="Human-readable vertex name.")
    type: VertexType = Field(..., description="Semantic role of the vertex.")
    map_name: str = Field(..., description="Name of the map this vertex belongs to.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")
    z: float = Field(..., description="Height of the ground the vertex stands on (metres).")


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


class OctomapStatus(str, Enum):
    """How this map's 3D map (``octomap.bt``) stands; ``GridStatus``'s values.

    Derived from disk alone by ``_octomap_status`` — the tree is built by the
    robot's ``clean_map``, a detached process this backend never sees, so
    there is no registry to consult. The same five words as the gridmap's,
    because a console shows them the same way, but a separate enum because
    the two are read off different files and may drift.
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
    octomap_status: OctomapStatus = Field(
        ...,
        description=(
            "How this map's 3D map (`octomap.bt`, written by the robot's map "
            "cleaning when asked for) stands: `none` (never built, or not asked "
            "for), `converting` (the cleaning is still running), `ok` (on disk, "
            "served by `GET /api/v1/maps/{name}/octomap`), `failed` (reason in "
            "`octomap_error`), `interrupted` (the cleaning never finished)."
        ),
    )
    octomap_error: Optional[str] = Field(
        None,
        description=(
            "Why there is no 3D map, as the cleaning reported it. Null for every "
            "status other than `failed`."
        ),
    )
    octomap_version: Optional[str] = Field(
        None,
        description=(
            "An opaque tag for the 3D map's current bytes (the GET's ETag, "
            "unquoted); it changes whenever the file is replaced, which is how "
            "a client knows to download it again. Null unless `ok`."
        ),
    )
    octomap_resolution: Optional[float] = Field(
        None,
        description=(
            "Cell edge of the 3D map in metres, from its header. Null unless "
            "`ok`, and null for a tree whose header this build cannot read."
        ),
    )


class OctomapEditsResponse(BaseModel):
    """The ground patches saved for a map's 3D map."""

    name: str
    res: Optional[float] = Field(
        None,
        description=(
            "Cell edge, metres, of the tree the keys index; null when no patches "
            "are saved."
        ),
    )
    fill_key: List[List[int]] = Field(
        ...,
        description=(
            "Absolute OcTree keys `[kx, ky, kz]` of the voxels to mark as floor, "
            "at most one per `[kx, ky]` column. `[]` when none are saved."
        ),
    )
    created_at: Optional[str] = Field(
        None, description="ISO 8601 UTC time of the save; null when no patches are saved."
    )


class SaveOctomapEditsRequest(BaseModel):
    fill_key: List[Tuple[int, int, int]] = Field(
        ...,
        description=(
            "The **whole** list of ground patches as absolute OcTree keys "
            "`[kx, ky, kz]` (0–65535 each), one per column. `[]` clears them."
        ),
    )


class SaveOctomapEditsResponse(BaseModel):
    name: str
    count: int = Field(..., description="Patches recorded.")
    message: str = Field(..., description="Human-readable result.")


class MapFloorModel(BaseModel):
    name: str = Field(..., description="What the floor is called, e.g. `1F` or `B1`.")
    level: float = Field(
        ...,
        description=(
            "Body height the robot stands at on this floor, metres in the map "
            "frame (the z of poses.txt, about half a metre above the ground)."
        ),
    )


class MapFloorsResponse(BaseModel):
    """The storeys a map spans, for showing one at a time."""

    name: str
    floors: List[MapFloorModel] = Field(
        ...,
        description=(
            "The floors to show, lowest first: the saved ones, else the "
            "detected ones. `[]` is one storey."
        ),
    )
    detected: List[MapFloorModel] = Field(
        ...,
        description=(
            "The floors found from the heights the robot walked at while "
            "mapping, lowest first; `[]` without a trajectory or for one storey."
        ),
    )
    saved: bool = Field(
        ..., description="Whether `floors` is a saved list rather than the detected one."
    )


class SaveMapFloorsRequest(BaseModel):
    floors: List[MapFloorModel] = Field(
        ...,
        description="The **whole** list of floors, in any order. `[]` saves one storey.",
    )


class SaveMapFloorsResponse(BaseModel):
    name: str
    floors: List[MapFloorModel] = Field(..., description="The floors saved, lowest first.")
    message: str = Field(..., description="Human-readable result.")


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
    grid: bool = Field(
        True,
        description=(
            "Convert the scan to a 2D gridmap after the save. False leaves the "
            "map with grid_status none, which POST "
            "/api/v1/maps/{name}/grid/convert can fill in later."
        ),
    )
    octomap: Optional[bool] = Field(
        None,
        description=(
            "Have the robot's map cleaning also build octomap.bt (the 3D map). "
            "Null leaves it to the robot's map_clean_octomap parameter. Watch "
            "the catalogue's octomap_status for the outcome: it is minutes "
            "behind the save and may fail for lack of memory."
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
            "to poll. A failure is a 502 (or a 409), never a 200 with false; "
            "the odometry has been reset either way, and the map pose is left "
            "where it was before the call."
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
    map_kind: Optional[str] = Field(
        None,
        description=(
            "'3d' when the map has an octomap.bt, so the robot's 3D planner "
            "(Octo3D) navigates it and the robot runs its 3D trees; '2d' for a "
            "gridmap-only map. Null on the no-op. Added 2026-10."
        ),
    )
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
            "Whether the planner now enforces this map's forbidden zones. On a "
            "2D map: filter_mask_server serves its keepout.yaml (written blank "
            "first if the map had none) — a map switch does not move the mask on "
            "its own, so the switch does this too, best-effort: false means the "
            "switch succeeded but the old map's zones may still be in force "
            "until the nav stack restarts. On a 3D map: always true, since the "
            "3D planner reads keepout.json beside the tree as part of loading "
            "it. Null on the switched: false no-op."
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
    name: Optional[str] = Field(
        None, max_length=128, description="A label for the operator; display only."
    )
    z_min: Optional[float] = Field(
        None,
        description=(
            "Lowest floor height (map-frame metres, the ground's z) the zone "
            "applies to on a 3D map; null = unbounded. Write the floor's ground "
            "z - 0.5. Ignored by the 2D mask."
        ),
    )
    z_max: Optional[float] = Field(
        None,
        description=(
            "Highest floor height the zone applies to on a 3D map; null = "
            "unbounded. A zone with neither covers every floor. Clients that "
            "edit only 2D must send back what they read, or a save widens a 3D "
            "zone to every storey."
        ),
    )


class KeepoutZoneResponse(BaseModel):
    id: str
    points: List[ZonePoint]
    name: Optional[str] = None
    z_min: Optional[float] = None
    z_max: Optional[float] = None


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
            "Whether the planner enforces the new zones now: on a 2D map the "
            "running filter_mask_server re-read the mask, on a 3D map the 3D "
            "planner re-read keepout.json (planner_server/Octo3D/load_keepout). "
            "False for any map that is not the active one, and for an active map "
            "whose reload failed — the save itself succeeded either way and the "
            "zones apply when the nav stack next loads this map."
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
        for label, value in (("z_min", zone.z_min), ("z_max", zone.z_max)):
            if value is not None and not math.isfinite(value):
                raise BadRequestError(f"Zone {index} has a {label} that is not a finite height.")
        if zone.z_min is not None and zone.z_max is not None and zone.z_min > zone.z_max:
            raise BadRequestError(
                f"Zone {index} has z_min {zone.z_min} above z_max {zone.z_max}."
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
                name=zone.name,
                z_min=zone.z_min,
                z_max=zone.z_max,
            )
        )
    return result


def _zone_responses(zones: List[KeepoutZone]) -> List[KeepoutZoneResponse]:
    return [
        KeepoutZoneResponse(
            id=zone.id,
            points=[ZonePoint(x=x, y=y) for x, y in zone.points],
            name=zone.name,
            z_min=zone.z_min,
            z_max=zone.z_max,
        )
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
        z=vertex.z,
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


def _octomap_status(
    stored: StoredMap, now: Optional[datetime] = None
) -> Tuple[OctomapStatus, Optional[str]]:
    """How the map's 3D map stands, off the tree on disk and the cleaning sidecar.

    Disk only: unlike the gridmap there is no thread registry, the tree being
    the robot's ``clean_map``'s to write. The check order is the logic:

    - A tree on disk is ``ok`` whatever the sidecar says. It is written by
      tmp + rename, so a present file is a complete one, and a re-clean in
      flight leaves the old tree in place until the new one lands.
    - No tree and no sidecar, or a clean that never asked for one, is
      ``none``: nothing was ever going to produce it.
    - A clean still ``converting`` is ``converting``, until ``started_at`` is
      OCTOMAP_STALE_AFTER_S ago; then ``interrupted`` (a clean killed hard
      never writes its outcome, and nothing will clear it).
    - A clean that ended ``ok`` without writing the tree is ``failed`` with
      the sidecar's reason (out of memory, usually): map.pcd was cleaned, the
      tree was dropped. One that says it wrote a tree that is gone is
      ``none`` — a save into the same directory deletes it first.
    - A clean that ``failed`` is ``failed`` with the clean's reason: a tree
      only comes out of a clean that finished.
    """
    if stored.octomap is not None:
        return OctomapStatus.OK, None

    record = stored.map_clean_record
    if record is None or not record.octomap_requested:
        return OctomapStatus.NONE, None

    if record.status is GridRecordStatus.CONVERTING:
        started = parse_iso_utc(record.started_at)
        current = now if now is not None else datetime.now(timezone.utc)
        if started is not None and (current - started).total_seconds() > OCTOMAP_STALE_AFTER_S:
            return OctomapStatus.INTERRUPTED, None
        return OctomapStatus.CONVERTING, None

    if record.status is GridRecordStatus.OK:
        if record.octomap_written:
            return OctomapStatus.NONE, None
        return (
            OctomapStatus.FAILED,
            record.octomap_error or "The map cleaning finished without building the 3D map.",
        )

    return (
        OctomapStatus.FAILED,
        record.error or "The map cleaning failed before the 3D map could be built.",
    )


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
    octomap_status, octomap_error = _octomap_status(stored)
    tree = stored.octomap if octomap_status is OctomapStatus.OK else None

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
        octomap_status=octomap_status,
        octomap_error=octomap_error,
        octomap_version=tree.version if tree is not None else None,
        octomap_resolution=tree.resolution if tree is not None else None,
    )


def _file_tag(stats: os.stat_result) -> str:
    """Build a strong ETag for a gridmap from its stat, without reading it.

    ``(inode, size, mtime_ns)``, not a content hash: the console revalidates
    every map's image and thumbnail on a timer, and hashing meant reading the
    whole .pgm (4-16 MB off the bind mount) per request, 304s included. The
    inode is what makes stat safe here -- an edited gridmap keeps its size, and
    a save inside one mtime tick would otherwise look unchanged -- because
    every writer of ``gridmap.pgm`` in this process (``write_pgm``: the grid
    PUT and the conversion) goes through a temp file + rename, so a save is a
    new inode every time. The blind spot is a hand edit that rewrites the file
    in place with its mtime preserved (``cp -p`` over it); nothing on the robot
    does that.
    """
    return f'"{stats.st_ino:x}-{stats.st_size:x}-{stats.st_mtime_ns:x}"'


def _not_modified(request: Request, tag: str) -> bool:
    """Report whether If-None-Match already names this content (for a 304)."""
    header = request.headers.get("if-none-match")
    return bool(header) and tag in [value.strip() for value in header.split(",")]


def _revalidate(request: Request, tag: str) -> Tuple[Dict[str, str], Optional[Response]]:
    """The validator headers for ``tag``, and the 304 to answer if the client
    already holds it (else None). Every ETag'd GET here revalidates the same
    way: ``no-cache`` so the browser asks every time, cheaply."""
    headers = {"ETag": tag, "Cache-Control": "no-cache"}
    if _not_modified(request, tag):
        return headers, Response(status_code=304, headers=headers)
    return headers, None


def _no_gridmap(name: str) -> NotFoundError:
    """The 404 for a map with no gridmap, naming the route that makes one."""
    return NotFoundError(
        f"Map '{name}' has no gridmap. Convert its map.pcd first "
        f"(POST /api/v1/maps/{name}/grid/convert)."
    )


# What the render cache may hold, in bytes. A map costs up to ~4-5 MB (the
# packed cloud is capped at MAP_CLOUD_MAX_POINTS x 12 B; the PNGs are far
# smaller), so this keeps a dozen-odd maps: more than one operator looks at,
# and a ceiling where there was none.
RENDER_CACHE_MAX_BYTES = 64 << 20


class _RenderCache:
    """Encoded renderings of each map, re-made only when their source changes,
    the least recently used dropped past ``max_bytes``.

    One entry per (kind, map); each remembers the source ``version`` it was
    made from, and ``get`` re-renders on any other version, so an entry is
    never served stale. It had no bound: every map ever opened kept its
    renderings until restart (``forget`` runs only on rename, delete and
    import), which read as a leak. The entry just stored is never evicted,
    even alone over budget, so a big map still renders once per version.

    The lock covers the bookkeeping only, never ``render`` (seconds for a
    cloud): two threads racing a miss both render, which is harmless.
    """

    def __init__(self, max_bytes: int = RENDER_CACHE_MAX_BYTES) -> None:
        self._max_bytes = max_bytes
        self._entries: "OrderedDict[Tuple[str, str], Tuple[Hashable, bytes]]" = OrderedDict()
        self._bytes = 0
        self._lock = threading.Lock()

    def get(
        self, kind: str, name: str, version: Hashable, render: Callable[[], bytes]
    ) -> bytes:
        key = (kind, name)
        with self._lock:
            cached = self._entries.get(key)
            if cached is not None and cached[0] == version:
                self._entries.move_to_end(key)
                return cached[1]

        data = render()

        with self._lock:
            self._drop(key)
            self._entries[key] = (version, data)
            self._bytes += len(data)
            while self._bytes > self._max_bytes and len(self._entries) > 1:
                oldest = next(iter(self._entries))
                self._drop(oldest)
        return data

    def forget(self, name: str) -> None:
        with self._lock:
            for key in [key for key in self._entries if key[1] == name]:
                self._drop(key)

    @property
    def size_bytes(self) -> int:
        return self._bytes

    def _drop(self, key: Tuple[str, str]) -> None:
        entry = self._entries.pop(key, None)
        if entry is not None:
            self._bytes -= len(entry[1])


# --- Router -----------------------------------------------------------------


def init_map_router(
    logger: structlog.stdlib.BoundLogger,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
    task_template_repo: TaskTemplateRepo,
    conversion_svc: GridmapConversionService,
    mapping_status_repo: MappingStatusRepo,
    map_switch_svc: MapSwitchService,
    map_usage: MapUsagePolicy,
    job_gate: JobGate,
    map_import_svc: MapImportService,
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

    # The image, thumbnail and packed cloud of each map, re-encoded only when
    # their source changes.
    renders = _RenderCache()

    def _vertex_count(name: str) -> int:
        # One map's count (GET /maps/{name}); the listing uses
        # count_vertices_by_map. Never len(list_vertices(...)).

        return map_repo.count_vertices(map=name)

    def _require(name: str) -> StoredMap:
        stored = map_catalog_repo.get_map(name)
        if stored is None:
            raise NotFoundError(f"No map named '{name}' on this robot.")
        return stored

    # Async so the Temporal call runs on the event loop while the plain
    # ``def`` route still runs in the threadpool. See services/map_usage.py.
    job_lock = [Depends(job_gate.refuse_while_a_job_holds)]

    @map_router.get("/api/v1/maps", response_model=List[MapSummaryResponse])
    def list_maps():
        active_name = map_catalog_repo.active_name()
        # One GROUP BY for the whole listing, not a count per map.
        counts = map_repo.count_vertices_by_map()
        return [
            _summary(stored, active_name, counts.get(stored.name, 0), conversion_svc)
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
        # Before the directory and before pgo: a save that builds nothing would
        # end the run into a map that cannot be loaded.
        if not request.grid and request.octomap is False:
            raise BadRequestError(
                "Choose at least one of the floor plan and the 3D map to build.",
                code="nothing_to_build",
            )
        _refuse_while_idle("save")

        directory = map_catalog_repo.create_map_dir(request.name)

        saved, detail = map_gw.save_map(directory, octomap=request.octomap)
        if not saved:
            map_catalog_repo.discard_empty_map_dir(request.name)
            logger.error("Failed to save map", map=request.name, error=detail)
            raise UpstreamError(detail)

        grid_pending = request.grid and conversion_svc.start(request.name, directory)

        # Operator copy: what is being built in the background, never a route.
        if grid_pending:
            follow_up = (
                " Building the floor plan in the background, once the robot "
                "has removed people from the scan."
            )
        elif request.grid:
            follow_up = " The floor plan could not be started; rebuild it from the map library."
        else:
            follow_up = ""
        if request.octomap:
            follow_up += " The 3D map is built after that and can take several minutes."

        return CreateMapResponse(
            name=request.name,
            has_pointcloud=True,
            grid_pending=grid_pending,
            message=(
                f"Saved '{request.name}'. Mapping has stopped — start it again "
                "for another map." + follow_up
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
        map_usage.refuse_while_converting(
            name, "export it once the conversion has finished."
        )

        vertices = [
            {"name": v.name, "type": v.type, "x": v.x, "y": v.y, "theta": v.theta, "z": v.z}
            for v in map_repo.list_vertices(map=name)
        ]
        # Built into an anonymous file on the maps' disk and streamed from it:
        # held in memory, the archive was about the map's size (map.pcd does
        # not compress) twice over at the peak, for as long as the download
        # took, in the process that also supervises the running job.
        started = time.perf_counter()
        spool = map_catalog_repo.scratch_file()
        try:
            write_archive(
                map_catalog_repo.resolve_dir(name),
                name,
                vertices,
                format,
                exported_at=iso_now(),
                dest=spool,
                exclude_dirs=(TRAVERSABLE_DEBUG_SUBDIR_NAME,),
            )
            size = spool.seek(0, os.SEEK_END)
            spool.seek(0)
        except BaseException:
            spool.close()
            raise
        logger.info(
            "Exported map",
            map=name,
            format=format.value,
            bytes=size,
            vertices=len(vertices),
            build_ms=round((time.perf_counter() - started) * 1000),
        )

        def _chunks():
            # Closing frees the disk; an abandoned download closes it when the
            # generator is collected.
            with spool:
                while chunk := spool.read(_SPOOL_CHUNK):
                    yield chunk

        return StreamingResponse(
            _chunks(),
            media_type=format.media_type,
            headers={
                # No quoting needed: _NAME_RE admits only [A-Za-z0-9._-].
                "Content-Disposition": f'attachment; filename="{name}.{format.extension}"',
                "Content-Length": str(size),
                "Cache-Control": "no-store",
            },
        )

    async def _spooled_upload(request: Request) -> AsyncIterator[BinaryIO]:
        """The request body, streamed into an anonymous file on the maps' disk.

        It was ``bytes = Body(...)``: the whole upload in memory before the
        route ran, with no limit, so one wrong file (a 4 GB bag) could take
        the process down mid-job. Now at most ``_SPOOL_CHUNK`` is held, and
        anything over ``MAP_IMPORT_MAX_BYTES`` is a 413 -- on the declared
        length before reading, and on the bytes actually received while
        reading. Writes go to a thread: this runs on the event loop.
        """
        declared = request.headers.get("content-length", "")
        if declared.isdigit() and int(declared) > MAP_IMPORT_MAX_BYTES:
            raise _too_large()

        spool = await asyncio.to_thread(map_catalog_repo.scratch_file)
        try:
            received = 0
            pending = bytearray()
            async for chunk in request.stream():
                received += len(chunk)
                if received > MAP_IMPORT_MAX_BYTES:
                    raise _too_large()
                pending += chunk
                if len(pending) >= _SPOOL_CHUNK:
                    full, pending = pending, bytearray()
                    await asyncio.to_thread(spool.write, full)
            if pending:
                await asyncio.to_thread(spool.write, pending)
            yield spool
        finally:
            spool.close()

    def _too_large() -> PayloadTooLargeError:
        return PayloadTooLargeError(
            f"The archive is larger than {MAP_IMPORT_MAX_BYTES >> 30} GiB; a map "
            "export is far smaller. Is this the right file?",
            code="archive_too_large",
        )

    @map_router.post(
        "/api/v1/maps/import",
        status_code=201,
        response_model=ImportMapResponse,
        openapi_extra={"requestBody": _IMPORT_REQUEST_BODY},
    )
    def import_map(
        name: Optional[str] = None,
        payload: BinaryIO = Depends(_spooled_upload),
    ):
        """Create — or replace — a map from an archive ``export_map`` produced.

        The raw body is the archive (zip or tar.gz by magic bytes); the target
        is ``?name=`` or the manifest's name. Spooled to disk as it arrives
        (``_spooled_upload``); 413 ``archive_too_large`` past
        ``MAP_IMPORT_MAX_BYTES``.

        Nothing is written until the archive is judged: a missing or mismatched
        manifest, a link or escaping member is 400, as is no ``map.pcd``, an
        unknown vertex type or a bad name. Replacing an existing map has
        delete's 409s (``map_active`` / ``conversion_running`` /
        ``template_bound``); 409 ``disk_low`` when the unpacked size exceeds
        free space (the replaced map's space does not count).

        The steps and their undos (staging, the one-rename publish, the vertex
        transaction, the parked map's ``rmtree`` last) are
        services/map_import.py. No conversion is started.
        """
        outcome = map_import_svc.import_archive(
            payload, name, vertex_types={member.value for member in VertexType}
        )
        target, replacing = outcome.name, outcome.replaced
        created, vertices_deleted = outcome.vertices_created, outcome.vertices_deleted

        # Renderings of an earlier map of this name are stale.
        renders.forget(target)

        return ImportMapResponse(
            name=target,
            replaced=replacing,
            files=outcome.files,
            bytes=outcome.bytes,
            vertices_created=created,
            vertices_deleted=vertices_deleted,
            message=(
                f"{'Replaced' if replacing else 'Imported'} '{target}': "
                f"{outcome.files} files and {created} "
                f"{'vertex' if created == 1 else 'vertices'}"
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
        map_usage.refuse_if_active(name, "renamed")
        map_usage.refuse_while_converting(
            name, "rename it once the conversion has finished."
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
        renders.forget(name)

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

        404, then ``MapUsagePolicy.refuse_destroy``'s 409s (``map_active``,
        ``conversion_running``, ``template_bound``) before any mutation.

        **Database first, filesystem last** (the inverse of rename): ``rmtree``
        cannot be undone, so it goes last. A failed ``rmtree`` after the rows
        are gone converges on retry; the reverse would leave orphaned rows that
        a future map of the same name silently inherits.

        Registered schedules keep firing their frozen steps; the response says
        so.
        """
        _require(name)
        map_usage.refuse_destroy(
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
        renders.forget(name)

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

    @map_router.post(
        "/api/v1/maps/{name}/activate", response_model=ActivateMapResponse
    )
    async def activate_map(name: str):
        """Switch the robot onto another map, live, without restarting the stack.

        Four parts must agree: the localizer's cloud (`relocalize`),
        map_server's grid (`load_map`), the 3D planner's tree
        (`planner_server/Octo3D/load_map`) and `[map] name` in the INI (what
        `active_name()` reports and a restart reads). The localizer goes first
        because a refusal there mutates nothing; each later step compensates the
        earlier ones on failure. The INI is last and preflighted for
        writability, since an unrecorded swap would leave REST naming the wrong
        map.

        A map is 2D (a usable gridmap) or 3D (an ``octomap.bt``; 2026-10). A 3D
        map is loaded into Octo3D, which is what makes the robot run its 3D
        trees, and needs a stack that has it (409 ``planner_3d_unavailable``);
        a 2D map *unloads* Octo3D when the stack has one, or the robot would
        keep planning on the previous map's octree. A 3D map's forbidden zones
        come in with its tree (Octo3D reads keepout.json beside octomap.bt), and
        map_server keeps the previous grid when the 3D map has none (it cannot
        unload; nothing plans on it while Octo3D has a map).

        All refusals come first. `stack_not_ready` (e.g. mapping mode) is
        detected by service discoverability, not the cached RobotState, which
        is absent on a robot that lost localization.

        `[initial_pose]` is zeroed: an old-frame pose is meaningless here. The
        operator sets the pose; `localized` says whether registration converged
        shortly after the swap. The steps and their undos are
        services/map_switch.py; this route owns only the Temporal question.
        """
        stored = _require(name)
        status, grid_error = _grid_status(stored, False)
        plan = map_switch_svc.plan(stored, status == GridStatus.OK, grid_error)
        if plan is None:
            return ActivateMapResponse(
                name=name,
                previous=name,
                switched=False,
                localized=None,
                message=f"'{name}' is already the map the robot is running on.",
            )

        # Any job, not only one holding a map: a switch re-points the localizer.
        await job_gate.refuse_while_any_job("a map switch", "switching maps")

        # The route is `async def` for the Temporal await (the client belongs to
        # the uvicorn loop); the switch blocks for up to a minute, so it runs in
        # a worker thread. Renderings are keyed by map name: nothing cached is
        # stale.
        outcome = await asyncio.to_thread(map_switch_svc.execute, plan)
        return ActivateMapResponse(
            name=outcome.name,
            map_kind=outcome.map_kind,
            previous=outcome.previous,
            switched=True,
            localized=outcome.localized,
            keepout_reloaded=outcome.keepout_reloaded,
            message=outcome.message,
        )

    # --- Localization -------------------------------------------------------

    # Held across one auto_localize call: a second press would queue behind the
    # first in the localizer and time out or search again. It is a 409 instead.
    auto_localize_lock = threading.Lock()

    @map_router.post(
        "/api/v1/localization/auto", response_model=AutoLocalizeResponse
    )
    async def auto_localize():
        """Find the robot on the loaded map with no hint — the recovery for a
        kidnapped robot. The localizer's ``reset_odometry`` with ``skip_seed``:
        Point-LIO is reset first (a carry often diverges it), then the
        hint-free search runs on the new run's scans. The robot must be
        standing still; ``odom → base_link`` jumps back to the origin once.

        No body. Blocks up to ~30 s; a 200 means localized, nothing to poll.
        Writes nothing (not the INI's ``[initial_pose]``).

        409 ``task_running`` while any job runs (it moves the map frame under
        the robot), 409 ``tasks_unknown``, 409 ``localize_running``. Every
        localizer refusal is the uniform 502 with its sentence. ``async`` for
        the Temporal await, blocking half on a worker thread, as activate.
        """
        await job_gate.refuse_while_any_job("finding the robot", "finding the robot")

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
                # Decided again now, not when the request came in: the
                # conversion can wait on a clean and run for minutes, and a
                # switch meanwhile is not refused (only the target map's
                # conversion is). Reloading then would put map_server on this
                # map's grid while the localizer, Octo3D and the INI say the
                # other one.
                current = map_catalog_repo.active_name()
                if current != name:
                    logger.warning(
                        "Converted a map the robot switched away from meanwhile; "
                        "map_server not reloaded",
                        map=name,
                        active=current,
                    )
                    return
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
                    "z": req.z,
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

    def _gridmap_file(name: str) -> Tuple[str, os.stat_result]:
        """The map's gridmap.pgm path and stat, 404ing with the reason if absent.

        Never served raw: /image's lossless PNG is the grid's wire format.
        """
        _require(name)
        path = map_catalog_repo.gridmap_path(name)
        if path is None:
            raise _no_gridmap(name)
        try:
            return path, os.stat(path)
        except OSError as exc:
            logger.error("Failed to stat gridmap", map=name, error=str(exc))
            raise NotFoundError(f"Map '{name}' has no readable gridmap.")

    def _png_response(
        name: str,
        request: Request,
        render: Callable[[bytes], bytes],
        what: str,
    ) -> Response:
        """Serve a PNG rendered from the map's gridmap, cached and revalidated.

        Cache and ETag key on the *source* .pgm's stat (``_file_tag``), not the
        encoded PNG; the file is read only when there is a PNG to render.
        """
        path, stats = _gridmap_file(name)
        tag = _file_tag(stats)
        # Before rendering, so a 304 costs no read and no PNG encode.
        headers, not_modified = _revalidate(request, tag)
        if not_modified is not None:
            return not_modified

        def _render() -> bytes:
            try:
                with open(path, "rb") as handle:
                    payload = handle.read()
            except OSError as exc:
                logger.error("Failed to read gridmap", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable gridmap.")
            try:
                return render(payload)
            except ValueError as exc:
                logger.error(f"Failed to render map {what}", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable gridmap.")

        png = renders.get(what, name, tag, _render)
        return Response(content=png, media_type="image/png", headers=headers)

    @map_router.get("/api/v1/maps/{name}/image")
    def get_map_image_by_name(name: str, request: Request):
        """The map's gridmap as a full-size PNG (raw bytes, not base64 JSON)."""
        return _png_response(name, request, render_png, "image")

    @map_router.get("/api/v1/maps/{name}/thumbnail")
    def get_map_thumbnail(name: str, request: Request):
        return _png_response(name, request, render_thumbnail, "thumbnail")

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

        # The conversion would overwrite the save, and the once-only
        # gridmap_raw.pgm snapshot could capture a half-finished grid.
        map_usage.refuse_while_converting(
            name,
            "save the edit once it has finished, or the conversion will overwrite it.",
        )

        # stored.grid parsed both the .pgm header and gridmap.yaml, so one None
        # test covers missing or torn files before map_server re-reads both.
        if stored.grid is None:
            raise _no_gridmap(name)

        expected = stored.grid.width * stored.grid.height
        if len(payload) != expected:
            raise BadRequestError(
                f"Gridmap body is {len(payload)} bytes; '{name}' is "
                f"{stored.grid.width}x{stored.grid.height} = {expected} cells."
            )

        map_catalog_repo.write_gridmap(name, payload)

        # No cache eviction by design: _png_response stats the file before
        # consulting the cache, so a stale entry is never served. The tag is
        # the new inode's, so it is the one the next GET answers with.
        _, stats = _gridmap_file(name)
        tag = _file_tag(stats)
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
        geometry may change), 404 with neither a gridmap nor an octomap, and
        200 ``reloaded: false`` when the save worked but the reload did not.

        The body is the *whole* list; ``[]`` clears the zones by rewriting the
        mask all-unknown (deleting files would not clear the running filter).
        Rasterised here to match ``gridmap.pgm`` cell for cell; see
        ``helpers/keepout.py``. A 3D-only map (no gridmap) gets the json alone.

        The reload follows the map's kind, as ``activate`` does ("3D wins"): an
        active 3D map's zones go to the 3D planner
        (``planner_server/Octo3D/load_keepout``), an active 2D map's mask to
        ``filter_mask_server``.
        """
        stored = _require(name)
        is_3d = map_catalog_repo.octomap_path(name) is not None

        map_usage.refuse_while_converting(
            name,
            "save the forbidden zones once it has finished, so the mask matches "
            "the new grid.",
        )

        if stored.grid is None and not is_3d:
            raise NotFoundError(
                f"Map '{name}' has neither a gridmap nor a 3D map to draw forbidden "
                f"zones on. Convert its map.pcd first "
                f"(POST /api/v1/maps/{name}/grid/convert)."
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
        if is_3d:
            enforcer = "the 3D planner"
            json_path = map_catalog_repo.keepout_json_path(name)
            if json_path is None:
                reloaded, detail = False, "keepout.json is missing"
            else:
                reloaded, detail = map_gw.reload_keepout_3d(json_path)
        else:
            enforcer = "the keepout filter"
            yaml_path = map_catalog_repo.keepout_yaml_path(name)
            if yaml_path is None:
                reloaded, detail = False, "keepout.yaml is missing"
            else:
                reloaded, detail = map_gw.reload_keepout(yaml_path)

        if not reloaded:
            logger.error(
                "Saved forbidden zones but the planner did not reload them",
                map=name,
                map_kind="3d" if is_3d else "2d",
                error=detail,
            )
            return SaveKeepoutResponse(
                name=name,
                zones=_zone_responses(zones),
                active=True,
                reloaded=False,
                message=f"{saved}, but {enforcer} did not reload: {detail}",
            )

        return SaveKeepoutResponse(
            name=name,
            zones=_zone_responses(zones),
            active=True,
            reloaded=True,
            message=f"{saved} and reloaded {enforcer}.",
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

        def _pack() -> bytes:
            try:
                points = read_pcd_xyz(path)
            except (OSError, ValueError) as exc:
                logger.error("Failed to read map cloud", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable map.pcd.")

            points = voxel_downsample(points=points, voxel_size=MAP_CLOUD_VOXEL_SIZE)
            points = cap_points(points=points, max_points=MAP_CLOUD_MAX_POINTS)
            logger.info("packed stored map cloud", map=name, num_points=int(points.shape[0]))
            return struct.pack("<I", points.shape[0]) + pack_xyz_f32(points)

        payload = renders.get("cloud", name, (stats.st_size, stats.st_mtime_ns), _pack)
        return Response(
            content=payload,
            media_type="application/octet-stream",
            headers={"Cache-Control": "no-store"},
        )

    # --- The 3D map ---------------------------------------------------------

    def _require_octomap(name: str) -> Tuple[StoredMap, str]:
        """The map and its tree's path, or the refusal that says why there is none.

        409 while the cleaning is still building it (a client that polls the
        catalogue will see `ok`), 404 with the reason for every other way of
        not having one.
        """
        stored = _require(name)
        path = map_catalog_repo.octomap_path(name)
        if path is not None and stored.octomap is not None:
            return stored, path

        status, error = _octomap_status(stored)
        if status is OctomapStatus.CONVERTING:
            raise ConflictError(
                f"The 3D map of '{name}' is still being built; try again once "
                "the map cleaning has finished.",
                code="octomap_converting",
            )
        if status is OctomapStatus.FAILED:
            raise NotFoundError(f"Map '{name}' has no 3D map: {error}")
        if status is OctomapStatus.INTERRUPTED:
            raise NotFoundError(f"The 3D map of '{name}' was never finished being built.")
        raise NotFoundError(f"Map '{name}' has no 3D map.")

    @map_router.get("/api/v1/maps/{name}/octomap")
    def get_map_octomap(name: str, request: Request):
        """The map's 3D map, ``octomap.bt`` byte for byte.

        Not decoded here: the file is the tree's own compact form (two bits a
        child, a third the size of its voxel centres as floats), and the
        console reads the keys straight off it for the ground editor. ETag'd
        on (size, mtime) through ``OctomapInfo.version`` — the same tag the
        catalogue reports as ``octomap_version``, so a client that sees it
        change knows the bytes did — and revalidated to 304 for a tab that
        already holds it. A content hash would cost a read of the whole file
        per listing for nothing: the tree is only ever replaced wholesale.
        """
        stored, path = _require_octomap(name)
        tree = stored.octomap
        assert tree is not None  # _require_octomap
        tag = f'"{tree.version}"'
        headers, not_modified = _revalidate(request, tag)
        if not_modified is not None:
            return not_modified

        return FileResponse(path, media_type="application/octet-stream", headers=headers)

    @map_router.get(
        "/api/v1/maps/{name}/octomap/edits", response_model=OctomapEditsResponse
    )
    def get_map_octomap_edits(name: str):
        """The ground patches saved for this map's 3D map; ``[]`` when none.

        Read from ``octomap_edits.json`` as saved, whether or not the tree on
        disk has had them applied: the file is the record, the tree is the
        robot's.
        """
        _require(name)
        edits = map_catalog_repo.read_octomap_edits(name)
        if edits is None:
            return OctomapEditsResponse(name=name, res=None, fill_key=[], created_at=None)
        return OctomapEditsResponse(
            name=name,
            res=edits.res,
            fill_key=[list(key) for key in edits.fill_key],
            created_at=edits.created_at,
        )

    def _validate_fill_keys(entries: List[Tuple[int, int, int]]) -> List[Tuple[int, int, int]]:
        """400 with a sentence for a key no tree has or a column named twice.

        Pydantic has already made them int triples; what it cannot say is
        that a key is 16-bit or that a column holds one patch at most (the
        editor fills a column's missing floor, which is one voxel).
        """
        if len(entries) > OCTOMAP_EDITS_MAX:
            raise BadRequestError(
                f"Too many ground patches ({len(entries)}); at most "
                f"{OCTOMAP_EDITS_MAX} can be saved."
            )
        columns = set()
        keys: List[Tuple[int, int, int]] = []
        for entry in entries:
            key = (int(entry[0]), int(entry[1]), int(entry[2]))
            if any(value < 0 or value > OCTOMAP_KEY_MAX for value in key):
                raise BadRequestError(
                    f"Ground patch {list(key)} is outside the 3D map (each key is 0 to "
                    f"{OCTOMAP_KEY_MAX})."
                )
            column = key[:2]
            if column in columns:
                raise BadRequestError(
                    f"Two ground patches name the same column {list(column)}; a column "
                    "takes one patch."
                )
            columns.add(column)
            keys.append(key)
        return keys

    @map_router.put(
        "/api/v1/maps/{name}/octomap/edits",
        response_model=SaveOctomapEditsResponse,
        dependencies=job_lock,
    )
    def save_map_octomap_edits(name: str, request: SaveOctomapEditsRequest):
        """Replace the map's ground patches with the body's whole list.

        A record, not an edit of the tree: ``octomap.bt`` is untouched and
        nothing running on the robot reads the file. The robot's
        ``apply_octomap_edits`` is what patches the tree from it (keeping the
        original as ``octomap.raw.bt``), and the 3D planner loads the tree
        when it starts, so the message says the patches take effect once
        applied. Held by a running job like every map write: the patches
        change where the planner may go.

        Refused with no tree (404: the keys index that tree) and while one is
        being built (409 ``octomap_converting``: the keys would be another
        save's, in another map frame).
        """
        stored, _ = _require_octomap(name)
        tree = stored.octomap
        assert tree is not None  # _require_octomap
        if tree.resolution is None:
            raise ConflictError(
                f"The 3D map of '{name}' could not be read, so ground patches "
                "cannot be saved onto it.",
                code="octomap_unreadable",
            )

        keys = _validate_fill_keys(request.fill_key)
        edits = map_catalog_repo.write_octomap_edits(name, keys, tree.resolution)
        count = len(edits.fill_key)
        if count == 0:
            message = f"Cleared the ground patches of '{name}'."
        else:
            message = (
                f"Saved {count} ground {'patch' if count == 1 else 'patches'} for "
                f"'{name}'. They take effect once applied to the robot's 3D map."
            )
        return SaveOctomapEditsResponse(name=name, count=count, message=message)

    def _floor_models(floors: List[MapFloor]) -> List[MapFloorModel]:
        return [MapFloorModel(name=floor.name, level=floor.level) for floor in floors]

    def _detected_floors(name: str) -> List[MapFloor]:
        heights = map_catalog_repo.read_pose_heights(name)
        levels = detect_floor_levels(heights) if heights else []
        # One level is one storey, which needs no split.
        if len(levels) < 2:
            return []
        return [MapFloor(name=floor_name(i), level=level) for i, level in enumerate(levels)]

    @map_router.get("/api/v1/maps/{name}/floors", response_model=MapFloorsResponse)
    def get_map_floors(name: str):
        """The floors of a map that spans storeys: saved, else detected.

        Detected on every read from poses.txt (a few thousand lines), so a
        map that has never been looked at needs no step to have floors.
        """
        _require(name)
        detected = _detected_floors(name)
        saved = map_catalog_repo.read_floors(name)
        return MapFloorsResponse(
            name=name,
            floors=_floor_models(saved if saved is not None else detected),
            detected=_floor_models(detected),
            saved=saved is not None,
        )

    def _validate_floors(entries: List[MapFloorModel]) -> List[MapFloor]:
        """400 with a sentence for a list the console could not draw bands from."""
        if len(entries) > FLOORS_MAX:
            raise BadRequestError(f"Too many floors ({len(entries)}); at most {FLOORS_MAX}.")
        floors: List[MapFloor] = []
        names = set()
        for entry in entries:
            label = entry.name.strip()
            if not label:
                raise BadRequestError("Every floor needs a name.")
            if len(label) > FLOOR_NAME_MAX:
                raise BadRequestError(
                    f"Floor name '{label}' is too long; at most {FLOOR_NAME_MAX} characters."
                )
            if label.casefold() in names:
                raise BadRequestError(f"Two floors are called '{label}'.")
            if not math.isfinite(entry.level):
                raise BadRequestError(f"Floor '{label}' has no height.")
            names.add(label.casefold())
            floors.append(MapFloor(name=label, level=float(entry.level)))
        floors.sort(key=lambda floor: floor.level)
        for lower, upper in zip(floors, floors[1:]):
            if upper.level - lower.level < FLOOR_MIN_SEPARATION_M:
                raise BadRequestError(
                    f"Floors '{lower.name}' and '{upper.name}' are less than "
                    f"{FLOOR_MIN_SEPARATION_M:g} m apart."
                )
        return floors

    @map_router.put("/api/v1/maps/{name}/floors", response_model=SaveMapFloorsResponse)
    def save_map_floors(name: str, request: SaveMapFloorsRequest):
        """Replace the map's floors with the body's whole list.

        Not held by a running job: the floors only decide what the console
        shows, and nothing on the robot reads them.
        """
        _require(name)
        floors = map_catalog_repo.write_floors(name, _validate_floors(request.floors))
        count = len(floors)
        message = (
            f"Saved {count} floor{'' if count == 1 else 's'} for '{name}'."
            if count
            else f"Saved '{name}' as one floor."
        )
        return SaveMapFloorsResponse(name=name, floors=_floor_models(floors), message=message)

    return map_router
