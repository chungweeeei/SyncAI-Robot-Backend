"""Request and response bodies of the map routes."""

import uuid
from enum import Enum
from typing import List, Optional, Tuple

from pydantic import BaseModel, Field


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
        "which storey it is on, and where on a slope. The console writes the 3D "
        "map's ground under the vertex (on its floor, for a map with floors); 0 "
        "is a vertex from before heights -- the planner snaps it to the ground "
        "only within 1 m, except where the ground is one layer.",
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


class OctomapStatus(str, Enum):
    """How this map's 3D map (``octomap.bt``) stands.

    Derived from disk alone by ``_octomap_status`` — the tree is built by the
    robot's ``clean_map``, a detached process this backend never sees, so
    there is no registry to consult. ``none`` (never built), ``failed``
    (reason in ``octomap_error``) and ``interrupted`` (the cleaning never
    finished) need different operator actions, so do not collapse them.
    """

    NONE = "none"
    CONVERTING = "converting"
    OK = "ok"
    FAILED = "failed"
    INTERRUPTED = "interrupted"


class MapSummaryResponse(BaseModel):
    name: str = Field(..., description="Directory name under the maps root.")
    active: bool = Field(..., description="Whether this is the map the stack was launched with.")
    has_pointcloud: bool = Field(
        ..., description="Whether map.pcd is present (the 3D localizer's source)."
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


class CreateMapResponse(BaseModel):
    name: str = Field(..., description="The map that was created.")
    has_pointcloud: bool = Field(..., description="Whether pgo wrote map.pcd (true on any 200).")
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
            "applies to; null = unbounded. Write the floor's ground z - 0.5."
        ),
    )
    z_max: Optional[float] = Field(
        None,
        description=(
            "Highest floor height the zone applies to; null = unbounded. A zone "
            "with neither covers every floor. Clients must send back what they "
            "read, or a save widens a zone to every storey."
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
            "Every forbidden zone of the map. An empty list clears them: "
            "keepout.json is rewritten with none and reloaded, never deleted."
        ),
    )


class KeepoutResponse(BaseModel):
    """GET /api/v1/maps/{name}/keepout — what keepout.json records."""

    name: str = Field(..., description="The map the zones belong to.")
    zones: List[KeepoutZoneResponse] = Field(
        ...,
        description=(
            "The zones as last saved through this API. Empty for a map whose "
            "zones were never drawn here, or whose keepout.json cannot be read."
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
            "Whether the planner enforces the new zones now: the 3D planner "
            "re-read keepout.json (planner_server/Octo3D/load_keepout). "
            "False for any map that is not the active one, and for an active map "
            "whose reload failed — the save itself succeeded either way and the "
            "zones apply when the nav stack next loads this map."
        ),
    )
    message: str = Field(..., description="What happened, for the operator to read.")
