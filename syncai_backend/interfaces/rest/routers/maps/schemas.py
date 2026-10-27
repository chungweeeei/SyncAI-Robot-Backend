"""Request and response bodies of the map routes."""

import uuid
from enum import Enum
from typing import List, Optional, Tuple

from pydantic import BaseModel, Field


from syncai_backend.services.gridmap_conversion import (
    FLOOR_REFERENCE_DEFAULT,
)


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
        "is a 2D map's, or a vertex from before heights -- the planner snaps it "
        "to the ground only within 1 m, except where the ground is one layer.",
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
