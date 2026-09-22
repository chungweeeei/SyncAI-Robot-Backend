import asyncio
import hashlib
import os
import struct
import uuid
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

import structlog
from fastapi import APIRouter, Body, Request
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
from syncai_backend.repositories.map.catalog import (
    GridRecordStatus,
    MapCatalogRepo,
    StoredMap,
)
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.task.task_template import TaskTemplateRepo

# The conversion itself -- recipes, the running-thread registry and the sidecar
# protocol -- lives a layer down; this module validates requests against it and
# turns its refusals into status codes. GRIDMAP_BANDS_ABOVE_FLOOR is here only
# to validate a re-convert's band overrides before the thread starts.
from syncai_backend.services.gridmap_conversion import (
    GRIDMAP_BANDS_ABOVE_FLOOR,
    GridmapConversionService,
    iso_now,
)


# Decimation for a stored map.pcd. Same numbers the point-cloud subscriber
# applies to the live localizer/map_cloud topic (see PointCloudSubscriber),
# duplicated rather than imported so this router does not depend on a
# subscriber; if one moves, move the other.
MAP_CLOUD_VOXEL_SIZE = 0.3
MAP_CLOUD_MAX_POINTS = 300000


# --- Schemas ----------------------------------------------------------------


class VertexType(str, Enum):
    """Semantic role of a map vertex: what the robot does when it visits.

    Persisted as a plain string in the ``map_vertices`` table; validated at the
    REST boundary only.
    """

    # A plain navigation stop (no pure path-only vertices exist in this
    # system, so every ordinary nav target is GENERAL).
    GENERAL = "GENERAL"
    # An IoT device station (pickup/drop/conveyor, etc.). The name used to
    # mirror ``StepType.ARTIFACT``; that step type went away with the conveyor
    # integration (2026-08), but the vertex label stays — it marks a place on
    # the map, existing rows carry it, and the frontend renders it.
    ARTIFACT = "ARTIFACT"
    # A charging dock.
    CHARGER = "CHARGER"
    # An idle/park base the robot returns to.
    HOME = "HOME"
    # A hold spot for queueing / yielding / waiting on a station to free up.
    WAITING = "WAITING"


class MapVertexRequest(BaseModel):
    """A vertex to create. The owning map comes from the URL, not the body.

    Deliberately no ``map_name`` field: the create route is nested under
    ``/api/v1/maps/{name}/vertices``, so accepting one here would let a request
    name a different map than the path it was posted to.
    """

    name: str = Field(..., min_length=1, description="Human-readable vertex name.")
    type: VertexType = Field(..., description="Semantic role of the vertex.")
    x: float = Field(..., description="World x-coordinate (metres, map frame).")
    y: float = Field(..., description="World y-coordinate (metres, map frame).")
    theta: float = Field(..., description="Yaw angle in degrees (map frame).")


class MapVertexUpdateRequest(BaseModel):
    """Fields to change on an existing vertex. All optional; omitted ones stay.

    No ``map_name``, for the same reason the create model has none: the route is
    nested under the owning map, so a body that renamed it would move the vertex
    out from under the URL that just addressed it. Moving a vertex between maps
    is a delete and a create.
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

    ``origin`` is the three-element origin from ``gridmap.yaml``, so its third
    component is a **yaw**, not a z. An OccupancyGrid message's origin is a full
    pose whose position is ``{x, y, z}``; the two are the same shape and are not
    interchangeable, which is worth remembering if a live-topic endpoint ever
    comes back.
    """

    resolution: float = Field(..., description="Metres per cell.")
    origin: GridOrigin = Field(..., description="Pose of the grid's lower-left cell.")
    width: int = Field(..., description="Grid width in cells.")
    height: int = Field(..., description="Grid height in cells.")


class GridStatus(str, Enum):
    """How this map's 2D gridmap stands: the conversion-status surface.

    Five states, of which the sidecar on disk can only ever hold three (see
    ``GridRecordStatus`` in the catalogue repo). ``interrupted`` and ``none`` are
    derived — the first from a sidecar claiming to be mid-conversion with no
    thread behind it, the second from the absence of both a grid and a record.

    A separate type from ``GridRecordStatus`` even though three of the values are
    spelled the same, because this one is the promise made to a client and that
    one is the vocabulary of a file on disk. ``_grid_status`` below is the only
    place the two meet.

    The distinction that matters is between ``none`` and the two failure states.
    All three leave a map the nav stack cannot load, but they call for different
    things from the operator: ``none`` means nobody has converted this map yet
    (press Build grid), ``failed`` means the pipeline rejected this cloud and
    the reason is in ``grid_error`` (read it before pressing anything), and
    ``interrupted`` means the backend went away mid-conversion and a retry is
    very likely to just work. Collapsing them into "no grid" is what this
    endpoint used to do, and it sent every one of those to the log.
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

    Not the bare ``DeleteResponse`` the vertex delete uses: the card that issued
    this request unmounts the moment it succeeds, so the count has to travel
    inside the sentence the map library shows in its place.
    """

    name: str = Field(..., description="The map that was deleted.")
    vertices_deleted: int = Field(
        ..., description="Stored vertices removed along with the map directory."
    )
    message: str = Field(..., description="What happened, for the operator to read.")


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


class GridRecipe(str, Enum):
    """The two pcd -> gridmap recipes an operator can convert with.

    No "auto" member on purpose: the automatic pick was removed 2026-09 (the
    comment above GRIDMAP_RECIPE in services/gridmap_conversion.py records the
    field failure), so a request either takes the default or names the recipe it
    wants.
    """

    Z_BAND = "z-band"
    TRAVERSABILITY = "traversability"


class ZBandOffsets(BaseModel):
    """Per-request overrides for the z-band recipe's bands.

    Offsets from the **measured** floor level, the same convention as
    GRIDMAP_BANDS_ABOVE_FLOOR — never absolute z, which was the per-site guess
    the offsets exist to remove. Omitted fields keep the recipe's values. The
    ranges are sanity rails, not tuning advice: a floor band 2 m off the floor
    or an obstacle band 8 m tall is a typo, not a site.
    """

    floor_zmin: Optional[float] = Field(None, ge=-2.0, le=2.0)
    floor_zmax: Optional[float] = Field(None, ge=-2.0, le=2.0)
    zmin: Optional[float] = Field(None, ge=-2.0, le=5.0)
    zmax: Optional[float] = Field(None, ge=-2.0, le=8.0)


class ConvertGridRequest(BaseModel):
    """POST /api/v1/maps/{name}/grid/convert — (re)build the 2D gridmap.

    Only ``recipe`` is surfaced in the frontend; the rest are the
    tune-with-the-intermediates-in-front-of-you parameters (curl / MCP), per the
    comment above GRIDMAP_RECIPE in services/gridmap_conversion.py.
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
        None, description="z-band only: band offsets from the measured floor."
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


# --- Helpers ----------------------------------------------------------------


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

    Lives here rather than in the repo because it needs both halves and the repo
    only has one: a sidecar is disk state, "a conversion thread is running" is
    this process's state, and the answer is a function of the two. Same division
    as ``active``, which the router resolves because only it can see the INI.

    The order of the checks is the whole of the logic:

    - A live thread wins over everything. Its sidecar already says ``converting``
      too, but only usually — the write is best-effort and the thread does it
      itself, so the registry is the authority while the process is up.
    - A sidecar still saying ``converting`` with no thread behind it is an
      ``interrupted`` conversion: the process that was running it is gone (a
      backend restart, a killed byobu pane, a mode switch mid-conversion, since
      ``switch_mode`` tears down the session the backend is a pane of). Nothing
      is coming to finish it and nothing would ever clear the flag.
    - ``failed`` outranks a grid being present, and that combination is real
      rather than defensive: a *re*-conversion that fails leaves the previous
      grid on disk, because ``archive_gridmap`` copies it aside instead of
      moving it precisely so the active map never has a window with no file. The
      map is loadable and the rebuild did not happen, and saying ``ok`` there
      would report the stale grid as the requested one.
    - Otherwise disk decides: a grid is a success (including every map converted
      before the sidecar carried a status), no grid is ``none``.
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

    # Read the registry once and derive both fields from it, rather than asking
    # twice: the two would otherwise be sampled either side of a conversion
    # finishing and could disagree, which is the one thing a deprecated alias
    # must never do.
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

    Deliberately not ``(mtime, size)``, which is the cheap and usual choice and
    is broken here. An edited gridmap has the **same dimensions**, so the same
    file size; and this workspace's filesystem hands out a coarse mtime — six
    consecutive writes measured identical ``st_mtime_ns``. The pair would
    therefore be unchanged across a save, and both this router's thumbnail cache
    and the browser would keep serving the pre-edit image.

    The extra cost is one blake2b pass (a few ms on 2.3 MB) over bytes the
    handler has already read, against re-encoding a PNG it did not need to.
    """
    return f'"{hashlib.blake2b(data, digest_size=16).hexdigest()}"'


def _not_modified(request: Request, tag: str) -> bool:
    """Report whether the client already holds this exact content.

    Both endpoints send ``no-cache``, which means "revalidate", not "do not
    store" — so the browser does come back with If-None-Match and this is what
    turns that into a 304 instead of another 2.3 MB body.
    """
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
) -> APIRouter:

    map_router = APIRouter(prefix="", tags=["Map"])

    # --- The loaded map -----------------------------------------------------

    # Plain (non-async) handlers throughout: these walk directories, read PGM
    # headers, encode PNGs and hit psycopg2 for the vertex counts, so FastAPI
    # must run them in its worker thread pool rather than on the event loop.

    # Renderings are re-encoded only when their source file changes. Four cards
    # on the catalogue page would otherwise re-decode and re-scale four multi-
    # megabyte PGMs on every visit; the cloud cache saves a ~20 MB .pcd parse.
    # Keyed by map name, so each holds at most one entry per map on disk.
    thumbnail_cache: Dict[str, Tuple[str, bytes]] = {}
    image_cache: Dict[str, Tuple[str, bytes]] = {}
    cloud_cache: Dict[str, Tuple[Tuple[int, int], bytes]] = {}

    def _vertex_count(name: str) -> int:
        # map_vertices.map holds the bare directory name, the same spelling
        # this catalogue uses. RobotState.map, by contrast, is a path
        # ("map/dp2f/gridmap.yaml") — reconciling the two is exactly why `active`
        # is resolved on this side and not in the UI.
        return len(map_repo.list_vertices(map=name))

    def _require(name: str) -> StoredMap:
        stored = map_catalog_repo.get_map(name)
        if stored is None:
            raise NotFoundError(f"No map named '{name}' on this robot.")
        return stored

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
                f"Saved '{request.name}'."
                + (
                    " Converting to a 2D gridmap in the background."
                    if grid_pending
                    else (
                        " Re-run the conversion through "
                        f"POST /api/v1/maps/{request.name}/grid/convert."
                    )
                )
            ),
        )

    @map_router.post("/api/v1/mapping/reset", response_model=ResetMappingResponse)
    def reset_mapping_run():
        """Discard the run in the robot's memory and start a new map.

        The one route here that touches no file: it is about the *run*, not the
        catalogue, which is why it sits under /api/v1/mapping/ rather than
        /api/v1/maps/. Nothing on disk changes, so a client has no cache to
        invalidate afterwards.

        No request body, and no `save first` flag. A save is POST /api/v1/maps
        and is a separate, deliberate act: the main use for this route is
        abandoning a run that went wrong in its first thirty seconds, and a
        combined endpoint would force a throwaway name and a throwaway
        directory onto exactly that case. It would also owe a compound answer
        for "the save worked but the reset did not", which nothing here wants.

        Failures are 502 with pgo's own sentence, same as a failed save. The
        one an operator actually hits is the wrong mode — pgo only exists in a
        mapping session — and the gateway words it that way.
        """
        ok, detail = map_gw.reset_mapping()
        if not ok:
            logger.error("Failed to reset the mapping run", error=detail)
            raise UpstreamError(detail)

        return ResetMappingResponse(reset=True, message=detail)

    @map_router.patch("/api/v1/maps/{name}", response_model=RenameMapResponse)
    def rename_map(name: str, request: RenameMapRequest):
        """Rename a map: move its directory and re-key the rows that name it.

        Every refusal comes before any mutation, in this order:

        - 404 for a map that is not there.
        - 409 ``map_active`` for the map the stack is running on. This is the
          one that matters. map_server and the FAST-LIO2 localizer opened
          ``map/<name>/…`` during construction, from ``[map] name`` in the
          instance INI; renaming the directory under them would leave both
          holding a path that no longer exists, and leave ``active_name()``
          naming a map the catalogue no longer lists — every card would read
          ``active: false``. This refusal is not the dead end it used to be:
          ``POST /api/v1/maps/{name}/activate`` moves the robot onto another map
          in place, and once it has, the old name is renameable. The refusal
          stands because that route re-points the two processes *and* rewrites
          the INI together, and a rename is not entitled to do half of it. The
          UI greys the control for the active map; this is what makes that more
          than a suggestion.
        - 409 ``conversion_running`` while a gridmap conversion is in flight.
          The conversion thread closed over the old directory path when it
          started, so a rename under it would make it die with ``OSError`` and
          leave the catalogue reporting ``converting`` for a name that is gone.
        - 400 / 409 ``name_taken`` from ``rename_map_dir`` for a bad or
          already-used new name.

        Filesystem first, database second. The directory move is the step most
        likely to fail (target exists, permissions), and failing there needs no
        compensation. The two UPDATEs -- vertices and template bindings -- run
        in one transaction (``MapRepo.transaction``), so the DB half lands or
        fails as a unit; if it fails the directory is moved back, because a
        renamed map whose vertices still answer to the old name is worse than a
        rename that did not happen. They used to be two sessions, and a failure
        in the second left the vertices committed under a name the directory
        had just been moved back from.
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
            # Best-effort compensation. A second failure here is logged and
            # reported, not raised over the first: the operator needs the
            # sentence about the database, and the log needs the path state.
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

        # The renderings are keyed by name. A stale entry under the old name
        # would never be *served* (_png_response re-hashes the file before
        # consulting the cache) but it would sit there forever; drop it.
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
            # The schedule memo is frozen at registration and is a display
            # label only (see routers/schedule.py); nothing here re-registers.
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

        The refusals are rename's, in the same order and before any mutation,
        plus one more:

        - 404 for a map that is not there.
        - 409 ``map_active`` for the map the stack is running on. The argument
          is rename's (see above) only harder: map_server and the FAST-LIO2
          localizer opened ``map/<name>/…`` at launch, and where a rename would
          leave them holding a stale path a delete leaves them holding nothing.
          Switching away with ``POST /api/v1/maps/{name}/activate`` lifts this
          one too — but note that it lifts it by loading the *other* map's files,
          so the map being deleted is genuinely no longer open.
        - 409 ``conversion_running``. The conversion thread closed over the
          directory path and would die writing its own sidecar into a directory
          that is gone, leaving the catalogue with no map and no record.
        - 409 ``template_bound``, which has no rename counterpart because rename
          could re-key the templates and this cannot. A template bound to a
          deleted map is stuck: it will not dispatch, and ``_require_map``
          refuses to *edit* it because the map it names does not exist, while
          clearing its ``map_name`` is blocked for any template holding MOVE
          steps. Deleting the templates instead would take saved work the
          operator never mentioned. So the map is kept and the operator is told
          which templates to unbind first — the one refusal that asks for work
          rather than a map switch.

        **Database first, filesystem last — the inverse of rename, on purpose.**
        A rename puts the directory first because ``os.rename`` back is a real
        compensation; ``shutil.rmtree`` has none, so here the irreversible step
        goes last, once everything that can still fail has already succeeded.
        The two failure modes are not symmetric:

        - Rows first, ``rmtree`` fails: the map is still on disk, minus its
          vertices. The operator retries and the delete converges, and those
          annotations belonged to a map they had just asked to destroy.
        - Directory first, the ``DELETE`` fails: the map is gone and its rows
          survive where nothing can reach them (every vertex route resolves the
          map first, which now 404s) until the next map saved under the same
          name silently inherits them. That one does not converge, and nobody
          would trace it back to a delete that half-failed weeks earlier.

        What survives a successful delete and cannot be helped from here: a
        Temporal schedule registered against this map. Schedules freeze
        concrete, already-resolved steps and carry ``map_name`` only as a memo
        fixed at registration, so one keeps firing coordinates from a map that
        no longer exists. ``template_bound`` catches the templates, not a
        schedule whose template was deleted afterwards, and reaching the
        schedules would mean handing this router ``workflow_gw`` for a warning.
        It is a sentence in the response instead.
        """
        _require(name)

        active_name = map_catalog_repo.active_name()
        if name == active_name:
            raise ConflictError(
                f"'{name}' is the map the stack is running on and cannot be "
                "deleted while it is in use. Switch the robot to another map "
                "first.",
                code="map_active",
            )
        if conversion_svc.is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; delete it "
                "once the conversion has finished.",
                code="conversion_running",
            )

        # Strictly this map's templates: include_map_independent=False leaves
        # the map_name IS NULL rows alone, because a posture-only template runs
        # anywhere and a map going away means nothing to it.
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
                f"{'it' if len(bound) == 1 else 'them'} first — a template "
                "bound to a map that is gone can neither run nor be edited.",
                code="template_bound",
            )

        try:
            vertices_deleted = map_repo.delete_vertices(name)
        except Exception as exc:
            # Nothing to compensate -- that is the whole point of going first --
            # but the operator has to be told the map is still there, or a
            # failed delete reads as a delete that worked and did not refresh.
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

        # Keyed by name, and unlike rename's there is no new name to move them
        # to: drop them or they sit in the process until it restarts.
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

    @map_router.post(
        "/api/v1/maps/{name}/activate", response_model=ActivateMapResponse
    )
    async def activate_map(name: str):
        """Switch the robot onto another map, live, without restarting the stack.

        For most of this file's history the answer to "can I switch maps?" was
        no: map_server and the localizer read `[map] map` / `[map] pcd` during
        construction, nothing in this backend wrote the INI, and so rename and
        delete both told the operator to restart the stack. This is the route
        that makes those refusals escapable, and it works because both
        processes turned out to already have a runtime door — `map_server/load_map`
        re-reads a yaml off disk, and `relocalize` takes a `pcd_path`.

        Three moving parts have to end up agreeing, and the whole design is about
        what happens when one of them does not:

        - the localizer's point cloud (what the robot matches against),
        - map_server's occupancy grid (what the planner and both costmaps see),
        - `[map] name` in the INI (what survives a restart, and what
          `active_name()` — and therefore every card's `active` flag, the
          dashboard's vertex scoping and the task templates' map binding —
          reports right now).

        Order is chosen so the likeliest failure is the cleanest one. The
        localizer goes first because its refusals happen before it mutates
        anything (`relocCB` returns early on a missing file, and
        `ICPLocalizer::loadMap` swaps its buffers only as its last statement), so
        a bad PCD leaves *nothing* changed. Each later step compensates the
        earlier ones by putting the old map back. The INI is last and is
        preflighted for writability, because a swap the INI does not record is
        the worst of the available outcomes: the stack would be driving on one
        map while every REST answer named another.

        Every refusal is before any of that. Notably `stack_not_ready`, which is
        how "the robot is in mapping mode" is detected — service discoverability
        rather than the cached RobotState's mode, because RobotRepo's write is
        gated on `localization_valid` and a robot that has lost localization
        therefore has no cached mode at all. That robot is precisely the one
        whose operator is reaching for this route.

        What this deliberately does not do is guarantee the robot knows where it
        is afterwards. `[initial_pose]` is zeroed rather than carried over — a
        pose measured in the old map's frame is not wrong-looking in the new one,
        it is wrong-looking nowhere, and it can drop the robot inside a wall with
        every indicator green. The operator sets the real pose from the
        dashboard. `localized` reports whether registration converged in the few
        seconds after the swap, which is the only honest signal available.
        """
        stored = _require(name)

        previous = map_catalog_repo.active_name()
        if name == previous:
            # The no-op, not a failure — same shape as sys_manager's switch_mode
            # answering "Already in AUTO; nothing to do". Tearing the localizer
            # down and rebuilding it onto the map it already holds would only
            # throw away a good registration.
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
            # map_server throws out of its constructor on an unloadable yaml and
            # main() has no try/catch, so at *launch* this would take the whole
            # AUTO session down. Here it would only fail the load_map call, but
            # refusing up front is what keeps that from being discovered with the
            # localizer already swapped.
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
            # Preflighted so the last step of the swap cannot realistically fail
            # after the robot has already moved onto the new map.
            raise ConflictError(
                f"The system INI ({ini_path}) is not writable, so a switch "
                "could not be recorded and would not survive a restart.",
                code="ini_not_writable",
            )

        try:
            active_tasks, _ = await workflow_gw.list_active_tasks()
        except Exception as exc:
            # Refuse rather than assume idle. The gateway caches for
            # ACTIVE_TASK_CACHE_TTL_S, so this is a real Temporal outage, and
            # swapping the map under a robot that might be mid-navigation is not
            # something to do on an unverified guess.
            logger.warning(
                "Could not confirm the robot is idle before a map switch",
                map=name,
                error=str(exc),
            )
            raise ConflictError(
                "Could not reach Temporal to confirm no task is running, and a "
                "map switch under a moving robot is not safe to guess at. Check "
                "the Tasks page and try again.",
                code="tasks_unknown",
            )

        if active_tasks:
            running = ", ".join(task.id for task in active_tasks)
            raise ConflictError(
                f"The robot is running {len(active_tasks)} "
                f"{'task' if len(active_tasks) == 1 else 'tasks'} ({running}). "
                "Cancel or wait for it before switching maps.",
                code="task_running",
            )

        # Everything from here on is blocking: two wait_for_service probes,
        # a relocalize that can take a minute, a load_map, an INI write and a
        # convergence poll. This route has to be `async def` because confirming
        # the robot is idle is a Temporal await and the client belongs to the
        # uvicorn loop -- so the blocking half goes to a worker thread instead,
        # which is what the rest of this file gets for free by being plain
        # `def`. Left on the loop it froze every WebSocket (telemetry,
        # pointcloud, and the teleop watchdog that stops the robot) for the
        # length of the swap.
        #
        # The preflight above stays on the loop deliberately: it is stat calls
        # and one small INI read, and moving it would buy microseconds while
        # changing the order failures are reported in.
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
                    # Either the INI named no map at all (which is itself what makes
                    # the write below fail, so this is the *likely* pairing, not an
                    # exotic one) or the map it named has since lost its map.pcd.
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
                # Nothing has changed: this is the one step whose failure needs no
                # compensation at all.
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
                # Undone in the order they were done, and as statements rather than
                # inside the message below: built by concatenation these run in
                # argument-evaluation order, which reads as the reverse of what it
                # does and would silently flip if the sentences were reordered.
                restored_cloud = _restore_localizer("INI write failed")

                previous_yaml = (
                    map_catalog_repo.gridmap_yaml_path(previous) if previous else None
                )
                if previous_yaml is None:
                    # The map the stack launched on has since lost its gridmap.yaml
                    # (map_server loaded that file to start at all), or the INI named
                    # no map. Say so rather than leave map_server quietly serving a
                    # grid that no longer matches the cloud the localizer went back
                    # to.
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

            # Only the vertex-scoping consumers care, and they re-query; the
            # renderings are keyed by map name, so nothing cached is now stale.
            localized = map_gw.localization_converged()

            logger.info(
                "Switched the active map",
                map=name,
                previous=previous,
                localized=localized,
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
            return ActivateMapResponse(
                name=name,
                previous=previous,
                switched=True,
                localized=localized,
                message=message,
            )

        return await asyncio.to_thread(_switch)

    @map_router.post(
        "/api/v1/maps/{name}/grid/convert", response_model=ConvertGridResponse
    )
    def convert_map_grid(name: str, request: ConvertGridRequest):
        """(Re)build a map's 2D gridmap from its map.pcd, in the background.

        This is the formal home of what used to be done with a one-off script
        that never made it into the repo: choosing the recipe when the default
        is wrong for the site (a warehouse too large to hand-edit wants
        traversability), re-converting after a parameter override, and getting
        the traversability pipeline's intermediate clouds for tuning a site the
        defaults cannot handle — the next outdoor venue tunes from those, not
        from constants guessed in advance.

        Two 409s can come back and the client must tell them apart (the `code`
        field): `conversion_running` means try later; `gridmap_hand_edited`
        means the current grid holds operator edits and the caller has to
        confirm with `overwrite_edits` — even then the edited grid survives as
        gridmap_prev.pgm.
        """
        stored = _require(name)
        if not stored.has_pointcloud:
            raise BadRequestError(
                f"Map '{name}' has no map.pcd — there is nothing to convert."
            )

        # Cross-field validation as readable 400s rather than schema 422s: which
        # parameter belongs to which recipe is domain knowledge, and the FastAPI
        # 422 for it would name a field, not the mismatch.
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

        band_overrides: Optional[Dict[str, float]] = None
        if request.z_band_offsets is not None:
            band_overrides = request.z_band_offsets.model_dump(exclude_none=True)
            merged = {**GRIDMAP_BANDS_ABOVE_FLOOR, **band_overrides}
            # Ordering is checked on the *merged* bands: a request overriding one
            # end of a band can invert it against the recipe's other end, and
            # that inversion selects no points and fails half a minute later in
            # the thread, where nothing answers the operator.
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

        param_overrides: Dict[str, object] = {}
        if band_overrides:
            param_overrides["z_band_offsets"] = band_overrides
        if grid_overrides:
            param_overrides.update(grid_overrides)
        override: Dict[str, object] = {
            "requested": request.recipe.value,
            "picked_by": f"POST /api/v1/maps/{name}/grid/convert",
            "reason": request.reason,
            "at": iso_now(),
            "param_overrides": param_overrides,
        }

        # An active map that re-converts must reach map_server, or it keeps
        # serving the grid the operator just replaced — the same reload the
        # gridmap-editor save does, deferred into the thread because the grid
        # does not exist yet when this handler answers.
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
            band_offset_overrides=band_overrides,
            grid_overrides=grid_overrides,
            debug=request.debug,
            override=override,
            archive=lambda: map_catalog_repo.archive_gridmap(name),
            on_success=on_success,
        )
        # started=False cannot happen past the has_pointcloud gate above short of
        # a race deleting map.pcd; report it honestly rather than asserting.
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
        # _require first: without it an unknown map name returns [] — the same
        # answer as a real map with no vertices yet, which is the harder of the
        # two states to debug from the UI side.
        _require(name)
        vertices = map_repo.list_vertices(map=name, type=type.value if type else None)
        return [_vertex_response(vertex) for vertex in vertices]

    @map_router.post("/api/v1/maps/{name}/vertices", response_model=List[MapVertexResponse])
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

        A vertex id is unique on its own, so the map name in the URL is not
        needed to find the row — it is here to be checked. Without this a client
        could read or delete any vertex through any map's URL, and the response
        would contradict the path it came from.

        A row that exists but belongs elsewhere is a 404 rather than a 403: from
        this URL's point of view the resource genuinely is not there, and saying
        "wrong map" would confirm the id exists to a caller who addressed the
        wrong map.
        """
        _require(name)
        vertex = map_repo.get_vertex(vertex_id=vertex_id)
        if vertex is None or vertex.map != name:
            raise NotFoundError(f"Map vertex {vertex_id} was not found in '{name}'.")
        return vertex

    @map_router.get("/api/v1/maps/{name}/vertices/{id}", response_model=MapVertexResponse)
    def get_map_vertex(name: str, id: uuid.UUID):
        return _vertex_response(_require_vertex(name, id))

    @map_router.put("/api/v1/maps/{name}/vertices/{id}", response_model=MapVertexResponse)
    def update_map_vertex(name: str, id: uuid.UUID, req: MapVertexUpdateRequest):
        _require_vertex(name, id)

        changes = req.model_dump(exclude_unset=True)
        if "type" in changes and changes["type"] is not None:
            changes["type"] = changes["type"].value

        vertex = map_repo.update_vertex(id, **changes)
        if vertex is None:
            raise NotFoundError(f"Map vertex {id} was not found in '{name}'.")
        return _vertex_response(vertex)

    @map_router.delete("/api/v1/maps/{name}/vertices/{id}", response_model=DeleteResponse)
    def delete_map_vertex(name: str, id: uuid.UUID):
        _require_vertex(name, id)
        map_repo.delete_vertex(vertex_id=id)
        return DeleteResponse(message=f"Map vertex {id} has been deleted.")

    def _read_gridmap(name: str) -> bytes:
        """Read the map's gridmap.pgm bytes, 404ing with the reason if absent.

        No endpoint serves these bytes as-is. /image is the wire format for the
        grid: a lossless full-size PNG of the same cells, which every browser
        can already decode, against a P5 the client would need a parser for —
        one that has to tolerate the ``#`` comment line GIMP writes.
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

        Shared by /image and /thumbnail because the only thing that differs
        between them is the render call: both key their cache and their ETag on
        the *source* .pgm bytes rather than the encoded PNG, so a rendering that
        is a deterministic function of the gridmap changes exactly when the
        gridmap does.
        """
        payload = _read_gridmap(name)
        tag = _content_tag(payload)
        headers = {"ETag": tag, "Cache-Control": "no-cache"}

        # Before rendering, not after: a client holding the current image must not
        # cost us a PNG encode we then throw away.
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
        """The map's gridmap as a full-size PNG.

        Raw bytes, not the base64-in-JSON envelope the loaded-map endpoint uses:
        the consumer is an <img>/texture load, and base64 would cost a third
        more bytes for a value nothing reads as a string.
        """
        return _png_response(name, request, image_cache, render_png, "image")

    @map_router.get("/api/v1/maps/{name}/thumbnail")
    def get_map_thumbnail(name: str, request: Request):
        return _png_response(name, request, thumbnail_cache, render_thumbnail, "thumbnail")

    @map_router.put("/api/v1/maps/{name}/grid", response_model=SaveGridmapResponse)
    def save_map_grid(
        name: str,
        response: Response,
        payload: bytes = Body(..., media_type="application/octet-stream"),
    ):
        """Write an edited gridmap back, and reload it if it is the live one.

        The body is the cells themselves: exactly ``width * height`` bytes in
        .pgm row order (row 0 is the top of the map, max y). Raw rather than a
        PNG or base64-in-JSON because it is what the editor already holds — the
        client's buffer goes out as a memcpy and the server writes it into a P5
        body verbatim, so there is no encode, no decode, and no chance of a
        colour-managed round trip shifting 205 to 204. (The GET side had to pass
        ``colorSpaceConversion: "none"`` to stop exactly that.) The cost is ~1.6
        MB on the wire per save, on a robot LAN, once per operator edit.

        A plain ``def``, like everything else here, and that is load-bearing: this
        handler fsyncs a multi-megabyte write and then parks on
        ``MapGateway.reload_map`` for up to 25 s. On the event loop that would
        stall every other request in the process, the telemetry WebSocket
        included. ``bytes = Body(...)`` is what makes it possible — FastAPI reads
        the body in its async layer and hands the finished bytes to the
        threadpool. The alternative (``async def`` + ``await request.body()`` +
        ``run_in_threadpool`` twice) buys nothing — a ``bytes`` body parameter
        already receives the raw payload whatever the Content-Type header says;
        the ``media_type`` here is OpenAPI documentation, not enforcement — and
        needs two threadpool hops a later edit can silently drop.

        Cell *values* are deliberately not validated. map_io.cpp classifies by
        range (occupied <= 89, unknown 90..205, free >= 206) under the
        ``negate: 0 / 0.65 / 0.196`` every gridmap.yaml here carries, and two of
        the real maps already contain 255s from a round of hand-editing in GIMP.
        A ``{0, 205, 254}`` whitelist would refuse to save a map this same round
        trip just handed the client. The length is the only thing that can make a
        file map_server would misread.

        No body-size cap either: the payload is already in memory by the time
        this runs, so rejecting on Content-Length would mean streaming, and this
        whole API is unauthenticated on a robot LAN — a cap is middleware's job.
        """
        stored = _require(name)

        if conversion_svc.is_converting(name):
            # The conversion thread is about to os.replace() gridmap.pgm and
            # gridmap.yaml with whatever it computes. A save landing in that
            # window is silently overwritten, and the once-only gridmap_raw.pgm
            # snapshot write_gridmap takes could capture the half-finished grid
            # as the "original". Same refusal rename, delete and activate make.
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; save the edit "
                "once it has finished, or the conversion will overwrite it.",
                code="conversion_running",
            )

        # stored.grid rather than a fresh read: _read_grid has already parsed the
        # .pgm header *and* gridmap.yaml, so one None test covers "no pgm", "no
        # yaml" and "torn pgm" — and it makes "the two agree" a precondition of
        # saving, which matters because map_server re-reads both a few lines down.
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

        # No cache eviction here, and that is the design rather than an oversight.
        # thumbnail_cache and image_cache are in this same closure, so reaching
        # them is trivial — but _png_response re-reads the file and re-hashes it
        # *before* consulting the cache, so a stale entry can never be served.
        # Evicting would add a second place that has to remember the caches exist,
        # making /image's correctness look like it depends on this handler; and
        # popping before the write would be actively wrong, throwing away a valid
        # rendering if the write then failed. There is no thumbnail file on disk
        # to update either: /thumbnail renders from these bytes on demand.
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

        # None only if gridmap.yaml vanished since stored.grid was read — the same
        # race the repo's own isfile check covers. Handled as a failed reload
        # rather than left to hand the gateway a None it would abspath().
        yaml_path = map_catalog_repo.gridmap_yaml_path(name)
        if yaml_path is None:
            reloaded, detail = False, "gridmap.yaml is missing"
        else:
            reloaded, detail = map_gw.reload_map(yaml_path)

        if not reloaded:
            # Still a 200. The bytes are on disk and every GET now returns them,
            # so a 5xx would tell the operator the save failed when it did not —
            # and they would either press save again or re-edit a grid they
            # believe was lost. `reloaded` is the machine-readable half of the
            # answer, `message` the human one. 202/207 were considered and
            # dropped: no client here understands them, and nothing is partial or
            # queued — the request completed, one of its two effects did not.
            logger.error("Saved gridmap but map_server did not reload", map=name, error=detail)
            # `detail` rides in the message, not only in the log: the gateway
            # goes out of its way to distinguish "service not there" from
            # "map_server rejected it" from a timeout, and the operator staring
            # at a stale map is the one who needs that distinction.
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

    @map_router.get("/api/v1/maps/{name}/pointcloud")
    def get_map_pointcloud_by_name(name: str):
        """The map's saved map.pcd, packed for the viewer.

        Wire format matches the live endpoint and the WebSocket stream: a
        little-endian uint32 point count followed by ``3 * count`` little-endian
        float32 xyz values. Decimated with the same voxel size and cap the
        point-cloud subscriber applies to ``localizer/map_cloud``, so a client
        gets a comparable cloud whichever endpoint it reads.

        Cached on the file's (size, mtime) rather than a content hash: parsing a
        ~20 MB .pcd is the expensive part and hashing it first would mean
        reading the whole file anyway. The objection that made ``_content_tag``
        hash gridmap bytes does not apply here — nothing edits map.pcd in place,
        it is only ever replaced wholesale by pgo/save_maps, which changes the
        point count and so the size.
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
