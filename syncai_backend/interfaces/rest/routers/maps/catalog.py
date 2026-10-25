"""The map catalogue: list, read, save a mapping run as a map, rename, delete."""

import os
from typing import List

from fastapi import APIRouter

from syncai_backend.exceptions import (
    BadRequestError,
    UpstreamError,
)

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    CreateMapRequest,
    CreateMapResponse,
    DeleteMapResponse,
    MapSummaryResponse,
    RenameMapRequest,
    RenameMapResponse,
)
from syncai_backend.interfaces.rest.routers.maps.common import (
    _summary,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_repo = ctx.map_repo
    map_catalog_repo = ctx.map_catalog_repo
    map_gw = ctx.map_gw
    task_template_repo = ctx.task_template_repo
    conversion_svc = ctx.conversion_svc
    map_usage = ctx.map_usage
    _refuse_while_idle = ctx.refuse_while_idle
    renders = ctx.renders
    _require = ctx.require

    def _vertex_count(name: str) -> int:
        # One map's count (GET /maps/{name}); the listing uses
        # count_vertices_by_map. Never len(list_vertices(...)).

        return map_repo.count_vertices(map=name)

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
