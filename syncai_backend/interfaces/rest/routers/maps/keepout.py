"""A map's forbidden zones: read and save keepout.json."""


from fastapi import APIRouter

from syncai_backend.exceptions import (
    NotFoundError,
)

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    KeepoutResponse,
    SaveKeepoutRequest,
    SaveKeepoutResponse,
)
from syncai_backend.interfaces.rest.routers.maps.common import (
    _validate_zones,
    _zone_responses,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_catalog_repo = ctx.map_catalog_repo
    map_gw = ctx.map_gw
    _require_exists = ctx.require_exists
    job_lock = ctx.job_lock

    @map_router.get("/api/v1/maps/{name}/keepout", response_model=KeepoutResponse)
    def get_map_keepout(name: str):
        """The map's forbidden zones, as polygons in map-frame metres.

        Read from ``keepout.json``; a map without one answers ``zones: []``.
        """
        _require_exists(name)
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

        Plain ``def``: it may park on the planner's reload. 404 without an
        ``octomap.bt`` (nothing to plan on, so nothing to forbid), and 200
        ``reloaded: false`` when the save worked but the reload did not.

        The body is the *whole* list; ``[]`` clears the zones by rewriting
        ``keepout.json`` with none. An active map's zones go to the 3D planner
        (``planner_server/Octo3D/load_keepout``); any other map's are read
        when it is next loaded.
        """
        _require_exists(name)
        if map_catalog_repo.octomap_path(name) is None:
            raise NotFoundError(
                f"Map '{name}' has no 3D map (octomap.bt) to draw forbidden zones on."
            )

        zones = _validate_zones(request.zones)
        map_catalog_repo.write_keepout(name, zones)

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
        json_path = map_catalog_repo.keepout_json_path(name)
        if json_path is None:
            reloaded, detail = False, "keepout.json is missing"
        else:
            reloaded, detail = map_gw.reload_keepout_3d(json_path)

        if not reloaded:
            logger.error(
                "Saved forbidden zones but the planner did not reload them",
                map=name,
                error=detail,
            )
            return SaveKeepoutResponse(
                name=name,
                zones=_zone_responses(zones),
                active=True,
                reloaded=False,
                message=f"{saved}, but the 3D planner did not reload: {detail}",
            )

        return SaveKeepoutResponse(
            name=name,
            zones=_zone_responses(zones),
            active=True,
            reloaded=True,
            message=f"{saved} and reloaded the 3D planner.",
        )
