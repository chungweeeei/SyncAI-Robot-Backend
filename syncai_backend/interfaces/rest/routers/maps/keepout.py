"""A map's forbidden zones: read and save keepout.json and its mask."""


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
    map_usage = ctx.map_usage
    _require = ctx.require
    job_lock = ctx.job_lock

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
