"""The 2D gridmap: convert one from map.pcd, serve it as PNG, save an edit."""

import os
from typing import Callable, Dict, Optional, Tuple

from fastapi import APIRouter, Body, Request
from fastapi.responses import Response

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    NotFoundError,
)
from syncai_backend.helpers.pgm import render_png, render_thumbnail

from syncai_backend.services.gridmap_conversion import (
    FLOOR_REFERENCE_DEFAULT,
    GRIDMAP_BANDS_ABOVE_FLOOR,
    iso_now,
)
from syncai_backend.interfaces.rest.routers.maps.schemas import (
    ConvertGridRequest,
    ConvertGridResponse,
    GridRecipe,
    SaveGridmapResponse,
)
from syncai_backend.interfaces.rest.routers.maps.common import (
    _file_tag,
    _no_gridmap,
    _revalidate,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_catalog_repo = ctx.map_catalog_repo
    map_gw = ctx.map_gw
    conversion_svc = ctx.conversion_svc
    map_usage = ctx.map_usage
    renders = ctx.renders
    _require = ctx.require
    _require_exists = ctx.require_exists
    job_lock = ctx.job_lock

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

    def _gridmap_file(name: str) -> Tuple[str, os.stat_result]:
        """The map's gridmap.pgm path and stat, 404ing with the reason if absent.

        Never served raw: /image's lossless PNG is the grid's wire format.
        """
        _require_exists(name)
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
