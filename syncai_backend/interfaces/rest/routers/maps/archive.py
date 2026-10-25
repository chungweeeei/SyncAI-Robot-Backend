"""Maps as archives: export one, import one (both spooled to disk)."""

import asyncio
import os
import time
from typing import AsyncIterator, BinaryIO, Optional

from fastapi import APIRouter, Depends, Request
from fastapi.responses import StreamingResponse

from syncai_backend.exceptions import (
    PayloadTooLargeError,
)
from syncai_backend.helpers.map_archive import (
    ArchiveFormat,
    write_archive,
)

from syncai_backend.services.gridmap_conversion import (
    TRAVERSABLE_DEBUG_SUBDIR_NAME,
    iso_now,
)
from syncai_backend.interfaces.rest.routers.maps.schemas import (
    ImportMapResponse,
    VertexType,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


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


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_repo = ctx.map_repo
    map_catalog_repo = ctx.map_catalog_repo
    map_usage = ctx.map_usage
    map_import_svc = ctx.map_import_svc
    renders = ctx.renders
    _require_exists = ctx.require_exists

    @map_router.get("/api/v1/maps/{name}/export")
    def export_map(name: str, format: ArchiveFormat = ArchiveFormat.ZIP):
        """The map directory as one archive, with its vertices and an md5 manifest.

        Every file under ``map/<name>/`` except ``traversable_debug/``, plus
        ``syncai_map.json`` (name, vertices without ids, md5 per file); layout
        in ``helpers/map_archive``. 409 ``conversion_running``, since the grid
        is about to be replaced. Exporting the active map is fine.
        """
        _require_exists(name)
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
