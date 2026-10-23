"""Create or replace a map from an archive ``GET /api/v1/maps/{name}/export`` made.

POST /api/v1/maps/import is the only caller. Like the map switch it is a
small saga over two stores that share no transaction -- the map directory on
disk and its rows in ``map_vertices`` -- so the order and the undo for each
step live here, beside each other, rather than inside the route:

1. Judge the archive (manifest, members, ``map.pcd``, vertex types) and the
   target (name, delete's 409s when replacing, free disk). Nothing written.
2. Unpack and md5-verify into ``map/.import-<token>/`` and publish it with one
   ``os.rename``, parking a same-named map as ``map/.import-old-<token>/``.
   Staging stays inside ``map/`` because it is a bind mount (a sibling path is
   another filesystem, and the rename would not be atomic).
3. One transaction deletes every row naming the map (orphans too) and inserts
   the manifest's vertices. A failure swaps the directories back.
4. ``rmtree`` the parked map: the irreversible step last.

No conversion is started.
"""

import time
from dataclasses import dataclass
from typing import AbstractSet, Optional

import structlog

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.helpers.map_archive import (
    ArchiveFormat,
    MapArchiveError,
    extract_archive,
    inspect_archive,
)
from syncai_backend.repositories.map.catalog import POINTCLOUD_PCD, MapCatalogRepo
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.services.map_usage import MapUsagePolicy


@dataclass(frozen=True)
class ImportOutcome:
    name: str
    replaced: bool
    format: ArchiveFormat
    files: int
    bytes: int
    vertices_created: int
    vertices_deleted: int


class MapImportService:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        map_repo: MapRepo,
        map_catalog_repo: MapCatalogRepo,
        map_usage: MapUsagePolicy,
    ):
        self._logger = logger
        self._map_repo = map_repo
        self._catalog = map_catalog_repo
        self._map_usage = map_usage

    def import_archive(
        self,
        payload: bytes,
        name: Optional[str],
        vertex_types: AbstractSet[str],
    ) -> ImportOutcome:
        """Import ``payload`` as ``name`` (or the manifest's name); blocking.

        ``vertex_types`` is what a manifest's vertices may be: the REST layer's
        ``VertexType``, the only place that set is defined.

        400 for anything wrong with the archive or the name; delete's 409s when
        it would replace a map in use; 409 ``disk_low``; 502 when the vertex
        rows could not be written (the directories are swapped back first).
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
        unknown = sorted(
            {str(v["type"]) for v in manifest.vertices if v["type"] not in vertex_types}
        )
        if unknown:
            raise BadRequestError(
                f"Unknown vertex type(s) in the manifest: {', '.join(unknown)}."
            )

        target = name if name is not None else manifest.name
        self._catalog.resolve_dir(target)
        replacing = self._catalog.get_map(target) is not None
        if replacing:
            self._map_usage.refuse_destroy(
                target,
                verb="replaced",
                template_consequence=(
                    "its MOVE steps point at this map's current vertices, which the "
                    "import replaces."
                ),
            )

        if inspected.total_bytes > self._catalog.free_bytes():
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
                "z": float(v.get("z", 0.0)),
            }
            for v in manifest.vertices
        ]

        staging = self._catalog.begin_import(target)
        started = time.perf_counter()
        try:
            written = extract_archive(payload, inspected, staging)
            _, displaced = self._catalog.commit_import(target, staging)
        except MapArchiveError as exc:
            self._catalog.abort_import(staging)
            raise BadRequestError(str(exc))
        except BaseException:
            self._catalog.abort_import(staging)
            raise
        extract_ms = round((time.perf_counter() - started) * 1000)

        started = time.perf_counter()
        try:
            with self._map_repo.transaction(op="import_map") as session:
                vertices_deleted = self._map_repo.delete_vertices(target, session=session)
                created = (
                    self._map_repo.create_vertices(target, new_vertices, session=session)
                    if new_vertices
                    else []
                )
        except Exception as exc:
            self._catalog.undo_import(target, displaced)
            self._logger.error(
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
            self._catalog.abort_import(displaced)

        self._logger.info(
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
        return ImportOutcome(
            name=target,
            replaced=replacing,
            format=inspected.format,
            files=len(manifest.files),
            bytes=written,
            vertices_created=len(created),
            vertices_deleted=vertices_deleted,
        )


def init_map_import_service(
    logger: structlog.stdlib.BoundLogger,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    map_usage: MapUsagePolicy,
) -> MapImportService:
    return MapImportService(
        logger=logger,
        map_repo=map_repo,
        map_catalog_repo=map_catalog_repo,
        map_usage=map_usage,
    )
