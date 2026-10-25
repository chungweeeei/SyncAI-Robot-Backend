"""The map routes: catalogue, archives, mapping run, the loaded map, gridmap,
vertices, forbidden zones, point cloud, 3D map and floors.

The routes live in ``routers/maps/``, one module per area; this composes them
under the one ``init_map_router`` that ``server.py`` calls. Plain (non-async)
handlers throughout, unless a route awaits Temporal: they block on disk, PNG
encoding and psycopg2, so FastAPI must run them in its thread pool.

The saved map is read from disk on request, never subscribed: every route here
reads ``map/<name>/`` when it is asked. ``map`` and ``localizer/map_cloud`` used
to be subscribed (TRANSIENT_LOCAL, latched), which held a copy of whichever map
was loaded and none of the others the console lists.
"""

import structlog
from fastapi import APIRouter

from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.mapping.mapping import MappingStatusRepo
from syncai_backend.repositories.task.task_template import TaskTemplateRepo
from syncai_backend.services.gridmap_conversion import GridmapConversionService
from syncai_backend.services.map_import import MapImportService
from syncai_backend.services.map_switch import MapSwitchService
from syncai_backend.services.map_usage import JobGate, MapUsagePolicy

from syncai_backend.interfaces.rest.routers.maps import (
    archive,
    catalog,
    cloud,
    floors,
    grid,
    keepout,
    mapping,
    octomap,
    switch,
    vertices,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext

# Registration order. Paths do not overlap across areas, so this only orders
# the OpenAPI listing.
_AREAS = (catalog, archive, mapping, switch, grid, vertices, keepout, cloud, octomap, floors)


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
    ctx = MapRouteContext(
        logger=logger,
        map_repo=map_repo,
        map_catalog_repo=map_catalog_repo,
        map_gw=map_gw,
        task_template_repo=task_template_repo,
        conversion_svc=conversion_svc,
        mapping_status_repo=mapping_status_repo,
        map_switch_svc=map_switch_svc,
        map_usage=map_usage,
        job_gate=job_gate,
        map_import_svc=map_import_svc,
    )
    map_router = APIRouter(prefix="", tags=["Map"])
    for area in _AREAS:
        area.register(map_router, ctx)
    return map_router
