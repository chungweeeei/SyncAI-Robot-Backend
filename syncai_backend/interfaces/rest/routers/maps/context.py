"""What the map route modules share: their dependencies, the render cache, and
the checks more than one of them makes.

Built once per router by ``init_map_router``, so a test's client gets a fresh
render cache, as it did when these were locals of that function.
"""

from dataclasses import dataclass, field
from typing import List, Optional

import structlog
from fastapi import Depends

from syncai_backend.exceptions import ConflictError, NotFoundError
from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.repositories.map.catalog import MapCatalogRepo, StoredMap
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.mapping.mapping import (
    MappingState,
    MappingStatusRepo,
)
from syncai_backend.repositories.task.task_template import TaskTemplateRepo
from syncai_backend.services.gridmap_conversion import GridmapConversionService
from syncai_backend.services.map_import import MapImportService
from syncai_backend.services.map_switch import MapSwitchService
from syncai_backend.services.map_usage import JobGate, MapUsagePolicy

from syncai_backend.interfaces.rest.routers.maps.common import _RenderCache


@dataclass
class MapRouteContext:
    logger: structlog.stdlib.BoundLogger
    map_repo: MapRepo
    map_catalog_repo: MapCatalogRepo
    map_gw: MapGateway
    task_template_repo: TaskTemplateRepo
    conversion_svc: GridmapConversionService
    mapping_status_repo: MappingStatusRepo
    map_switch_svc: MapSwitchService
    map_usage: MapUsagePolicy
    job_gate: JobGate
    map_import_svc: MapImportService

    # The image, thumbnail and packed cloud of each map, re-encoded only when
    # their source changes.
    renders: _RenderCache = field(default_factory=_RenderCache)

    # Async so the Temporal call runs on the event loop while the plain
    # ``def`` route still runs in the threadpool. See services/map_usage.py.
    job_lock: List = field(init=False)

    def __post_init__(self) -> None:
        self.job_lock = [Depends(self.job_gate.refuse_while_a_job_holds)]

    def require(self, name: str) -> StoredMap:
        stored = self.map_catalog_repo.get_map(name)
        if stored is None:
            raise NotFoundError(f"No map named '{name}' on this robot.")
        return stored

    # pgo's latched run state, or None if it has not reported. The run routes
    # turn pgo's refusals into a coded 409 instead of a 502. Check-then-act,
    # not a lock: pgo's own precondition still holds; None lets pgo answer.
    def run_state(self) -> Optional[MappingState]:
        status = self.mapping_status_repo.get()
        return None if status is None else status.state

    def refuse_while_resetting(self) -> None:
        if self.run_state() is MappingState.RESETTING:
            raise ConflictError(
                "The robot is starting or resetting a map right now; try again "
                "in a moment.",
                code="mapping_busy",
            )

    def refuse_while_idle(self, nothing_to: str) -> None:
        state = self.run_state()
        if state is MappingState.IDLE:
            raise ConflictError(
                f"There is no map to {nothing_to} — mapping has not been started.",
                code="mapping_idle",
            )
        self.refuse_while_resetting()
