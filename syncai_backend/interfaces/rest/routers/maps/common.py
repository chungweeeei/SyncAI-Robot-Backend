"""What more than one map route module needs: statuses, summaries, the ETag
helpers and the render cache."""

import math
import threading
import uuid
from collections import OrderedDict
from datetime import datetime, timezone
from typing import Callable, Dict, Hashable, List, Optional, Tuple

from fastapi import Request
from fastapi.responses import Response

from syncai_backend.exceptions import (
    BadRequestError,
)
from syncai_backend.database.models import MapPoint
from syncai_backend.repositories.map.catalog import (
    KeepoutZone,
    MapCleanStatus,
    StoredMap,
)

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    KeepoutZoneRequest,
    KeepoutZoneResponse,
    MapSummaryResponse,
    MapVertexResponse,
    OctomapStatus,
    ZonePoint,
)


# --- Helpers ----------------------------------------------------------------


def iso_now() -> str:
    """UTC, ISO 8601, ``Z``-suffixed, like ``MapSummaryResponse.modified_at``."""
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _validate_zones(zones: List[KeepoutZoneRequest]) -> List[KeepoutZone]:
    """Turn the request's zones into repo zones, or raise a 400 with a sentence.

    The pydantic models are structural only, so these rules answer 400 with a
    sentence rather than pydantic's 422 list.
    """
    seen: set = set()
    result: List[KeepoutZone] = []
    for index, zone in enumerate(zones, start=1):
        if len(zone.points) < 3:
            raise BadRequestError(
                f"Zone {index} has {len(zone.points)} point"
                f"{'' if len(zone.points) == 1 else 's'}; a forbidden zone needs at least 3."
            )
        for point in zone.points:
            if not (math.isfinite(point.x) and math.isfinite(point.y)):
                raise BadRequestError(
                    f"Zone {index} has a corner that is not a finite coordinate."
                )
        for label, value in (("z_min", zone.z_min), ("z_max", zone.z_max)):
            if value is not None and not math.isfinite(value):
                raise BadRequestError(f"Zone {index} has a {label} that is not a finite height.")
        if zone.z_min is not None and zone.z_max is not None and zone.z_min > zone.z_max:
            raise BadRequestError(
                f"Zone {index} has z_min {zone.z_min} above z_max {zone.z_max}."
            )
        zone_id = zone.id if zone.id is not None else uuid.uuid4().hex
        if not zone_id:
            raise BadRequestError(f"Zone {index} has an empty id.")
        if zone_id in seen:
            raise BadRequestError(f"Zone id {zone_id!r} appears more than once.")
        seen.add(zone_id)
        result.append(
            KeepoutZone(
                id=zone_id,
                points=tuple((point.x, point.y) for point in zone.points),
                name=zone.name,
                z_min=zone.z_min,
                z_max=zone.z_max,
            )
        )
    return result


def _zone_responses(zones: List[KeepoutZone]) -> List[KeepoutZoneResponse]:
    return [
        KeepoutZoneResponse(
            id=zone.id,
            points=[ZonePoint(x=x, y=y) for x, y in zone.points],
            name=zone.name,
            z_min=zone.z_min,
            z_max=zone.z_max,
        )
        for zone in zones
    ]


def _vertex_response(vertex: MapPoint) -> MapVertexResponse:
    return MapVertexResponse(
        id=vertex.id,
        name=vertex.name,
        type=vertex.type,
        map_name=vertex.map,
        x=vertex.x,
        y=vertex.y,
        theta=vertex.theta,
        z=vertex.z,
    )


def _octomap_status(
    stored: StoredMap, now: Optional[datetime] = None
) -> Tuple[OctomapStatus, Optional[str]]:
    """How the map's 3D map stands, off the tree on disk and the cleaning sidecar.

    Disk only: there is no registry, the tree being the robot's
    ``clean_map``'s to write. The check order is the logic:

    - A tree on disk is ``ok`` whatever the sidecar says. It is written by
      tmp + rename, so a present file is a complete one, and a re-clean in
      flight leaves the old tree in place until the new one lands.
    - No tree and no sidecar, or a clean that never asked for one, is
      ``none``: nothing was ever going to produce it.
    - A clean still ``converting`` is ``converting``, until ``started_at`` is
      MAP_CLEAN_STALE_AFTER_S ago (``MapCleanRecord.running``); then
      ``interrupted`` (a clean killed hard
      never writes its outcome, and nothing will clear it).
    - A clean that ended ``ok`` without writing the tree is ``failed`` with
      the sidecar's reason (out of memory, usually): map.pcd was cleaned, the
      tree was dropped. One that says it wrote a tree that is gone is
      ``none`` — a save into the same directory deletes it first.
    - A clean that ``failed`` is ``failed`` with the clean's reason: a tree
      only comes out of a clean that finished.
    """
    if stored.octomap is not None:
        return OctomapStatus.OK, None

    record = stored.map_clean_record
    if record is None or not record.octomap_requested:
        return OctomapStatus.NONE, None

    if record.status is MapCleanStatus.CONVERTING:
        if record.running(now):
            return OctomapStatus.CONVERTING, None
        return OctomapStatus.INTERRUPTED, None

    if record.status is MapCleanStatus.OK:
        if record.octomap_written:
            return OctomapStatus.NONE, None
        return (
            OctomapStatus.FAILED,
            record.octomap_error or "The map cleaning finished without building the 3D map.",
        )

    return (
        OctomapStatus.FAILED,
        record.error or "The map cleaning failed before the 3D map could be built.",
    )


def _summary(
    stored: StoredMap,
    active_name: Optional[str],
    vertex_count: int,
) -> MapSummaryResponse:
    octomap_status, octomap_error = _octomap_status(stored)
    tree = stored.octomap if octomap_status is OctomapStatus.OK else None

    return MapSummaryResponse(
        name=stored.name,
        active=stored.name == active_name,
        has_pointcloud=stored.has_pointcloud,
        size_bytes=stored.size_bytes,
        modified_at=stored.modified_at.isoformat().replace("+00:00", "Z"),
        vertex_count=vertex_count,
        octomap_status=octomap_status,
        octomap_error=octomap_error,
        octomap_version=tree.version if tree is not None else None,
        octomap_resolution=tree.resolution if tree is not None else None,
    )


def _not_modified(request: Request, tag: str) -> bool:
    """Report whether If-None-Match already names this content (for a 304)."""
    header = request.headers.get("if-none-match")
    return bool(header) and tag in [value.strip() for value in header.split(",")]


def _revalidate(request: Request, tag: str) -> Tuple[Dict[str, str], Optional[Response]]:
    """The validator headers for ``tag``, and the 304 to answer if the client
    already holds it (else None). Every ETag'd GET here revalidates the same
    way: ``no-cache`` so the browser asks every time, cheaply."""
    headers = {"ETag": tag, "Cache-Control": "no-cache"}
    if _not_modified(request, tag):
        return headers, Response(status_code=304, headers=headers)
    return headers, None


# What the render cache may hold, in bytes. A map costs up to ~4-5 MB (the
# packed cloud is capped at MAP_CLOUD_MAX_POINTS x 12 B), so this keeps a
# dozen-odd maps: more than one operator looks at,
# and a ceiling where there was none.
RENDER_CACHE_MAX_BYTES = 64 << 20


class _RenderCache:
    """Encoded renderings of each map, re-made only when their source changes,
    the least recently used dropped past ``max_bytes``.

    One entry per (kind, map); each remembers the source ``version`` it was
    made from, and ``get`` re-renders on any other version, so an entry is
    never served stale. It had no bound: every map ever opened kept its
    renderings until restart (``forget`` runs only on rename, delete and
    import), which read as a leak. The entry just stored is never evicted,
    even alone over budget, so a big map still renders once per version.

    The lock covers the bookkeeping only, never ``render`` (seconds for a
    cloud): two threads racing a miss both render, which is harmless.
    """

    def __init__(self, max_bytes: int = RENDER_CACHE_MAX_BYTES) -> None:
        self._max_bytes = max_bytes
        self._entries: "OrderedDict[Tuple[str, str], Tuple[Hashable, bytes]]" = OrderedDict()
        self._bytes = 0
        self._lock = threading.Lock()

    def get(
        self, kind: str, name: str, version: Hashable, render: Callable[[], bytes]
    ) -> bytes:
        key = (kind, name)
        with self._lock:
            cached = self._entries.get(key)
            if cached is not None and cached[0] == version:
                self._entries.move_to_end(key)
                return cached[1]

        data = render()

        with self._lock:
            self._drop(key)
            self._entries[key] = (version, data)
            self._bytes += len(data)
            while self._bytes > self._max_bytes and len(self._entries) > 1:
                oldest = next(iter(self._entries))
                self._drop(oldest)
        return data

    def forget(self, name: str) -> None:
        with self._lock:
            for key in [key for key in self._entries if key[1] == name]:
                self._drop(key)

    @property
    def size_bytes(self) -> int:
        return self._bytes

    def _drop(self, key: Tuple[str, str]) -> None:
        entry = self._entries.pop(key, None)
        if entry is not None:
            self._bytes -= len(entry[1])
