"""A map's vertices (waypoints): list, create, read, update, delete."""

import uuid
from typing import List, Optional

from fastapi import APIRouter, Body

from syncai_backend.exceptions import (
    NotFoundError,
)
from syncai_backend.database.models import MapPoint

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    DeleteResponse,
    MapVertexRequest,
    MapVertexResponse,
    MapVertexUpdateRequest,
    VertexType,
)
from syncai_backend.interfaces.rest.routers.maps.common import (
    _vertex_response,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    map_repo = ctx.map_repo
    _require = ctx.require
    job_lock = ctx.job_lock

    @map_router.get("/api/v1/maps/{name}/vertices", response_model=List[MapVertexResponse])
    def list_map_vertices(name: str, type: Optional[VertexType] = None):
        # _require first, so an unknown map is a 404 rather than [].
        _require(name)
        vertices = map_repo.list_vertices(map=name, type=type.value if type else None)
        return [_vertex_response(vertex) for vertex in vertices]

    @map_router.post(
        "/api/v1/maps/{name}/vertices",
        response_model=List[MapVertexResponse],
        dependencies=job_lock,
    )
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
                    "z": req.z,
                }
                for req in reqs
            ],
        )
        return [_vertex_response(vertex) for vertex in vertices]

    def _require_vertex(name: str, vertex_id: uuid.UUID) -> MapPoint:
        """Fetch a vertex, but only as a member of ``name``.

        The id alone finds the row; the URL's map is checked so no vertex is
        reachable through another map's URL. A row on another map is a 404.
        """
        _require(name)
        vertex = map_repo.get_vertex(vertex_id=vertex_id)
        if vertex is None or vertex.map != name:
            raise NotFoundError(f"Map vertex {vertex_id} was not found in '{name}'.")
        return vertex

    @map_router.get("/api/v1/maps/{name}/vertices/{id}", response_model=MapVertexResponse)
    def get_map_vertex(name: str, id: uuid.UUID):
        return _vertex_response(_require_vertex(name, id))

    @map_router.put(
        "/api/v1/maps/{name}/vertices/{id}",
        response_model=MapVertexResponse,
        dependencies=job_lock,
    )
    def update_map_vertex(name: str, id: uuid.UUID, req: MapVertexUpdateRequest):
        _require_vertex(name, id)

        changes = req.model_dump(exclude_unset=True)
        if "type" in changes and changes["type"] is not None:
            changes["type"] = changes["type"].value

        vertex = map_repo.update_vertex(id, **changes)
        if vertex is None:
            raise NotFoundError(f"Map vertex {id} was not found in '{name}'.")
        return _vertex_response(vertex)

    @map_router.delete(
        "/api/v1/maps/{name}/vertices/{id}",
        response_model=DeleteResponse,
        dependencies=job_lock,
    )
    def delete_map_vertex(name: str, id: uuid.UUID):
        _require_vertex(name, id)
        map_repo.delete_vertex(vertex_id=id)
        return DeleteResponse(message=f"Map vertex {id} has been deleted.")
