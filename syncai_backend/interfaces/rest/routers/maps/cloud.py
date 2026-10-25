"""A stored map's point cloud, packed for the 3D viewer."""

import os
import struct

from fastapi import APIRouter
from fastapi.responses import Response

from syncai_backend.exceptions import (
    NotFoundError,
)
from syncai_backend.helpers.pointcloud import (
    cap_points,
    pack_xyz_f32,
    read_pcd_xyz,
    voxel_downsample,
)

from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


# Decimation for a stored map.pcd. Duplicates PointCloudSubscriber's numbers so
# the router does not depend on a subscriber; keep the two in step.
MAP_CLOUD_VOXEL_SIZE = 0.3

MAP_CLOUD_MAX_POINTS = 300000


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_catalog_repo = ctx.map_catalog_repo
    renders = ctx.renders
    _require = ctx.require

    @map_router.get("/api/v1/maps/{name}/pointcloud")
    def get_map_pointcloud_by_name(name: str):
        """The map's saved map.pcd, packed for the viewer.

        Same wire format and decimation as the live stream: LE uint32 count,
        then ``3 * count`` LE float32 xyz.

        Cached on (size, mtime), not a content hash: map.pcd is only ever
        replaced wholesale, never edited in place.
        """
        _require(name)
        path = map_catalog_repo.pointcloud_path(name)
        if path is None:
            raise NotFoundError(f"Map '{name}' has no map.pcd.")

        try:
            stats = os.stat(path)
        except OSError as exc:
            logger.error("Failed to stat map cloud", map=name, error=str(exc))
            raise NotFoundError(f"Map '{name}' has no readable map.pcd.")

        def _pack() -> bytes:
            try:
                points = read_pcd_xyz(path)
            except (OSError, ValueError) as exc:
                logger.error("Failed to read map cloud", map=name, error=str(exc))
                raise NotFoundError(f"Map '{name}' has no readable map.pcd.")

            points = voxel_downsample(points=points, voxel_size=MAP_CLOUD_VOXEL_SIZE)
            points = cap_points(points=points, max_points=MAP_CLOUD_MAX_POINTS)
            logger.info("packed stored map cloud", map=name, num_points=int(points.shape[0]))
            return struct.pack("<I", points.shape[0]) + pack_xyz_f32(points)

        payload = renders.get("cloud", name, (stats.st_size, stats.st_mtime_ns), _pack)
        return Response(
            content=payload,
            media_type="application/octet-stream",
            headers={"Cache-Control": "no-store"},
        )
