import struct
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo


def init_pointcloud_router(
    logger: structlog.stdlib.BoundLogger,
    pointcloud_repo: PointCloudRepo,
    map_cloud_repo: PointCloudRepo,
) -> APIRouter:
    pointcloud_router = APIRouter(prefix="", tags=["PointCloud"])

    async def _pump(ws: WebSocket, repo: PointCloudRepo):
        """Drain one single-slot cloud repo onto one WebSocket, forever.

        Wire format per frame (both streams): a little-endian uint32 point
        count, then ``3 * count`` little-endian float32 xyz values.

        Frame-driven, not timer-driven: polling the slot added latency and
        dropped ~5% of frames to clock beat. See ``PointCloudRepo``. A slow
        client just skips superseded frames (single slot), so there is no rate
        cap and nothing queues.
        """
        await ws.accept()
        last_seq = 0
        try:
            # Subscribe before the first read, and clear() before get_latest:
            # clear-then-check can only cause a spurious wakeup, while
            # check-then-clear can lose a frame's wakeup.
            with repo.subscribe() as new_frame:
                while True:
                    new_frame.clear()
                    frame = repo.get_latest(after_seq=last_seq)
                    if frame is None:
                        await new_frame.wait()
                        continue
                    last_seq = frame.seq
                    header = struct.pack("<I", frame.num_points)
                    await ws.send_bytes(header + frame.data)
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("point cloud stream failed", exc_info=True)
            return

    @pointcloud_router.websocket("/api/v1/robot/pointcloud/stream")
    async def stream_pointcloud(ws: WebSocket):
        """Stream the latest map-frame body_cloud as binary frames (~10 Hz)."""
        await _pump(ws, pointcloud_repo)

    @pointcloud_router.websocket("/api/v1/robot/pointcloud/map/stream")
    async def stream_map_cloud(ws: WebSocket):
        """Stream pgo's merged "map so far" cloud as binary frames.

        Only while mapping (MANUAL), every few seconds at most; each frame
        replaces the last wholesale (loop-closure corrected).
        """
        await _pump(ws, map_cloud_repo)

    return pointcloud_router
