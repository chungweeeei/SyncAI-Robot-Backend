import asyncio
import contextlib
import threading
import structlog

from dataclasses import dataclass
from typing import Iterator, Optional


@dataclass(frozen=True)
class PointCloudFrame:
    """A single packed point-cloud frame ready to ship over the wire.

    ``data`` is little-endian float32 xyz triplets (3 * num_points values) in
    the map frame. ``seq`` lets the WebSocket loop skip frames already sent.
    """

    seq: int
    num_points: int
    data: bytes


class PointCloudRepo:
    """In-process single-slot cache holding only the latest point-cloud frame.

    Written from the ROS thread, read from asyncio, under a lock. A slow
    client drops stale frames instead of queuing them.

    Writers **wake** readers (``subscribe``). Do not go back to polling on a
    timer: it beat against the 10 Hz producer, adding ~50 ms latency and
    dropping ~5% of frames.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger

        self._lock = threading.Lock()
        self._seq = 0
        self._frame: Optional[PointCloudFrame] = None
        # (loop, event) per WS pump; a shared Event would let one reader
        # swallow another's wakeup.
        self._waiters: list[tuple[asyncio.AbstractEventLoop, asyncio.Event]] = []

    def update_frame(self, num_points: int, data: bytes):
        with self._lock:
            self._seq += 1
            self._frame = PointCloudFrame(
                seq=self._seq, num_points=num_points, data=data
            )
            waiters = list(self._waiters)

        # Outside the lock, so the asyncio readers never wait on the hand-off.
        for loop, event in waiters:
            try:
                loop.call_soon_threadsafe(event.set)
            except RuntimeError:
                # The reader's loop closed; its `subscribe` unregisters it.
                pass

    @contextlib.contextmanager
    def subscribe(self) -> Iterator[asyncio.Event]:
        """Register the calling coroutine's loop for frame notifications.

        Yields an Event that ``update_frame`` sets. Enter it from the loop that
        will await it; that loop is captured for the ROS-thread writer.
        """
        loop = asyncio.get_running_loop()
        entry = (loop, asyncio.Event())
        with self._lock:
            self._waiters.append(entry)
        try:
            yield entry[1]
        finally:
            with self._lock:
                # A double exit must not raise out of WebSocket teardown.
                if entry in self._waiters:
                    self._waiters.remove(entry)

    def get_latest(self, after_seq: int = 0) -> Optional[PointCloudFrame]:
        with self._lock:
            if self._frame is None:
                return None
            if self._frame.seq <= after_seq:
                return None
            return self._frame


def init_pointcloud_repo(logger: structlog.stdlib.BoundLogger) -> PointCloudRepo:
    return PointCloudRepo(logger)
