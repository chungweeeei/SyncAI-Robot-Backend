import os
import tempfile
import time

import structlog
from fastapi import APIRouter
from fastapi.responses import StreamingResponse

from syncai_backend.gateways.logs.logs import LogBundleGateway


# Streaming granularity, as for a map export.
_SPOOL_CHUNK = 1 << 20

_TAR_RESPONSE = {
    200: {
        "content": {"application/x-tar": {}},
        "description": "The bundle; its layout is in the route's description.",
    }
}


def init_logs_router(
    logger: structlog.stdlib.BoundLogger,
    log_bundle_gw: LogBundleGateway,
) -> APIRouter:
    logs_router = APIRouter(prefix="", tags=["Logs"])

    @logs_router.get(
        "/api/v1/logs/export", response_class=StreamingResponse, responses=_TAR_RESPONSE
    )
    def export_logs():
        """The last 24 h of the robot's logs as one uncompressed tar.

        Under ``<robot_id>-logs-<UTC stamp>/``:

        - ``stack/<pane>/``: every byobu pane's multilog files (``current`` and
          the gzipped ``@<tai64n>.s`` rotations) written to in the window,
          each whole, so the oldest reaches somewhat past 24 h;
        - ``journal.log``: the host's systemd journal since then
          (``journalctl -o short-iso-precise``), kernel included;
        - ``manifest.json``: what was collected, and why a half is missing.

        Either half failing (not mounted, no permission, journalctl absent or
        wedged) leaves it out and says so in the manifest; the rest is still
        returned. Plain ``def``: walks the log tree and runs journalctl.
        """
        # Built into an anonymous file on the gateway's scratch disk and
        # streamed from it, as a map export is: a day of logs is far too big
        # to hold in the process that supervises the running job.
        started = time.perf_counter()
        spool = tempfile.TemporaryFile(dir=log_bundle_gw.scratch_dir)
        try:
            manifest = log_bundle_gw.write_bundle(spool)
            size = spool.seek(0, os.SEEK_END)
            spool.seek(0)
        except BaseException:
            spool.close()
            raise
        logger.info(
            "Exported logs",
            bytes=size,
            journal=manifest["journal"]["included"],
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
            media_type="application/x-tar",
            headers={
                # robot_id and a UTC stamp: nothing in it needs quoting.
                "Content-Disposition": f'attachment; filename="{manifest["name"]}.tar"',
                "Content-Length": str(size),
                "Cache-Control": "no-store",
            },
        )

    return logs_router
