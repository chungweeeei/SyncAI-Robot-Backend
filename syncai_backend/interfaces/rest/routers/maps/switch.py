"""The loaded map: switch the robot onto another, and find it on this one."""

import asyncio
import math
import threading

from fastapi import APIRouter

from syncai_backend.exceptions import (
    ConflictError,
    UpstreamError,
)

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    ActivateMapResponse,
    AutoLocalizeResponse,
    GridStatus,
)
from syncai_backend.interfaces.rest.routers.maps.common import (
    _grid_status,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_gw = ctx.map_gw
    map_switch_svc = ctx.map_switch_svc
    job_gate = ctx.job_gate
    _require = ctx.require

    @map_router.post(
        "/api/v1/maps/{name}/activate", response_model=ActivateMapResponse
    )
    async def activate_map(name: str):
        """Switch the robot onto another map, live, without restarting the stack.

        Four parts must agree: the localizer's cloud (`relocalize`),
        map_server's grid (`load_map`), the 3D planner's tree
        (`planner_server/Octo3D/load_map`) and `[map] name` in the INI (what
        a restart reads; `active_name()` reports it once written). The
        localizer goes first because a refusal there mutates nothing; each
        later step compensates the earlier ones on failure. The INI is last and preflighted for
        writability, since an unrecorded swap would leave REST naming the wrong
        map.

        A map is 2D (a usable gridmap) or 3D (an ``octomap.bt``; 2026-10). A 3D
        map is loaded into Octo3D, which is what makes the robot run its 3D
        trees, and needs a stack that has it (409 ``planner_3d_unavailable``);
        a 2D map *unloads* Octo3D when the stack has one, or the robot would
        keep planning on the previous map's octree. A 3D map's forbidden zones
        come in with its tree (Octo3D reads keepout.json beside octomap.bt), and
        map_server keeps the previous grid when the 3D map has none (it cannot
        unload; nothing plans on it while Octo3D has a map).

        All refusals come first. `stack_not_ready` (e.g. mapping mode) is
        detected by service discoverability, not the cached RobotState, which
        is absent on a robot that lost localization: the localizer always,
        map_server for a 2D map only (a 3D session runs without it; a 3D map's
        grid is then not loaded).

        `[initial_pose]` is zeroed: an old-frame pose is meaningless here. The
        operator sets the pose; `localized` says whether registration converged
        shortly after the swap. The steps and their undos are
        services/map_switch.py; this route owns only the Temporal question.
        """
        def _plan():
            stored = _require(name)
            status, grid_error = _grid_status(stored, False)
            return map_switch_svc.plan(stored, status == GridStatus.OK, grid_error)

        # Off the event loop, like the switch below: reading the map walks every
        # file under it (hundreds under patches/, the files least likely to be
        # in the page cache) and plan() reads the INI, and this route is async.
        # On the loop, a slow eMMC (a bag recording, a conversion writing) froze
        # teleop and the telemetry streams for as long as that took.
        plan = await asyncio.to_thread(_plan)
        if plan is None:
            return ActivateMapResponse(
                name=name,
                previous=name,
                switched=False,
                localized=None,
                message=f"'{name}' is already the map the robot is running on.",
            )

        # Any job, not only one holding a map: a switch re-points the localizer.
        await job_gate.refuse_while_any_job("a map switch", "switching maps")

        # The route is `async def` for the Temporal await (the client belongs to
        # the uvicorn loop); the switch blocks for up to a minute, so it runs in
        # a worker thread. Renderings are keyed by map name: nothing cached is
        # stale.
        outcome = await asyncio.to_thread(map_switch_svc.execute, plan)
        return ActivateMapResponse(
            name=outcome.name,
            map_kind=outcome.map_kind,
            previous=outcome.previous,
            switched=True,
            localized=outcome.localized,
            keepout_reloaded=outcome.keepout_reloaded,
            message=outcome.message,
        )

    # --- Localization -------------------------------------------------------

    # Held across one auto_localize call: a second press would queue behind the
    # first in the localizer and time out or search again. It is a 409 instead.
    auto_localize_lock = threading.Lock()

    @map_router.post(
        "/api/v1/localization/auto", response_model=AutoLocalizeResponse
    )
    async def auto_localize():
        """Find the robot on the loaded map with no hint — the recovery for a
        kidnapped robot. The localizer's ``reset_odometry`` with ``skip_seed``:
        Point-LIO is reset first (a carry often diverges it), then the
        hint-free search runs on the new run's scans. The robot must be
        standing still; ``odom → base_link`` jumps back to the origin once.

        No body. Blocks up to ~30 s; a 200 means localized, nothing to poll.
        Writes nothing (not the INI's ``[initial_pose]``).

        409 ``task_running`` while any job runs (it moves the map frame under
        the robot), 409 ``tasks_unknown``, 409 ``localize_running``. Every
        localizer refusal is the uniform 502 with its sentence. ``async`` for
        the Temporal await, blocking half on a worker thread, as activate.
        """
        await job_gate.refuse_while_any_job("finding the robot", "finding the robot")

        if not auto_localize_lock.acquire(blocking=False):
            raise ConflictError(
                "The robot is already searching for where it stands; wait for "
                "that answer.",
                code="localize_running",
            )

        def _localize() -> AutoLocalizeResponse:
            try:
                ok, detail, found = map_gw.auto_localize()
            finally:
                auto_localize_lock.release()

            if not ok:
                logger.error("Auto-localization failed", error=detail)
                raise UpstreamError(detail)

            return AutoLocalizeResponse(
                localized=True,
                message=detail,
                x=found.x,
                y=found.y,
                z=found.z,
                # REST speaks degrees.
                theta=math.degrees(found.yaw),
                fitness=found.fitness,
                inlier_ratio=found.inlier_ratio,
                candidates=found.candidates,
            )

        # No await between the acquire and this submit, so _localize always
        # runs and releases the lock even if the client goes away.
        return await asyncio.to_thread(_localize)
