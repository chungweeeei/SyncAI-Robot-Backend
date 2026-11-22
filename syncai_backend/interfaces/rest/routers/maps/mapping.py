"""The mapping run: start it, resume a saved one, reset it, read its state."""

import math

from fastapi import APIRouter

from syncai_backend.exceptions import (
    ConflictError,
    UpstreamError,
)
from syncai_backend.repositories.mapping.mapping import (
    MappingState,
)

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    MappingRunState,
    MappingStatusResponse,
    ResetMappingResponse,
    ResumeMappingRequest,
    ResumeMappingResponse,
    ResumeMethod,
    StartMappingResponse,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_gw = ctx.map_gw
    map_catalog_repo = ctx.map_catalog_repo
    mapping_status_repo = ctx.mapping_status_repo
    _require_exists = ctx.require_exists
    _run_state = ctx.run_state
    _refuse_while_resetting = ctx.refuse_while_resetting
    _refuse_while_idle = ctx.refuse_while_idle

    @map_router.post("/api/v1/mapping/reset", response_model=ResetMappingResponse)
    def reset_mapping_run():
        """Discard the run in the robot's memory and start a new map.

        Touches no file. No body and no "save first" flag: saving is
        POST /api/v1/maps, a separate act. 409 ``mapping_idle`` while pgo is
        IDLE (nothing to discard); other failures are 502 with pgo's sentence
        (typically: not in a mapping session).
        """
        _refuse_while_idle("discard")

        ok, detail = map_gw.reset_mapping()
        if not ok:
            logger.error("Failed to reset the mapping run", error=detail)
            raise UpstreamError(detail)

        return ResetMappingResponse(reset=True, message=detail)

    @map_router.post("/api/v1/mapping/start", response_model=StartMappingResponse)
    def start_mapping():
        """Begin a mapping run: ``pgo/start_mapping``, IDLE -> MAPPING.

        A mapping session starts idle, so the drive to the starting point is
        not mapped; a save closes the run. Always resets LIO (the map origin is
        the odometry origin), so the robot must stay **stationary** until the
        live scan returns.

        409 ``mapping_running`` while a run is on, 409 ``mapping_busy`` while a
        start or reset is in flight, else 502 on pgo's refusal. The status slot
        is left to the latched topic so every console agrees.
        """
        if _run_state() is MappingState.MAPPING:
            raise ConflictError(
                "The robot is already building a map. Save it, or start a new "
                "one with a reset.",
                code="mapping_running",
            )
        _refuse_while_resetting()

        ok, detail = map_gw.start_mapping()
        if not ok:
            logger.error("Failed to start the mapping run", error=detail)
            raise UpstreamError(detail)

        return StartMappingResponse(started=True, message=detail)

    @map_router.post("/api/v1/mapping/resume", response_model=ResumeMappingResponse)
    def resume_mapping(request: ResumeMappingRequest):
        """Begin a mapping run on top of a saved map: ``pgo/resume_mapping``,
        IDLE -> MAPPING with that map loaded.

        For a map that is missing a room: pgo loads its keyframes, finds the
        robot in it and maps on in the same frame, so what was drawn on the
        map stays where it was. The robot must stand **still, inside the
        saved map**, until this returns, which takes tens of seconds (the
        load, a LIO reset, a second of scans, the search).

        The saved map is only read, whichever it is: the loaded one (the usual
        case — navigate there, switch to mapping, resume) or one still being
        cleaned, since the clean leaves ``poses.txt`` and ``patches/`` alone.
        The run is then saved with POST /api/v1/maps under **another** name;
        pgo refuses a save into the map it resumed.

        404 for an unknown map, 409 ``mapping_running`` / ``mapping_busy`` as
        for a start, else 502 with pgo's sentence (the robot was not found,
        the place looks like another one, the map was saved without its
        keyframes, not in a mapping session). After a failure pgo is idle.
        """
        _require_exists(request.name)

        if _run_state() is MappingState.MAPPING:
            raise ConflictError(
                "The robot is already building a map. Save it before resuming "
                "another one.",
                code="mapping_running",
            )
        _refuse_while_resetting()

        hint = None
        if request.hint is not None:
            # REST speaks degrees.
            hint = (
                request.hint.x,
                request.hint.y,
                request.hint.z,
                math.radians(request.hint.theta),
            )

        ok, detail, resumed = map_gw.resume_mapping(
            map_catalog_repo.resolve_dir(request.name), hint
        )
        if not ok:
            logger.error("Failed to resume the map", map=request.name, error=detail)
            raise UpstreamError(detail)

        return ResumeMappingResponse(
            resumed=True,
            message=detail,
            name=request.name,
            prior_key_poses=resumed.prior_key_poses,
            method=ResumeMethod(resumed.method),
            x=resumed.x,
            y=resumed.y,
            z=resumed.z,
            theta=math.degrees(resumed.yaw),
        )

    @map_router.get("/api/v1/mapping", response_model=MappingStatusResponse)
    async def get_mapping_status():
        """pgo's run state, as it last reported it.

        ``async``: it only reads a slot, no I/O. ``unknown`` is a normal answer
        (every navigating robot, or pgo quiet past the repo's TTL).
        """
        status = mapping_status_repo.get()
        if status is None:
            return MappingStatusResponse(
                state=MappingRunState.UNKNOWN, key_poses=0, loop_closures=0
            )
        return MappingStatusResponse(
            state=MappingRunState(status.state.value),
            key_poses=status.key_poses,
            loop_closures=status.loop_closures,
        )
