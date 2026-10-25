"""The mapping run: start it, reset it, read its state."""


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
    StartMappingResponse,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    logger = ctx.logger
    map_gw = ctx.map_gw
    mapping_status_repo = ctx.mapping_status_repo
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
