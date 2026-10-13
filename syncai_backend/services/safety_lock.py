"""Stop every task when the driver's safety lock engages.

While syncai_driver_manager's `SafetyLock` is engaged the robot cannot move,
so every task on this robot's queue is cancelled the moment it engages.

Two inputs feed one last-seen state: the operator's POST /api/v1/robot/estop
(`engage_requested`, cancels at once) and the driver's own trigger (low
battery, overheat), seen only via RobotState.low_level_mode.safety_state at
1 Hz (`observe`). Only the false -> true edge cancels. A first sample that is
already true counts as an edge: runs outlive a backend restart.

`observe` runs on the ROS executor and must never block or raise (that ends
spin()). The cancel runs on the uvicorn loop, which owns WorkflowGateway;
`bind_loop` hands that loop in at REST startup, and an earlier edge waits for it.
"""

import asyncio
import threading
from typing import Optional

import structlog


class SafetyLockService:
    """The lock's last observed state, and the cancel its rising edge starts.

    ``robot_gw`` needs ``cancel_active_moves()``; ``workflow_gw`` needs
    ``cancel_active_tasks()``.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger, robot_gw, workflow_gw):
        self._logger = logger
        self._robot_gw = robot_gw
        self._workflow_gw = workflow_gw
        self._lock = threading.Lock()
        self._last: Optional[bool] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # An edge seen before the loop was bound, run once it is.
        self._pending = False

    def observe(self, engaged: bool) -> None:
        """Feed one safety_state sample. Never blocks, never raises."""
        self._transition(engaged)

    def engage_requested(self) -> bool:
        """The operator engaged the lock (POST /api/v1/robot/estop).

        Cancels now and records the lock as engaged, so the next sample is not
        a second edge. Returns False when it was already engaged.
        """
        return self._transition(True)

    def _transition(self, engaged: bool) -> bool:
        with self._lock:
            rising = engaged and self._last is not True
            self._last = engaged
            if not rising:
                return False
            loop = self._loop
            if loop is None:
                self._pending = True

        self._logger.warning("Safety lock engaged; cancelling every task")
        if loop is None:
            self._logger.warning(
                "REST loop not up yet; the cancel runs once it is"
            )
            return True
        self._submit(loop)
        return True

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Hand in the uvicorn loop. Runs a cancel an earlier edge left pending."""
        with self._lock:
            self._loop = loop
            pending, self._pending = self._pending, False
        if pending:
            self._submit(loop)

    def _submit(self, loop: asyncio.AbstractEventLoop) -> None:
        try:
            asyncio.run_coroutine_threadsafe(self._stop_all(), loop)
        except RuntimeError as err:
            # The loop closed under us: the process is shutting down.
            self._logger.error("Could not schedule the safety cancel", error=str(err))

    async def _stop_all(self) -> None:
        # Nav first and directly: a Temporal cancel only reaches a MOVE on its
        # next heartbeat. In the executor, since it blocks on a ROS round trip.
        loop = asyncio.get_running_loop()
        try:
            ok, message = await loop.run_in_executor(
                None, self._robot_gw.cancel_active_moves
            )
            if not ok:
                self._logger.error("Failed to cancel nav goals", message=message)
        except Exception as err:
            self._logger.error("Failed to cancel nav goals", error=str(err))

        # Nobody awaits this coroutine's future, so nothing may escape it.
        try:
            cancelled = await self._workflow_gw.cancel_active_tasks()
        except Exception as err:
            self._logger.error(
                "Failed to cancel tasks on safety lock", error=str(err)
            )
            return
        self._logger.warning(
            "Cancelled tasks on safety lock", task_ids=cancelled
        )


def init_safety_lock_service(
    logger: structlog.stdlib.BoundLogger, robot_gw, workflow_gw
) -> SafetyLockService:
    return SafetyLockService(logger=logger, robot_gw=robot_gw, workflow_gw=workflow_gw)
