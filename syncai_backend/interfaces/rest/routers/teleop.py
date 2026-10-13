import asyncio
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.gateways.robot.robot import RobotGateway


# Stale-input watchdog: five missed 10 Hz ticks. It lives here because the
# driver has none (a quiet cmd_vel never becomes a stop), so whoever opens a
# command channel owns stopping it.
_WATCHDOG_S = 0.5

# The driver->controller hop is lossy UDP, so the final zero is repeated.
_STOP_REPEATS = 3


def init_teleop_router(
    logger: structlog.stdlib.BoundLogger, robot_gw: RobotGateway
) -> APIRouter:
    teleop_router = APIRouter(prefix="", tags=["Teleop"])

    @teleop_router.websocket("/api/v1/robot/teleop")
    async def teleop(ws: WebSocket):
        """Inbound velocity-command channel for the console's manual control.

        Wire format, client -> server, JSON text frames at ~10 Hz:

            {"vx": .., "vy": .., "wz": ..}

        each axis clamped gateway-side to [-1, 1], in m/s and rad/s (REP-103
        body frame: +vx forward, +vy left, +wz CCW).

        Server -> client frames are only refusals:

            {"error": "autonomous move in progress"}   (teleop refused; the
                socket stays open — cancel the task, then drive)
            {"error": "malformed frame..."}            (that frame skipped)

        Publishes on the event loop: Publisher.publish() does not block.
        """
        await ws.accept()
        try:
            while True:
                try:
                    # Cancelling the receive on timeout is safe: uvicorn's
                    # queue read is cancellation-atomic.
                    msg = await asyncio.wait_for(
                        ws.receive_json(), timeout=_WATCHDOG_S
                    )
                except asyncio.TimeoutError:
                    # Quiet client: keep the robot stopped.
                    robot_gw.teleop_stop()
                    continue
                except ValueError:
                    await ws.send_json({"error": "malformed frame; expected JSON"})
                    continue

                try:
                    vx = float(msg["vx"])
                    vy = float(msg["vy"])
                    wz = float(msg["wz"])
                except (KeyError, TypeError, ValueError):
                    await ws.send_json(
                        {"error": "malformed frame; expected {vx, vy, wz} floats"}
                    )
                    continue

                ok, message = robot_gw.teleop_cmd_vel(vx=vx, vy=vy, wz=wz)
                if not ok:
                    await ws.send_json({"error": message})
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("teleop stream failed", exc_info=True)
            return
        finally:
            # The real stop-on-disconnect, on every exit path; the client's
            # zero-then-close is only best effort.
            for _ in range(_STOP_REPEATS):
                robot_gw.teleop_stop()

    return teleop_router
