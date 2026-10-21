import asyncio
import structlog

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from syncai_backend.gateways.robot.robot import RobotGateway


# Stale-input watchdog: five missed 10 Hz ticks. It lives here because the
# driver has none (a quiet cmd_vel never becomes a stop), so whoever opens a
# command channel owns stopping it.
_WATCHDOG_S = 0.5

# The driver->controller hop is lossy UDP, so every stop is repeated: since a
# stop is no longer re-sent each watchdog tick, one lost zero would otherwise
# leave the robot holding its last velocity.
_STOP_REPEATS = 3


def init_teleop_router(
    logger: structlog.stdlib.BoundLogger, robot_gw: RobotGateway
) -> APIRouter:
    teleop_router = APIRouter(prefix="", tags=["Teleop"])

    @teleop_router.websocket("/api/v1/robot/teleop")
    async def teleop(ws: WebSocket):
        """Inbound velocity-command channel for the console's manual control.

        Wire format, client -> server, JSON text frames at ~10 Hz while the
        operator deflects an input, then one zero on release:

            {"vx": .., "vy": .., "wz": ..}

        each axis clamped gateway-side to [-1, 1], in m/s and rad/s (REP-103
        body frame: +vx forward, +vy left, +wz CCW).

        Every publish is a cmd_vel on the robot, so an open socket with no
        input publishes nothing: a stop (a zero frame, the watchdog, the
        disconnect) goes out only when this socket last set the robot moving,
        and a zero while already stopped is skipped.

        Server -> client frames are only refusals:

            {"error": "autonomous move in progress"}   (teleop refused; the
                socket stays open — cancel the task, then drive)
            {"error": "malformed frame..."}            (that frame skipped)

        Publishes on the event loop: Publisher.publish() does not block.
        """
        await ws.accept()
        # Whether this socket's last published command moved the robot: the
        # one fact that decides whether a stop has anything to stop.
        driving = False

        def stop() -> None:
            nonlocal driving
            if not driving:
                return
            for _ in range(_STOP_REPEATS):
                robot_gw.teleop_stop()
            driving = False

        try:
            while True:
                try:
                    # Cancelling the receive on timeout is safe: uvicorn's
                    # queue read is cancellation-atomic.
                    msg = await asyncio.wait_for(
                        ws.receive_json(), timeout=_WATCHDOG_S
                    )
                except asyncio.TimeoutError:
                    # Quiet client: stop the robot once if it was moving.
                    stop()
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

                if vx == 0.0 and vy == 0.0 and wz == 0.0:
                    # The client's release: a stop, repeated like every
                    # other, and nothing at all when already stopped.
                    stop()
                    continue

                ok, message = robot_gw.teleop_cmd_vel(vx=vx, vy=vy, wz=wz)
                if ok:
                    driving = True
                else:
                    await ws.send_json({"error": message})
        except WebSocketDisconnect:
            return
        except Exception:
            logger.error("teleop stream failed", exc_info=True)
            return
        finally:
            # The real stop-on-disconnect, on every exit path; the client's
            # zero-then-close is only best effort.
            stop()

    return teleop_router
