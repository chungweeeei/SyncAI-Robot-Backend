import structlog
import threading
from contextlib import contextmanager
from datetime import datetime, timezone
from typing import Iterator, List, Optional
from fastapi import APIRouter, Response
from pydantic import BaseModel, Field

from syncai_backend.exceptions import BadRequestError, ConflictError, UpstreamError
from syncai_backend.gateways.robot.robot import RobotGateway
from syncai_backend.helpers.single_flight import SingleFlight


class WifiNetwork(BaseModel):
    bssid: str = Field(..., description="The BSSID (access point MAC) of the network.")
    ssid: str = Field(..., description="The SSID of the network.")
    rssi: int = Field(..., description="The signal strength of the network, in dBm.")


class ScanWifiNetworksResponse(BaseModel):
    networks: List[WifiNetwork] = Field(
        ..., description="The WiFi networks visible to the robot."
    )


class ConnectWifiNetworkRequest(BaseModel):
    ssid: str = Field(
        ..., min_length=1, description="The SSID of the network to connect to."
    )
    password: str = Field(
        "", description="The network password; leave empty for open networks."
    )


class ConnectWifiNetworkResponse(BaseModel):
    message: str = Field(..., description="Human-readable result of the connection.")


class WifiConnection(BaseModel):
    uuid: str = Field(..., description="The saved network's id; the path key below.")
    name: str = Field(..., description="The saved network's name, usually its SSID.")
    ssid: str = Field(..., description="The SSID the robot joins with it.")
    active: bool = Field(..., description="Whether the robot is on this network now.")
    last_used_at: Optional[str] = Field(
        None,
        description="When the robot last joined it (ISO 8601, UTC), or null if never.",
    )


class ListWifiConnectionsResponse(BaseModel):
    connections: List[WifiConnection] = Field(
        ..., description="The WiFi networks saved on the robot."
    )


def _last_used_at(seconds: int) -> Optional[str]:
    # NetworkManager writes 0 for a profile that has never come up.
    if seconds <= 0:
        return None
    return (
        datetime.fromtimestamp(seconds, tz=timezone.utc)
        .isoformat()
        .replace("+00:00", "Z")
    )


def init_network_router(
    logger: structlog.stdlib.BoundLogger, robot_gw: RobotGateway
) -> APIRouter:
    network_router = APIRouter(prefix="", tags=["Network"])

    # Plain `def`: the gateway calls block for up to tens of seconds on ROS.

    # A scan is read-only and the same for every caller, so a press while one
    # runs (the console's button, pressed again after 20 s of nothing) joins
    # it instead of holding a second threadpool thread for ~40 s and queueing
    # a second scan behind the first in driver_manager.
    scans: SingleFlight = SingleFlight()

    # Connect, activate and delete each change NetworkManager's state and take
    # up to 70 s; two at once would race (connect to A while activating B), and
    # joining one to the other would answer a request that was never run. So
    # one at a time, and the rest are a 409 the operator can retry.
    changing = threading.Lock()

    @contextmanager
    def _one_change_at_a_time() -> Iterator[None]:
        if not changing.acquire(blocking=False):
            raise ConflictError(
                "The robot is already changing its WiFi connection; wait for "
                "that to finish.",
                code="wifi_busy",
            )
        try:
            yield
        finally:
            changing.release()

    @network_router.get(
        "/api/v1/network/wifi/scan", response_model=ScanWifiNetworksResponse
    )
    def scan_wifi_networks():
        success, message, networks = scans.run(robot_gw.scan_wifi_networks)
        if not success:
            logger.error("Failed to scan WiFi networks", message=message)
            raise UpstreamError(message)

        return ScanWifiNetworksResponse(
            networks=[
                WifiNetwork(bssid=network.bssid, ssid=network.ssid, rssi=network.rssi)
                for network in networks
            ]
        )

    @network_router.post(
        "/api/v1/network/wifi/connect", response_model=ConnectWifiNetworkResponse
    )
    def connect_wifi_network(request: ConnectWifiNetworkRequest):
        with _one_change_at_a_time():
            success, message = robot_gw.connect_wifi(
                ssid=request.ssid, password=request.password
            )
        if not success:
            logger.error(
                "Failed to connect to WiFi network", ssid=request.ssid, message=message
            )
            raise BadRequestError(message)

        return ConnectWifiNetworkResponse(
            message=f"Successfully connected to {request.ssid}"
        )

    @network_router.get(
        "/api/v1/network/wifi/connections", response_model=ListWifiConnectionsResponse
    )
    def list_wifi_connections():
        success, message, connections = robot_gw.list_wifi_connections()
        if not success:
            logger.error("Failed to list saved WiFi networks", message=message)
            raise UpstreamError(message)

        return ListWifiConnectionsResponse(
            connections=[
                WifiConnection(
                    uuid=connection.uuid,
                    name=connection.name,
                    ssid=connection.ssid,
                    active=connection.active,
                    last_used_at=_last_used_at(connection.last_used),
                )
                for connection in connections
            ]
        )

    @network_router.post(
        "/api/v1/network/wifi/connections/{uuid}/activate",
        response_model=ConnectWifiNetworkResponse,
    )
    def activate_wifi_connection(uuid: str):
        with _one_change_at_a_time():
            success, message, ssid = robot_gw.activate_wifi_connection(uuid=uuid)
        if not success:
            # 400 like a failed connect: nmcli's sentence (a changed password,
            # a network out of range) is the operator's to act on.
            logger.error(
                "Failed to activate saved WiFi network", uuid=uuid, message=message
            )
            raise BadRequestError(message)

        return ConnectWifiNetworkResponse(message=f"Successfully connected to {ssid}")

    @network_router.delete(
        "/api/v1/network/wifi/connections/{uuid}",
        status_code=204,
        response_class=Response,
    )
    def delete_wifi_connection(uuid: str):
        with _one_change_at_a_time():
            success, message = robot_gw.delete_wifi_connection(uuid=uuid)
        if not success:
            logger.error(
                "Failed to delete saved WiFi network", uuid=uuid, message=message
            )
            raise BadRequestError(message)

        return Response(status_code=204)

    return network_router
