"""A multi-storey map's floors: detected from poses.txt, or saved."""

import math
from typing import List

from fastapi import APIRouter

from syncai_backend.exceptions import (
    BadRequestError,
)
from syncai_backend.helpers.floors import detect_floors, floor_name
from syncai_backend.repositories.map.catalog import (
    MapFloor,
)

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    MapFloorModel,
    MapFloorsResponse,
    SaveMapFloorsRequest,
    SaveMapFloorsResponse,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


# Floors a save may name, the longest name, and how close two levels may sit.
# The console draws one band per floor between neighbouring levels, so two
# levels closer than a body height would leave a band with nothing in it.
FLOORS_MAX = 16

FLOOR_NAME_MAX = 16

FLOOR_MIN_SEPARATION_M = 1.0


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    map_catalog_repo = ctx.map_catalog_repo
    _require_exists = ctx.require_exists

    def _floor_models(floors: List[MapFloor]) -> List[MapFloorModel]:
        return [
            MapFloorModel(name=floor.name, level=floor.level, low=floor.low, high=floor.high)
            for floor in floors
        ]

    def _detected_floors(name: str) -> List[MapFloor]:
        poses = map_catalog_repo.read_poses(name)
        found = detect_floors(poses) if poses else []
        # One storey needs no split.
        if len(found) < 2:
            return []
        return [
            MapFloor(name=floor_name(i), level=floor.level, low=floor.low, high=floor.high)
            for i, floor in enumerate(found)
        ]

    @map_router.get("/api/v1/maps/{name}/floors", response_model=MapFloorsResponse)
    def get_map_floors(name: str):
        """The floors of a map that spans storeys: saved, else detected.

        Detected on every read from poses.txt (a few thousand lines), so a
        map that has never been looked at needs no step to have floors.
        """
        _require_exists(name)
        detected = _detected_floors(name)
        saved = map_catalog_repo.read_floors(name)
        return MapFloorsResponse(
            name=name,
            floors=_floor_models(saved if saved is not None else detected),
            detected=_floor_models(detected),
            saved=saved is not None,
        )

    def _validate_floors(entries: List[MapFloorModel]) -> List[MapFloor]:
        """400 with a sentence for a list the console could not draw bands from."""
        if len(entries) > FLOORS_MAX:
            raise BadRequestError(f"Too many floors ({len(entries)}); at most {FLOORS_MAX}.")
        floors: List[MapFloor] = []
        names = set()
        for entry in entries:
            label = entry.name.strip()
            if not label:
                raise BadRequestError("Every floor needs a name.")
            if len(label) > FLOOR_NAME_MAX:
                raise BadRequestError(
                    f"Floor name '{label}' is too long; at most {FLOOR_NAME_MAX} characters."
                )
            if label.casefold() in names:
                raise BadRequestError(f"Two floors are called '{label}'.")
            if not math.isfinite(entry.level):
                raise BadRequestError(f"Floor '{label}' has no height.")
            names.add(label.casefold())
            # A walked range rides along when the console sends one back
            # (a detected list saved under new names); one that does not
            # hold the level is no range, and the floor keeps a fixed band.
            low, high = entry.low, entry.high
            if (
                low is None
                or high is None
                or not (math.isfinite(low) and math.isfinite(high))
                or not low <= entry.level <= high
            ):
                low = high = None
            floors.append(MapFloor(name=label, level=float(entry.level), low=low, high=high))
        floors.sort(key=lambda floor: floor.level)
        for lower, upper in zip(floors, floors[1:]):
            if upper.level - lower.level < FLOOR_MIN_SEPARATION_M:
                raise BadRequestError(
                    f"Floors '{lower.name}' and '{upper.name}' are less than "
                    f"{FLOOR_MIN_SEPARATION_M:g} m apart."
                )
        return floors

    @map_router.put("/api/v1/maps/{name}/floors", response_model=SaveMapFloorsResponse)
    def save_map_floors(name: str, request: SaveMapFloorsRequest):
        """Replace the map's floors with the body's whole list.

        Not held by a running job: the floors only decide what the console
        shows, and nothing on the robot reads them.
        """
        _require_exists(name)
        floors = map_catalog_repo.write_floors(name, _validate_floors(request.floors))
        count = len(floors)
        message = (
            f"Saved {count} floor{'' if count == 1 else 's'} for '{name}'."
            if count
            else f"Saved '{name}' as one floor."
        )
        return SaveMapFloorsResponse(name=name, floors=_floor_models(floors), message=message)
