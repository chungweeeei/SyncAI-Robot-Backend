"""A live map switch: the localizer, Octo3D and the INI, in step.

POST /api/v1/maps/{name}/activate is the only caller. The switch is a small
saga -- three parts that must agree, each later failure compensating the
earlier ones -- and lives here rather than in the router so the steps and
their undos read as one list and can be driven without HTTP.

Two halves, because the router has to await Temporal between them (the
"is a job running" question belongs to the uvicorn loop):

- ``plan`` is every refusal that reads disk alone, in the order the operator
  should hear them. It mutates nothing.
- ``execute`` blocks for up to a minute on ROS services, so the router runs
  it in a worker thread. On the loop it froze every WebSocket, including the
  teleop watchdog.

Order and why:

1. The localizer (`relocalize`) first, because a refusal there mutates
   nothing.
2. Octo3D (`planner_server/Octo3D/load_map`), the map every goal plans on.
   Its tree takes seconds to build, so it goes after the cheap refusal.
3. `[map] name` in the INI last, preflighted for writability in ``plan``,
   since an unrecorded swap would leave REST naming the wrong map.

Compensations run in the order the steps were done, not reversed, and each
returns the sentence the operator reads; they are run as statements before
the message is built, so the order is the one done, not the one the
sentences would imply.

The stack navigates on 3D maps only (2026-10-28): a map without
``octomap.bt`` is refused, and there is no map_server grid or keepout mask to
move along (both went with the 2D stack). A map's forbidden zones come in
with its tree: Octo3D reads ``keepout.json`` beside it as part of the load.
"""

import os
from dataclasses import dataclass
from typing import Callable, List, Optional

import structlog

from syncai_backend.exceptions import ConflictError, UpstreamError
from syncai_backend.gateways.map.map import MapGateway
from syncai_backend.helpers.system_config import system_ini_path
from syncai_backend.repositories.map.catalog import MapCatalogRepo


@dataclass(frozen=True)
class SwitchPlan:
    """What ``plan`` judged; everything ``execute`` needs, read once."""

    name: str
    previous: Optional[str]
    pcd_path: str
    octomap_path: str
    ini_path: str


@dataclass(frozen=True)
class SwitchOutcome:
    name: str
    previous: Optional[str]
    localized: Optional[bool]
    message: str


# A compensation: given why the switch failed, put one part back and return
# the operator's sentence about it ("" when there is nothing to say).
_Undo = Callable[[str], str]


class MapSwitchService:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        map_catalog_repo: MapCatalogRepo,
        map_gw: MapGateway,
    ):
        self._logger = logger
        self._catalog = map_catalog_repo
        self._map_gw = map_gw

    # --- Preflight ---------------------------------------------------------

    def plan(self, name: str) -> Optional[SwitchPlan]:
        """Every refusal that needs no ROS call, or None for the no-op."""
        previous = self._catalog.active_name()
        if name == previous:
            # The no-op: reloading would only throw away a good registration.
            return None

        octomap_path = self._catalog.octomap_path(name)
        if octomap_path is None:
            # Refused up front, before the localizer is swapped: Octo3D is
            # all the robot plans on, and it would have nothing to load.
            raise ConflictError(
                f"'{name}' has no 3D map (octomap.bt), so the robot could not "
                "plan on it. Wait for the robot's map cleaning to build it, or "
                "save the map again, before switching to it.",
                code="octomap_missing",
            )

        pcd_path = self._catalog.pointcloud_path(name)
        if pcd_path is None:
            raise ConflictError(
                f"'{name}' has no map.pcd, so the localizer has nothing to "
                "match against. Only a map saved from a mapping run can be "
                "switched to.",
                code="pointcloud_missing",
            )

        ini_path = system_ini_path()
        if not os.access(ini_path, os.W_OK):
            # Preflighted so the swap's last step cannot fail after the move.
            raise ConflictError(
                f"The system INI ({ini_path}) is not writable, so a switch "
                "could not be recorded and would not survive a restart.",
                code="ini_not_writable",
            )

        return SwitchPlan(
            name=name,
            previous=previous,
            pcd_path=pcd_path,
            octomap_path=octomap_path,
            ini_path=ini_path,
        )

    # --- The switch --------------------------------------------------------

    def execute(self, plan: SwitchPlan) -> SwitchOutcome:
        """Run the switch; blocking. Raises ConflictError before anything
        moves, UpstreamError (with what was put back) after."""
        name, previous = plan.name, plan.previous
        map_gw = self._map_gw

        if not map_gw.localizer_ready():
            raise ConflictError(
                "The localizer is not reachable, so there is no running map to "
                "switch. The robot is in mapping mode, or the nav stack is down.",
                code="stack_not_ready",
            )

        # Asked before anything moves. Absent on a stack built without
        # OctomapPlanner3D, which then has nothing to navigate a map with.
        if not map_gw.planner_3d_available():
            raise ConflictError(
                f"The nav stack has no 3D planner (planner_server/Octo3D/load_map) "
                f"to navigate '{name}' with.",
                code="planner_3d_unavailable",
            )

        # What has been done, as the undo for each step, in the order done.
        done: List[_Undo] = []

        swapped, detail = map_gw.swap_localizer_map(plan.pcd_path, 0.0, 0.0, 0.0)
        if not swapped:
            # Nothing has changed yet, so nothing to compensate.
            self._logger.error("Map switch failed at the localizer", map=name, error=detail)
            raise UpstreamError(
                f"Could not point the localizer at '{name}': {detail}. The robot "
                f"is still on '{previous}'."
            )
        done.append(self._localizer_undo(plan))

        # Registered before the call: Octo3D may have dropped the previous
        # tree on its way to failing.
        done.append(self._octomap_undo(plan))
        octomap_loaded, detail = map_gw.reload_octomap(plan.octomap_path)
        if not octomap_loaded:
            self._logger.error("Map switch failed at the 3D planner", map=name, error=detail)
            raise UpstreamError(
                f"The localizer moved to '{name}' but the 3D planner would not "
                f"load its 3D map: {detail}." + _undo(done, "Octo3D load_map failed")
            )

        try:
            # The INI and the catalogue's cache of it, in that order.
            self._catalog.set_active(name)
        except (OSError, ValueError) as exc:
            self._logger.error(
                "Map switch failed at the INI write", map=name, error=str(exc)
            )
            raise UpstreamError(
                f"The robot moved to '{name}' but the switch could not be "
                f"recorded in {plan.ini_path}: {exc}." + _undo(done, "INI write failed")
            )

        # The switch is recorded; nothing can fail it now. The map's
        # forbidden zones need no carrying: the 3D planner read its
        # keepout.json (if any) as part of the load_map that just succeeded.
        count = len(self._catalog.read_keepout_zones(name))
        localized = map_gw.localization_converged()

        self._logger.info(
            "Switched the active map",
            map=name,
            previous=previous,
            localized=localized,
            keepout_zones=count,
        )

        message = f"The robot is now on '{name}'"
        message += f" (was '{previous}')." if previous else "."
        if localized:
            message += " The localizer converged on the new map."
        else:
            message += (
                " The pose was reset to the map origin; set an initial pose on "
                "the dashboard, because the localizer has not converged"
            )
            message += " yet." if localized is False else " and could not be asked."
        if count:
            message += f" Its {count} forbidden zone{'s' if count != 1 else ''} are active."
        return SwitchOutcome(
            name=name,
            previous=previous,
            localized=localized,
            message=message,
        )

    # --- Compensations -----------------------------------------------------

    def _localizer_undo(self, plan: SwitchPlan) -> _Undo:
        name, previous = plan.name, plan.previous
        previous_pcd = self._catalog.pointcloud_path(previous) if previous else None

        def _restore(reason: str) -> str:
            """Put the localizer back on the old cloud."""
            if previous_pcd is None:
                # The INI named no map (the likely case when its write fails),
                # or that map lost its map.pcd.
                where = (
                    f"'{previous}' has no map.pcd"
                    if previous
                    else "the INI named no previous map"
                )
                return (
                    f" The localizer is now on '{name}' and could not be put "
                    f"back — {where}."
                )
            restored, detail = self._map_gw.swap_localizer_map(previous_pcd, 0.0, 0.0, 0.0)
            if restored:
                return f" The localizer was put back on '{previous}'."
            self._logger.error(
                "Could not restore the localizer after a failed map switch",
                map=name,
                previous=previous,
                reason=reason,
                error=detail,
            )
            return (
                f" The localizer is still on '{name}' and could not be put back "
                f"({detail}) — set the map by hand and restart the stack."
            )

        return _restore

    def _octomap_undo(self, plan: SwitchPlan) -> _Undo:
        name, previous = plan.name, plan.previous

        def _restore(reason: str) -> str:
            """Put Octo3D back as the old map had it: its tree, or none."""
            previous_bt = self._catalog.octomap_path(previous) if previous else None
            back, back_detail = self._map_gw.reload_octomap(previous_bt or "")
            if back:
                return ""
            self._logger.error(
                "Could not restore the 3D planner after a failed map switch",
                map=name,
                previous=previous,
                reason=reason,
                error=back_detail,
            )
            return (
                f" The 3D planner could not be put back on '{previous}' "
                f"({back_detail}) — restart the nav stack before navigating."
            )

        return _restore


def _undo(done: List[_Undo], reason: str) -> str:
    """Run every compensation in the order its step was done; their sentences."""
    return "".join([undo(reason) for undo in done])


def init_map_switch_service(
    logger: structlog.stdlib.BoundLogger,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
) -> MapSwitchService:
    return MapSwitchService(
        logger=logger,
        map_catalog_repo=map_catalog_repo,
        map_gw=map_gw,
    )
