"""Whether a map may be changed, destroyed, switched to or driven on right now.

Every refusal of that kind, in one place. Two classes, split by which thread
may ask:

- ``MapUsagePolicy`` answers from disk and this process's registries alone
  (the INI's active map, the conversion registry, the template table). Plain
  calls, safe from any thread: the map routes ask it, and so does
  ``RobotActivities.execute_move`` on the worker's executor.
- ``JobGate`` answers "is a job running", which means asking Temporal, so it
  holds the ``WorkflowGateway`` and is awaited on the uvicorn loop only (that
  gateway belongs to the loop; see CLAUDE.md). The worker never gets one.

All of these are check-then-act, not locks: the operation's own precondition
(pgo's, map_server's, ``execute_move``'s check before every goal) is what
stops a request that slipped through the window. When a question cannot be
answered -- Temporal down -- the answer is a refusal, never "idle".

Codes and sentences are operator prose, shown verbatim by the console.
"""

from typing import List, Optional

import structlog

from syncai_backend.exceptions import ConflictError
from syncai_backend.gateways.workflow.schema import ActiveTask
from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.helpers.move_guard import MoveRefusal, move_refusal
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.task.task_template import TaskTemplateRepo
from syncai_backend.services.gridmap_conversion import GridmapConversionService


class MapUsagePolicy:
    def __init__(
        self,
        map_catalog_repo: MapCatalogRepo,
        conversion_svc: GridmapConversionService,
        task_template_repo: TaskTemplateRepo,
    ):
        self._catalog = map_catalog_repo
        self._conversion_svc = conversion_svc
        self._task_template_repo = task_template_repo

    def refuse_while_converting(self, name: str, advice: str) -> None:
        """409 ``conversion_running`` while ``name``'s grid is being rebuilt.

        ``advice`` finishes the sentence after the semicolon, saying when to
        retry and, where it matters, why (the conversion would overwrite it).
        """
        if self._conversion_svc.is_converting(name):
            raise ConflictError(
                f"A gridmap conversion for '{name}' is running; {advice}",
                code="conversion_running",
            )

    def refuse_if_active(self, name: str, verb: str) -> None:
        """409 ``map_active``: map_server and the localizer hold ``map/<name>/``."""
        if name == self._catalog.active_name():
            raise ConflictError(
                f"'{name}' is the map the stack is running on and cannot be "
                f"{verb} while it is in use. Switch the robot to another map "
                "first.",
                code="map_active",
            )

    def refuse_destroy(self, name: str, *, verb: str, template_consequence: str) -> None:
        """The three 409s that guard destroying a map's contents, in order.

        Shared by delete and by an import that replaces a map:

        - ``map_active``: map_server and the localizer hold ``map/<name>/``.
        - ``conversion_running``: the thread would write its sidecar into a
          directory that is gone (or into the *new* one).
        - ``template_bound``: a template's MOVE steps name this map's vertex ids,
          which would dangle. The operator is told which templates to unbind;
          they are never deleted silently. Map-independent templates are
          ignored.
        """
        self.refuse_if_active(name, verb)
        self.refuse_while_converting(
            name, "try again once the conversion has finished."
        )

        bound = self._task_template_repo.list_task_templates(
            map_name=name, include_map_independent=False
        )
        if bound:
            names = ", ".join(sorted(f"'{row.name}'" for row in bound))
            raise ConflictError(
                f"{len(bound)} task "
                f"{'template' if len(bound) == 1 else 'templates'} still "
                f"{'targets' if len(bound) == 1 else 'target'} '{name}' "
                f"({names}). Point {'it' if len(bound) == 1 else 'them'} at "
                "another map or delete "
                f"{'it' if len(bound) == 1 else 'them'} first — "
                f"{template_consequence}",
                code="template_bound",
            )

    def move_refusal(
        self, expected_map: Optional[str], active_map: Optional[str]
    ) -> Optional[MoveRefusal]:
        """Why a job planned on ``expected_map`` must not drive on
        ``active_map`` now, or None; the rule itself is helpers/move_guard.py.

        ``active_map`` is the caller's one read of the INI, so a dispatch
        stamps the very map it was checked against.
        """
        return move_refusal(
            expected_map,
            active_map,
            active_map is not None and self._conversion_svc.is_converting(active_map),
        )


class JobGate:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        map_catalog_repo: MapCatalogRepo,
        workflow_gw: WorkflowGateway,
    ):
        self._logger = logger
        self._catalog = map_catalog_repo
        self._workflow_gw = workflow_gw

    async def running_tasks(self, change: str) -> List[ActiveTask]:
        """What is running now, or 409 ``tasks_unknown`` if that cannot be known.

        Refuse rather than assume idle: the gateway caches failures too, so an
        error here is a real outage. ``change`` names what was refused.
        """
        try:
            tasks, _ = await self._workflow_gw.list_active_tasks()
        except Exception as exc:
            self._logger.warning(
                "Could not confirm the robot is idle before changing a map",
                change=change,
                error=str(exc),
            )
            raise ConflictError(
                "Could not confirm that no job is running, and "
                f"{change} under a moving robot is not safe to guess at. Check "
                "the Tasks page and try again.",
                code="tasks_unknown",
            )
        return tasks

    async def refuse_while_a_job_holds(self, name: str) -> None:
        """409 ``task_running`` while a running job's positions are on ``name``.

        Guards every route that edits what the planner, costmaps or a job's
        waypoints read (vertices, grid, rebuild, keepout).

        A map other than the loaded one passes without asking Temporal: a run
        can only hold the loaded map, and editing an idle map should not need
        Temporal up. execute_move's own check stops a run that slipped through
        the window.
        """
        active = self._catalog.active_name()
        if name != active:
            return
        tasks = await self.running_tasks("changing the map in use")
        if any(task.map_in_use(active) == name for task in tasks):
            raise ConflictError(
                "The robot is running a job on this map. Cancel it or wait for "
                "it to finish before changing the map.",
                code="task_running",
            )

    async def refuse_while_any_job(self, change: str, before: str) -> None:
        """409 ``task_running`` while *any* job runs, whatever map it holds.

        Stricter than ``refuse_while_a_job_holds``, for the operations that
        move the map frame under the robot (a switch, a kidnap search).
        ``before`` ends the sentence: "Cancel or wait for it before <before>."
        """
        active_tasks = await self.running_tasks(change)
        if active_tasks:
            running = ", ".join(task.id for task in active_tasks)
            raise ConflictError(
                f"The robot is running {len(active_tasks)} "
                f"{'task' if len(active_tasks) == 1 else 'tasks'} ({running}). "
                f"Cancel or wait for it before {before}.",
                code="task_running",
            )


def init_map_usage_policy(
    map_catalog_repo: MapCatalogRepo,
    conversion_svc: GridmapConversionService,
    task_template_repo: TaskTemplateRepo,
) -> MapUsagePolicy:
    return MapUsagePolicy(
        map_catalog_repo=map_catalog_repo,
        conversion_svc=conversion_svc,
        task_template_repo=task_template_repo,
    )


def init_job_gate(
    logger: structlog.stdlib.BoundLogger,
    map_catalog_repo: MapCatalogRepo,
    workflow_gw: WorkflowGateway,
) -> JobGate:
    return JobGate(logger=logger, map_catalog_repo=map_catalog_repo, workflow_gw=workflow_gw)
