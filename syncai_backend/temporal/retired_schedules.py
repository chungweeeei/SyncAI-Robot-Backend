"""Strip DOCK / UNDOCK steps out of this robot's Temporal schedules at startup.

A schedule's ``WorkflowTask`` is frozen into its start-workflow action when it is
created, and ``create_schedule`` is the only writer. Re-reading the template
when a run starts (RobotWorkflow._resolve_template) does not help: the run
first has to decode its own input, the frozen ``WorkflowTask``, and with
``StepType.DOCK`` gone that decode fails before any workflow code runs --
Temporal then retries the failing workflow task for as long as the run lives,
and SKIP overlap drops every later trigger meanwhile. So the frozen copy itself
is rewritten (tasks/retired_steps.py has the whole story):

- **steps left:** the action's argument is replaced with the stripped
  ``WorkflowTask``, validated first; spec, policy, state and memo go back as
  described, so the trigger, a pause and SKIP's record of a running run all
  survive (the same describe-then-update ``update_schedule_trigger`` uses);
- **nothing left, or what is left does not validate:** the schedule is deleted.
  A template-backed schedule whose template was DOCK-only lands here, since its
  snapshot is the template as it was registered; a template-backed one whose
  template was edited since keeps the stripped snapshot, which is also what it
  ran before whenever its template could not be read.

Called from ``run_worker`` before either worker polls, so no run fired from a
stale schedule is picked up first. Never fatal, like ``ensure_search_attributes``
beside it: Temporal unreachable here means the worker is about to fail anyway,
and one bad schedule must not keep the rest from being cleaned. Idempotent: a
cleaned schedule has nothing left to strip.

The raw JSON is read off the payload rather than decoded into ``WorkflowTask``,
because that decode is exactly what fails.
"""

import json
from typing import Any, Dict, List, Optional

import structlog
from pydantic import ValidationError
from temporalio.api.common.v1 import Payload
from temporalio.client import (
    Client,
    ScheduleActionStartWorkflow,
    ScheduleUpdate,
    ScheduleUpdateInput,
)

from syncai_backend.tasks.retired_steps import RETIRED_REASON, strip_retired_steps
from syncai_backend.tasks.schema import WorkflowTask

# The pydantic converter's JSON encoding; anything else is not ours to read.
_JSON_ENCODING = b"json/plain"


def _raw_task(arg: Any) -> Optional[Dict[str, Any]]:
    """The frozen WorkflowTask as a plain dict, or None when it is not JSON."""
    if not isinstance(arg, Payload):
        return None
    if arg.metadata.get("encoding") != _JSON_ENCODING:
        return None
    try:
        raw = json.loads(arg.data)
    except ValueError:
        return None
    return raw if isinstance(raw, dict) else None


async def purge_retired_schedule_steps(
    client: Client, logger: structlog.stdlib.BoundLogger, task_queue: str
) -> Dict[str, List[str]]:
    """Strip DOCK / UNDOCK out of every schedule dispatching onto ``task_queue``.

    ``task_queue`` is the ownership test (the namespace is shared between
    robots; the same fact WorkflowGateway._require_owned_schedule checks).
    Returns the ids it updated and deleted, which it also logs.
    """
    summary: Dict[str, List[str]] = {"updated": [], "deleted": []}
    try:
        items = [item async for item in await client.list_schedules()]
    except Exception as err:
        logger.warning(
            "[RetiredSchedules] Could not list schedules; any holding a DOCK / UNDOCK "
            "step will fail to start its runs until the next startup",
            error=str(err),
        )
        return summary

    for item in items:
        try:
            await _purge_one(client, logger, task_queue, item.id, summary)
        except Exception as err:
            logger.warning(
                "[RetiredSchedules] Could not check a schedule for DOCK / UNDOCK steps; "
                "left as it is until the next startup",
                schedule_id=item.id,
                error=str(err),
            )

    if summary["updated"] or summary["deleted"]:
        logger.warning(
            "[RetiredSchedules] Purged DOCK / UNDOCK steps from schedules",
            updated=summary["updated"],
            deleted=summary["deleted"],
            reason=RETIRED_REASON,
        )
    return summary


async def _purge_one(
    client: Client,
    logger: structlog.stdlib.BoundLogger,
    task_queue: str,
    schedule_id: str,
    summary: Dict[str, List[str]],
) -> None:
    handle = client.get_schedule_handle(schedule_id)
    desc = await handle.describe()
    action = desc.schedule.action
    if not isinstance(action, ScheduleActionStartWorkflow) or action.task_queue != task_queue:
        return
    if not action.args:
        return
    raw = _raw_task(action.args[0])
    if raw is None:
        return
    definition = raw.get("definition")
    steps = definition.get("steps") if isinstance(definition, dict) else None
    if not isinstance(steps, list):
        return

    kept, removed = strip_retired_steps(steps)
    if not removed:
        return

    task: Optional[WorkflowTask] = None
    problem = "no step left"
    if kept:
        definition["steps"] = kept
        try:
            task = WorkflowTask.model_validate(raw)
        except ValidationError as err:
            problem = f"the steps left do not validate: {err}"

    if task is None:
        await handle.delete()
        summary["deleted"].append(schedule_id)
        logger.warning(
            "[RetiredSchedules] Deleted a schedule: removing its DOCK / UNDOCK steps "
            "left nothing it could run",
            schedule_id=schedule_id,
            removed_step_ids=removed,
            problem=problem,
            reason=RETIRED_REASON,
        )
        return

    def _rewrite(inp: ScheduleUpdateInput) -> ScheduleUpdate:
        schedule = inp.description.schedule
        # A Python object, not a Payload: the SDK encodes it with the client's
        # (pydantic) converter, as at create, and passes the other args through.
        schedule.action.args = [task]
        return ScheduleUpdate(schedule=schedule)

    await handle.update(_rewrite)
    summary["updated"].append(schedule_id)
    logger.warning(
        "[RetiredSchedules] Removed DOCK / UNDOCK steps from a schedule",
        schedule_id=schedule_id,
        removed_step_ids=removed,
        steps_left=len(kept),
        reason=RETIRED_REASON,
    )
