"""The step types removed from StepType, and what stored step lists do about them.

DOCK and UNDOCK were removed on 2026-11-12, with the robot stack's dock.xml /
undock.xml, its DockingController and its dock_to_pose server: a goal on
either tree is rejected before it runs, so neither step could do anything any
more. Unlike ARTIFACT (2026-08, "purge before deploying", by hand), the stored
ones are purged by the backend itself, because a step list that no longer
parses is not a quiet failure here:

- a template (``task_templates.steps``) is validated on every read, so one
  DOCK step turns ``GET /api/v1/task_templates`` into a 500 for every
  template, not only its own;
- a Temporal schedule freezes its ``WorkflowTask`` as the start-workflow
  argument, so every run it fires fails to decode its input, and Temporal
  retries a workflow task that fails that way for as long as the run lives.

The purge runs at startup, idempotently, in the two places that already
reconcile stored state with the code: ``init_task_template_repo`` for the
templates (as ``init_map_repo`` adds ``map_vertices.z``) and ``run_worker``
for the schedules (beside ``ensure_search_attributes``, before polling starts).
Both work on the raw JSON, never through ``Step``, since parsing the very
thing being purged is what fails.

Task history (``task_record.steps``) is left alone: it is read as raw dicts
(the CSV export writes ``type`` as a string), so a DOCK row exports as it
always did and is aged out with its partition.
"""

from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from pydantic import ValidationError

from syncai_backend.tasks.schema import Step, step_group_problem

# Kept as strings: the enum members are gone, which is the point.
RETIRED_STEP_TYPES = frozenset({"DOCK", "UNDOCK"})

# Named in every log line the purge writes, so a line found months later says
# which change it belongs to.
RETIRED_REASON = (
    "DOCK / UNDOCK steps were removed on 2026-11-12 with the robot stack's dock "
    "trees; a stored one would no longer parse"
)


def strip_retired_steps(
    entries: Sequence[Mapping[str, Any]],
) -> Tuple[List[Dict[str, Any]], List[str]]:
    """``entries`` without their retired steps, and the ids of those removed.

    Order and every key of the kept entries are untouched (a stored step may
    carry keys this module does not know: vertex ids, labels). An entry that is
    not a mapping is kept as it is; it is not this purge's to judge.
    """
    kept: List[Dict[str, Any]] = []
    removed: List[str] = []
    for entry in entries:
        if isinstance(entry, Mapping) and entry.get("type") in RETIRED_STEP_TYPES:
            removed.append(str(entry.get("id", "?")))
            continue
        kept.append(dict(entry) if isinstance(entry, Mapping) else entry)
    return kept, removed


def stored_steps_problem(entries: Sequence[Mapping[str, Any]]) -> Optional[str]:
    """Why a stripped step list still could not run, as a sentence; None if it can.

    Each entry as a run builds it (``resolve_stored_steps``' Step), then the
    group rules. Removing steps cannot break either rule on a list that kept
    them before -- a group only loses members, and two members of one group
    could not have had a step of another between them -- so this is the check
    that the list was sound to begin with, not a repair.
    """
    try:
        steps = [
            Step(
                id=entry["id"],
                type=entry["type"],
                params=entry.get("params"),
                group=entry.get("group"),
            )
            for entry in entries
        ]
    except (KeyError, TypeError, ValidationError) as err:
        return f"a remaining step does not parse: {err}"
    problem = step_group_problem(steps)
    return str(problem) if problem is not None else None
