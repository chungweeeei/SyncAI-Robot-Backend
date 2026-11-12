"""Tests for the startup purge of DOCK / UNDOCK steps from Temporal schedules
(temporal/retired_schedules.py).

A fake client stands in for Temporal: what matters is what the purge reads off
a described schedule's frozen argument -- the raw JSON payload, since that is
what no longer decodes into a WorkflowTask -- and what it does about it.
"""

import asyncio
import json
from types import SimpleNamespace

import pytest

pytest.importorskip("temporalio")

from temporalio.api.common.v1 import Payload  # noqa: E402
from temporalio.client import ScheduleActionStartWorkflow  # noqa: E402

from syncai_backend.tasks.schema import WorkflowTask  # noqa: E402
from syncai_backend.temporal.retired_schedules import (  # noqa: E402
    purge_retired_schedule_steps,
)

QUEUE = "robot01.ROBOT_TASK_QUEUE"

MOVE = {"id": "1-move", "type": "MOVE", "params": {"x": 1.0, "y": 2.0, "theta": 90.0}}
DOCK = {"id": "2-dock", "type": "DOCK", "params": {"x": 3.0, "y": 1.0, "theta": 180.0}}
UNDOCK = {"id": "3-undock", "type": "UNDOCK"}


def _payload(steps, template_id=None):
    raw = {"id": "sched", "definition": {"steps": steps}, "taskTemplateId": template_id}
    return Payload(metadata={"encoding": b"json/plain"}, data=json.dumps(raw).encode())


class _Handle:
    def __init__(self, schedule_id, steps, task_queue=QUEUE):
        self.id = schedule_id
        self.action = ScheduleActionStartWorkflow(
            "RobotWorkflow", args=[_payload(steps)], id=schedule_id, task_queue=task_queue
        )
        self.deleted = False
        self.updates = 0

    async def describe(self):
        return SimpleNamespace(id=self.id, schedule=SimpleNamespace(action=self.action))

    async def update(self, change):
        update = change(SimpleNamespace(description=await self.describe()))
        self.action = update.schedule.action
        self.updates += 1

    async def delete(self):
        self.deleted = True


class _Client:
    def __init__(self, *handles):
        self.handles = {handle.id: handle for handle in handles}

    async def list_schedules(self):
        async def rows():
            for handle in list(self.handles.values()):
                if not handle.deleted:
                    yield SimpleNamespace(id=handle.id)

        return rows()

    def get_schedule_handle(self, schedule_id):
        return self.handles[schedule_id]


def _run(client, logger):
    return asyncio.run(purge_retired_schedule_steps(client, logger, QUEUE))


def test_a_mixed_schedule_keeps_its_other_steps(logger):
    handle = _Handle("mixed", [MOVE, DOCK, UNDOCK])

    summary = _run(_Client(handle), logger)

    assert summary == {"updated": ["mixed"], "deleted": []}
    # Rewritten as a WorkflowTask the SDK encodes, as create_schedule does.
    (task,) = handle.action.args
    assert isinstance(task, WorkflowTask)
    assert [step.id for step in task.definition.steps] == ["1-move"]
    assert not handle.deleted


def test_a_dock_only_schedule_is_deleted(logger):
    handle = _Handle("charge", [DOCK, UNDOCK])

    summary = _run(_Client(handle), logger)

    assert summary == {"updated": [], "deleted": ["charge"]}
    assert handle.deleted


def test_a_schedule_without_dock_steps_is_untouched(logger):
    handle = _Handle("plain", [MOVE])

    assert _run(_Client(handle), logger) == {"updated": [], "deleted": []}
    assert handle.updates == 0 and not handle.deleted


def test_another_robots_schedule_is_left_alone(logger):
    # The namespace is shared: only schedules on this robot's queue are ours.
    handle = _Handle("theirs", [DOCK], task_queue="robot02.ROBOT_TASK_QUEUE")

    assert _run(_Client(handle), logger) == {"updated": [], "deleted": []}
    assert not handle.deleted


def test_a_second_run_finds_nothing(logger):
    client = _Client(_Handle("mixed", [MOVE, DOCK]), _Handle("charge", [UNDOCK]))
    _run(client, logger)

    # The rewritten action now holds a WorkflowTask, as a described one would
    # hold its payload; give it back as one to read it the way Temporal would.
    mixed = client.handles["mixed"]
    mixed.action.args = [
        _payload([step.model_dump(mode="json", by_alias=True)
                  for step in mixed.action.args[0].definition.steps])
    ]

    assert _run(client, logger) == {"updated": [], "deleted": []}


def test_one_bad_schedule_does_not_stop_the_rest(logger):
    broken = _Handle("broken", [DOCK])

    async def refuse():
        raise RuntimeError("describe failed")

    broken.describe = refuse
    client = _Client(broken, _Handle("charge", [DOCK]))

    assert _run(client, logger) == {"updated": [], "deleted": ["charge"]}
