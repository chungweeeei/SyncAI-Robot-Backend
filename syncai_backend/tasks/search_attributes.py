"""The custom search attributes this backend stamps on every run it starts.

``TaskKind`` (see schema.py) and ``TaskName`` (the template) exist for
``GET /api/v1/task_history`` and ``/stats``, which filter and count on them; the
memo cannot be queried.

``TaskMap`` is the map a run's MOVE coordinates are in. An attribute rather than memo
because a scheduled run inherits only its action's search attributes. Read by the
active-task list (which map a job holds, for the map routes' refusal) and by
RobotActivities.execute_move (checked against the loaded map).

``RobotId`` is on the *schedule*, not its runs: the namespace is shared by the fleet,
and ``GET /api/v1/schedules`` asks the server for this robot's schedules (plus those
from before the attribute, which carry none) instead of pulling every robot's and
filtering on the memo here.

Keyword, not Text: SQL visibility allows ``=`` / ``IN`` / ``STARTS_WITH`` only on
Keyword. On the pinned server (auto-setup 1.29.7, Postgres) these take four of the
``Keyword01..10`` columns.

auto-setup registers custom attributes only on a fresh install, with no knob for
extra names, so the worker registers them at connect, idempotently; the README gives
the CLI equivalent.
"""

from typing import Dict

import structlog
from temporalio.api.enums.v1 import IndexedValueType
from temporalio.api.operatorservice.v1 import (
    AddSearchAttributesRequest,
    ListSearchAttributesRequest,
)
from temporalio.client import Client
from temporalio.common import SearchAttributeKey

TASK_KIND_KEY = SearchAttributeKey.for_keyword("TaskKind")
TASK_NAME_KEY = SearchAttributeKey.for_keyword("TaskName")
TASK_MAP_KEY = SearchAttributeKey.for_keyword("TaskMap")
ROBOT_ID_KEY = SearchAttributeKey.for_keyword("RobotId")

# Name -> type, the shape AddSearchAttributesRequest takes.
CUSTOM_SEARCH_ATTRIBUTES: Dict[str, "IndexedValueType.ValueType"] = {
    TASK_KIND_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
    TASK_NAME_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
    TASK_MAP_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
    ROBOT_ID_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
}


async def ensure_search_attributes(
    client: Client, logger: structlog.stdlib.BoundLogger
) -> bool:
    """Register the attributes on the client's namespace if they are missing.

    Lists first and adds only what is absent (AddSearchAttributes answers
    ALREADY_EXISTS). Never raises: without them everything else still works, and
    the kind/name filters and /stats answer 502. True when known to be registered.
    """
    try:
        listed = await client.operator_service.list_search_attributes(
            ListSearchAttributesRequest(namespace=client.namespace)
        )
        missing = {
            name: value_type
            for name, value_type in CUSTOM_SEARCH_ATTRIBUTES.items()
            if name not in listed.custom_attributes
        }
        if missing:
            await client.operator_service.add_search_attributes(
                AddSearchAttributesRequest(
                    namespace=client.namespace, search_attributes=missing
                )
            )
            logger.info(
                "Registered search attributes",
                component="Temporal",
                names=sorted(missing),
            )
        return True
    except Exception as err:
        logger.error(
            "Could not register search attributes; the task-history kind/name "
            "filters and /stats will fail until they exist",
            component="Temporal",
            names=sorted(CUSTOM_SEARCH_ATTRIBUTES),
            error=str(err),
        )
        return False
