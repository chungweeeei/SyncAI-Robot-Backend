"""The custom search attributes this backend stamps on every run it starts.

Two Keyword attributes: ``TaskKind`` (how the run was started -- see
``TaskKind`` in schema.py) and ``TaskName`` (the template it came from). They
exist for one consumer, ``GET /api/v1/task_history`` and its ``/stats``
sibling, which filter and count on them -- the memo cannot be queried, and
the workflow id only carries the kind for runs the console itself minted.

Keyword rather than Text: the filters are exact matches, and SQL visibility
only allows ``=`` / ``IN`` / ``STARTS_WITH`` on Keyword columns (Text is
full-text, no equality). On the pinned server (auto-setup 1.29.7 on Postgres)
Keyword attributes map onto the pre-allocated ``Keyword01..10`` columns; the
auto-setup image's own test set takes one of them, these take two more.

Registration is a namespace-level operation the backend has to do itself:
``temporalio/auto-setup`` registers custom attributes only on a fresh
install, from a list that has no environment knob for extra names. So the
worker registers them at connect, idempotently, and the README gives the CLI
equivalent for a stack where the backend has no operator rights.
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

# Name -> type, the shape AddSearchAttributesRequest takes.
CUSTOM_SEARCH_ATTRIBUTES: Dict[str, "IndexedValueType.ValueType"] = {
    TASK_KIND_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
    TASK_NAME_KEY.name: IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
}


async def ensure_search_attributes(
    client: Client, logger: structlog.stdlib.BoundLogger
) -> bool:
    """Register the attributes on the client's namespace if they are missing.

    Idempotent: lists first and adds only what is absent, because
    AddSearchAttributes answers ALREADY_EXISTS for a known name. Never raises.
    A backend that cannot register them (no operator rights, an old server)
    still serves everything else; the kind/name filters and the stats endpoint
    then answer 502, and the gateway logs the INVALID_ARGUMENT that says why.
    Answers True when the attributes are known to be registered.
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
