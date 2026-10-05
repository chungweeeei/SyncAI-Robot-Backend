"""Tests for the idempotent search-attribute registration the worker runs.

The operator service is stubbed at the client: what is pinned is that only
the attributes the namespace lacks are added, that a namespace that has them
is left alone, and that nothing here can take the worker down -- a failure
is logged and answered False, because a backend that cannot register them
still has every other job to do.
"""

import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
import structlog

pytest.importorskip("temporalio")

from temporalio.api.enums.v1 import IndexedValueType  # noqa: E402

from syncai_backend.gateways.workflow.search_attributes import (  # noqa: E402
    CUSTOM_SEARCH_ATTRIBUTES,
    TASK_KIND_KEY,
    TASK_MAP_KEY,
    TASK_NAME_KEY,
    ensure_search_attributes,
)

logger = structlog.get_logger()


def _client(existing: dict) -> SimpleNamespace:
    operator = SimpleNamespace(
        list_search_attributes=AsyncMock(
            return_value=SimpleNamespace(custom_attributes=existing)
        ),
        add_search_attributes=AsyncMock(),
    )
    return SimpleNamespace(namespace="default", operator_service=operator)


def test_the_three_keys_are_keyword_attributes():
    # Keyword, because the filters are equality and SQL visibility has no
    # equality on Text.
    assert CUSTOM_SEARCH_ATTRIBUTES == {
        "TaskKind": IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
        "TaskName": IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
        "TaskMap": IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
    }
    assert (TASK_KIND_KEY.name, TASK_NAME_KEY.name, TASK_MAP_KEY.name) == (
        "TaskKind",
        "TaskName",
        "TaskMap",
    )


def test_adds_only_what_the_namespace_lacks():
    client = _client({"TaskKind": IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD})

    assert asyncio.run(ensure_search_attributes(client, logger)) is True

    request = client.operator_service.add_search_attributes.await_args.args[0]
    assert request.namespace == "default"
    assert dict(request.search_attributes) == {
        "TaskName": IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
        "TaskMap": IndexedValueType.INDEXED_VALUE_TYPE_KEYWORD,
    }


def test_leaves_a_namespace_that_has_them_alone():
    client = _client(dict(CUSTOM_SEARCH_ATTRIBUTES))

    assert asyncio.run(ensure_search_attributes(client, logger)) is True
    client.operator_service.add_search_attributes.assert_not_awaited()


def test_a_failure_is_answered_not_raised():
    client = _client({})
    client.operator_service.add_search_attributes.side_effect = RuntimeError(
        "permission denied"
    )

    assert asyncio.run(ensure_search_attributes(client, logger)) is False
