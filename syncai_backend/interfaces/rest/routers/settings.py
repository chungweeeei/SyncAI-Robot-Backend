import structlog
from typing import Any, Dict, List, Optional, Union
from fastapi import APIRouter, Query
from pydantic import BaseModel, Field

from syncai_backend.services.node_settings import (
    NodeSettingsService,
    SettingsSnapshot,
)

SettingValue = Union[bool, float, int]


class SettingGroup(BaseModel):
    id: str = Field(..., description="The group's key; a field's `group` names it.")
    label: str = Field(..., description="The group's heading on the page.")
    description: str = Field(..., description="One sentence under the heading.")


class SettingField(BaseModel):
    id: str = Field(..., description="The field's key in a PUT body.")
    group: str = Field(..., description="The id of the group it is shown under.")
    label: str = Field(..., description="The field's label on the page.")
    description: str = Field(..., description="What the setting does, for the operator.")
    kind: str = Field(..., description="`number`, `integer` or `boolean`.")
    unit: Optional[str] = Field(None, description="The unit the value is in, if any.")
    min: Optional[float] = Field(None, description="The lowest value accepted.")
    max: Optional[float] = Field(None, description="The highest value accepted.")
    step: Optional[float] = Field(None, description="A sensible increment for an input.")
    apply: str = Field(
        ...,
        description=(
            "How a change takes effect: `live` at once, `idle` at once but only "
            "while the robot is not navigating (409 `controller_busy` otherwise), "
            "`restart` from the next start of the navigation stack."
        ),
    )
    value: Optional[SettingValue] = Field(
        None,
        description=(
            "The value to show: the one in force, or for a `restart` field the "
            "one the next start loads. Null when nothing knows it."
        ),
    )
    current: Optional[SettingValue] = Field(
        None, description="The running node's value; null while it is not running."
    )
    default: Optional[SettingValue] = Field(
        None, description="The robot's default (factory) value."
    )
    modified: bool = Field(
        ..., description="Whether the robot keeps a value of its own instead of the default."
    )
    pending_restart: bool = Field(
        ...,
        description="A `restart` field whose saved value is not yet in force.",
    )


class NodeSettingsResponse(BaseModel):
    live_available: bool = Field(
        ...,
        description=(
            "Whether the navigation stack is running. When false, values are "
            "the saved ones and a change is saved for its next start."
        ),
    )
    groups: List[SettingGroup]
    fields: List[SettingField]


class UpdateNodeSettingsRequest(BaseModel):
    # Any here, typed by the schema: a 400 sentence about the one value beats
    # a 422 listing the union's every branch.
    values: Dict[str, Any] = Field(
        ...,
        description="Field id -> new value; only the fields being changed.",
    )


def _response(snapshot: SettingsSnapshot) -> NodeSettingsResponse:
    return NodeSettingsResponse(
        live_available=snapshot.live_available,
        groups=[
            SettingGroup(id=g.id, label=g.label, description=g.description)
            for g in snapshot.groups
        ],
        fields=[
            SettingField(
                id=state.setting.id,
                group=state.setting.group,
                label=state.setting.label,
                description=state.setting.description,
                kind=state.setting.kind,
                unit=state.setting.unit,
                min=state.setting.min,
                max=state.setting.max,
                step=state.setting.step,
                apply=state.setting.apply,
                value=state.value,
                current=state.current,
                default=state.default,
                modified=state.persisted is not None,
                pending_restart=state.pending_restart,
            )
            for state in snapshot.fields
        ],
    )


def init_settings_router(
    logger: structlog.stdlib.BoundLogger,
    settings_svcs: Dict[str, NodeSettingsService],
) -> APIRouter:
    """``/api/v1/settings/<page>`` for each page in ``settings_svcs``
    (``controller``, ``planner``): one fixed path each rather than a path
    parameter, so an unknown page is FastAPI's own 404 and /docs lists them."""
    settings_router = APIRouter(prefix="", tags=["Settings"])

    # Plain `def`: each call waits on a node's parameter services (seconds
    # when the nav session is down) and reads files.
    for page, svc in settings_svcs.items():
        _register(settings_router, logger, page, svc)

    return settings_router


def _register(
    router: APIRouter,
    logger: structlog.stdlib.BoundLogger,
    page: str,
    svc: NodeSettingsService,
) -> None:
    path = f"/api/v1/settings/{page}"

    @router.get(path, response_model=NodeSettingsResponse, name=f"get_{page}_settings")
    def get_settings():
        return _response(svc.snapshot())

    @router.put(path, response_model=NodeSettingsResponse, name=f"update_{page}_settings")
    def update_settings(request: UpdateNodeSettingsRequest):
        logger.info("Changing settings", page=page, values=request.values)
        return _response(svc.apply(request.values))

    @router.delete(path, response_model=NodeSettingsResponse, name=f"reset_{page}_settings")
    def reset_settings(
        id: Optional[List[str]] = Query(
            None, description="The fields to reset; every field when absent."
        ),
    ):
        logger.info("Resetting settings", page=page, ids=id or "all")
        return _response(svc.reset(id))
