"""The steps an operator can build a job from, described for a client to draw.

``GET /api/v1/task_steps`` answers with this. What the robot can do is this
process's to say, so the console builds its add menus and each step's form
from it instead of keeping its own list of types, limits and words: a step
added here, or a limit moved in a params model, reaches the editor without a
console release.

Each step says whether it occupies the robot's body (the group rule,
BODY_STEP_TYPES), whether it is offered on the walk to a stop
(``while_walking``), and lists its parameters as one of four field kinds a client
knows how to draw: ``waypoint`` (a pose on the job's map, which on the wire
spreads into the params model's ``x`` / ``y`` / ``theta`` / ``z`` and a
template's ``vertex_id``), ``text``, ``integer`` and ``number``. The limits are
read off the params models' own constraints, so the two cannot drift; the
validators stay the gate, and a client's own check only spares the operator a
422.

The labels, hints and units are written for an operator on the console, as its
copy is: what the robot does, never a node or a vendor name.
"""

from typing import List, Literal, Optional, Union

from annotated_types import Ge, Gt, Le, Lt, MaxLen
from pydantic import BaseModel, Field

from syncai_backend.tasks.schema import (
    BODY_STEP_TYPES,
    InspectionId,
    SpeakParams,
    StepType,
    WaitParams,
)


class WaypointField(BaseModel):
    kind: Literal["waypoint"] = "waypoint"
    key: str
    label: str
    filter: Optional[Literal["charger"]] = Field(
        None, description="`charger`: only charging waypoints are offered."
    )


class TextField(BaseModel):
    kind: Literal["text"] = "text"
    key: str
    label: str
    max_length: int
    placeholder: Optional[str] = None


class IntegerField(BaseModel):
    kind: Literal["integer"] = "integer"
    key: str
    label: str
    min: int
    max: int
    default: int = Field(..., description="Sent only when the operator changes it.")
    unit: Optional[str] = None


class NumberField(BaseModel):
    kind: Literal["number"] = "number"
    key: str
    label: str
    min: float
    min_exclusive: bool = Field(False, description="True when `min` itself is refused.")
    max: float
    unit: Optional[str] = None
    placeholder: Optional[str] = None


# Demo branch only, for DEMO: one of a fixed set of values, drawn as a select.
class ChoiceOption(BaseModel):
    value: str
    label: str


class ChoiceField(BaseModel):
    kind: Literal["choice"] = "choice"
    key: str
    label: str
    options: List[ChoiceOption]


StepParamField = Union[WaypointField, TextField, IntegerField, NumberField, ChoiceField]


class StepSpec(BaseModel):
    type: StepType
    label: str = Field(..., description="What the console calls the step.")
    hint: str = Field(..., description="One line on what the robot does.")
    occupies_body: bool = Field(
        ...,
        description=(
            "Moves the robot's body. A group of steps that start together "
            "holds at most one."
        ),
    )
    while_walking: bool = Field(
        ...,
        description=(
            "Offered on the walk to a stop: started with the MOVE, in its "
            "group. Never a body step, and not WAIT either, which in a walk's "
            "group would only hold the next stop back until it ran out."
        ),
    )
    params: List[StepParamField]


class StepCatalogue(BaseModel):
    steps: List[StepSpec]


def _bounds(model: type[BaseModel], field: str) -> dict:
    """The field's constraints, by name, off its pydantic metadata."""
    found: dict = {}
    for item in model.model_fields[field].metadata:
        if isinstance(item, MaxLen):
            found["max_length"] = item.max_length
        elif isinstance(item, Ge):
            found["min"], found["min_exclusive"] = item.ge, False
        elif isinstance(item, Gt):
            found["min"], found["min_exclusive"] = item.gt, True
        elif isinstance(item, Le):
            found["max"] = item.le
        elif isinstance(item, Lt):
            found["max"] = item.lt
    return found


# What an operator means by doing something on the way. A WAIT in a MOVE's
# group is legal, and the group waits for both, so it reads as "walk for at
# least this long" — never what was meant, so it is not offered.
WHILE_WALKING_STEP_TYPES = frozenset({StepType.SPEAK})


# DEMO's options in operator words; the capture service gets the Chinese
# description (schema.INSPECTION_DESCRIPTIONS) instead.
INSPECTION_LABELS: dict[InspectionId, str] = {
    InspectionId.HYDRANT_OK: "Fire hydrant: OK",
    InspectionId.LIGHT_ON: "Light: on",
    InspectionId.LIGHT_OFF: "Light: off",
    InspectionId.DOOR_OPEN: "Door: open",
    InspectionId.DOOR_CLOSED: "Door: closed",
}


def _spec(step_type: StepType, label: str, hint: str, params: list) -> StepSpec:
    return StepSpec(
        type=step_type,
        label=label,
        hint=hint,
        occupies_body=step_type in BODY_STEP_TYPES,
        while_walking=step_type in WHILE_WALKING_STEP_TYPES,
        params=params,
    )


def step_catalogue() -> StepCatalogue:
    """Every step an operator authors, in the order the console offers them.

    UNDOCK is left out: a MOVE backs the robot off the dock it last docked on
    by itself (services/dock_state.py), so an operator never writes one, and
    the console no longer loads a template that has one.
    """
    text = _bounds(SpeakParams, "text")
    repeat = _bounds(SpeakParams, "repeat")
    wait = _bounds(WaitParams, "seconds")
    return StepCatalogue(
        steps=[
            _spec(
                StepType.MOVE,
                "Move",
                "Walk to a waypoint.",
                [WaypointField(key="target", label="Waypoint")],
            ),
            _spec(
                StepType.DOCK,
                "Dock",
                "Go to a waypoint, then walk straight onto the charger there and "
                "stay standing on it.",
                [WaypointField(key="target", label="Charger", filter="charger")],
            ),
            _spec(
                StepType.SPEAK,
                "Speak",
                "Say a line on the robot’s speaker.",
                # voice and speed are left to their defaults, as the console
                # has always sent them: a field here would freeze today's
                # default into every template saved with it.
                [
                    TextField(
                        key="text",
                        label="Say",
                        max_length=text["max_length"],
                        placeholder="Delivery arrived — please take your items.",
                    ),
                    IntegerField(
                        key="repeat",
                        label="Times",
                        min=repeat["min"],
                        max=repeat["max"],
                        default=SpeakParams.model_fields["repeat"].default,
                    ),
                ],
            ),
            _spec(
                StepType.WAIT,
                "Wait",
                "Stay put for a set time, then go on.",
                [
                    NumberField(
                        key="seconds",
                        label="Duration",
                        min=wait["min"],
                        min_exclusive=wait["min_exclusive"],
                        max=wait["max"],
                        unit="seconds",
                        placeholder="10",
                    )
                ],
            ),
            _spec(StepType.STANDUP, "Stand", "Stand up. Nothing to set.", []),
            _spec(StepType.LIEDOWN, "Lie down", "Lie down. Nothing to set.", []),
            _spec(
                StepType.DEMO,
                "Inspection",
                "Report what the robot saw to the capture service.",
                [
                    ChoiceField(
                        key="inspection",
                        label="Result",
                        options=[
                            ChoiceOption(value=item.value, label=INSPECTION_LABELS[item])
                            for item in InspectionId
                        ],
                    )
                ],
            ),
        ]
    )
