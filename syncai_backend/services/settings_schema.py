"""What a settings page is made of: groups of fields, each writing one or more
ROS parameters of one node, and the rules that tie them together.

The nodes declare their parameters with no descriptor -- no description, no
range -- so ``DescribeParameters`` answers a name and a type and nothing an
operator could act on. A ``SettingsSchema`` per node (``controller_schema``,
``planner_schema``) is where the console's form comes from instead.

A field is not one ROS parameter: ``targets`` lists every parameter it writes,
because some are one calibration that must move together. The value read back
is the first target's.

``apply`` is what the node does with a live set, read off its C++ callbacks:

- ``live``: stored by a callback into the running node; in force at once.
- ``idle``: refused while the node is busy (controller_server's own
  parameters, mid-goal), so sent in a set of their own.
- ``restart``: read once at startup; a live set would be stored and used by
  nothing, or refused. Written to the overlay only, in force from the node's
  next start.

Ranges are what the console may write, not what the node accepts.
"""

from dataclasses import dataclass, field
from functools import cached_property
from typing import Callable, Optional, Union

Value = Union[float, int, bool]

APPLY_LIVE = "live"
APPLY_IDLE = "idle"
APPLY_RESTART = "restart"

KIND_NUMBER = "number"
KIND_INTEGER = "integer"
KIND_BOOLEAN = "boolean"


@dataclass(frozen=True)
class Group:
    id: str
    label: str
    description: str


@dataclass(frozen=True)
class SettingField:
    id: str
    group: str
    label: str
    description: str
    targets: tuple[str, ...]
    apply: str
    kind: str = KIND_NUMBER
    unit: Optional[str] = None
    min: Optional[float] = None
    max: Optional[float] = None
    step: Optional[float] = None

    @property
    def source(self) -> str:
        """The parameter the field's value is read from."""
        return self.targets[0]


# ValueOf answers a ROS parameter's value as it will be after the change being
# checked: the new value for a target in the request, else the live one, else
# the overlay's, else the factory file's. None when nothing knows it.
ValueOf = Callable[[str], Optional[Value]]


@dataclass(frozen=True)
class Rule:
    """A cross-parameter constraint, checked on every change.

    ``check`` returns the operator's sentence when the combination is refused,
    None when it is fine (or when a value it needs is unknown -- refusing a
    change for want of a number nobody can supply would lock the page).
    ``fields`` are the settings the sentence is about, for the console to
    mark.
    """

    fields: tuple[str, ...]
    check: Callable[[ValueOf], Optional[str]] = field(compare=False)


@dataclass(frozen=True)
class SettingsSchema:
    groups: tuple[Group, ...]
    fields: tuple[SettingField, ...]
    rules: tuple[Rule, ...] = ()

    @cached_property
    def by_id(self) -> dict[str, SettingField]:
        return {f.id: f for f in self.fields}


def below(
    upper: str, lower: str, sentence: str
) -> Callable[[ValueOf], Optional[str]]:
    """A rule that ``lower`` stays strictly under ``upper`` (ROS names)."""

    def check(value_of: ValueOf) -> Optional[str]:
        hi, lo = value_of(upper), value_of(lower)
        if hi is None or lo is None or lo < hi:
            return None
        return sentence.format(hi=hi, lo=lo)

    return check


def coerce(setting: SettingField, value: object) -> Value:
    """``value`` as the field's type, within its limits; ValueError otherwise.

    The type matters past validation: the nodes declare their parameters
    without dynamic typing, so a double parameter set with an int (JSON's
    ``1`` for ``1.0``) is refused by rclcpp itself.
    """
    if setting.kind == KIND_BOOLEAN:
        if not isinstance(value, bool):
            raise ValueError(f"{setting.label} must be true or false")
        return value

    # bool is an int subclass; true for a number is a client bug, not 1.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{setting.label} must be a number")
    if setting.kind == KIND_INTEGER:
        if float(value) != int(value):
            raise ValueError(f"{setting.label} must be a whole number")
        number: Value = int(value)
    else:
        number = float(value)
        if number != number or number in (float("inf"), float("-inf")):
            raise ValueError(f"{setting.label} must be a finite number")

    unit = f" {setting.unit}" if setting.unit else ""
    if setting.min is not None and number < setting.min:
        raise ValueError(f"{setting.label} must be at least {setting.min}{unit}")
    if setting.max is not None and number > setting.max:
        raise ValueError(f"{setting.label} must be at most {setting.max}{unit}")
    return number
