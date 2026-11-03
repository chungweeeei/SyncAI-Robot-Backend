"""A ROS node's parameters, over its standard parameter services.

Every rclcpp node serves ``<node>/get_parameters`` and
``<node>/set_parameters_atomically`` (rcl_interfaces); this is a client of
those two for one node, by its name relative to this process's namespace
(``controller_server`` -> ``/<robot_id>/controller_server``). It holds no
handle that can move the robot.

Atomically, not ``set_parameters``: rclcpp hands the atomic call's whole list
to each on-set callback at once, so a callback that judges a pair
(DockingController's ``v_linear_min`` against ``v_linear_max``) sees both new
values, where ``set_parameters`` would try them one at a time and refuse the
first of a pair that is only valid together.

A refusal is not always a clean one. The callbacks of a node run in turn and
rclcpp stops at the first that refuses -- after the ones before it already
applied their share to their plugin. The controller's validating callbacks
(RPP 3D, Docking) refuse before touching anything; controller_server's own,
which refuses every undotted parameter while a goal runs, is registered first
and so runs last. The caller therefore sends undotted parameters in a set of
their own, and reads back after any refusal rather than assuming nothing moved.

Types are sent as given: controller_server declares without dynamic typing, so
a float parameter set with an int is refused by rclcpp. The schema's
``coerce`` is what makes a JSON ``1`` a ``1.0``.
"""

from typing import Optional, Union

import structlog
from rclpy.client import Client
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically

from syncai_backend.gateways import ros
from syncai_backend.gateways.failure import Failure, fail

Value = Union[float, int, bool, str]

# controller_server's refusal while a goal holds its lock. Matched on the
# reason string because SetParametersResult has no code; the sentence is
# controller_server.cpp's own (and upstream nav2's), unchanged since the port.
_BUSY_REASON = "currently running"

SERVICE_WAIT_S = 2.0
CALL_TIMEOUT_S = 5.0


def to_parameter_value(value: Value) -> ParameterValue:
    """``value`` as a ParameterValue of its own Python type."""
    # bool before int: bool is an int subclass.
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    if isinstance(value, str):
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    raise TypeError(f"unsupported parameter value {value!r}")


def from_parameter_value(value: ParameterValue) -> Optional[Value]:
    """A ParameterValue's Python value; None for NOT_SET (an undeclared name)
    and for the array types, which no setting uses."""
    return {
        ParameterType.PARAMETER_BOOL: lambda: value.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: value.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda: value.double_value,
        ParameterType.PARAMETER_STRING: lambda: value.string_value,
    }.get(value.type, lambda: None)()


class ParameterGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node, target: str):
        self._logger = logger
        self._node = node
        self._target = target
        self._get_client: Client = node.create_client(
            GetParameters, f"{target}/get_parameters"
        )
        self._set_client: Client = node.create_client(
            SetParametersAtomically, f"{target}/set_parameters_atomically"
        )

    @property
    def target(self) -> str:
        return self._target

    def available(self, timeout_sec: float = SERVICE_WAIT_S) -> bool:
        """Is the node up? Usually "is the nav session (AUTO) running"."""
        return self._get_client.wait_for_service(timeout_sec=timeout_sec)

    def get(self, names: list[str]) -> tuple[bool, str, dict[str, Optional[Value]]]:
        """The live values of ``names``; None for a name the node lacks."""
        if not self._get_client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return False, f"{self._target} is not running (the robot is not in AUTO mode)", {}

        response = ros.call_service(
            self._get_client, GetParameters.Request(names=list(names)), CALL_TIMEOUT_S
        )
        if response is None:
            return False, f"Timeout reading {self._target}'s parameters", {}
        if len(response.values) != len(names):
            # rclcpp answers an empty list when any name is malformed.
            return (
                False,
                f"{self._target} answered {len(response.values)} of {len(names)} parameters",
                {},
            )
        return True, "", {
            name: from_parameter_value(value) for name, value in zip(names, response.values)
        }

    def set_atomically(self, values: dict[str, Value]) -> tuple[bool, str]:
        """Set every one of ``values`` in one call; all or (per rclcpp) none.

        A refusal comes back tagged: CONTROLLER_BUSY for controller_server's
        mid-goal lock, PARAMETER_REFUSED for a callback's reason. Untagged
        failures are the service being gone or not answering.
        """
        if not values:
            return True, ""
        if not self._set_client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return False, f"{self._target} is not running (the robot is not in AUTO mode)"

        request = SetParametersAtomically.Request(
            parameters=[
                Parameter(name=name, value=to_parameter_value(value))
                for name, value in values.items()
            ]
        )
        response = ros.call_service(self._set_client, request, CALL_TIMEOUT_S)
        if response is None:
            return False, f"Timeout setting {self._target}'s parameters"

        result = response.result
        if result.successful:
            self._logger.info(
                "[ParameterGateway] Parameters set", node=self._target, values=values
            )
            return True, ""

        reason = result.reason or "no reason given"
        self._logger.warning(
            "[ParameterGateway] Parameters refused",
            node=self._target,
            values=values,
            reason=reason,
        )
        if _BUSY_REASON in reason:
            return False, fail(
                Failure.CONTROLLER_BUSY,
                "This setting can only be changed while the robot is not navigating.",
            )
        return False, fail(Failure.PARAMETER_REFUSED, f"The robot refused the change: {reason}")


def init_parameter_gateway(
    logger: structlog.stdlib.BoundLogger, node: Node, target: str
) -> ParameterGateway:
    return ParameterGateway(logger=logger, node=node, target=target)
