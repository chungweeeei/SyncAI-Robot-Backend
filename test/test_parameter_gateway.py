"""Tests for ParameterGateway: the wire types and how a refusal is tagged."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")

from rcl_interfaces.msg import ParameterType, ParameterValue  # noqa: E402

from syncai_backend.gateways.failure import Failure, failure_code  # noqa: E402
from syncai_backend.gateways.parameters import parameters  # noqa: E402
from syncai_backend.gateways.parameters.parameters import (  # noqa: E402
    ParameterGateway,
    from_parameter_value,
    to_parameter_value,
)


@pytest.mark.parametrize(
    "value, kind",
    [
        (True, ParameterType.PARAMETER_BOOL),
        (3, ParameterType.PARAMETER_INTEGER),
        (3.0, ParameterType.PARAMETER_DOUBLE),
        ("x", ParameterType.PARAMETER_STRING),
    ],
)
def test_a_value_keeps_its_type_on_the_wire(value, kind):
    """No dynamic typing on controller_server: 3 and 3.0 are different sets."""
    wire = to_parameter_value(value)
    assert wire.type == kind
    assert from_parameter_value(wire) == value


def test_an_undeclared_parameter_reads_none():
    assert from_parameter_value(ParameterValue(type=ParameterType.PARAMETER_NOT_SET)) is None


def _gateway(logger, set_response):
    node = MagicMock()
    gw = ParameterGateway(logger=logger, node=node, target="controller_server")
    gw._set_client = MagicMock()
    gw._set_client.wait_for_service.return_value = True
    return gw, set_response


def _answer(monkeypatch, response):
    monkeypatch.setattr(parameters.ros, "call_service", lambda *a, **k: response)


def _result(successful, reason=""):
    return SimpleNamespace(result=SimpleNamespace(successful=successful, reason=reason))


def test_the_mid_goal_refusal_is_controller_busy(logger, monkeypatch):
    gw, _ = _gateway(logger, None)
    _answer(
        monkeypatch,
        _result(
            False,
            "Unable to dynamically change Parameters while the controller is currently running",
        ),
    )

    ok, message = gw.set_atomically({"failure_tolerance": 4.0})
    assert not ok
    assert failure_code(message) == Failure.CONTROLLER_BUSY


def test_a_callbacks_reason_is_parameter_refused(logger, monkeypatch):
    gw, _ = _gateway(logger, None)
    _answer(monkeypatch, _result(False, "Docking.v_linear_min above Docking.v_linear_max"))

    ok, message = gw.set_atomically({"Docking.v_linear_max": 0.01})
    assert not ok
    assert failure_code(message) == Failure.PARAMETER_REFUSED
    assert "v_linear_min above" in message


def test_no_answer_is_untagged(logger, monkeypatch):
    gw, _ = _gateway(logger, None)
    _answer(monkeypatch, None)

    ok, message = gw.set_atomically({"FollowPath.vx_max": 0.6})
    assert not ok
    assert failure_code(message) is None


def test_an_empty_set_sends_nothing(logger, monkeypatch):
    gw, _ = _gateway(logger, None)
    monkeypatch.setattr(
        parameters.ros, "call_service", MagicMock(side_effect=AssertionError("sent"))
    )
    assert gw.set_atomically({}) == (True, "")
