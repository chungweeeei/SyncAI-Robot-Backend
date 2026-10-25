"""gateways/ros.py: the one way a gateway calls a ROS service.

What is pinned is the cleanup: a request whose response did not arrive in time
is removed from the client's pending table, so a server restarted under it
(switch_mode / restart_mode rebuild every service this process calls) leaves
nothing behind for the life of the process.
"""

import pathlib
import re
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")

from syncai_backend.gateways import ros  # noqa: E402


class _Future:
    def __init__(self, result=None, completed=True):
        self._result = result
        self._done = completed
        self._callbacks = []

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def complete(self, result):
        self._result = result
        self._done = True
        for callback in self._callbacks:
            callback(self)

    def result(self):
        if isinstance(self._result, Exception):
            raise self._result
        return self._result


def _client(future):
    client = MagicMock()
    client.call_async.return_value = future
    return client


def test_an_answered_call_returns_the_response_and_removes_nothing():
    """rclpy already dropped the entry when it delivered the response."""
    response = SimpleNamespace(success=True)
    client = _client(_Future(result=response))

    assert ros.call_service(client, "req", timeout_s=1.0) is response
    client.call_async.assert_called_once_with("req")
    client.remove_pending_request.assert_not_called()


def test_an_unanswered_call_is_removed_from_the_pending_table():
    future = _Future(completed=False)
    client = _client(future)

    assert ros.call_service(client, "req", timeout_s=0.01) is None
    client.remove_pending_request.assert_called_once_with(future)


def test_a_response_from_another_thread_wakes_the_caller():
    future = _Future(completed=False)
    client = _client(future)
    threading.Timer(0.01, future.complete, args=("late but in time",)).start()

    assert ros.call_service(client, "req", timeout_s=5.0) == "late but in time"
    client.remove_pending_request.assert_not_called()


def test_a_server_side_failure_still_raises():
    """Not folded into None: "the service broke" is not "no response"."""
    client = _client(_Future(result=RuntimeError("server raised")))

    with pytest.raises(RuntimeError, match="server raised"):
        ros.call_service(client, "req", timeout_s=1.0)
    client.remove_pending_request.assert_not_called()


def test_the_wait_bridge_times_out_false():
    assert ros.wait_for_future(_Future(completed=False), timeout=0.01) is False
    assert ros.wait_for_future(_Future(completed=True)) is True


_GATEWAYS = pathlib.Path(__file__).resolve().parent.parent / "syncai_backend" / "gateways"


@pytest.mark.parametrize("relative", ["robot/robot.py", "map/map.py"])
def test_service_calls_go_through_call_service(relative):
    """A raw call_async is how the leak comes back. The one allowed exception
    is restart_mode, which keeps its request pending on purpose so a late
    answer can still report the restart's outcome."""
    source = (_GATEWAYS / relative).read_text()
    raw = re.findall(r"(\w+)\.call_async\(", source)

    assert raw == (["restart_mode_client"] if relative == "robot/robot.py" else [])
