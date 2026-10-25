"""The two rclpy bridges every ROS gateway needs off the executor thread.

REST handlers and Temporal activities wait on ROS futures from their own
threads while the executor delivers the answers; ``spin_until_future_complete``
would deadlock there. Its own module rather than ``gateways/__init__.py`` so
the gateways that hold no node (tts, webrtc, recording) keep importing without
rclpy.
"""

import threading
from typing import Any, Optional

from rclpy.client import Client


def wait_for_future(future, timeout: Optional[float] = None) -> bool:
    """Block until ``future`` is done or ``timeout`` passes; True if done.

    An Event the future's done callback sets: the callback runs on the executor
    thread, the wait on the caller's.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout=timeout)


def call_service(client: Client, request: Any, timeout_s: float) -> Optional[Any]:
    """Send one request and wait up to ``timeout_s`` for its response.

    None on timeout, after ``remove_pending_request``. Without that, the
    request stays in the client's pending table until a response that may
    never come -- the server was restarted under it (``switch_mode`` and
    ``restart_mode`` rebuild every service this process calls), or crashed --
    for the life of the process: memory, and a longer linear scan under the
    client's lock on every response the executor delivers after it. A late
    response for a removed request is dropped by rclpy, so nothing reads the
    abandoned future.

    ``wait_for_service`` stays with the caller: "service is not available" and
    "no response" are different answers to the operator. A response the server
    failed with is re-raised by ``result()`` here, as it always was.
    """
    future = client.call_async(request)
    if wait_for_future(future, timeout=timeout_s):
        return future.result()
    client.remove_pending_request(future)
    return None
