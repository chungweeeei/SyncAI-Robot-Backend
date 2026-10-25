"""Collapse concurrent calls of one slow, read-only operation into one.

For a blocking route whose answer does not depend on who asked -- the wifi
scan, ~40 s on the robot -- a second request arriving while the first runs
waits for that run and gets its outcome, instead of holding a second threadpool
thread for the same 40 s and queueing a second request behind the first in the
ROS service. Not for an operation with side effects: two different requests
must not silently get one answer (those want a 409 instead).
"""

import threading
from concurrent.futures import Future
from typing import Callable, Generic, Optional, TypeVar

T = TypeVar("T")


class SingleFlight(Generic[T]):
    """One run of ``fn`` at a time; callers that arrive meanwhile share it.

    The run that starts is the caller's own thread (no executor of its own);
    the others park on a ``Future`` it settles, result or exception alike. The
    slot is cleared once that run ends, so the next caller starts a fresh one:
    nothing is cached past the run.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._inflight: Optional["Future[T]"] = None

    def run(self, fn: Callable[[], T]) -> T:
        with self._lock:
            shared = self._inflight
            if shared is None:
                own: "Future[T]" = Future()
                self._inflight = own
        if shared is not None:
            return shared.result()

        try:
            result = fn()
        except BaseException as exc:
            own.set_exception(exc)
            raise
        else:
            own.set_result(result)
            return result
        finally:
            with self._lock:
                self._inflight = None
