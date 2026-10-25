"""helpers/single_flight.py: concurrent callers of one slow read share one run."""

import threading

import pytest

from syncai_backend.helpers.single_flight import SingleFlight


def _parked(flight: SingleFlight, count: int) -> bool:
    """Whether ``count`` callers are waiting on the run in flight.

    Reads the Future's condition variable: the only way to know a follower
    has *joined* (not merely started) without sleeping and hoping.
    """
    inflight = flight._inflight
    return inflight is not None and len(inflight._condition._waiters) == count


def _wait_until(predicate, timeout=5.0):
    done = threading.Event()
    for _ in range(int(timeout / 0.005)):
        if predicate():
            return True
        done.wait(0.005)
    return False


def test_callers_during_a_run_get_its_result_and_do_not_run_again():
    flight = SingleFlight()
    release = threading.Event()
    calls = []

    def slow():
        calls.append(1)
        release.wait(5)
        return ["net-a", "net-b"]

    results = []
    threads = [
        threading.Thread(target=lambda: results.append(flight.run(slow)))
        for _ in range(3)
    ]
    threads[0].start()
    assert _wait_until(lambda: calls == [1])
    for thread in threads[1:]:
        thread.start()
    assert _wait_until(lambda: _parked(flight, 2))

    release.set()
    for thread in threads:
        thread.join(5)

    assert calls == [1]
    assert results == [["net-a", "net-b"]] * 3


def test_callers_during_a_run_get_its_exception_too():
    flight = SingleFlight()
    release = threading.Event()

    def failing():
        release.wait(5)
        raise RuntimeError("scan_wifi went away")

    errors = []

    def call():
        try:
            flight.run(failing)
        except RuntimeError as exc:
            errors.append(str(exc))

    first, second = threading.Thread(target=call), threading.Thread(target=call)
    first.start()
    assert _wait_until(lambda: flight._inflight is not None)
    second.start()
    assert _wait_until(lambda: _parked(flight, 1))
    release.set()
    first.join(5)
    second.join(5)

    assert errors == ["scan_wifi went away"] * 2


def test_nothing_is_cached_after_a_run_ends():
    """The next press scans again: a single flight, not a cache."""
    flight = SingleFlight()
    answers = iter(["first", "second"])

    assert flight.run(lambda: next(answers)) == "first"
    assert flight.run(lambda: next(answers)) == "second"


def test_a_failed_run_frees_the_slot():
    flight = SingleFlight()

    with pytest.raises(ValueError):
        flight.run(lambda: (_ for _ in ()).throw(ValueError("no")))

    assert flight.run(lambda: "after") == "after"
