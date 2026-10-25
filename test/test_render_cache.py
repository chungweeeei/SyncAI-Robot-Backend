"""The map router's _RenderCache: never stale, and now bounded (LRU by bytes)."""

import pytest

pytest.importorskip("rclpy")  # the router module imports the ROS gateways

from syncai_backend.interfaces.rest.routers.map import _RenderCache  # noqa: E402


def _renderer(data: bytes):
    calls = []

    def render():
        calls.append(1)
        return data

    return render, calls


def test_a_hit_on_the_same_version_does_not_render_again():
    cache = _RenderCache(max_bytes=100)
    render, calls = _renderer(b"x" * 10)

    assert cache.get("image", "hall", "v1", render) == b"x" * 10
    assert cache.get("image", "hall", "v1", render) == b"x" * 10
    assert calls == [1]


def test_a_new_version_replaces_the_entry_in_place():
    cache = _RenderCache(max_bytes=100)
    cache.get("image", "hall", "v1", lambda: b"a" * 10)

    assert cache.get("image", "hall", "v2", lambda: b"b" * 20) == b"b" * 20
    assert cache.size_bytes == 20


def test_the_least_recently_used_goes_first_past_the_budget():
    cache = _RenderCache(max_bytes=30)
    cache.get("cloud", "a", 1, lambda: b"a" * 10)
    cache.get("cloud", "b", 1, lambda: b"b" * 10)
    cache.get("cloud", "c", 1, lambda: b"c" * 10)
    # A hit makes 'a' the most recent; 'b' is now the oldest.
    cache.get("cloud", "a", 1, lambda: pytest.fail("a was cached"))

    cache.get("cloud", "d", 1, lambda: b"d" * 10)

    assert cache.size_bytes == 30
    render_b, calls = _renderer(b"b" * 10)
    cache.get("cloud", "b", 1, render_b)
    assert calls == [1]  # evicted, so rendered again


def test_one_entry_over_budget_is_still_kept_until_the_next():
    """A big map renders once per version, not on every request."""
    cache = _RenderCache(max_bytes=10)
    render, calls = _renderer(b"x" * 50)

    cache.get("cloud", "huge", 1, render)
    cache.get("cloud", "huge", 1, render)

    assert calls == [1]
    assert cache.size_bytes == 50


def test_forget_drops_every_kind_of_one_map_and_its_bytes():
    cache = _RenderCache(max_bytes=100)
    cache.get("image", "hall", 1, lambda: b"i" * 10)
    cache.get("thumbnail", "hall", 1, lambda: b"t" * 5)
    cache.get("image", "lab", 1, lambda: b"l" * 7)

    cache.forget("hall")

    assert cache.size_bytes == 7
