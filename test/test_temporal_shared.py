"""The Temporal address is read when asked for, not when imported.

Pins the fix for a bug that was invisible in deployment: main.py imports
temporal.worker before it calls load_dotenv(), so a module-level
``os.getenv("TEMPORAL_ADDRESS")`` evaluated before .env existed and both
Temporal clients silently used the default whenever the variable lived only in
that file.
"""

from syncai_backend.temporal import shared


def test_the_address_reflects_the_environment_at_call_time(monkeypatch):
    monkeypatch.setenv("TEMPORAL_ADDRESS", "temporal:7233")
    assert shared.temporal_server_url() == "temporal:7233"

    # Changing it afterwards is seen too -- there is no cached value.
    monkeypatch.setenv("TEMPORAL_ADDRESS", "10.0.0.5:7233")
    assert shared.temporal_server_url() == "10.0.0.5:7233"


def test_the_default_applies_when_unset(monkeypatch):
    monkeypatch.delenv("TEMPORAL_ADDRESS", raising=False)
    assert shared.temporal_server_url() == shared.DEFAULT_TEMPORAL_ADDRESS


def test_nothing_is_evaluated_at_import():
    """No module-level name holds a resolved address any more."""
    assert not hasattr(shared, "TEMPORAL_SERVER_URL")
