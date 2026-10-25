"""logger.py: LOG_LEVEL and LOG_FORMAT, read once at startup."""

import json
import logging

import pytest
import structlog

from syncai_backend import logger as logger_module


@pytest.fixture(autouse=True)
def _restore_structlog():
    yield
    structlog.reset_defaults()


def _configure(monkeypatch, level=None, fmt=None):
    for name, value in (("LOG_LEVEL", level), ("LOG_FORMAT", fmt)):
        if value is None:
            monkeypatch.delenv(name, raising=False)
        else:
            monkeypatch.setenv(name, value)
    logger_module.setup_log_handler()
    # cache_logger_on_first_use: a fresh logger per test.
    return structlog.get_logger()


def test_info_is_the_default_so_debug_stays_out_of_production(monkeypatch, capsys):
    """It was NOTSET: every debug call reached stdout with no way to stop it."""
    log = _configure(monkeypatch)

    log.debug("hidden")
    log.info("shown")

    out = capsys.readouterr().out
    assert "hidden" not in out
    assert "shown" in out


def test_the_level_is_settable(monkeypatch, capsys):
    log = _configure(monkeypatch, level="debug")

    log.debug("now shown")

    assert "now shown" in capsys.readouterr().out


def test_json_is_one_parseable_object_per_line(monkeypatch, capsys):
    log = _configure(monkeypatch, fmt="json")

    log.info("Map switched", map="hall", localized=True)

    record = json.loads(capsys.readouterr().out.strip())
    assert record["event"] == "Map switched"
    assert record["map"] == "hall"
    assert record["level"] == "info"
    assert "timestamp" in record


def test_json_carries_a_traceback_as_text(monkeypatch, capsys):
    log = _configure(monkeypatch, fmt="json")

    try:
        raise RuntimeError("boom")
    except RuntimeError:
        log.error("It failed", exc_info=True)

    record = json.loads(capsys.readouterr().out.strip())
    assert "RuntimeError: boom" in record["exception"]


@pytest.mark.parametrize("level, fmt", [("loud", None), (None, "xml")])
def test_a_typo_falls_back_and_says_so(monkeypatch, capsys, level, fmt):
    log = _configure(monkeypatch, level=level, fmt=fmt)

    log.info("still logging")

    out = capsys.readouterr().out
    assert "Ignoring a logging setting" in out
    assert "still logging" in out


def test_levels_resolve_by_name():
    assert logger_module._level_from("WARNING") == (logging.WARNING, None)
    assert logger_module._level_from(" error ") == (logging.ERROR, None)
    assert logger_module._level_from(None) == (logging.INFO, None)
