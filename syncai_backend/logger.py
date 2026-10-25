import logging
import os
from typing import List, Optional, Tuple

import structlog

# Read once, after load_dotenv (main.py), so `.env` can set them.
LOG_LEVEL_ENV = "LOG_LEVEL"
LOG_FORMAT_ENV = "LOG_FORMAT"

DEFAULT_LEVEL = logging.INFO

# `console` (the default) is the human one, colours and all, for byobu and
# `docker compose logs`; `json` is one object per line for a log collector to
# parse, which the console renderer's padding and ANSI codes defeat.
FORMATS = ("console", "json")


def _level_from(value: Optional[str]) -> Tuple[int, Optional[str]]:
    """The numeric level ``value`` names, and a complaint if it names none.

    INFO when unset. It used to be NOTSET, which let every ``debug`` call
    through to production stdout with no way to turn it off.
    """
    if not value:
        return DEFAULT_LEVEL, None
    level = logging.getLevelName(value.strip().upper())
    if isinstance(level, int):
        return level, None
    return DEFAULT_LEVEL, f"{LOG_LEVEL_ENV}={value!r} is not a level; using INFO"


def _format_from(value: Optional[str]) -> Tuple[str, Optional[str]]:
    fmt = (value or "console").strip().lower()
    if fmt in FORMATS:
        return fmt, None
    return "console", f"{LOG_FORMAT_ENV}={value!r} is not one of {FORMATS}; using console"


def setup_log_handler() -> None:
    level, level_problem = _level_from(os.getenv(LOG_LEVEL_ENV))
    fmt, format_problem = _format_from(os.getenv(LOG_FORMAT_ENV))

    processors: List = [structlog.processors.add_log_level]
    if fmt == "json":
        processors += [
            structlog.processors.TimeStamper(fmt="iso", utc=True),
            # The console renderer formats exc_info itself; JSON needs it as text.
            structlog.processors.format_exc_info,
            structlog.processors.JSONRenderer(),
        ]
    else:
        processors += [
            structlog.processors.TimeStamper(fmt="%Y-%m-%d %H:%M:%S", utc=False),
            structlog.dev.ConsoleRenderer(),
        ]

    structlog.configure(
        processors=processors,
        wrapper_class=structlog.make_filtering_bound_logger(level),
        context_class=dict,
        logger_factory=structlog.PrintLoggerFactory(),
        cache_logger_on_first_use=True,
    )

    # A typo must not silently mean "the default": say so, once, at startup.
    logger = structlog.get_logger()
    for problem in (level_problem, format_problem):
        if problem:
            logger.warning("Ignoring a logging setting", problem=problem)
