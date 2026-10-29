"""SQLAlchemy ORM models persisted in the per-robot ``<robot_id>_db`` PostgreSQL database."""

import uuid
from datetime import datetime, timezone
from typing import Any, Optional

from sqlalchemy import JSON, DateTime, Float, String, UniqueConstraint, Uuid
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class Base(DeclarativeBase):
    """Declarative base shared by every ORM model."""


class MapPoint(Base):
    """A named point placed on a map, stored in world (map-frame) coordinates.

    ``x``/``y`` in metres, ``theta`` in degrees, map frame: a MOVE goal as-is.
    ``z`` is the height of the ground the point stands on, in metres: which
    storey of a multi-floor map it is on, and what the 3D planner snaps the
    goal to. 0 is the ground floor, and what every point saved before the
    column existed reads as.
    """

    __tablename__ = "map_vertices"

    # Names are unique per map (every map has its own "dock"). In the schema
    # because only the DB can check and insert in one step. No migrations:
    # tables created before this constraint still accept duplicates.
    __table_args__ = (UniqueConstraint("map", "name", name="uq_map_vertices_map_name"),)

    id: Mapped[uuid.UUID] = mapped_column(Uuid, primary_key=True, default=uuid.uuid4)
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    type: Mapped[str] = mapped_column(String(64), nullable=False)
    map: Mapped[str] = mapped_column(String(255), nullable=False, index=True)
    x: Mapped[float] = mapped_column(Float, nullable=False)
    y: Mapped[float] = mapped_column(Float, nullable=False)
    theta: Mapped[float] = mapped_column(Float, nullable=False)
    z: Mapped[float] = mapped_column(Float, nullable=False, default=0.0, server_default="0")
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, onupdate=_utcnow, nullable=False
    )


class TaskTemplate(Base):
    """An operator-authored step list, kept so it can be re-dispatched.

    Temporal keeps closed workflows only for the namespace retention (5 days
    with the workspace compose), so saved step lists live here.

    ``steps`` is a JSON array, not a child table, because:

    1. **There are no migrations.** A typed table would freeze the step
       vocabulary into DDL; a JSON array grows an optional key.
    2. ``StepRequest`` + ``validate_step_params`` already enforce the schema at
       the request boundary.
    3. The array is the order; a child table would rewrite everything anyway.
    4. ``sqlalchemy.JSON`` works on the SQLite test engine too.

    Only ever *add* optional keys to a stored step; never rename, retype or
    repurpose one (an older backend reads it after a rollback).

    Traps:

    - **JSON columns are not mutation-tracked.** ``row.steps.append(...)`` is
      silently not flushed; always assign a fresh list.
    - **``Base.metadata`` is shared**, so adding a model here changes what every
      repo factory's ``create_all`` creates.
    """

    __tablename__ = "task_templates"

    id: Mapped[uuid.UUID] = mapped_column(Uuid, primary_key=True, default=uuid.uuid4)
    # A label, not unique: a ``(map_name, name)`` constraint would not apply to
    # NULL map_name rows (NULLs are distinct in a unique index), which are the
    # ones most likely to collide.
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    description: Mapped[str] = mapped_column(String(1000), nullable=False, default="")
    # The map of the MOVE coordinates; NULL (no MOVE) runs anywhere.
    map_name: Mapped[Optional[str]] = mapped_column(String(255), nullable=True, index=True)
    # Element shape: ``_StoredStep`` in interfaces/rest/routers/task_template.py.
    # JSONB on PostgreSQL (chosen now: no ALTER path later), JSON on SQLite.
    steps: Mapped[list[dict[str, Any]]] = mapped_column(
        JSON().with_variant(JSONB(), "postgresql"), nullable=False
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, onupdate=_utcnow, nullable=False
    )
