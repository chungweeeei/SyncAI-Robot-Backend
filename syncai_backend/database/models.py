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


class TaskRecordRow(Base):
    """One finished run, as RobotWorkflow left it: the source of the report export.

    Temporal forgets a closed run after the namespace retention (5 days); a soak test
    runs longer than that, so each run writes its row here as it closes
    (``record_task``). Pydantic's ``TaskRecord`` (tasks/schema.py) is the shape sent;
    this is the row.

    **Range-partitioned by day** on ``closed_at`` (UTC) on PostgreSQL, so a day ages
    out as one ``DETACH`` + ``DROP`` instead of a ``DELETE`` that bloats the table
    (TaskRecordRetention keeps 30 days). The partitions themselves are not in the
    metadata: TaskRecordRepo creates them. SQLite (the tests) ignores the dialect
    kwarg and gets a plain table.

    The primary key carries ``closed_at`` because PostgreSQL requires every unique
    constraint of a partitioned table to contain the partition key; ``run_id`` alone
    is what identifies the run. ``steps`` holds ``Step`` dumps, timing included;
    like ``TaskTemplate.steps``, only ever add optional keys to it.
    """

    __tablename__ = "task_record"
    __table_args__ = {"postgresql_partition_by": "RANGE (closed_at)"}

    run_id: Mapped[str] = mapped_column(String(255), primary_key=True)
    closed_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), primary_key=True, index=True
    )
    task_id: Mapped[str] = mapped_column(String(255), nullable=False)
    # COMPLETED / FAILED / CANCELED.
    status: Mapped[str] = mapped_column(String(32), nullable=False)
    started_at: Mapped[datetime] = mapped_column(DateTime(timezone=True), nullable=False)
    kind: Mapped[Optional[str]] = mapped_column(String(32), nullable=True)
    name: Mapped[Optional[str]] = mapped_column(String(255), nullable=True)
    map_name: Mapped[Optional[str]] = mapped_column(String(255), nullable=True)
    schedule_id: Mapped[Optional[str]] = mapped_column(String(255), nullable=True)
    steps: Mapped[list[dict[str, Any]]] = mapped_column(
        JSON().with_variant(JSONB(), "postgresql"), nullable=False
    )
