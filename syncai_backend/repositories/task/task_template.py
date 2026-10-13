import uuid
import structlog

from collections.abc import Generator
from contextlib import contextmanager
from typing import Any, Optional

from sqlalchemy import Engine, or_, select, update
from sqlalchemy.orm import Session, sessionmaker

from syncai_backend.database.models import TaskTemplate, _utcnow


class TaskTemplateRepo:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        engine: Engine,
    ):
        self.logger = logger

        # Register database session factory (per-repo session convention).
        self.session_maker = sessionmaker(
            autocommit=False,
            autoflush=False,
            expire_on_commit=False,
            bind=engine,
        )

    @contextmanager
    def _session(self, op: str) -> Generator[Session, None, None]:
        """A session for one repo operation, naming ``op`` if it fails.

        Log-and-reraise: the SQLAlchemy error carries the failed statement.
        """
        with self.session_maker() as session:
            try:
                yield session
            except Exception:
                self.logger.error(
                    f"[TaskTemplateRepo][{op}] database operation failed", exc_info=True
                )
                raise

    def create_task_template(
        self,
        *,
        name: str,
        description: str,
        map_name: Optional[str],
        steps: list[dict[str, Any]],
    ) -> TaskTemplate:
        with self._session("create_task_template") as session:
            row = TaskTemplate(
                name=name,
                description=description,
                map_name=map_name,
                steps=steps,
            )
            session.add(row)
            session.commit()
            session.refresh(row)
            return row

    def list_task_templates(
        self,
        map_name: Optional[str] = None,
        include_map_independent: bool = True,
    ) -> list[TaskTemplate]:
        """Task templates, by name then creation time.

        A ``map_name`` returns that map's templates plus the map-independent
        (``map_name IS NULL``) ones, unless ``include_map_independent=False``;
        ``map_name=None`` returns every row.

        By name, not ``updated_at``, so a save does not reshuffle the list the
        console renders as-is; ``created_at`` breaks ties.
        """
        with self._session("list_task_templates") as session:
            stmt = select(TaskTemplate)
            if map_name is not None:
                stmt = stmt.where(
                    or_(
                        TaskTemplate.map_name == map_name,
                        TaskTemplate.map_name.is_(None),
                    )
                    if include_map_independent
                    else TaskTemplate.map_name == map_name
                )
            stmt = stmt.order_by(TaskTemplate.name, TaskTemplate.created_at)
            return list(session.scalars(stmt).all())

    def get_task_template(self, task_id: uuid.UUID) -> Optional[TaskTemplate]:
        with self._session("get_task_template") as session:
            return session.get(TaskTemplate, task_id)

    def update_task_template(self, task_id: uuid.UUID, **fields) -> Optional[TaskTemplate]:
        # Only these columns may be updated through the API.
        allowed = {"name", "description", "map_name", "steps"}
        # Not filtering None, unlike MapRepo.update_vertex: `map_name=None` makes
        # a template map-independent. The router's `exclude_unset=True` keeps
        # omitted fields out.
        changes = {k: v for k, v in fields.items() if k in allowed}

        with self._session("update_task_template") as session:
            row = session.get(TaskTemplate, task_id)
            if row is None:
                return None

            for key, value in changes.items():
                # Assign `steps` whole, never mutate: JSON columns are not
                # mutation-tracked (see database/models.py).
                setattr(row, key, value)

            session.commit()
            session.refresh(row)
            return row

    def rebind_map(
        self, old_name: str, new_name: str, session: Optional[Session] = None
    ) -> int:
        """Point every template bound to ``old_name`` at ``new_name``; return how many.

        With ``session``, joins the caller's transaction (the rename runs this
        and ``MapRepo.move_vertices`` under one ``MapRepo.transaction``).
        Without it, a renamed map's templates would refuse to run (dispatch
        compares ``map_name`` by string). Steps need no rewriting: MOVE refers
        to vertices by UUID.

        A schedule memo's ``map_name`` stays stale on purpose (a display label).
        """
        statement = (
            update(TaskTemplate)
            .where(TaskTemplate.map_name == old_name)
            .values(map_name=new_name, updated_at=_utcnow())
        )
        if session is not None:
            return session.execute(statement).rowcount

        with self._session("rebind_map") as own:
            result = own.execute(statement)
            own.commit()
            return result.rowcount

    def delete_task_template(self, task_id: uuid.UUID) -> bool:
        with self._session("delete_task_template") as session:
            row = session.get(TaskTemplate, task_id)
            if row is None:
                return False

            session.delete(row)
            session.commit()
            return True


def init_task_template_repo(
    logger: structlog.stdlib.BoundLogger,
    engine: Engine,
) -> TaskTemplateRepo:
    # Redundant with init_map_repo's (shared `Base.metadata`), but keeps each
    # factory self-sufficient and construction order irrelevant.
    TaskTemplate.metadata.create_all(engine)
    return TaskTemplateRepo(logger=logger, engine=engine)
