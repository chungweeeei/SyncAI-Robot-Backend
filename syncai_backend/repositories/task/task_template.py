import uuid
import structlog

from typing import Any, Optional

from sqlalchemy import Engine, or_, select, update
from sqlalchemy.orm import Session

from syncai_backend.database.models import TaskTemplate, _utcnow
from syncai_backend.database.session import logged_session, session_factory
from syncai_backend.tasks.retired_steps import (
    RETIRED_REASON,
    stored_steps_problem,
    strip_retired_steps,
)
from syncai_backend.tasks.schema import POSE_STEP_TYPES


class TaskTemplateRepo:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        engine: Engine,
    ):
        self.logger = logger

        self.session_maker = session_factory(engine)

    def _session(self, op: str):
        """A session for one repo operation, naming ``op`` if it fails."""
        return logged_session(self.session_maker, self.logger, f"[TaskTemplateRepo][{op}]")

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
    purge_retired_steps(logger, engine)
    return TaskTemplateRepo(logger=logger, engine=engine)


# What a stripped template's map_name is checked against: the router's rule that
# a template with no pose step names no map (_check_map_scope).
_POSE_TYPES = frozenset(step_type.value for step_type in POSE_STEP_TYPES)


def purge_retired_steps(logger: structlog.stdlib.BoundLogger, engine: Engine) -> dict:
    """Strip DOCK / UNDOCK steps from every stored template (tasks/retired_steps.py).

    Per template that has one:

    - **Nothing left: the template is deleted.** A template must have at least
      one step (``TaskTemplateRequest``), and one that only docked does nothing
      now. Its schedules go at the worker's startup (temporal/retired_schedules.py),
      since their frozen copy is DOCK-only too.
    - **Steps left, but no MOVE / PATROL:** ``map_name`` is cleared, because the
      router refuses a map on a template with no pose step, and the next PUT
      would 400 on a rule the operator never broke.
    - **What is left does not parse or breaks the group rules:** deleted, and
      said at ERROR. Removing steps cannot cause that, so it means the row was
      unsound before; kept, it would still fail every template list.

    One transaction, so a crash half-way leaves every row as it was. A second
    run finds nothing (idempotent), so it costs one SELECT per startup. Returns
    the counts it logs, for the tests.
    """
    summary = {"stripped": [], "deleted": [], "unmapped": []}
    with Session(engine) as session, session.begin():
        for row in session.scalars(select(TaskTemplate)).all():
            kept, removed = strip_retired_steps(row.steps or [])
            if not removed:
                continue
            template = str(row.id)
            if not kept:
                session.delete(row)
                summary["deleted"].append(template)
                logger.warning(
                    "[TaskTemplateRepo] Deleted a template whose only steps were DOCK / "
                    "UNDOCK",
                    template_id=template,
                    name=row.name,
                    removed_step_ids=removed,
                    reason=RETIRED_REASON,
                )
                continue
            problem = stored_steps_problem(kept)
            if problem is not None:
                session.delete(row)
                summary["deleted"].append(template)
                logger.error(
                    "[TaskTemplateRepo] Deleted a template whose steps left after "
                    "removing DOCK / UNDOCK cannot run; they could not before either",
                    template_id=template,
                    name=row.name,
                    removed_step_ids=removed,
                    problem=problem,
                    reason=RETIRED_REASON,
                )
                continue
            # Assigned whole: JSON columns are not mutation-tracked.
            row.steps = kept
            row.updated_at = _utcnow()
            unmapped = row.map_name is not None and not any(
                entry.get("type") in _POSE_TYPES for entry in kept
            )
            if unmapped:
                row.map_name = None
                summary["unmapped"].append(template)
            summary["stripped"].append(template)
            logger.warning(
                "[TaskTemplateRepo] Removed DOCK / UNDOCK steps from a template",
                template_id=template,
                name=row.name,
                removed_step_ids=removed,
                steps_left=len(kept),
                map_name_cleared=unmapped,
                reason=RETIRED_REASON,
            )
    if summary["stripped"] or summary["deleted"]:
        logger.warning(
            "[TaskTemplateRepo] Purged DOCK / UNDOCK steps from stored templates",
            stripped=len(summary["stripped"]),
            deleted=len(summary["deleted"]),
            map_name_cleared=len(summary["unmapped"]),
        )
    return summary
