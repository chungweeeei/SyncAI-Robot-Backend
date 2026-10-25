import uuid
import structlog

from collections.abc import Generator, Iterable
from contextlib import contextmanager
from typing import Optional, TypedDict

from sqlalchemy import Engine, delete, func, inspect, select, text, update
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import Session

from syncai_backend.database.models import MapPoint, _utcnow
from syncai_backend.database.session import logged_session, session_factory
from syncai_backend.exceptions import ConflictError


class _VertexFieldsRequired(TypedDict):
    name: str
    type: str
    x: float
    y: float
    theta: float


class VertexFields(_VertexFieldsRequired, total=False):
    # Optional so a caller that predates storeys (and every test) still
    # inserts a ground-floor point; the column default fills it in.
    z: float


class MapRepo:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        engine: Engine,
    ):
        self.logger = logger

        self.session_maker = session_factory(engine)

    def _session(self, op: str):
        # ConflictError is a domain outcome (a duplicate name), not a DB fault.
        return logged_session(
            self.session_maker, self.logger, f"[MapRepo][{op}]", quiet=(ConflictError,)
        )

    @contextmanager
    def transaction(self, op: str) -> Generator[Session, None, None]:
        """One session for several writes that must land or fail together.

        Commits on a clean exit, rolls back otherwise. Used by the map rename
        and import so e.g. ``move_vertices`` and ``TaskTemplateRepo.rebind_map``
        cannot half-land (orphaned waypoints). Both tables share one engine, so
        this session serves both repos' ``session=`` argument.
        """
        with self._session(op=op) as session:
            yield session
            session.commit()

    def create_vertices(
        self,
        map: str,
        vertices: list[VertexFields],
        session: Optional[Session] = None,
    ) -> list[MapPoint]:
        """Batch-insert vertices in a single transaction.

        All rows land together or none do. With ``session``, they are flushed
        into the caller's transaction, not committed (see ``transaction``).

        Raises ConflictError when a name is taken on this map or repeated in
        the batch (``uq_map_vertices_map_name`` cannot tell which).
        """
        rows = [MapPoint(map=map, **fields) for fields in vertices]
        names = [fields["name"] for fields in vertices]

        if session is not None:
            session.add_all(rows)
            try:
                session.flush()
            except IntegrityError as exc:
                raise ConflictError(
                    self._name_taken_detail(map, names), code="vertex_name_taken"
                ) from exc
            return rows

        with self._session(op="create_vertices") as own:
            own.add_all(rows)
            try:
                own.commit()
            except IntegrityError as exc:
                # The constraint is the check (a SELECT first would race); see
                # MapPoint.__table_args__.
                raise ConflictError(
                    self._name_taken_detail(map, names), code="vertex_name_taken"
                ) from exc
            return rows

    @staticmethod
    def _name_taken_detail(map: str, names: list[str]) -> str:
        """The 409 sentence for a rejected insert.

        The constraint does not say which row tripped it, so a batch's message
        names the whole set.
        """
        if len(names) == 1:
            return f"A vertex in '{map}' is already named '{names[0]}'."
        listed = ", ".join(repr(name) for name in names)
        return (
            f"Vertex names must be unique within '{map}', and one of "
            f"{listed} is already taken or repeated in this request."
        )

    @staticmethod
    def _scope(stmt, map: Optional[str], type: Optional[str]):
        """AND the two optional filters onto a statement over ``map_vertices``.

        Shared so ``list_vertices`` and ``count_vertices`` cannot disagree.
        """
        if map is not None:
            stmt = stmt.where(MapPoint.map == map)
        if type is not None:
            stmt = stmt.where(MapPoint.type == type)
        return stmt

    def list_vertices(
        self, map: Optional[str] = None, type: Optional[str] = None
    ) -> list[MapPoint]:
        """Vertices matching both filters, oldest first.

        Optional filters, ANDed; zero rows is normal. Unbounded, so use
        ``count_vertices`` for a count and ``get_vertices`` for a subset.
        """
        with self._session(op="list_vertices") as session:
            stmt = self._scope(select(MapPoint), map, type)
            # id is a random UUID; creation time gives a stable order.
            stmt = stmt.order_by(MapPoint.created_at)
            return list(session.scalars(stmt).all())

    def count_vertices(self, map: Optional[str] = None, type: Optional[str] = None) -> int:
        """How many vertices match the same filters ``list_vertices`` takes.

        A ``SELECT count(*)``: the catalogue asks once per map per page load,
        and ``len(list_vertices(...))`` would hydrate every row.
        """
        with self._session(op="count_vertices") as session:
            stmt = self._scope(select(func.count()).select_from(MapPoint), map, type)
            return session.scalars(stmt).one()

    def count_vertices_by_map(self) -> dict[str, int]:
        """Every map's vertex count in one ``GROUP BY``, for the map listing.

        ``count_vertices`` per map was one session and one round trip per map
        on every ``GET /api/v1/maps``. A map with no vertices has no row here;
        read it with ``.get(name, 0)``.
        """
        with self._session(op="count_vertices_by_map") as session:
            stmt = select(MapPoint.map, func.count()).group_by(MapPoint.map)
            return {name: count for name, count in session.execute(stmt).all()}

    def get_vertex(self, vertex_id: uuid.UUID) -> Optional[MapPoint]:
        with self._session(op="get_vertex") as session:
            return session.get(MapPoint, vertex_id)

    def get_vertices(
        self, map: str, vertex_ids: Iterable[uuid.UUID]
    ) -> list[MapPoint]:
        """The vertices of ``map`` whose ids are in ``vertex_ids``.

        Keep the ``map`` filter: task templates rely on absence to reject a
        step pointing at another map's vertex. Callers tell "deleted" from
        "elsewhere" with ``get_vertex`` on the missing ids.
        """
        ids = list(vertex_ids)
        if not ids:
            # Skip a round trip for `IN ()`.
            return []

        with self._session(op="get_vertices") as session:
            stmt = (
                select(MapPoint)
                .where(MapPoint.map == map)
                .where(MapPoint.id.in_(ids))
                .order_by(MapPoint.created_at)
            )
            return list(session.scalars(stmt).all())

    def update_vertex(self, vertex_id: uuid.UUID, **fields) -> Optional[MapPoint]:
        """Apply ``fields`` to one vertex; None when no such vertex exists.

        Raises ConflictError when a rename collides with another vertex on the
        same map (renaming to its own name does not).
        """
        # Only these columns may be updated through the API.
        allowed = {"name", "type", "x", "y", "theta", "z"}
        changes = {k: v for k, v in fields.items() if k in allowed and v is not None}

        with self._session(op="update_vertex") as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return None

            # Read before the commit: a rollback expires the instance.
            owning_map = vertex.map
            new_name = changes.get("name", vertex.name)

            for key, value in changes.items():
                setattr(vertex, key, value)

            try:
                session.commit()
            except IntegrityError as exc:
                raise ConflictError(
                    self._name_taken_detail(owning_map, [new_name]),
                    code="vertex_name_taken",
                ) from exc
            return vertex

    def delete_vertex(self, vertex_id: uuid.UUID) -> bool:
        with self._session(op="delete_vertex") as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return False

            session.delete(vertex)
            session.commit()
            return True

    def move_vertices(
        self, old_map: str, new_map: str, session: Optional[Session] = None
    ) -> int:
        """Re-key every vertex of ``old_map`` to ``new_map``; return how many.

        The DB half of a map rename (the name is a foreign key by convention).
        With ``session``, runs in the caller's transaction, uncommitted. Not
        part of ``update_vertex``'s ``allowed`` set: moving one vertex to
        another map is deliberately not API.

        ``updated_at`` is set by hand: ``onupdate`` does not fire for a Core
        bulk update.
        """
        statement = (
            update(MapPoint)
            .where(MapPoint.map == old_map)
            .values(map=new_map, updated_at=_utcnow())
        )
        if session is not None:
            return session.execute(statement).rowcount

        with self._session(op="move_vertices") as own:
            result = own.execute(statement)
            own.commit()
            return result.rowcount

    def delete_vertices(self, map: str, session: Optional[Session] = None) -> int:
        """Delete every vertex of ``map``; return how many rows went.

        The DB half of a map delete or import. With ``session``, runs in the
        caller's transaction, uncommitted. Left behind, the rows would be
        adopted by the next map saved under the same name.
        """
        statement = delete(MapPoint).where(MapPoint.map == map)
        if session is not None:
            return session.execute(statement).rowcount

        with self._session(op="delete_vertices") as own:
            result = own.execute(statement)
            own.commit()
            return result.rowcount


def init_map_repo(
    logger: structlog.stdlib.BoundLogger,
    engine: Engine,
) -> MapRepo:
    MapPoint.metadata.create_all(engine)
    _add_vertex_z(engine)
    return MapRepo(logger=logger, engine=engine)


def _add_vertex_z(engine: Engine) -> None:
    """Give a ``map_vertices`` table created before ``z`` existed its column.

    There are no migrations and ``create_all`` never alters a table that is
    already there, so a robot's existing waypoints would otherwise fail every
    read. Checked by inspection rather than ``ADD COLUMN IF NOT EXISTS``,
    which the SQLite test engine does not speak. The default puts every old
    waypoint on the ground floor, which is where they all were.
    """
    columns = {column["name"] for column in inspect(engine).get_columns("map_vertices")}
    if "z" in columns:
        return
    with engine.begin() as connection:
        connection.execute(
            text("ALTER TABLE map_vertices ADD COLUMN z FLOAT NOT NULL DEFAULT 0")
        )
