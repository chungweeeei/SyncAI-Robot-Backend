import uuid
import structlog

from collections.abc import Generator
from contextlib import contextmanager
from typing import Optional, TypedDict

from sqlalchemy import Engine, delete, select, update
from sqlalchemy.exc import IntegrityError
from sqlalchemy.orm import Session, sessionmaker

from syncai_backend.database.models import MapPoint, _utcnow
from syncai_backend.exceptions import ConflictError


class VertexFields(TypedDict):
    name: str
    type: str
    x: float
    y: float
    theta: float


class MapRepo:
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
        with self.session_maker() as session:
            try:
                yield session
            except ConflictError:
                # A domain outcome this repo produced on purpose (a duplicate
                # vertex name), not a database that misbehaved. Logging it at
                # ERROR with a traceback would put a stack trace in the journal
                # every time an operator types a name twice.
                raise
            except Exception:
                self.logger.error(f"[MapRepo][{op}] database operation failed", exc_info=True)
                raise

    @contextmanager
    def transaction(self, op: str) -> Generator[Session, None, None]:
        """One session for several writes that must land or fail together.

        Yields a session; commits when the block exits cleanly, and lets the
        sessionmaker's context manager roll back when it does not. Exists for
        the map rename: ``move_vertices`` and ``TaskTemplateRepo.rebind_map``
        used to each open and commit their own session, so a failure in the
        second left the vertices already re-keyed to a name the directory had
        just been moved back from -- permanently orphaned waypoints. Both repos
        take a ``session`` argument for exactly this caller; the two tables
        share one engine, so a session from this repo's maker serves both.
        """
        with self._session(op=op) as session:
            yield session
            session.commit()

    def create_vertices(self, map: str, vertices: list[VertexFields]) -> list[MapPoint]:
        """Batch-insert vertices in a single transaction.

        Each dict carries the column values (name/type/x/y/theta).
        All rows are committed together, so a failure inserts none of them.

        Raises ConflictError when a name is already taken on this map, or when
        the batch repeats one within itself — ``uq_map_vertices_map_name``
        cannot tell those apart, and neither answer differs to the caller.
        """
        with self._session(op="create_vertices") as session:
            rows = [MapPoint(map=map, **fields) for fields in vertices]
            session.add_all(rows)
            try:
                session.commit()
            except IntegrityError as exc:
                # The constraint is the check; this only translates it. A
                # SELECT-then-INSERT here would race, and would have to be
                # repeated in update_vertex -- see MapPoint.__table_args__.
                raise ConflictError(
                    self._name_taken_detail(map, [fields["name"] for fields in vertices]),
                    code="vertex_name_taken",
                ) from exc
            return rows

    @staticmethod
    def _name_taken_detail(map: str, names: list[str]) -> str:
        """The 409 sentence for a rejected insert.

        The database reports the constraint, not which row tripped it, and a
        batch arrives as one statement. Rather than issue another query on the
        error path to find out, the message names the whole submitted set and
        says the collision is somewhere in it — true for both the "already on
        the map" and the "repeated within this batch" case.
        """
        if len(names) == 1:
            return f"A vertex in '{map}' is already named '{names[0]}'."
        listed = ", ".join(repr(name) for name in names)
        return (
            f"Vertex names must be unique within '{map}', and one of "
            f"{listed} is already taken or repeated in this request."
        )

    def list_vertices(
        self, map: Optional[str] = None, type: Optional[str] = None
    ) -> list[MapPoint]:
        """Vertices matching both filters, oldest first.

        The filters are optional and AND together; omitting both lists every
        vertex on the robot, and callers treat an empty result as normal (a map
        with no vertices yet), so this must never raise on zero rows.
        """
        with self._session(op="list_vertices") as session:
            # database statements
            stmt = select(MapPoint)
            if map is not None:
                stmt = stmt.where(MapPoint.map == map)
            if type is not None:
                stmt = stmt.where(MapPoint.type == type)
            # id is a random UUID, so order by creation time for a stable,
            # meaningful listing order.
            stmt = stmt.order_by(MapPoint.created_at)
            # scalars(), not query(): Session.query() takes entities, not an
            # already-built Select, and returns Rows rather than MapPoints.
            return list(session.scalars(stmt).all())

    def get_vertex(self, vertex_id: uuid.UUID) -> Optional[MapPoint]:
        with self._session(op="get_vertex") as session:
            return session.get(MapPoint, vertex_id)

    def update_vertex(self, vertex_id: uuid.UUID, **fields) -> Optional[MapPoint]:
        """Apply ``fields`` to one vertex; None when no such vertex exists.

        Raises ConflictError when the rename would collide with another vertex
        on the same map. Renaming to the name it already has is not a collision
        -- the row is excluded from its own constraint check.
        """
        # Only these columns may be updated through the API.
        allowed = {"name", "type", "x", "y", "theta"}
        changes = {k: v for k, v in fields.items() if k in allowed and v is not None}

        with self._session(op="update_vertex") as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return None

            # Read for the error message before the commit that may fail: a
            # rolled-back session expires its objects, so touching the instance
            # afterwards would re-query inside a dead transaction.
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

        With ``session`` given, the UPDATE runs inside the caller's transaction
        and is *not* committed here -- see ``transaction``. Without it, the
        method is its own transaction, as every other write in this repo is.

        The cascade half of a map rename: ``map_vertices.map`` holds the bare
        directory name with no foreign key behind it, so when the directory
        moves the rows have to be told. A separate method rather than widening
        ``update_vertex``'s ``allowed`` set, because that set *is* the
        per-vertex API surface and "move one vertex to another map" is
        deliberately not on it (the MCP tools document a move as delete +
        create). This is a whole-map operation with one legitimate caller.

        One ``UPDATE`` statement, not a load-modify-save loop: a conference map
        carries dozens of vertices and the router is holding a half-renamed map
        while this runs. ``updated_at`` is set by hand because ``onupdate``
        fires for ORM unit-of-work flushes, not for a Core bulk update.
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

    def delete_vertices(self, map: str) -> int:
        """Delete every vertex of ``map``; return how many rows went.

        The cascade half of a map delete, and the counterpart to
        ``move_vertices``: ``map_vertices.map`` holds the bare directory name
        with no foreign key behind it, so nothing removes these rows when the
        directory goes. Left behind they are not merely untidy — they are
        unreachable (every vertex route resolves the map first and now 404s) and
        they would be silently adopted by the next map saved under the same
        name, handing a fresh mapping run someone else's waypoints.

        One ``DELETE`` statement rather than a loop over ``delete_vertex``, for
        the same reason ``move_vertices`` is one ``UPDATE``: this is a
        whole-map operation with one legitimate caller, and the router is
        holding a half-deleted map while it runs. No ``updated_at`` to set —
        the rows are gone, not changed.
        """
        with self._session(op="delete_vertices") as session:
            result = session.execute(delete(MapPoint).where(MapPoint.map == map))
            session.commit()
            return result.rowcount


def init_map_repo(
    logger: structlog.stdlib.BoundLogger,
    engine: Engine,
) -> MapRepo:
    MapPoint.metadata.create_all(engine)
    return MapRepo(logger=logger, engine=engine)
