"""The 3D map: serve octomap.bt, read and save the console's ground edits."""

from typing import List, Tuple

from fastapi import APIRouter, Request
from fastapi.responses import FileResponse

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    NotFoundError,
)
from syncai_backend.repositories.map.catalog import (
    OCTOMAP_KEY_MAX,
    StoredMap,
)

from syncai_backend.interfaces.rest.routers.maps.schemas import (
    OctomapEditsResponse,
    OctomapStatus,
    SaveOctomapEditsRequest,
    SaveOctomapEditsResponse,
)
from syncai_backend.interfaces.rest.routers.maps.common import (
    _octomap_status,
    _revalidate,
)
from syncai_backend.interfaces.rest.routers.maps.context import MapRouteContext


# Ground patches a save may carry. A whole floor at 0.1 m is a few hundred
# thousand columns; this is a bound on the body, not a budget.
OCTOMAP_EDITS_MAX = 1_000_000


def register(map_router: APIRouter, ctx: MapRouteContext) -> None:
    map_catalog_repo = ctx.map_catalog_repo
    _require = ctx.require
    _require_exists = ctx.require_exists
    job_lock = ctx.job_lock

    def _require_octomap(name: str) -> Tuple[StoredMap, str]:
        """The map and its tree's path, or the refusal that says why there is none.

        409 while the cleaning is still building it (a client that polls the
        catalogue will see `ok`), 404 with the reason for every other way of
        not having one.
        """
        stored = _require(name)
        path = map_catalog_repo.octomap_path(name)
        if path is not None and stored.octomap is not None:
            return stored, path

        status, error = _octomap_status(stored)
        if status is OctomapStatus.CONVERTING:
            raise ConflictError(
                f"The 3D map of '{name}' is still being built; try again once "
                "the map cleaning has finished.",
                code="octomap_converting",
            )
        if status is OctomapStatus.FAILED:
            raise NotFoundError(f"Map '{name}' has no 3D map: {error}")
        if status is OctomapStatus.INTERRUPTED:
            raise NotFoundError(f"The 3D map of '{name}' was never finished being built.")
        raise NotFoundError(f"Map '{name}' has no 3D map.")

    @map_router.get("/api/v1/maps/{name}/octomap")
    def get_map_octomap(name: str, request: Request):
        """The map's 3D map, ``octomap.bt`` byte for byte.

        Not decoded here: the file is the tree's own compact form (two bits a
        child, a third the size of its voxel centres as floats), and the
        console reads the keys straight off it for the ground editor. ETag'd
        on (size, mtime) through ``OctomapInfo.version`` — the same tag the
        catalogue reports as ``octomap_version``, so a client that sees it
        change knows the bytes did — and revalidated to 304 for a tab that
        already holds it. A content hash would cost a read of the whole file
        per listing for nothing: the tree is only ever replaced wholesale.
        """
        stored, path = _require_octomap(name)
        tree = stored.octomap
        assert tree is not None  # _require_octomap
        tag = f'"{tree.version}"'
        headers, not_modified = _revalidate(request, tag)
        if not_modified is not None:
            return not_modified

        return FileResponse(path, media_type="application/octet-stream", headers=headers)

    @map_router.get(
        "/api/v1/maps/{name}/octomap/edits", response_model=OctomapEditsResponse
    )
    def get_map_octomap_edits(name: str):
        """The ground patches saved for this map's 3D map; ``[]`` when none.

        Read from ``octomap_edits.json`` as saved, whether or not the tree on
        disk has had them applied: the file is the record, the tree is the
        robot's.
        """
        _require_exists(name)
        edits = map_catalog_repo.read_octomap_edits(name)
        if edits is None:
            return OctomapEditsResponse(name=name, res=None, fill_key=[], created_at=None)
        return OctomapEditsResponse(
            name=name,
            res=edits.res,
            fill_key=[list(key) for key in edits.fill_key],
            created_at=edits.created_at,
        )

    def _validate_fill_keys(entries: List[Tuple[int, int, int]]) -> List[Tuple[int, int, int]]:
        """400 with a sentence for a key no tree has or a column named twice.

        Pydantic has already made them int triples; what it cannot say is
        that a key is 16-bit or that a column holds one patch at most (the
        editor fills a column's missing floor, which is one voxel).
        """
        if len(entries) > OCTOMAP_EDITS_MAX:
            raise BadRequestError(
                f"Too many ground patches ({len(entries)}); at most "
                f"{OCTOMAP_EDITS_MAX} can be saved."
            )
        columns = set()
        keys: List[Tuple[int, int, int]] = []
        for entry in entries:
            key = (int(entry[0]), int(entry[1]), int(entry[2]))
            if any(value < 0 or value > OCTOMAP_KEY_MAX for value in key):
                raise BadRequestError(
                    f"Ground patch {list(key)} is outside the 3D map (each key is 0 to "
                    f"{OCTOMAP_KEY_MAX})."
                )
            column = key[:2]
            if column in columns:
                raise BadRequestError(
                    f"Two ground patches name the same column {list(column)}; a column "
                    "takes one patch."
                )
            columns.add(column)
            keys.append(key)
        return keys

    @map_router.put(
        "/api/v1/maps/{name}/octomap/edits",
        response_model=SaveOctomapEditsResponse,
        dependencies=job_lock,
    )
    def save_map_octomap_edits(name: str, request: SaveOctomapEditsRequest):
        """Replace the map's ground patches with the body's whole list.

        A record, not an edit of the tree: ``octomap.bt`` is untouched and
        nothing running on the robot reads the file. The robot's
        ``apply_octomap_edits`` is what patches the tree from it (keeping the
        original as ``octomap.raw.bt``), and the 3D planner loads the tree
        when it starts, so the message says the patches take effect once
        applied. Held by a running job like every map write: the patches
        change where the planner may go.

        Refused with no tree (404: the keys index that tree) and while one is
        being built (409 ``octomap_converting``: the keys would be another
        save's, in another map frame).
        """
        stored, _ = _require_octomap(name)
        tree = stored.octomap
        assert tree is not None  # _require_octomap
        if tree.resolution is None:
            raise ConflictError(
                f"The 3D map of '{name}' could not be read, so ground patches "
                "cannot be saved onto it.",
                code="octomap_unreadable",
            )

        keys = _validate_fill_keys(request.fill_key)
        edits = map_catalog_repo.write_octomap_edits(name, keys, tree.resolution)
        count = len(edits.fill_key)
        if count == 0:
            message = f"Cleared the ground patches of '{name}'."
        else:
            message = (
                f"Saved {count} ground {'patch' if count == 1 else 'patches'} for "
                f"'{name}'. They take effect once applied to the robot's 3D map."
            )
        return SaveOctomapEditsResponse(name=name, count=count, message=message)
