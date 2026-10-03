"""Tests for the on-disk map catalogue repository.

Real directories via the ``maps_dir`` fixture; nothing is mocked. The INI lookup
is redirected with monkeypatch.setenv so no test depends on the host having a
~/robot_ws.
"""

import json
import os

import pytest

pytest.importorskip("cv2")
pytest.importorskip("yaml")

from syncai_backend.exceptions import BadRequestError, NotFoundError  # noqa: E402
from syncai_backend.helpers.system_config import (  # noqa: E402
    SYSTEM_INI_ENV,
)
from syncai_backend.repositories.map.catalog import (  # noqa: E402
    init_map_catalog_repo,
)


def _names(maps):
    return [stored.name for stored in maps]


# --- list_maps --------------------------------------------------------------


def test_lists_directories_only(catalog_repo):
    """The loose stray.pgm at the root is not a map under this layout."""
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_converted_map_reports_its_geometry(catalog_repo):
    full = catalog_repo.get_map("full")

    assert full.grid is not None
    assert full.grid.width == 6
    assert full.grid.height == 4
    assert full.grid.resolution == pytest.approx(0.05)
    assert full.grid.origin == pytest.approx((-6.94, -11.09, 0.0))
    assert full.has_pointcloud is True


def test_unconverted_map_has_no_grid(catalog_repo):
    """pgo/save_maps has run but pcd_to_gridmap.py has not."""
    rawonly = catalog_repo.get_map("rawonly")

    assert rawonly.grid is None
    assert rawonly.has_pointcloud is True


def test_size_and_mtime_cover_the_directory(catalog_repo, maps_dir):
    full = catalog_repo.get_map("full")

    on_disk = sum(
        os.path.getsize(maps_dir / "full" / name)
        for name in os.listdir(maps_dir / "full")
    )
    assert full.size_bytes == on_disk
    assert full.modified_at.tzinfo is not None


def test_missing_map_returns_none(catalog_repo):
    assert catalog_repo.get_map("nosuchmap") is None


def test_missing_maps_dir_lists_empty(logger, tmp_path):
    repo = init_map_catalog_repo(logger=logger)
    repo.maps_dir = str(tmp_path / "absent")

    assert repo.list_maps() == []


def test_unreadable_gridmap_degrades_to_no_grid(catalog_repo, maps_dir):
    """One torn gridmap must not take the whole listing down."""
    (maps_dir / "full" / "gridmap.pgm").write_bytes(b"P5\n999 999\n255\nshort")

    assert catalog_repo.get_map("full").grid is None
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_gridmap_yaml_without_pgm_has_no_grid(catalog_repo, maps_dir):
    (maps_dir / "full" / "gridmap.pgm").unlink()

    assert catalog_repo.get_map("full").grid is None


# --- resolve_dir / gridmap_path ---------------------------------------------


@pytest.mark.parametrize(
    "name",
    [
        "..",
        ".",
        "../etc",
        "sub/dir",
        "/etc/passwd",
        "with space",
        "",
        "a" * 65,
        "full/../../etc",
    ],
)
def test_resolve_rejects_unsafe_names(catalog_repo, name):
    with pytest.raises(BadRequestError):
        catalog_repo.resolve_dir(name)


def test_resolve_rejects_symlink_escaping_the_root(catalog_repo, maps_dir, tmp_path):
    """The name pattern cannot see through a link; the realpath check must."""
    outside = tmp_path / "outside"
    outside.mkdir()
    os.symlink(outside, maps_dir / "escape")

    with pytest.raises(BadRequestError):
        catalog_repo.resolve_dir("escape")


def test_gridmap_path_is_none_without_a_gridmap(catalog_repo):
    assert catalog_repo.gridmap_path("rawonly") is None
    assert catalog_repo.gridmap_path("full").endswith("full/gridmap.pgm")


def test_gridmap_yaml_path_is_none_without_a_yaml(catalog_repo):
    assert catalog_repo.gridmap_yaml_path("rawonly") is None

    path = catalog_repo.gridmap_yaml_path("full")
    assert path.endswith("full/gridmap.yaml")
    # What map_server's LoadMap needs: it resolves the yaml's relative image key
    # against dirname() of the string it was handed, unexpanded.
    assert os.path.isabs(path)
    assert "~" not in path


# --- active_name ------------------------------------------------------------


def test_active_name_from_ini(catalog_repo, tmp_path, monkeypatch):
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    assert catalog_repo.active_name() == "full"


def test_active_name_falls_back_to_the_map_path(catalog_repo, tmp_path, monkeypatch):
    """Older instance files set only the derived paths."""
    ini = tmp_path / "system.ini"
    ini.write_text("[map]\nmap: map/full/gridmap.yaml\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    assert catalog_repo.active_name() == "full"


def test_active_name_is_none_without_an_ini(catalog_repo, tmp_path, monkeypatch):
    """A robot with no INI must still be able to list its maps."""
    monkeypatch.setenv(SYSTEM_INI_ENV, str(tmp_path / "absent.ini"))

    assert catalog_repo.active_name() is None
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_active_name_is_none_without_a_map_section(catalog_repo, tmp_path,
                                                   monkeypatch):
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))

    assert catalog_repo.active_name() is None


# --- write_gridmap ----------------------------------------------------------


def test_write_gridmap_replaces_the_cells(catalog_repo, maps_dir):
    path = maps_dir / "full" / "gridmap.pgm"

    catalog_repo.write_gridmap("full", b"\x00" * 24)

    assert path.read_bytes() == b"P5\n6 4\n255\n" + b"\x00" * 24
    assert catalog_repo.get_map("full").grid.width == 6


def test_write_gridmap_returns_the_file_contents(catalog_repo, maps_dir):
    """The REST layer hashes this for the ETag."""
    written = catalog_repo.write_gridmap("full", b"\x00" * 24)

    assert written == (maps_dir / "full" / "gridmap.pgm").read_bytes()


def test_write_gridmap_creates_the_raw_backup_once(catalog_repo, maps_dir):
    """gridmap_raw.pgm must keep holding the pcd_to_gridmap.py output.

    Re-taking it on every save would, on the second save, replace the pristine
    conversion output with the first save's edit — losing the only way back.
    """
    pristine = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    raw = maps_dir / "full" / "gridmap_raw.pgm"

    catalog_repo.write_gridmap("full", b"\x00" * 24)
    assert raw.read_bytes() == pristine

    catalog_repo.write_gridmap("full", b"\xfe" * 24)
    assert raw.read_bytes() == pristine


def test_write_gridmap_rejects_a_wrong_length_body(catalog_repo, maps_dir):
    path = maps_dir / "full" / "gridmap.pgm"
    before = path.read_bytes()

    with pytest.raises(BadRequestError):
        catalog_repo.write_gridmap("full", b"\x00" * 23)

    assert path.read_bytes() == before


def test_write_gridmap_without_a_gridmap_raises_not_found(catalog_repo, maps_dir):
    """And must not create one: a pgm with no yaml is an invisible half-map."""
    with pytest.raises(NotFoundError):
        catalog_repo.write_gridmap("rawonly", b"\x00" * 24)

    assert not (maps_dir / "rawonly" / "gridmap.pgm").exists()


def test_write_gridmap_rejects_an_unsafe_name(catalog_repo):
    with pytest.raises(BadRequestError):
        catalog_repo.write_gridmap("../etc", b"\x00" * 24)


def test_write_gridmap_leaves_gridmap_yaml_untouched(catalog_repo, maps_dir):
    """The geometry is a property of the conversion, never of a save."""
    yaml_path = maps_dir / "full" / "gridmap.yaml"
    before = yaml_path.read_bytes()

    catalog_repo.write_gridmap("full", b"\x00" * 24)

    assert yaml_path.read_bytes() == before


# --- gridmap_edited / archive_gridmap -----------------------------------------


def test_gridmap_edited_is_false_without_a_raw_snapshot(catalog_repo):
    assert catalog_repo.gridmap_edited("full") is False


def test_gridmap_edited_is_false_when_raw_matches_the_grid(catalog_repo, maps_dir):
    """An operator who opened the editor and saved unchanged cells triggered the
    snapshot without editing anything — existence alone must not count."""
    catalog_repo.write_gridmap("full", b"\xfe" * 24)  # snapshot, cells unchanged

    assert catalog_repo.gridmap_edited("full") is False


def test_gridmap_edited_is_true_when_the_cells_differ(catalog_repo, maps_dir):
    catalog_repo.write_gridmap("full", b"\x00" * 24)

    assert catalog_repo.gridmap_edited("full") is True


def test_archive_gridmap_sets_the_grid_aside(catalog_repo, maps_dir):
    grid = (maps_dir / "full" / "gridmap.pgm").read_bytes()
    meta = (maps_dir / "full" / "gridmap.yaml").read_bytes()

    catalog_repo.archive_gridmap("full")

    # Copies, not moves: the live grid keeps serving until a new conversion
    # atomically replaces it.
    assert (maps_dir / "full" / "gridmap.pgm").read_bytes() == grid
    assert (maps_dir / "full" / "gridmap_prev.pgm").read_bytes() == grid
    assert (maps_dir / "full" / "gridmap_prev.yaml").read_bytes() == meta


def test_archive_gridmap_moves_the_raw_snapshot_aside(catalog_repo, maps_dir):
    """write_gridmap snapshots only-when-absent, so a stale raw left in place
    would claim to be the pristine copy of a grid it may not even match."""
    catalog_repo.write_gridmap("full", b"\x00" * 24)
    raw = (maps_dir / "full" / "gridmap_raw.pgm").read_bytes()

    catalog_repo.archive_gridmap("full")

    assert not (maps_dir / "full" / "gridmap_raw.pgm").exists()
    assert (maps_dir / "full" / "gridmap_prev_raw.pgm").read_bytes() == raw


def test_archive_gridmap_is_a_noop_without_a_grid(catalog_repo, maps_dir):
    catalog_repo.archive_gridmap("rawonly")

    assert not (maps_dir / "rawonly" / "gridmap_prev.pgm").exists()


# --- rename_map_dir -----------------------------------------------------------


def test_rename_map_dir_moves_the_directory_and_its_files(catalog_repo, maps_dir):
    grid = (maps_dir / "full" / "gridmap.pgm").read_bytes()

    new_dir = catalog_repo.rename_map_dir("full", "hall")

    assert new_dir == str(maps_dir / "hall")
    assert not (maps_dir / "full").exists()
    assert (maps_dir / "hall" / "gridmap.pgm").read_bytes() == grid
    assert (maps_dir / "hall" / "map.pcd").is_file()
    # The catalogue follows the directory: same geometry under the new name.
    assert catalog_repo.get_map("full") is None
    assert catalog_repo.get_map("hall").grid is not None


def test_rename_map_dir_leaves_gridmap_yaml_untouched(catalog_repo, maps_dir):
    """No file inside a map names the map, so nothing is rewritten on rename."""
    before = (maps_dir / "full" / "gridmap.yaml").read_bytes()

    catalog_repo.rename_map_dir("full", "hall")

    assert (maps_dir / "hall" / "gridmap.yaml").read_bytes() == before


def test_rename_map_dir_rejects_the_same_name(catalog_repo, maps_dir):
    from syncai_backend.exceptions import BadRequestError

    with pytest.raises(BadRequestError):
        catalog_repo.rename_map_dir("full", "full")
    assert (maps_dir / "full").is_dir()


@pytest.mark.parametrize("new", ["../evil", "a/b", "", ".", "..", "x" * 65])
def test_rename_map_dir_rejects_an_unsafe_new_name(catalog_repo, maps_dir, new):
    from syncai_backend.exceptions import BadRequestError

    with pytest.raises(BadRequestError):
        catalog_repo.rename_map_dir("full", new)
    assert (maps_dir / "full").is_dir()


def test_rename_map_dir_of_a_missing_map_raises_not_found(catalog_repo, maps_dir):
    from syncai_backend.exceptions import NotFoundError

    with pytest.raises(NotFoundError):
        catalog_repo.rename_map_dir("nope", "hall")
    assert not (maps_dir / "hall").exists()


def test_rename_map_dir_refuses_an_existing_target(catalog_repo, maps_dir):
    """POSIX rename onto an empty directory would succeed; the check must not."""
    from syncai_backend.exceptions import ConflictError

    (maps_dir / "empty").mkdir()

    with pytest.raises(ConflictError) as excinfo:
        catalog_repo.rename_map_dir("full", "empty")
    assert excinfo.value.code == "name_taken"
    assert (maps_dir / "full" / "gridmap.pgm").is_file()
    assert list((maps_dir / "empty").iterdir()) == []


# --- keepout ----------------------------------------------------------------


def _zone(zone_id, *points):
    from syncai_backend.repositories.map.catalog import KeepoutZone

    return KeepoutZone(id=zone_id, points=tuple(points))


def _grid_of(catalog_repo, name="full"):
    return catalog_repo.get_map(name).grid


def test_keepout_yaml_path_is_none_without_the_pair(catalog_repo, maps_dir, make_pgm):
    assert catalog_repo.keepout_yaml_path("full") is None

    # A yaml alone is unloadable: filter_mask_server reads the pgm through it.
    (maps_dir / "full" / "keepout.yaml").write_text("image: keepout.pgm\n")
    assert catalog_repo.keepout_yaml_path("full") is None

    make_pgm(maps_dir / "full" / "keepout.pgm", 6, 4, fill=205)
    path = catalog_repo.keepout_yaml_path("full")
    assert path.endswith("full/keepout.yaml")
    assert os.path.isabs(path)
    assert "~" not in path


def test_read_keepout_zones_is_empty_without_a_json(catalog_repo):
    assert catalog_repo.read_keepout_zones("full") == []


def test_write_keepout_writes_the_three_files_and_reads_back(catalog_repo, maps_dir):
    zones = [_zone("a", (-6.9, -11.0), (-6.8, -11.0), (-6.8, -10.9))]

    catalog_repo.write_keepout("full", zones, _grid_of(catalog_repo))

    directory = maps_dir / "full"
    pgm = (directory / "keepout.pgm").read_bytes()
    assert pgm.startswith(b"P5\n6 4\n255\n")
    assert len(pgm) - len(b"P5\n6 4\n255\n") == 24
    assert (directory / "keepout.yaml").read_text().startswith("image: keepout.pgm\n")
    assert "origin: [-6.940000, -11.090000, 0.0]" in (directory / "keepout.yaml").read_text()
    document = json.loads((directory / "keepout.json").read_text())
    assert document["version"] == 1
    assert document["zones"][0]["id"] == "a"
    assert document["zones"][0]["points"][0] == {"x": -6.9, "y": -11.0}

    assert catalog_repo.read_keepout_zones("full") == zones
    assert catalog_repo.keepout_yaml_path("full") is not None


def test_write_keepout_with_no_zones_is_an_all_unknown_mask(catalog_repo, maps_dir):
    catalog_repo.write_keepout("full", [], _grid_of(catalog_repo))

    body = (maps_dir / "full" / "keepout.pgm").read_bytes()[len(b"P5\n6 4\n255\n"):]
    # 205 everywhere: the same blank the nav session writes, and a no-op for
    # the filter -- never 254, which would free every unknown cell.
    assert body == bytes([205]) * 24
    assert json.loads((maps_dir / "full" / "keepout.json").read_text())["zones"] == []


def test_write_keepout_paints_the_zone_black(catalog_repo, maps_dir):
    # Cell (col 1, row 2 from the top) of the 6x4 grid at 0.05 m: its centre.
    x = -6.94 + 1.5 * 0.05
    y = -11.09 + (4 - 2 - 0.5) * 0.05
    zone = _zone("z", (x, y), (x, y), (x, y))
    catalog_repo.write_keepout("full", [zone], _grid_of(catalog_repo))

    body = (maps_dir / "full" / "keepout.pgm").read_bytes()[len(b"P5\n6 4\n255\n"):]
    assert body[2 * 6 + 1] == 0
    assert sum(1 for value in body if value == 0) == 1


def test_write_keepout_rejects_an_unsafe_name(catalog_repo):
    with pytest.raises(BadRequestError):
        catalog_repo.write_keepout("../full", [], _grid_of(catalog_repo))


@pytest.mark.parametrize(
    "text",
    [
        "{not json",
        '{"version": 2, "zones": []}',
        '{"version": 1, "zones": "nope"}',
        "[]",
    ],
)
def test_read_keepout_zones_reports_nothing_for_an_unusable_json(catalog_repo, maps_dir, text):
    (maps_dir / "full" / "keepout.json").write_text(text)

    assert catalog_repo.read_keepout_zones("full") == []


def test_read_keepout_zones_skips_malformed_entries_one_at_a_time(catalog_repo, maps_dir):
    (maps_dir / "full" / "keepout.json").write_text(
        json.dumps(
            {
                "version": 1,
                "zones": [
                    {"id": "ok", "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]},
                    {"id": "short", "points": [{"x": 0, "y": 0}]},
                    {"id": "", "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]},
                    {"points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]},
                    {
                        "id": "bad",
                        "points": [{"x": "a", "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}],
                    },
                    "not a zone",
                ],
            }
        )
    )

    zones = catalog_repo.read_keepout_zones("full")

    assert [zone.id for zone in zones] == ["ok"]
    assert zones[0].points == ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))


def test_rename_map_dir_carries_the_keepout_along(catalog_repo, maps_dir):
    zones = [_zone("a", (-6.9, -11.0), (-6.8, -11.0), (-6.8, -10.9))]
    catalog_repo.write_keepout("full", zones, _grid_of(catalog_repo))
    before = (maps_dir / "full" / "keepout.yaml").read_bytes()

    catalog_repo.rename_map_dir("full", "hall")

    # Nothing in the three files names the map, so nothing was rewritten.
    assert (maps_dir / "hall" / "keepout.yaml").read_bytes() == before
    assert catalog_repo.read_keepout_zones("hall") == zones
    assert catalog_repo.keepout_yaml_path("hall").endswith("hall/keepout.yaml")


# --- import staging ---------------------------------------------------------


def _hidden(maps_dir):
    return sorted(p.name for p in maps_dir.iterdir() if p.name.startswith(".import-"))


def test_list_maps_ignores_import_staging_directories(catalog_repo, maps_dir):
    (maps_dir / ".import-abc123").mkdir()
    (maps_dir / ".import-abc123" / "map.pcd").write_bytes(b"half unpacked")

    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_resolve_dir_refuses_a_staging_name(catalog_repo):
    with pytest.raises(BadRequestError):
        catalog_repo.resolve_dir(".import-abc123")


def test_begin_import_makes_a_hidden_directory_under_maps_dir(catalog_repo, maps_dir):
    staging = catalog_repo.begin_import("copy")

    assert os.path.dirname(staging) == str(maps_dir)
    assert os.path.basename(staging).startswith(".import-")
    assert os.path.isdir(staging)
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_begin_import_validates_the_name_only(catalog_repo):
    # An existing name is allowed -- an import may replace a map.
    assert os.path.isdir(catalog_repo.begin_import("full"))

    with pytest.raises(BadRequestError):
        catalog_repo.begin_import("../escape")


def test_commit_import_publishes_a_new_map(catalog_repo, maps_dir):
    staging = catalog_repo.begin_import("copy")
    (maps_dir / os.path.basename(staging) / "map.pcd").write_bytes(b"new")

    target, displaced = catalog_repo.commit_import("copy", staging)

    assert displaced is None
    assert target == str(maps_dir / "copy")
    assert (maps_dir / "copy" / "map.pcd").read_bytes() == b"new"
    assert _hidden(maps_dir) == []


def test_commit_import_parks_the_map_it_replaces(catalog_repo, maps_dir):
    before = (maps_dir / "rawonly" / "map.pcd").read_bytes()
    staging = catalog_repo.begin_import("rawonly")
    (maps_dir / os.path.basename(staging) / "map.pcd").write_bytes(b"new")

    target, displaced = catalog_repo.commit_import("rawonly", staging)

    assert (maps_dir / "rawonly" / "map.pcd").read_bytes() == b"new"
    assert os.path.basename(displaced).startswith(".import-old-")
    assert (maps_dir / os.path.basename(displaced) / "map.pcd").read_bytes() == before
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_commit_import_refuses_a_path_that_is_not_staging(catalog_repo, maps_dir):
    with pytest.raises(BadRequestError):
        catalog_repo.commit_import("copy", str(maps_dir / "full"))

    assert (maps_dir / "full").is_dir()
    assert not (maps_dir / "copy").exists()


def test_undo_import_restores_the_previous_map(catalog_repo, maps_dir):
    before = (maps_dir / "rawonly" / "map.pcd").read_bytes()
    staging = catalog_repo.begin_import("rawonly")
    (maps_dir / os.path.basename(staging) / "map.pcd").write_bytes(b"new")
    _, displaced = catalog_repo.commit_import("rawonly", staging)

    catalog_repo.undo_import("rawonly", displaced)

    assert (maps_dir / "rawonly" / "map.pcd").read_bytes() == before
    assert _hidden(maps_dir) == []


def test_undo_import_of_a_new_map_removes_it(catalog_repo, maps_dir):
    staging = catalog_repo.begin_import("copy")
    (maps_dir / os.path.basename(staging) / "map.pcd").write_bytes(b"new")
    catalog_repo.commit_import("copy", staging)

    catalog_repo.undo_import("copy", None)

    assert not (maps_dir / "copy").exists()
    assert _hidden(maps_dir) == []


def test_abort_import_removes_a_staging_directory(catalog_repo, maps_dir):
    staging = catalog_repo.begin_import("copy")
    (maps_dir / os.path.basename(staging) / "map.pcd").write_bytes(b"half")

    catalog_repo.abort_import(staging)

    assert _hidden(maps_dir) == []


def test_abort_import_leaves_anything_that_is_not_staging_alone(catalog_repo, maps_dir, tmp_path):
    catalog_repo.abort_import(str(maps_dir / "full"))
    catalog_repo.abort_import(str(tmp_path))

    assert (maps_dir / "full" / "map.pcd").is_file()
    assert tmp_path.is_dir()


def test_free_bytes_is_positive(catalog_repo):
    assert catalog_repo.free_bytes() > 0
