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

from syncai_backend.exceptions import BadRequestError  # noqa: E402
from syncai_backend.helpers.system_config import (  # noqa: E402
    SYSTEM_INI_ENV,
)
from syncai_backend.repositories.map import catalog as catalog_module  # noqa: E402
from syncai_backend.repositories.map.catalog import (  # noqa: E402
    MapCleanStatus,
    init_map_catalog_repo,
)


def _names(maps):
    return [stored.name for stored in maps]


# --- list_maps --------------------------------------------------------------


def test_lists_directories_only(catalog_repo):
    """The loose stray.pgm at the root is not a map under this layout."""
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_a_legacy_gridmap_is_listed_as_a_plain_file(
    catalog_repo, maps_dir, make_pgm, make_gridmap_yaml
):
    """A map saved before 2026-10-28 may still carry a 2D gridmap. Nothing
    reads it any more; it only counts toward the map's size."""
    before = catalog_repo.get_map("full").size_bytes
    make_pgm(maps_dir / "full" / "gridmap.pgm", 6, 4)
    make_gridmap_yaml(maps_dir / "full" / "gridmap.yaml")

    stored = catalog_repo.get_map("full")

    assert not hasattr(stored, "grid")
    assert stored.size_bytes > before


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


def test_exists_agrees_with_get_map(catalog_repo):
    """The cheap yes/no and the full read must never disagree about a name."""
    for name in ("full", "rawonly", "nosuchmap"):
        assert catalog_repo.exists(name) is (catalog_repo.get_map(name) is not None)


def test_exists_does_not_read_the_map(catalog_repo, monkeypatch):
    """One isdir: no walk of the map's files, no sidecar."""
    monkeypatch.setattr(
        catalog_repo, "_read", lambda *a: pytest.fail("exists read the map")
    )
    assert catalog_repo.exists("full") is True


@pytest.mark.parametrize("bad", ["../etc", ".import-abc", "a/b"])
def test_exists_refuses_what_get_map_refuses(catalog_repo, bad):
    with pytest.raises(BadRequestError):
        catalog_repo.exists(bad)


def test_missing_maps_dir_lists_empty(logger, tmp_path):
    repo = init_map_catalog_repo(logger=logger)
    repo.maps_dir = str(tmp_path / "absent")

    assert repo.list_maps() == []


# --- resolve_dir ----------------------------------------------------------------


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


# --- active_name ------------------------------------------------------------


def test_active_name_from_ini(catalog_repo, tmp_path, monkeypatch):
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: full\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    catalog_repo.load_active()

    assert catalog_repo.active_name() == "full"


def test_active_name_falls_back_to_the_map_path(catalog_repo, tmp_path, monkeypatch):
    """Older instance files set only the derived paths."""
    ini = tmp_path / "system.ini"
    ini.write_text("[map]\nmap: map/full/gridmap.yaml\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    catalog_repo.load_active()

    assert catalog_repo.active_name() == "full"


def test_active_name_is_none_without_an_ini(catalog_repo, tmp_path, monkeypatch):
    """A robot with no INI must still be able to list its maps."""
    monkeypatch.setenv(SYSTEM_INI_ENV, str(tmp_path / "absent.ini"))
    catalog_repo.load_active()

    assert catalog_repo.active_name() is None
    assert _names(catalog_repo.list_maps()) == ["full", "rawonly"]


def test_active_name_is_none_without_a_map_section(catalog_repo, tmp_path,
                                                   monkeypatch):
    ini = tmp_path / "system.ini"
    ini.write_text("[system]\nrobot_id: robot01\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    catalog_repo.load_active()

    assert catalog_repo.active_name() is None


def _ini_naming(tmp_path, monkeypatch, name):
    ini = tmp_path / "system.ini"
    ini.write_text(f"[system]\nrobot_id: robot01\n\n[map]\nname: {name}\n")
    monkeypatch.setenv(SYSTEM_INI_ENV, str(ini))
    return ini


def test_active_name_is_read_when_the_repo_is_built(logger, tmp_path, monkeypatch):
    """At startup, so no request ever waits on the INI."""
    _ini_naming(tmp_path, monkeypatch, "full")

    assert init_map_catalog_repo(logger=logger).active_name() == "full"


def test_active_name_does_not_read_the_ini_again(catalog_repo, tmp_path, monkeypatch):
    """A hand edit is seen at the next restart, as README *Gotchas* says."""
    ini = _ini_naming(tmp_path, monkeypatch, "full")
    catalog_repo.load_active()
    ini.write_text("[system]\nrobot_id: robot01\n\n[map]\nname: rawonly\n")

    def _no_read(logger):
        raise AssertionError("active_name read the INI")

    monkeypatch.setattr(catalog_module, "active_map_name", _no_read)

    assert catalog_repo.active_name() == "full"


def test_set_active_writes_the_ini_and_then_answers_from_it(
    catalog_repo, tmp_path, monkeypatch
):
    ini = _ini_naming(tmp_path, monkeypatch, "full")
    catalog_repo.load_active()

    assert catalog_repo.set_active("rawonly") == str(ini)

    assert catalog_repo.active_name() == "rawonly"
    assert "name: rawonly" in ini.read_text()


def test_a_failed_set_active_keeps_the_previous_map(catalog_repo, tmp_path, monkeypatch):
    """The next boot loads what the INI says; the cache must not run ahead."""
    _ini_naming(tmp_path, monkeypatch, "full")
    catalog_repo.load_active()

    def _boom(name, logger):
        raise OSError("read-only file system")

    monkeypatch.setattr(catalog_module, "set_active_map", _boom)

    with pytest.raises(OSError):
        catalog_repo.set_active("rawonly")
    assert catalog_repo.active_name() == "full"


# --- rename_map_dir -----------------------------------------------------------


def test_rename_map_dir_moves_the_directory_and_its_files(catalog_repo, maps_dir):
    tree = (maps_dir / "full" / "octomap.bt").read_bytes()

    new_dir = catalog_repo.rename_map_dir("full", "hall")

    assert new_dir == str(maps_dir / "hall")
    assert not (maps_dir / "full").exists()
    assert (maps_dir / "hall" / "octomap.bt").read_bytes() == tree
    assert (maps_dir / "hall" / "map.pcd").is_file()
    # The catalogue follows the directory: the same tree under the new name.
    assert catalog_repo.get_map("full") is None
    assert catalog_repo.get_map("hall").octomap is not None


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
    assert (maps_dir / "full" / "octomap.bt").is_file()
    assert list((maps_dir / "empty").iterdir()) == []


# --- keepout ----------------------------------------------------------------


def _zone(zone_id, *points):
    from syncai_backend.repositories.map.catalog import KeepoutZone

    return KeepoutZone(id=zone_id, points=tuple(points))


def test_read_keepout_zones_is_empty_without_a_json(catalog_repo):
    assert catalog_repo.read_keepout_zones("full") == []


def test_write_keepout_writes_the_json_alone_and_reads_back(catalog_repo, maps_dir):
    zones = [_zone("a", (-6.9, -11.0), (-6.8, -11.0), (-6.8, -10.9))]

    catalog_repo.write_keepout("full", zones)

    directory = maps_dir / "full"
    # No 2D mask: keepout.pgm / keepout.yaml went with the 2D stack.
    assert not (directory / "keepout.pgm").exists()
    assert not (directory / "keepout.yaml").exists()
    document = json.loads((directory / "keepout.json").read_text())
    assert document["version"] == 1
    assert document["zones"][0]["id"] == "a"
    assert document["zones"][0]["points"][0] == {"x": -6.9, "y": -11.0}

    assert catalog_repo.read_keepout_zones("full") == zones
    assert catalog_repo.keepout_json_path("full").endswith("full/keepout.json")


def test_write_keepout_with_no_zones_rewrites_the_json_empty(catalog_repo, maps_dir):
    catalog_repo.write_keepout("full", [_zone("a", (0, 0), (1, 0), (1, 1))])

    catalog_repo.write_keepout("full", [])

    document = json.loads((maps_dir / "full" / "keepout.json").read_text())
    assert document == {"version": 1, "zones": []}


def test_keepout_name_and_band_round_trip_and_unset_ones_are_omitted(catalog_repo, maps_dir):
    from syncai_backend.repositories.map.catalog import KeepoutZone

    zones = [
        KeepoutZone(
            id="a", points=((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)), name="rack",
            z_min=4.5, z_max=5.5,
        ),
        _zone("b", (0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
    ]
    catalog_repo.write_keepout("full", zones)

    document = json.loads((maps_dir / "full" / "keepout.json").read_text())
    assert document["zones"][0]["name"] == "rack"
    assert document["zones"][0]["z_min"] == 4.5
    assert set(document["zones"][1]) == {"id", "points"}
    assert catalog_repo.read_keepout_zones("full") == zones


def test_read_keepout_takes_null_as_absent_and_skips_a_bad_band(catalog_repo, maps_dir):
    points = [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}]
    (maps_dir / "full" / "keepout.json").write_text(
        json.dumps(
            {
                "version": 1,
                "zones": [
                    {"id": "null", "points": points, "name": None, "z_min": None, "z_max": 1},
                    {"id": "upside", "points": points, "z_min": 2, "z_max": 1},
                    {"id": "text", "points": points, "z_min": "low"},
                    {"id": "flag", "points": points, "z_max": True},
                ],
            }
        )
    )

    zones = catalog_repo.read_keepout_zones("full")

    assert [zone.id for zone in zones] == ["null"]
    assert zones[0].name is None
    assert zones[0].z_min is None
    assert zones[0].z_max == 1.0


def test_write_keepout_rejects_an_unsafe_name(catalog_repo):
    with pytest.raises(BadRequestError):
        catalog_repo.write_keepout("../full", [])


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
    catalog_repo.write_keepout("full", zones)
    before = (maps_dir / "full" / "keepout.json").read_bytes()

    catalog_repo.rename_map_dir("full", "hall")

    # Nothing in the file names the map, so nothing was rewritten.
    assert (maps_dir / "hall" / "keepout.json").read_bytes() == before
    assert catalog_repo.read_keepout_zones("hall") == zones
    assert catalog_repo.keepout_json_path("hall").endswith("hall/keepout.json")


# --- octomap ----------------------------------------------------------------
#
# The robot's 3D map. The catalogue reads the tree's header and the cleaning
# sidecar; it never parses the tree and never writes it.


def _plant_clean(directory, payload):
    (directory / "map_clean.recipe.json").write_text(json.dumps(payload), encoding="utf-8")


def test_octomap_is_none_without_a_tree(catalog_repo):
    assert catalog_repo.get_map("rawonly").octomap is None
    assert catalog_repo.octomap_path("rawonly") is None


def test_octomap_reads_the_header_and_versions_the_file(catalog_repo, maps_dir, make_bt):
    tree = make_bt(maps_dir / "full" / "octomap.bt", res=0.05)

    info = catalog_repo.get_map("full").octomap

    assert info is not None
    assert info.resolution == pytest.approx(0.05)
    assert info.size_bytes == tree.stat().st_size
    assert info.version == f"{tree.stat().st_size}-{tree.stat().st_mtime_ns}"
    assert catalog_repo.octomap_path("full") == str(tree)


def test_octomap_resolution_is_none_for_a_header_this_build_cannot_read(
    catalog_repo, maps_dir, make_bt
):
    """Present is still present (the file is served as-is); only ``res`` is unknown."""
    (maps_dir / "full" / "octomap.bt").write_bytes(b"not a tree")
    assert catalog_repo.get_map("full").octomap.resolution is None

    make_bt(maps_dir / "full" / "octomap.bt", res=None)
    assert catalog_repo.get_map("full").octomap.resolution is None


def test_octomap_version_changes_when_the_file_is_replaced(catalog_repo, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt", body=b"\x02\x00")
    before = catalog_repo.get_map("full").octomap.version

    make_bt(maps_dir / "full" / "octomap.bt", body=b"\x02\x00\x02\x00")

    assert catalog_repo.get_map("full").octomap.version != before


@pytest.mark.parametrize(
    "status, started_ago_s, running",
    [
        ("converting", 60, True),
        ("converting", None, True),  # nothing to age it by
        ("converting", 31 * 60, False),  # a clean killed hard
        ("ok", 60, False),
        ("failed", 60, False),
    ],
)
def test_map_clean_running_reads_the_sidecar(
    catalog_repo, maps_dir, status, started_ago_s, running
):
    from datetime import datetime, timedelta, timezone

    document = {"status": status, "params": {"octomap": True}}
    if started_ago_s is not None:
        started = datetime.now(timezone.utc) - timedelta(seconds=started_ago_s)
        document["started_at"] = started.isoformat().replace("+00:00", "Z")
    _plant_clean(maps_dir / "rawonly", document)

    assert catalog_repo.map_clean_running("rawonly") is running


def test_map_clean_running_is_false_without_a_sidecar(catalog_repo):
    assert catalog_repo.map_clean_running("rawonly") is False


def test_map_clean_record_is_none_without_a_sidecar(catalog_repo):
    assert catalog_repo.get_map("full").map_clean_record is None


def test_map_clean_record_reads_the_tree_fields(catalog_repo, maps_dir):
    _plant_clean(
        maps_dir / "full",
        {
            "status": "ok",
            "started_at": "2026-10-13T02:49:28Z",
            "params": {"voxel_size": 0.1, "octomap": True},
            "measurements": {"octomap_written": False, "octomap_error": "out of memory"},
        },
    )

    record = catalog_repo.get_map("full").map_clean_record

    assert record.status is MapCleanStatus.OK
    assert record.started_at == "2026-10-13T02:49:28Z"
    assert record.octomap_requested is True
    assert record.octomap_written is False
    assert record.octomap_error == "out of memory"
    assert record.error is None


def test_map_clean_record_defaults_the_tree_fields(catalog_repo, maps_dir):
    """A clean from before the tree existed, or one that never asked for it."""
    _plant_clean(maps_dir / "full", {"status": "failed", "error": "no patches"})

    record = catalog_repo.get_map("full").map_clean_record

    assert record.status is MapCleanStatus.FAILED
    assert record.octomap_requested is False
    assert record.octomap_written is False
    assert record.octomap_error is None
    assert record.error == "no patches"


@pytest.mark.parametrize("text", ['{"status": "ok"', "[]", '{"status": "sideways"}', ""])
def test_map_clean_record_is_none_for_an_unusable_sidecar(catalog_repo, maps_dir, text):
    (maps_dir / "full" / "map_clean.recipe.json").write_text(text)

    assert catalog_repo.get_map("full").map_clean_record is None


def test_read_octomap_edits_is_none_without_a_file(catalog_repo):
    assert catalog_repo.read_octomap_edits("full") is None


def test_write_octomap_edits_round_trips_in_the_viewer_shape(catalog_repo, maps_dir):
    keys = [(32770, 32768, 32763), (32771, 32768, 32763)]

    written = catalog_repo.write_octomap_edits("full", keys, 0.1)
    document = json.loads((maps_dir / "full" / "octomap_edits.json").read_text())

    assert written.fill_key == tuple(keys)
    assert written.created_at.endswith("Z")
    assert document["format"] == "syncai-octomap-edits"
    assert document["version"] == 1
    assert document["res"] == pytest.approx(0.1)
    assert document["source"] == "octomap.bt"
    assert document["fill_key"] == [list(key) for key in keys]
    # The metric centres, for a reader without the key arithmetic.
    assert document["fill_xyz"][0] == pytest.approx([0.25, 0.05, -0.45])
    assert catalog_repo.read_octomap_edits("full") == written


def test_write_octomap_edits_puts_one_cell_per_line(catalog_repo, maps_dir):
    catalog_repo.write_octomap_edits("full", [(1, 2, 3), (4, 5, 6)], 0.1)

    lines = (maps_dir / "full" / "octomap_edits.json").read_text().splitlines()

    assert lines[0] == "{"
    assert '  "format": "syncai-octomap-edits",' in lines
    assert "    [1, 2, 3]," in lines
    assert "    [4, 5, 6]" in lines


def test_write_octomap_edits_with_no_keys_clears_them(catalog_repo, maps_dir):
    catalog_repo.write_octomap_edits("full", [(1, 2, 3)], 0.1)

    catalog_repo.write_octomap_edits("full", [], 0.1)

    assert catalog_repo.read_octomap_edits("full").fill_key == ()
    assert (maps_dir / "full" / "octomap_edits.json").exists()


@pytest.mark.parametrize(
    "text",
    [
        "{",
        "[]",
        '{"format": "other", "version": 1, "res": 0.1, "fill_key": []}',
        '{"format": "syncai-octomap-edits", "version": 2, "res": 0.1, "fill_key": []}',
        '{"format": "syncai-octomap-edits", "version": 1, "res": "0.1", "fill_key": []}',
        '{"format": "syncai-octomap-edits", "version": 1, "res": 0.1, "fill_key": {}}',
    ],
)
def test_read_octomap_edits_reports_none_for_an_unusable_file(catalog_repo, maps_dir, text):
    (maps_dir / "full" / "octomap_edits.json").write_text(text)

    assert catalog_repo.read_octomap_edits("full") is None


def test_read_octomap_edits_skips_malformed_keys_one_at_a_time(catalog_repo, maps_dir):
    (maps_dir / "full" / "octomap_edits.json").write_text(
        json.dumps(
            {
                "format": "syncai-octomap-edits",
                "version": 1,
                "res": 0.1,
                "fill_key": [
                    [1, 2],
                    [1, 2, 70000],
                    [1, 2, "3"],
                    [1, 2, 3.0],
                    [True, 2, 3],
                    [4, 5, 6],
                ],
            }
        )
    )

    assert catalog_repo.read_octomap_edits("full").fill_key == ((4, 5, 6),)


def test_write_octomap_edits_rejects_an_unsafe_name(catalog_repo):
    with pytest.raises(BadRequestError):
        catalog_repo.write_octomap_edits("../full", [], 0.1)


def test_rename_map_dir_carries_the_tree_and_its_patches_along(catalog_repo, maps_dir, make_bt):
    make_bt(maps_dir / "full" / "octomap.bt")
    catalog_repo.write_octomap_edits("full", [(1, 2, 3)], 0.1)
    before = (maps_dir / "full" / "octomap_edits.json").read_bytes()

    catalog_repo.rename_map_dir("full", "hall")

    # Neither file names the map, so neither was rewritten.
    assert catalog_repo.octomap_path("hall").endswith("hall/octomap.bt")
    assert (maps_dir / "hall" / "octomap_edits.json").read_bytes() == before
    assert catalog_repo.read_octomap_edits("hall").fill_key == ((1, 2, 3),)


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
