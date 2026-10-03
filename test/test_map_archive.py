"""Tests for the map archive helper: build, inspect, extract.

Pure standard library, real files under tmp_path. The refusal tests build their
archives by hand with zipfile/tarfile so each one perturbs exactly one thing
about an otherwise valid export; ``_manifest`` computes the real md5s so the
only lie in a test archive is the one the test tells.
"""

import gzip
import hashlib
import io
import json
import os
import stat
import tarfile
import zipfile

import pytest

from syncai_backend.helpers.map_archive import (
    MANIFEST_NAME,
    MAX_MEMBERS,
    ArchiveFormat,
    MapArchiveError,
    build_archive,
    extract_archive,
    inspect_archive,
    sniff_format,
)


_FILES = {
    "map.pcd": b"# .PCD v0.7\nDATA ascii\n0 0 0\n",
    "gridmap.pgm": b"P5\n2 1\n255\n\xcd\xfe",
    "patches/000001.pcd": b"patch one",
}
_VERTICES = [
    {"name": "dock", "type": "CHARGER", "x": 1.5, "y": -2.0, "theta": 90.0},
    {"name": "home", "type": "HOME", "x": 0.0, "y": 0.0, "theta": 0.0},
]


def _md5(data):
    return hashlib.md5(data, usedforsecurity=False).hexdigest()


@pytest.fixture
def map_dir(tmp_path):
    root = tmp_path / "full"
    for rel, data in _FILES.items():
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    (root / "traversable_debug").mkdir()
    (root / "traversable_debug" / "step1.pcd").write_bytes(b"debug cloud")
    return root


def _manifest(files=_FILES, vertices=_VERTICES, **overrides):
    document = {
        "format": "syncai-map",
        "version": 1,
        "name": "full",
        "exported_at": "2026-10-03T00:00:00Z",
        "files": {rel: _md5(data) for rel, data in files.items()},
        "vertices": list(vertices),
    }
    document.update(overrides)
    return json.dumps(document).encode()


def _zip(files=_FILES, manifest=None, extra_infos=()):
    """A zip of ``files`` plus a manifest (default: a correct one)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for rel, data in files.items():
            archive.writestr(rel, data)
        for info, data in extra_infos:
            archive.writestr(info, data)
        if manifest is not None:
            archive.writestr(MANIFEST_NAME, manifest)
    return buffer.getvalue()


def _tar(files=_FILES, manifest=None, extra_members=()):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        for rel, data in files.items():
            info = tarfile.TarInfo(rel)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
        for info, data in extra_members:
            archive.addfile(info, io.BytesIO(data) if data is not None else None)
        if manifest is not None:
            info = tarfile.TarInfo(MANIFEST_NAME)
            info.size = len(manifest)
            archive.addfile(info, io.BytesIO(manifest))
    return buffer.getvalue()


def _members(payload):
    if sniff_format(payload) is ArchiveFormat.ZIP:
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            return archive.namelist()
    with tarfile.open(fileobj=io.BytesIO(payload), mode="r:gz") as archive:
        return archive.getnames()


# --- build + inspect + extract round trip ------------------------------------


@pytest.mark.parametrize("fmt", list(ArchiveFormat))
def test_round_trip_reproduces_the_directory(fmt, map_dir, tmp_path):
    payload = build_archive(
        str(map_dir), "full", _VERTICES, fmt,
        exported_at="2026-10-03T00:00:00Z",
        exclude_dirs=("traversable_debug",),
    )

    assert sniff_format(payload) is fmt
    inspected = inspect_archive(payload)
    assert inspected.format is fmt
    assert set(inspected.manifest.files) == set(_FILES)
    assert inspected.manifest.name == "full"
    assert inspected.manifest.exported_at == "2026-10-03T00:00:00Z"
    assert list(inspected.manifest.vertices) == _VERTICES
    assert inspected.total_bytes == sum(len(d) for d in _FILES.values())

    dest = tmp_path / "copy"
    dest.mkdir()
    written = extract_archive(payload, inspected, str(dest))

    assert written == inspected.total_bytes
    for rel, data in _FILES.items():
        assert (dest / rel).read_bytes() == data
    assert not (dest / "traversable_debug").exists()
    # The manifest is archive metadata, not a file of the map.
    assert not (dest / MANIFEST_NAME).exists()


@pytest.mark.parametrize("fmt", list(ArchiveFormat))
def test_manifest_is_the_last_member(fmt, map_dir):
    payload = build_archive(str(map_dir), "full", [], fmt, exported_at="t")

    assert _members(payload)[-1] == MANIFEST_NAME


def test_build_records_the_md5_of_each_file(map_dir):
    payload = build_archive(str(map_dir), "full", [], ArchiveFormat.ZIP, exported_at="t",
                            exclude_dirs=("traversable_debug",))

    manifest = inspect_archive(payload).manifest
    assert manifest.files == {rel: _md5(data) for rel, data in _FILES.items()}


def test_build_skips_symlinks(map_dir, tmp_path):
    outside = tmp_path / "secret.txt"
    outside.write_text("not part of the map")
    os.symlink(outside, map_dir / "link.txt")

    payload = build_archive(str(map_dir), "full", [], ArchiveFormat.TAR_GZ, exported_at="t")

    assert "link.txt" not in inspect_archive(payload).manifest.files


def test_build_without_exclusions_keeps_debug_clouds(map_dir):
    payload = build_archive(str(map_dir), "full", [], ArchiveFormat.ZIP, exported_at="t")

    assert "traversable_debug/step1.pcd" in inspect_archive(payload).manifest.files


def test_build_coerces_vertex_numbers_to_float(map_dir):
    payload = build_archive(
        str(map_dir), "full",
        [{"name": "a", "type": "GENERAL", "x": 1, "y": 2, "theta": 3}],
        ArchiveFormat.ZIP, exported_at="t",
    )

    vertex = inspect_archive(payload).manifest.vertices[0]
    assert vertex == {"name": "a", "type": "GENERAL", "x": 1.0, "y": 2.0, "theta": 3.0}


def test_extract_does_not_apply_archive_modes(tmp_path):
    info = zipfile.ZipInfo("map.pcd")
    info.external_attr = (stat.S_IFREG | 0o777) << 16
    payload = _zip(files={}, extra_infos=[(info, _FILES["map.pcd"])],
                   manifest=_manifest(files={"map.pcd": _FILES["map.pcd"]}))

    dest = tmp_path / "copy"
    dest.mkdir()
    extract_archive(payload, inspect_archive(payload), str(dest))

    mode = stat.S_IMODE(os.stat(dest / "map.pcd").st_mode)
    assert mode & 0o111 == 0


# --- sniff_format --------------------------------------------------------------


def test_sniff_rejects_random_bytes():
    with pytest.raises(MapArchiveError, match="Unsupported"):
        sniff_format(b"\x00\x01\x02 definitely not an archive")


def test_sniff_rejects_a_plain_tar():
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w") as archive:
        info = tarfile.TarInfo("map.pcd")
        info.size = 0
        archive.addfile(info, io.BytesIO(b""))

    with pytest.raises(MapArchiveError, match="Unsupported"):
        sniff_format(buffer.getvalue())


def test_truncated_gzip_is_not_readable():
    payload = _tar(manifest=_manifest())[: len(_tar(manifest=_manifest())) // 2]

    with pytest.raises(MapArchiveError, match="Not a readable"):
        inspect_archive(payload)


def test_gzip_that_is_not_a_tar_is_not_readable():
    payload = gzip.compress(b"just some gzipped text, no tar inside")

    with pytest.raises(MapArchiveError, match="Not a readable"):
        inspect_archive(payload)


# --- manifest ------------------------------------------------------------------


@pytest.mark.parametrize("make", [_zip, _tar])
def test_missing_manifest_is_refused(make):
    with pytest.raises(MapArchiveError, match=f"No {MANIFEST_NAME}"):
        inspect_archive(make(manifest=None))


def test_invalid_json_manifest_is_refused():
    with pytest.raises(MapArchiveError, match="not valid JSON"):
        inspect_archive(_zip(manifest=b"{not json"))


def test_non_object_manifest_is_refused():
    with pytest.raises(MapArchiveError, match="JSON object"):
        inspect_archive(_zip(manifest=b"[1, 2, 3]"))


def test_wrong_format_marker_is_refused():
    with pytest.raises(MapArchiveError, match="not a 'syncai-map' manifest"):
        inspect_archive(_zip(manifest=_manifest(format="something-else")))


def test_newer_version_is_refused_with_a_pointer():
    with pytest.raises(MapArchiveError, match="newer backend"):
        inspect_archive(_zip(manifest=_manifest(version=2)))


def test_unknown_version_is_refused():
    with pytest.raises(MapArchiveError, match="Unsupported manifest version"):
        inspect_archive(_zip(manifest=_manifest(version="1")))


def test_manifest_without_a_name_is_refused():
    with pytest.raises(MapArchiveError, match="no map name"):
        inspect_archive(_zip(manifest=_manifest(name="")))


def test_files_must_be_an_object():
    manifest = json.loads(_manifest())
    manifest["files"] = list(manifest["files"])

    with pytest.raises(MapArchiveError, match="files table"):
        inspect_archive(_zip(manifest=json.dumps(manifest).encode()))


def test_md5_must_be_32_hex():
    bad = {rel: _md5(d) for rel, d in _FILES.items()}
    bad["map.pcd"] = "DEADBEEF"
    manifest = json.loads(_manifest())
    manifest["files"] = bad

    with pytest.raises(MapArchiveError, match="no valid md5"):
        inspect_archive(_zip(manifest=json.dumps(manifest).encode()))


def test_manifest_may_not_list_itself():
    manifest = json.loads(_manifest())
    manifest["files"][MANIFEST_NAME] = "0" * 32

    with pytest.raises(MapArchiveError, match="must not list itself"):
        inspect_archive(_zip(manifest=json.dumps(manifest).encode()))


def test_manifest_may_not_list_an_unsafe_path():
    manifest = json.loads(_manifest())
    manifest["files"]["../escape"] = "0" * 32

    with pytest.raises(MapArchiveError, match="unsafe path"):
        inspect_archive(_zip(manifest=json.dumps(manifest).encode()))


def test_vertices_must_be_present():
    manifest = json.loads(_manifest())
    del manifest["vertices"]

    with pytest.raises(MapArchiveError, match="vertices list"):
        inspect_archive(_zip(manifest=json.dumps(manifest).encode()))


@pytest.mark.parametrize(
    "vertex, fragment",
    [
        ({"name": "", "type": "GENERAL", "x": 0, "y": 0, "theta": 0}, "has no name"),
        ({"name": "a", "type": "", "x": 0, "y": 0, "theta": 0}, "has no type"),
        ({"name": "a", "type": "GENERAL", "x": 0, "y": 0}, "finite theta"),
        ({"name": "a", "type": "GENERAL", "x": "1", "y": 0, "theta": 0}, "finite x"),
        ({"name": "a", "type": "GENERAL", "x": True, "y": 0, "theta": 0}, "finite x"),
        ("not an object", "not an object"),
    ],
)
def test_malformed_vertices_are_refused(vertex, fragment):
    with pytest.raises(MapArchiveError, match=fragment):
        inspect_archive(_zip(manifest=_manifest(vertices=[vertex])))


def test_nan_coordinates_are_refused():
    # json.dumps writes NaN as a bare token Python's loader accepts, so a
    # manifest can carry one; the check has to be explicit.
    manifest = _manifest(vertices=[{"name": "a", "type": "GENERAL",
                                    "x": float("nan"), "y": 0, "theta": 0}])

    with pytest.raises(MapArchiveError, match="finite x"):
        inspect_archive(_zip(manifest=manifest))


def test_duplicate_vertex_names_are_refused():
    twice = [_VERTICES[0], dict(_VERTICES[0], x=9.0)]

    with pytest.raises(MapArchiveError, match="twice"):
        inspect_archive(_zip(manifest=_manifest(vertices=twice)))


def test_unknown_manifest_keys_are_tolerated():
    payload = _zip(manifest=_manifest(robot_id="robot01", future_key={"a": 1}))

    assert inspect_archive(payload).manifest.name == "full"


# --- set mismatch ---------------------------------------------------------------


@pytest.mark.parametrize("make", [_zip, _tar])
def test_unlisted_member_is_refused(make):
    files = dict(_FILES, **{"extra.bin": b"smuggled"})

    with pytest.raises(MapArchiveError, match="Not listed in the manifest: 'extra.bin'"):
        inspect_archive(make(files=files, manifest=_manifest()))


@pytest.mark.parametrize("make", [_zip, _tar])
def test_listed_member_that_is_absent_is_refused(make):
    files = {rel: d for rel, d in _FILES.items() if rel != "gridmap.pgm"}

    with pytest.raises(MapArchiveError, match="missing from the archive: 'gridmap.pgm'"):
        inspect_archive(make(files=files, manifest=_manifest()))


@pytest.mark.parametrize("make", [_zip, _tar])
def test_md5_mismatch_is_caught_on_extract(make, tmp_path):
    files = dict(_FILES, **{"map.pcd": b"altered after export"})
    payload = make(files=files, manifest=_manifest())  # manifest hashes the originals
    inspected = inspect_archive(payload)  # sizes and names still agree

    dest = tmp_path / "copy"
    dest.mkdir()
    with pytest.raises(MapArchiveError, match="md5 mismatch for 'map.pcd'"):
        extract_archive(payload, inspected, str(dest))


def test_duplicate_member_names_are_refused():
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for rel, data in _FILES.items():
            archive.writestr(rel, data)
        archive.writestr("map.pcd", b"second copy")
        archive.writestr(MANIFEST_NAME, _manifest())

    with pytest.raises(MapArchiveError, match="twice"):
        inspect_archive(buffer.getvalue())


# --- unsafe members --------------------------------------------------------------


@pytest.mark.parametrize(
    "bad_name",
    ["../evil", "/abs/path", "a/./b", "a//b", "a\\b", "./map.pcd", "patches/../x"],
)
def test_unsafe_zip_member_names_are_refused(bad_name):
    payload = _zip(extra_infos=[(bad_name, b"x")], manifest=_manifest())

    with pytest.raises(MapArchiveError, match="Archive member|Unsafe"):
        inspect_archive(payload)


@pytest.mark.parametrize("bad_name", ["../evil", "/abs/path", "a/../b"])
def test_unsafe_tar_member_names_are_refused(bad_name):
    info = tarfile.TarInfo(bad_name)
    info.size = 1
    payload = _tar(extra_members=[(info, b"x")], manifest=_manifest())

    with pytest.raises(MapArchiveError, match="Archive member"):
        inspect_archive(payload)


def test_unsafe_directory_entry_is_refused_too():
    payload = _zip(extra_infos=[("../out/", b"")], manifest=_manifest())

    with pytest.raises(MapArchiveError, match="Archive member"):
        inspect_archive(payload)


@pytest.mark.parametrize("tartype", [tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.CHRTYPE,
                                     tarfile.FIFOTYPE])
def test_tar_links_and_devices_are_refused(tartype):
    info = tarfile.TarInfo("sneaky")
    info.type = tartype
    info.linkname = "/etc/passwd"
    payload = _tar(extra_members=[(info, None)], manifest=_manifest())

    with pytest.raises(MapArchiveError, match="link, device or encrypted"):
        inspect_archive(payload)


def test_zip_symlink_is_refused():
    info = zipfile.ZipInfo("sneaky")
    info.external_attr = (stat.S_IFLNK | 0o777) << 16
    payload = _zip(extra_infos=[(info, b"/etc/passwd")], manifest=_manifest())

    with pytest.raises(MapArchiveError, match="link, device or encrypted"):
        inspect_archive(payload)


def test_zip_encrypted_entry_is_refused():
    # zipfile clears flag_bits on write, so the "encrypted" bit has to be set
    # in the bytes afterwards: bit 0 of the general-purpose flags, at offset 6
    # of the first local header and offset 8 of the first central entry --
    # both describe map.pcd, the first member written.
    payload = bytearray(_zip(manifest=_manifest()))
    local = payload.find(b"PK\x03\x04")
    central = payload.find(b"PK\x01\x02")
    payload[local + 6] |= 0x1
    payload[central + 8] |= 0x1

    with pytest.raises(MapArchiveError, match="link, device or encrypted"):
        inspect_archive(bytes(payload))


def test_windows_zip_entries_with_no_mode_bits_are_files(tmp_path):
    """An MS-DOS writer leaves external_attr at 0; that is still a file."""
    infos = []
    for rel, data in _FILES.items():
        info = zipfile.ZipInfo(rel)
        info.create_system = 0
        info.external_attr = 0
        infos.append((info, data))
    payload = _zip(files={}, extra_infos=infos, manifest=_manifest())

    inspected = inspect_archive(payload)
    dest = tmp_path / "copy"
    dest.mkdir()
    extract_archive(payload, inspected, str(dest))

    assert (dest / "map.pcd").read_bytes() == _FILES["map.pcd"]


def test_directory_entries_are_ignored_not_refused():
    payload = _zip(extra_infos=[("patches/", b"")], manifest=_manifest())

    assert set(inspect_archive(payload).manifest.files) == set(_FILES)


def test_too_many_members_is_refused():
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for index in range(MAX_MEMBERS + 1):
            archive.writestr(f"d/{index}", b"")

    with pytest.raises(MapArchiveError, match="members"):
        inspect_archive(buffer.getvalue())


def test_total_bytes_sums_only_listed_files():
    payload = _zip(extra_infos=[("patches/", b"")], manifest=_manifest())

    assert inspect_archive(payload).total_bytes == sum(len(d) for d in _FILES.values())
