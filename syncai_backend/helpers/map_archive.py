"""Checksummed ``.zip`` / ``.tar.gz`` archives of a map directory.

The transport format for ``GET /api/v1/maps/{name}/export`` and
``POST /api/v1/maps/import``: the files under ``map/<name>/`` stored relative to
the map root, plus one manifest, ``syncai_map.json``, carrying an md5 for every
file, the map's name and its vertices. The manifest is archive metadata — it is
never written into the map directory and never lists itself — so a round trip
leaves the directory byte-identical and a re-export has nothing to exclude.

This module knows nothing about maps, ROS, repositories or FastAPI. It is "a
directory archive with a manifest", and every refusal is a ``MapArchiveError``
whose message is the sentence the operator reads; the router decides the status
code, the same contract ``helpers/pgm.py`` has with its ``ValueError``.

**Inspect, then extract.** ``inspect_archive`` validates every member's name and
type and parses and cross-checks the manifest *without writing anything*, so an
archive that was not produced by this API — or was altered since — is refused
before a byte reaches the maps directory. ``extract_archive`` then streams only
the files the manifest lists, hashing as it writes, and refuses on the first
mismatch. Neither ever calls ``extractall``/``extract``: on Python 3.10 those
honour absolute names, create links and apply modes and owners from the archive
(``tarfile.data_filter`` only arrived in 3.12). ``ZipFile.open`` and
``TarFile.extractfile`` hand back bytes and nothing else.
"""

import hashlib
import io
import json
import math
import os
import re
import stat
import tarfile
import time
import zipfile
import zlib
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

MANIFEST_NAME = "syncai_map.json"
MANIFEST_FORMAT = "syncai-map"
MANIFEST_VERSION = 1

# A zip of a million empty entries is a few kilobytes on the wire and a million
# name checks here; a map directory has a few hundred patches at most.
MAX_MEMBERS = 50_000
# The manifest carries a few hundred vertices at most; anything larger is not
# ours.
MAX_MANIFEST_BYTES = 16 << 20

_CHUNK = 1 << 20
_MD5_RE = re.compile(r"^[0-9a-f]{32}$")
_VERTEX_NUMBERS = ("x", "y", "theta")


class ArchiveFormat(str, Enum):
    """The two containers an export comes in. ``str`` mixin, not ``StrEnum``:
    the runtime is 3.10, and FastAPI needs the mixin to accept the value as a
    query parameter.
    """

    ZIP = "zip"
    TAR_GZ = "tar.gz"

    @property
    def media_type(self) -> str:
        return "application/zip" if self is ArchiveFormat.ZIP else "application/gzip"

    @property
    def extension(self) -> str:
        return self.value


class MapArchiveError(ValueError):
    """The archive cannot be used; the message is the sentence for the operator."""


@dataclass(frozen=True)
class ArchiveManifest:
    """What ``syncai_map.json`` says, after shape-checking.

    ``files`` maps a posix relpath from the map root to the lowercase hex md5 of
    that file's bytes. ``vertices`` are plain dicts of ``name``/``type``/``x``/
    ``y``/``theta`` with the numbers already coerced to float; ``type`` is
    passed through as a string because which types exist is the caller's
    vocabulary, not this module's.
    """

    name: str
    exported_at: str
    files: Dict[str, str]
    vertices: Tuple[Dict[str, object], ...]


@dataclass(frozen=True)
class InspectedArchive:
    """The outcome of ``inspect_archive``: safe to hand to ``extract_archive``."""

    format: ArchiveFormat
    manifest: ArchiveManifest
    # Sum of the declared, uncompressed sizes of the files the manifest lists.
    # Authoritative for both containers (a tar member's size delimits the
    # stream; a ZipExtFile stops at file_size), so a caller can compare it with
    # free disk before anything is written.
    total_bytes: int


def _md5():
    # An integrity tag, not a signature: ``usedforsecurity=False`` is what keeps
    # md5 constructible on a FIPS-mode kernel.
    return hashlib.md5(usedforsecurity=False)


# --- Building ---------------------------------------------------------------


def _collect_files(map_dir: str, exclude_dirs: Sequence[str]) -> List[Tuple[str, str]]:
    """Every regular file under ``map_dir`` as ``(relpath, abspath)``, sorted.

    Symlinks are skipped (a link out of the map directory must not pull its
    target into the archive), as are the top-level directories in
    ``exclude_dirs`` and a stray manifest left over from somewhere.
    """
    root = os.path.realpath(map_dir)
    found: List[Tuple[str, str]] = []
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        rel_dir = os.path.relpath(dirpath, root)
        if rel_dir == ".":
            rel_dir = ""
        dirnames[:] = sorted(
            d
            for d in dirnames
            if not os.path.islink(os.path.join(dirpath, d))
            and not (rel_dir == "" and d in exclude_dirs)
        )
        for filename in sorted(filenames):
            full = os.path.join(dirpath, filename)
            if os.path.islink(full) or not os.path.isfile(full):
                continue
            rel = filename if not rel_dir else f"{rel_dir}/{filename}"
            rel = rel.replace(os.sep, "/")
            if rel == MANIFEST_NAME:
                continue
            found.append((rel, full))
    return sorted(found)


def _manifest_bytes(
    name: str,
    exported_at: str,
    files: Mapping[str, str],
    vertices: Sequence[Mapping[str, object]],
) -> bytes:
    document = {
        "format": MANIFEST_FORMAT,
        "version": MANIFEST_VERSION,
        "name": name,
        "exported_at": exported_at,
        "files": {rel: files[rel] for rel in sorted(files)},
        "vertices": [
            {
                "name": str(vertex["name"]),
                "type": str(vertex["type"]),
                "x": float(vertex["x"]),
                "y": float(vertex["y"]),
                "theta": float(vertex["theta"]),
            }
            for vertex in vertices
        ],
    }
    return (json.dumps(document, indent=2) + "\n").encode("utf-8")


class _HashingReader:
    """A ``read(n)`` wrapper that feeds a hash with everything it hands out.

    ``TarFile.addfile`` pulls the member's bytes through ``read``, so wrapping
    the source is how one pass over the file both hashes it and stores it.
    """

    def __init__(self, source, digest):
        self._source = source
        self._digest = digest

    def read(self, size: int = -1) -> bytes:
        chunk = self._source.read(size)
        self._digest.update(chunk)
        return chunk


def build_archive(
    map_dir: str,
    name: str,
    vertices: Sequence[Mapping[str, object]],
    fmt: ArchiveFormat,
    exported_at: str,
    exclude_dirs: Sequence[str] = (),
) -> bytes:
    """Archive ``map_dir`` with its manifest and return the bytes.

    Each file is read once: hashed and written in the same pass, so the md5
    recorded is of the bytes actually stored — a hash-then-add pair of reads
    could record one generation of ``gridmap.pgm`` and store the next.

    No top-level ``<name>/`` folder: the name is an import-time choice (the
    import route takes ``?name=``), so a folder would carry a label the import
    may override. The manifest carries it, and goes in **last**.

    Members carry no owner and a fixed ``0o644`` (tar) or whatever the zip
    writer defaults to; the import side never applies either, so none of it
    matters beyond keeping the archive free of this machine's uids.
    """
    files = _collect_files(map_dir, exclude_dirs)
    hashes: Dict[str, str] = {}
    buffer = io.BytesIO()

    if fmt is ArchiveFormat.ZIP:
        with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for rel, path in files:
                info = zipfile.ZipInfo.from_file(path, arcname=rel, strict_timestamps=False)
                info.compress_type = zipfile.ZIP_DEFLATED
                digest = _md5()
                with open(path, "rb") as source, archive.open(info, "w") as sink:
                    while chunk := source.read(_CHUNK):
                        digest.update(chunk)
                        sink.write(chunk)
                hashes[rel] = digest.hexdigest()
            archive.writestr(MANIFEST_NAME, _manifest_bytes(name, exported_at, hashes, vertices))
    else:
        with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
            for rel, path in files:
                info = archive.gettarinfo(path, arcname=rel)
                info.uid = info.gid = 0
                info.uname = info.gname = ""
                info.mode = 0o644
                digest = _md5()
                with open(path, "rb") as source:
                    archive.addfile(info, _HashingReader(source, digest))
                hashes[rel] = digest.hexdigest()
            payload = _manifest_bytes(name, exported_at, hashes, vertices)
            info = tarfile.TarInfo(MANIFEST_NAME)
            info.size = len(payload)
            info.mode = 0o644
            info.mtime = int(time.time())
            archive.addfile(info, io.BytesIO(payload))

    return buffer.getvalue()


# --- Reading ----------------------------------------------------------------


def sniff_format(payload: bytes) -> ArchiveFormat:
    """Which container ``payload`` is, from its magic bytes.

    A plain, uncompressed ``.tar`` is deliberately not accepted: nothing here
    produces one, and the export's two formats are the import's two formats.
    """
    if payload[:4] in (b"PK\x03\x04", b"PK\x05\x06"):
        return ArchiveFormat.ZIP
    if payload[:2] == b"\x1f\x8b":
        return ArchiveFormat.TAR_GZ
    raise MapArchiveError("Unsupported archive: expected a .zip or .tar.gz export of a map.")


@dataclass(frozen=True)
class _Member:
    raw_name: str
    # "file" | "dir" | "other" (link, device, fifo, encrypted entry …)
    kind: str
    size: int
    handle: object  # ZipInfo or TarInfo, for ``_Reader.open``


class _Reader:
    """One open archive, read-only, uniform over the two containers."""

    def __init__(self, payload: bytes, fmt: ArchiveFormat):
        self._buffer = io.BytesIO(payload)
        self._zip: Optional[zipfile.ZipFile] = None
        self._tar: Optional[tarfile.TarFile] = None
        try:
            if fmt is ArchiveFormat.ZIP:
                self._zip = zipfile.ZipFile(self._buffer)
            else:
                self._tar = tarfile.open(fileobj=self._buffer, mode="r:gz")
        except (zipfile.BadZipFile, tarfile.TarError, EOFError, OSError, zlib.error) as exc:
            raise MapArchiveError(f"Not a readable {fmt.value} archive: {exc}") from exc

    def __enter__(self) -> "_Reader":
        return self

    def __exit__(self, *_exc) -> None:
        if self._zip is not None:
            self._zip.close()
        if self._tar is not None:
            self._tar.close()

    def members(self) -> List[_Member]:
        try:
            if self._zip is not None:
                return [self._zip_member(info) for info in self._zip.infolist()]
            assert self._tar is not None
            return [self._tar_member(info) for info in self._tar.getmembers()]
        except (zipfile.BadZipFile, tarfile.TarError, EOFError, OSError, zlib.error) as exc:
            raise MapArchiveError(f"Not a readable archive: {exc}") from exc

    @staticmethod
    def _zip_member(info: zipfile.ZipInfo) -> _Member:
        if info.is_dir():
            kind = "dir"
        elif info.flag_bits & 0x1:
            # Encrypted entries cannot be read without a password, and nothing
            # here writes one.
            kind = "other"
        else:
            # The high 16 bits of external_attr are the Unix mode when the
            # archive was made on Unix; an MS-DOS/Windows writer leaves them 0,
            # which has no file-type bits and means "an ordinary file" here.
            mode = info.external_attr >> 16
            kind = "file" if stat.S_IFMT(mode) == 0 or stat.S_ISREG(mode) else "other"
        return _Member(info.filename, kind, info.file_size, info)

    @staticmethod
    def _tar_member(info: tarfile.TarInfo) -> _Member:
        if info.isdir():
            kind = "dir"
        elif info.isreg():
            kind = "file"
        else:
            kind = "other"
        return _Member(info.name, kind, info.size, info)

    def open(self, member: _Member):
        if self._zip is not None:
            return self._zip.open(member.handle)
        assert self._tar is not None
        handle = self._tar.extractfile(member.handle)
        if handle is None:  # pragma: no cover - kind == "file" rules it out
            raise MapArchiveError(f"Archive member {member.raw_name!r} is not a file.")
        return handle


def _check_member_name(raw: str) -> str:
    r"""The member's path relative to the map root, or raise.

    Rejects everything that could land a write outside the destination or
    hide one: empty names, NULs, backslashes (the zip spec mandates ``/``, and a
    backslash would be a literal filename character on Linux that hides
    ``..\\``), absolute paths, any ``.``/``..``/empty component, and names that
    ``normpath`` would rewrite (``./a``, ``a//b``). Any depth of *relative*
    subdirectory is fine — ``patches/`` today, whatever a later sidecar needs
    tomorrow; an allowlist of directory names would make every addition to the
    map directory an archive-format change.
    """
    name = raw[:-1] if raw.endswith("/") else raw
    if not name or "\x00" in name or "\\" in name:
        raise MapArchiveError(f"Unsafe archive member name {raw!r}.")
    if name.startswith("/") or os.path.isabs(name):
        raise MapArchiveError(f"Archive member {raw!r} has an absolute path.")
    if any(part in ("", ".", "..") for part in name.split("/")):
        raise MapArchiveError(f"Archive member {raw!r} is not a plain relative path.")
    if os.path.normpath(name) != name:
        raise MapArchiveError(f"Archive member {raw!r} is not a normalised path.")
    return name


def _some(names: Sequence[str], limit: int = 5) -> str:
    shown = ", ".join(repr(n) for n in names[:limit])
    if len(names) > limit:
        shown += f" and {len(names) - limit} more"
    return shown


def _parse_manifest(raw: bytes) -> ArchiveManifest:
    try:
        document = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise MapArchiveError(f"{MANIFEST_NAME} is not valid JSON: {exc}") from exc
    if not isinstance(document, dict):
        raise MapArchiveError(f"{MANIFEST_NAME} must be a JSON object.")

    if document.get("format") != MANIFEST_FORMAT:
        raise MapArchiveError(
            f"{MANIFEST_NAME} is not a {MANIFEST_FORMAT!r} manifest; only exports made "
            "by this API can be imported."
        )
    version = document.get("version")
    if version != MANIFEST_VERSION:
        newer = (
            isinstance(version, int)
            and not isinstance(version, bool)
            and version > MANIFEST_VERSION
        )
        if newer:
            raise MapArchiveError(
                f"This archive was written by a newer backend (manifest version {version}); "
                f"this one reads version {MANIFEST_VERSION}."
            )
        raise MapArchiveError(f"Unsupported manifest version {version!r}.")

    name = document.get("name")
    if not isinstance(name, str) or not name:
        raise MapArchiveError(f"{MANIFEST_NAME} has no map name.")
    exported_at = document.get("exported_at")
    if not isinstance(exported_at, str):
        raise MapArchiveError(f"{MANIFEST_NAME} has no exported_at timestamp.")

    files = document.get("files")
    if not isinstance(files, dict):
        raise MapArchiveError(f"{MANIFEST_NAME} has no files table.")
    checked: Dict[str, str] = {}
    for rel, digest in files.items():
        try:
            safe = isinstance(rel, str) and _check_member_name(rel) == rel
        except MapArchiveError:
            safe = False
        if not safe:
            raise MapArchiveError(f"{MANIFEST_NAME} lists an unsafe path {rel!r}.")
        if rel == MANIFEST_NAME:
            raise MapArchiveError(f"{MANIFEST_NAME} must not list itself.")
        if not isinstance(digest, str) or not _MD5_RE.match(digest):
            raise MapArchiveError(f"{MANIFEST_NAME} has no valid md5 for {rel!r}.")
        checked[rel] = digest

    vertices = document.get("vertices")
    if not isinstance(vertices, list):
        raise MapArchiveError(f"{MANIFEST_NAME} has no vertices list.")
    normalised: List[Dict[str, object]] = []
    seen = set()
    for index, vertex in enumerate(vertices):
        if not isinstance(vertex, dict):
            raise MapArchiveError(f"Vertex #{index} in {MANIFEST_NAME} is not an object.")
        vertex_name = vertex.get("name")
        if not isinstance(vertex_name, str) or not vertex_name:
            raise MapArchiveError(f"Vertex #{index} in {MANIFEST_NAME} has no name.")
        if vertex_name in seen:
            raise MapArchiveError(f"{MANIFEST_NAME} names vertex {vertex_name!r} twice.")
        seen.add(vertex_name)
        vertex_type = vertex.get("type")
        if not isinstance(vertex_type, str) or not vertex_type:
            raise MapArchiveError(f"Vertex {vertex_name!r} in {MANIFEST_NAME} has no type.")
        row: Dict[str, object] = {"name": vertex_name, "type": vertex_type}
        for key in _VERTEX_NUMBERS:
            value = vertex.get(key)
            finite = (
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(value)
            )
            if not finite:
                raise MapArchiveError(
                    f"Vertex {vertex_name!r} in {MANIFEST_NAME} has no finite {key}."
                )
            row[key] = float(value)
        normalised.append(row)

    return ArchiveManifest(
        name=name, exported_at=exported_at, files=checked, vertices=tuple(normalised)
    )


def inspect_archive(payload: bytes) -> InspectedArchive:
    """Validate ``payload`` without writing anything; raise or describe it.

    In order: the container is one we produce and opens; it has a sane number
    of members; every member — directories included — has a safe relative
    name; no member is a link, device or encrypted entry; exactly one manifest
    sits at the root and parses; and the set of files in the archive is
    *exactly* the set the manifest lists. Only the manifest's bytes are read.
    """
    fmt = sniff_format(payload)
    with _Reader(payload, fmt) as reader:
        members = reader.members()
        if len(members) > MAX_MEMBERS:
            raise MapArchiveError(
                f"The archive has {len(members)} members; a map export has far fewer "
                f"than {MAX_MEMBERS}."
            )

        files: Dict[str, _Member] = {}
        manifest_member: Optional[_Member] = None
        for member in members:
            name = _check_member_name(member.raw_name)
            if member.kind == "dir":
                continue
            if member.kind == "other":
                raise MapArchiveError(
                    f"Archive member {name!r} is a link, device or encrypted entry, "
                    "not a file."
                )
            if name in files or (name == MANIFEST_NAME and manifest_member is not None):
                raise MapArchiveError(f"The archive holds {name!r} twice.")
            if name == MANIFEST_NAME:
                manifest_member = member
            else:
                files[name] = member

        if manifest_member is None:
            raise MapArchiveError(
                f"No {MANIFEST_NAME} manifest in the archive; only exports made by "
                "this API can be imported."
            )
        if manifest_member.size > MAX_MANIFEST_BYTES:
            raise MapArchiveError(f"{MANIFEST_NAME} is implausibly large.")
        with reader.open(manifest_member) as handle:
            manifest = _parse_manifest(handle.read(MAX_MANIFEST_BYTES + 1))

        listed = set(manifest.files)
        present = set(files)
        missing = sorted(listed - present)
        if missing:
            raise MapArchiveError(
                f"Listed in the manifest but missing from the archive: {_some(missing)}."
            )
        extra = sorted(present - listed)
        if extra:
            raise MapArchiveError(f"Not listed in the manifest: {_some(extra)}.")

        total = sum(files[rel].size for rel in listed)

    return InspectedArchive(format=fmt, manifest=manifest, total_bytes=total)


def extract_archive(payload: bytes, inspected: InspectedArchive, dest_dir: str) -> int:
    """Write the manifest's files under ``dest_dir``; return the bytes written.

    ``dest_dir`` is expected to be a fresh, empty directory the caller owns (a
    staging directory it will rename or remove). Every file is hashed as it is
    written and compared with the manifest; the first mismatch raises, leaving
    whatever landed for the caller to remove. Modes, mtimes and owners from the
    archive are never applied — the files are created with the process umask.
    """
    root = os.path.realpath(dest_dir)
    written = 0
    with _Reader(payload, inspected.format) as reader:
        by_name: Dict[str, _Member] = {}
        for member in reader.members():
            if member.kind == "file":
                by_name[_check_member_name(member.raw_name)] = member

        for rel in sorted(inspected.manifest.files):
            member = by_name.get(rel)
            if member is None:  # pragma: no cover - inspect_archive saw the same payload
                raise MapArchiveError(f"{rel!r} is listed in the manifest but not in the archive.")

            target = os.path.realpath(os.path.join(root, rel))
            # Belt and braces over _check_member_name: a write must land
            # strictly inside the destination.
            if os.path.commonpath([root, target]) != root or target == root:
                raise MapArchiveError(f"Archive member {rel!r} escapes the destination.")
            os.makedirs(os.path.dirname(target), exist_ok=True)

            digest = _md5()
            with reader.open(member) as source, open(target, "xb") as sink:
                while chunk := source.read(_CHUNK):
                    digest.update(chunk)
                    sink.write(chunk)
                    written += len(chunk)
            if digest.hexdigest() != inspected.manifest.files[rel]:
                raise MapArchiveError(
                    f"md5 mismatch for {rel!r}: the archive is corrupt or was altered "
                    "after it was exported."
                )
    return written
