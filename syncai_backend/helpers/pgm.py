"""Binary PGM (P5) helpers for the stored gridmaps under ``map/<name>/``.

Readers tolerate ``#`` comments (GIMP writes one); ``write_pgm`` never emits
one, so a saved map is byte-identical in form to a freshly converted one.

``read_pgm_size`` parses only the header: ``gridmap.yaml`` does not carry the
cell extent, and decoding every map to list the catalogue would cost tens of MB
of I/O. Pixel decoding is left to OpenCV.

A legacy ``gridmap_preview.png`` is ignored: it is optional, full resolution,
and nothing produces it any more.

Only ``write_text_atomic`` is still used (the map catalogue's json writes).
No route calls the rest since 2026-10-28, when the stack went 3D-only and the
backend's 2D gridmap surface was removed; kept as a library.
"""

import os
import tempfile
from typing import List, Tuple

import cv2
import numpy as np

# nav2 map_server only writes, and assumes, 8-bit maps.
_MAXVAL = 255

# magic, width, height, maxval
_HEADER_TOKENS = 4

# Thumbnail longest edge, px: keeps the widest real map (1613 cells) legible in
# a card tile at ~30 KB.
DEFAULT_MAX_EDGE = 512


def _read_header(path: str) -> Tuple[List[bytes], int]:
    """Tokenize a PGM header; return its tokens and the body's byte offset.

    Headers are whitespace-delimited and may hold ``#`` comments anywhere
    (GIMP writes one after the magic). Reads a byte at a time so a bad file
    cannot pull a multi-MB body into memory looking for a token.
    """
    tokens: List[bytes] = []
    current = bytearray()
    offset = 0

    with open(path, "rb") as handle:
        while len(tokens) < _HEADER_TOKENS:
            byte = handle.read(1)
            offset += 1
            if not byte:
                raise ValueError(f"Unexpected EOF reading PGM header: {path}")

            if byte == b"#":
                # A comment ends the current token and runs to end of line.
                if current:
                    tokens.append(bytes(current))
                    current = bytearray()
                while byte and byte not in b"\r\n":
                    byte = handle.read(1)
                    offset += 1
                continue

            if byte.isspace():
                if current:
                    tokens.append(bytes(current))
                    current = bytearray()
                continue

            current += byte

    # The whitespace byte ending the last token is consumed; the body starts here.
    return tokens, offset


def read_pgm_size(path: str) -> Tuple[int, int]:
    """Return ``(width, height)`` in cells from a binary PGM header.

    Raises ``ValueError`` for anything this stack cannot have produced: not P5,
    maxval != 255, a bad dimension, or a body shorter than ``width * height``
    (a map caught mid-write).
    """
    (magic, raw_width, raw_height, raw_maxval), body_offset = _read_header(path)

    if magic != b"P5":
        raise ValueError(f"Not a binary PGM (magic {magic!r}, expected P5): {path}")

    try:
        width = int(raw_width)
        height = int(raw_height)
        maxval = int(raw_maxval)
    except ValueError as exc:
        raise ValueError(f"Malformed PGM header: {path}") from exc

    if width <= 0 or height <= 0:
        raise ValueError(f"PGM has a non-positive extent {width}x{height}: {path}")
    if maxval != _MAXVAL:
        raise ValueError(f"Unsupported PGM maxval {maxval} (expected 255): {path}")

    body_bytes = os.path.getsize(path) - body_offset
    if body_bytes < width * height:
        raise ValueError(
            f"PGM body is short: {body_bytes} bytes for {width}x{height}: {path}"
        )

    return width, height


def write_pgm(path: str, width: int, height: int, body: bytes) -> bytes:
    r"""Replace ``path`` with a binary PGM of ``body``, atomically.

    Header ``P5\n{w} {h}\n255\n``, no comment; ``pcd_to_gridmap.py`` writes
    through here too. Atomic because readers arrive mid-write: every
    ``GET /api/v1/maps`` (a short body degrades the map to ``grid: None``),
    ``syncai_map_server``'s ``LoadMap`` right after a save (a torn P5 can decode
    shorter and mis-register the map nav2 plans on), and the router's ETag cache.

    Returns the whole file (header and body) for callers that want it. The
    rename is also what the router's ETag leans on: a save is a new inode, so
    a stat-based tag changes even when the size does not.
    """
    if len(body) != width * height:
        raise ValueError(
            f"PGM body is {len(body)} bytes for {width}x{height} "
            f"({width * height} cells expected): {path}"
        )

    blob = f"P5\n{width} {height}\n255\n".encode("ascii") + body

    directory = os.path.dirname(path) or "."
    # Same directory: os.replace is only atomic within one filesystem, and map/
    # is a bind mount.
    handle = tempfile.NamedTemporaryFile(
        dir=directory, prefix=".gridmap-", suffix=".tmp", delete=False
    )
    try:
        with handle:
            handle.write(blob)
            handle.flush()
            # fsync before the rename publishes it, or a power loss can leave a
            # correctly named map full of zeroes.
            os.fsync(handle.fileno())
        os.replace(handle.name, path)
    except BaseException:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise

    # fsync the directory so the rename survives too; best effort, as some
    # filesystems refuse an O_DIRECTORY open.
    try:
        dir_fd = os.open(directory, os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except OSError:
        pass

    return blob


def write_text_atomic(path: str, content: str) -> None:
    """Write a small text file via a temp file + rename, like ``write_pgm``.

    Same reader race: the catalogue parses gridmap.yaml and the recipe sidecar
    on every listing. Callers write the yaml after the .pgm, so a reader that
    sees the yaml finds its pgm. Lives here so writers need not import scipy.
    """
    directory = os.path.dirname(path) or "."
    handle = tempfile.NamedTemporaryFile(
        mode="w", dir=directory, prefix=".gridmap-", suffix=".tmp", delete=False
    )
    try:
        with handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(handle.name, path)
    except BaseException:
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise


def _decode_pgm(data: bytes) -> np.ndarray:
    """Decode PGM bytes to a greyscale array.

    Takes bytes: callers already read the file to hash it for the ETag.
    """
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("Failed to decode PGM data")
    return image


def _encode_png(image: np.ndarray) -> bytes:
    ok, buffer = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("Failed to encode PNG")

    return np.asarray(buffer).tobytes()


def render_png(data: bytes) -> bytes:
    """Re-encode a gridmap as a PNG at its native size, no resampling.

    For the 3D view's ground texture, which needs every cell. PNG because
    browsers cannot decode P5; lossless, and usually smaller than the .pgm.
    """
    return _encode_png(_decode_pgm(data))


def render_thumbnail(data: bytes, max_edge: int = DEFAULT_MAX_EDGE) -> bytes:
    """Downscale a gridmap to a PNG for the catalogue card.

    INTER_AREA averages the dropped cells; nearest sampling would erase thin
    walls. Greyscale is kept as is: the frontend tiles expect the .pgm
    convention (white free, black obstacle, 205 unknown).
    """
    image = _decode_pgm(data)

    height, width = image.shape[:2]
    longest = max(width, height)
    if longest > max_edge:
        scale = max_edge / longest
        image = cv2.resize(
            image,
            (max(int(round(width * scale)), 1), max(int(round(height * scale)), 1)),
            interpolation=cv2.INTER_AREA,
        )

    return _encode_png(image)
