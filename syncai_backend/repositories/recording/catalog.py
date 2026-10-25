import os
import re
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import List, Optional

import structlog
import yaml

from syncai_backend.exceptions import BadRequestError, NotFoundError
from syncai_backend.helpers.disk import walk_stats


# As strict as MapCatalogRepo's: the only thing between a URL path segment and
# an rmtree. Also a rosbag2 constraint: split files are named ``<name>_0.db3``,
# so a separator or space would break metadata.yaml's paths.
_NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")

# rosbag2 writes this last, on a clean shutdown: its presence means "finished".
METADATA_FILE = "metadata.yaml"

# The top-level key rosbag2 puts everything under.
_METADATA_ROOT = "rosbag2_bagfile_information"


@dataclass(frozen=True)
class BagInfo:
    """What ``metadata.yaml`` says about a finished recording.

    Only the fields an operator picks a bag by; the rest stays in the file.
    """

    duration_seconds: float
    message_count: int
    topics: List[str]
    compression: Optional[str]


@dataclass(frozen=True)
class StoredRecording:
    name: str
    size_bytes: int
    modified_at: datetime
    # None without metadata.yaml: still recording or interrupted. The REST
    # layer tells them apart with RecordingGateway's live slot.
    bag: Optional[BagInfo]


class RecordingCatalogRepo:
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self.logger = logger
        self.records_dir = os.path.expanduser("~/robot_ws/record")
        # Logged: the path is not configurable (see MapCatalogRepo).
        self.logger.info(
            "[RecordingCatalogRepo] Serving recordings", path=self.records_dir
        )

    # --- Paths --------------------------------------------------------------

    def resolve_dir(self, name: str) -> str:
        """Return the absolute path of a recording directory, or raise.

        Both checks are needed: the pattern rejects separators and ``..``, the
        realpath check catches a symlink pointing out of ``records_dir``.
        Copied from MapCatalogRepo rather than shared: a bag is not a map.
        """
        if not _NAME_RE.match(name) or name in (".", ".."):
            raise BadRequestError(f"Invalid recording name: {name!r}")

        candidate = os.path.realpath(os.path.join(self.records_dir, name))
        root = os.path.realpath(self.records_dir)
        if os.path.dirname(candidate) != root:
            raise BadRequestError(f"Invalid recording name: {name!r}")

        return candidate

    def exists(self, name: str) -> bool:
        """Whether a directory of this name is already there.

        Not a ``create_dir``: ``ros2 bag record -o`` creates the directory and
        refuses an existing one. This gives an early 409 ``name_taken`` instead.
        """
        return os.path.exists(self.resolve_dir(name))

    # --- Listing ------------------------------------------------------------

    def list_recordings(self) -> List[StoredRecording]:
        """Return every recording directory, newest first.

        Directories only: a rosbag2 bag is a directory.
        """
        try:
            entries = list(os.scandir(self.records_dir))
        except FileNotFoundError:
            # Not a warning: normal on a robot that has never recorded.
            self.logger.info(
                "[RecordingCatalogRepo] Recordings directory does not exist",
                path=self.records_dir,
            )
            return []

        recordings: List[StoredRecording] = []
        for entry in entries:
            if not entry.is_dir():
                continue
            stored = self._read(entry.name, entry.path)
            if stored is not None:
                recordings.append(stored)

        # Newest first (unlike maps): a bag is looked up as "the run I just did".
        recordings.sort(key=lambda item: item.modified_at, reverse=True)
        return recordings

    def get_recording(self, name: str) -> Optional[StoredRecording]:
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            return None
        return self._read(name, path)

    # --- Writing ------------------------------------------------------------

    def delete_dir(self, name: str) -> None:
        """Remove ``record/<name>/`` and everything under it. There is no undo.

        ``resolve_dir`` confines the path under the records root before
        ``rmtree``. Refusing the live recording is the router's job.
        """
        path = self.resolve_dir(name)
        if not os.path.isdir(path):
            raise NotFoundError(f"No recording named '{name}' on this robot.")

        shutil.rmtree(path)
        self.logger.info("[RecordingCatalogRepo] Deleted recording", recording=name)

    # --- Internals ----------------------------------------------------------

    def _read(self, name: str, path: str) -> Optional[StoredRecording]:
        """Describe one directory; None only if it disappeared mid-scan."""
        try:
            size_bytes, newest_mtime = walk_stats(path)
        except FileNotFoundError:
            return None

        return StoredRecording(
            name=name,
            size_bytes=size_bytes,
            modified_at=datetime.fromtimestamp(newest_mtime, tz=timezone.utc),
            bag=self._read_metadata(name, path),
        )

    def _read_metadata(self, name: str, path: str) -> Optional[BagInfo]:
        """Read ``metadata.yaml``, or None if it says nothing usable.

        Never raises: a half-written file must degrade one entry to
        "unfinished", not fail the listing (as MapCatalogRepo._read_grid_record).
        """
        metadata_path = os.path.join(path, METADATA_FILE)
        try:
            with open(metadata_path, "r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle)
        except FileNotFoundError:
            return None
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.logger.debug(
                "[RecordingCatalogRepo] Unreadable bag metadata",
                recording=name,
                error=str(exc),
            )
            return None

        if not isinstance(document, dict):
            return None
        info = document.get(_METADATA_ROOT)
        if not isinstance(info, dict):
            return None

        topics = []
        for entry in info.get("topics_with_message_count") or []:
            if not isinstance(entry, dict):
                continue
            metadata = entry.get("topic_metadata")
            if isinstance(metadata, dict) and isinstance(metadata.get("name"), str):
                topics.append(metadata["name"])

        # ``duration: {nanoseconds: N}`` in the file; seconds on the wire.
        duration = info.get("duration")
        nanoseconds = duration.get("nanoseconds") if isinstance(duration, dict) else None

        compression = info.get("compression_format")
        message_count = info.get("message_count")

        return BagInfo(
            duration_seconds=(
                nanoseconds / 1e9 if isinstance(nanoseconds, (int, float)) else 0.0
            ),
            message_count=message_count if isinstance(message_count, int) else 0,
            topics=topics,
            # rosbag2 writes "" for an uncompressed bag.
            compression=compression or None,
        )


def init_recording_catalog_repo(
    logger: structlog.stdlib.BoundLogger,
) -> RecordingCatalogRepo:
    return RecordingCatalogRepo(logger=logger)
