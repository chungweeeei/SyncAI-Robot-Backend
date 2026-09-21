"""The recipe sidecar is written atomically, like the yaml beside it.

It used to be a plain ``open("w")`` -- the one non-atomic write in the map
directory. ``MapCatalogRepo`` reads the sidecar on every listing and tolerates
a torn file by returning None, at which point ``_grid_status`` falls through to
"the grid on disk decides" and reports ``ok`` for a re-conversion that had just
written ``failed``. The reader's tolerance is right; the writer should not be
producing the torn file in the first place.
"""

import json
import os

import pytest

pytest.importorskip("numpy")
pytest.importorskip("scipy")

from syncai_backend.repositories.map.catalog import GRIDMAP_RECIPE_SIDECAR  # noqa: E402
from syncai_backend.services import gridmap_conversion as svc  # noqa: E402


def _sidecar(directory):
    return directory / GRIDMAP_RECIPE_SIDECAR


def test_the_sidecar_lands_whole_and_leaves_no_temp_file(logger, tmp_path):
    svc.write_recipe_sidecar(logger, str(tmp_path), {"status": "converting"})

    assert json.loads(_sidecar(tmp_path).read_text()) == {"status": "converting"}
    assert os.listdir(tmp_path) == [GRIDMAP_RECIPE_SIDECAR]


def test_a_failed_write_leaves_the_previous_record_untouched(logger, tmp_path, monkeypatch):
    """The atomic property: either the old file or the new one, never a torn
    one -- and no stray temp file for the catalogue to trip over."""
    svc.write_recipe_sidecar(logger, str(tmp_path), {"status": "ok", "recipe": "zband"})
    before = _sidecar(tmp_path).read_text()

    def _disk_full(src, dst):
        raise OSError("No space left on device")

    monkeypatch.setattr(svc.os, "replace", _disk_full)

    svc.write_recipe_sidecar(logger, str(tmp_path), {"status": "failed"})  # must not raise

    assert _sidecar(tmp_path).read_text() == before
    assert os.listdir(tmp_path) == [GRIDMAP_RECIPE_SIDECAR]
