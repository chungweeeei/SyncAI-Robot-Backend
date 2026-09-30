"""Tests for the keepout (forbidden-zone) rasteriser.

Pure numpy in, numpy out; nothing touches disk. The geometry is the one the
``maps_dir`` fixture's ``full`` map has (6x4 cells, 0.05 m, origin at
(-6.94, -11.09)), so a mask asserted here is the mask the router tests write.
"""

import numpy as np
import pytest

pytest.importorskip("cv2")
pytest.importorskip("yaml")

import yaml  # noqa: E402

from syncai_backend.helpers.keepout import (  # noqa: E402
    KEEPOUT_OCCUPIED,
    KEEPOUT_UNKNOWN,
    keepout_yaml_text,
    rasterize_zones,
)

WIDTH, HEIGHT, RES = 6, 4, 0.05
ORIGIN = (-6.94, -11.09)


def _centre(col, row):
    """World coordinates of the centre of cell (col, row), row 0 = top."""
    return (
        ORIGIN[0] + (col + 0.5) * RES,
        ORIGIN[1] + (HEIGHT - row - 0.5) * RES,
    )


def _rasterize(zones):
    return rasterize_zones(
        zones, width=WIDTH, height=HEIGHT, resolution=RES, origin_xy=ORIGIN
    )


def test_no_zones_is_all_unknown():
    mask = _rasterize([])

    assert mask.shape == (HEIGHT, WIDTH)
    assert mask.dtype == np.uint8
    # 205, not 254: a free mask cell would overwrite unknown costmap cells.
    assert (mask == KEEPOUT_UNKNOWN).all()
    assert KEEPOUT_UNKNOWN == 205
    assert KEEPOUT_OCCUPIED == 0


def test_a_rectangle_on_cell_centres_paints_exactly_those_cells():
    # Corners snapped to cell centres, the way the console hands them over:
    # columns 1..3, rows 1..2 (top-indexed), drawn counter-clockwise.
    zone = [_centre(1, 2), _centre(3, 2), _centre(3, 1), _centre(1, 1)]

    mask = _rasterize([zone])

    expected = np.full((HEIGHT, WIDTH), KEEPOUT_UNKNOWN, dtype=np.uint8)
    expected[1:3, 1:4] = KEEPOUT_OCCUPIED
    np.testing.assert_array_equal(mask, expected)


def test_row_zero_is_the_top_of_the_map():
    # A zone along the max-y edge must land on row 0 -- a mirrored mask would
    # put it on the bottom row and keep the robot out of the wrong place.
    zone = [_centre(0, 0), _centre(5, 0), _centre(5, 0), _centre(0, 0)]

    mask = _rasterize([zone])

    assert (mask[0] == KEEPOUT_OCCUPIED).all()
    assert (mask[1:] == KEEPOUT_UNKNOWN).all()


def test_zones_partly_outside_are_clipped_not_rejected():
    # Hugs the left edge and runs off it by a metre.
    zone = [(-8.0, -11.09), (_centre(1, 0)[0], -11.09), (_centre(1, 0)[0], -10.8), (-8.0, -10.8)]

    mask = _rasterize([zone])

    assert (mask[:, :2] == KEEPOUT_OCCUPIED).all()
    assert (mask[:, 2:] == KEEPOUT_UNKNOWN).all()


def test_a_zone_entirely_off_the_map_paints_nothing():
    zone = [(10.0, 10.0), (11.0, 10.0), (11.0, 11.0)]

    assert (_rasterize([zone]) == KEEPOUT_UNKNOWN).all()


def test_two_zones_union():
    mask = _rasterize(
        [
            [_centre(0, 0), _centre(0, 0), _centre(0, 0)],
            [_centre(5, 3), _centre(5, 3), _centre(5, 3)],
        ]
    )

    assert mask[0, 0] == KEEPOUT_OCCUPIED
    assert mask[3, 5] == KEEPOUT_OCCUPIED
    assert (mask == KEEPOUT_OCCUPIED).sum() == 2


def test_fewer_than_three_points_paints_nothing():
    # Validation is the router's; a degenerate zone must not raise here.
    assert (_rasterize([[_centre(0, 0), _centre(1, 1)]]) == KEEPOUT_UNKNOWN).all()


def test_absurd_coordinates_do_not_overflow_into_the_map():
    zone = [(1e12, 1e12), (1e12 + 1, 1e12), (1e12, 1e12 + 1)]

    assert (_rasterize([zone]) == KEEPOUT_UNKNOWN).all()


def test_yaml_text_matches_the_gridmap_writer_and_loads_back():
    text = keepout_yaml_text(resolution=RES, origin_xy=ORIGIN, image="keepout.pgm")

    assert text == (
        "image: keepout.pgm\n"
        "mode: trinary\n"
        "resolution: 0.05\n"
        "origin: [-6.940000, -11.090000, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    document = yaml.safe_load(text)
    assert document["resolution"] == RES
    assert document["origin"] == [-6.94, -11.09, 0.0]
    # Relative on purpose: map_server resolves it against the yaml's directory.
    assert document["image"] == "keepout.pgm"
