"""Tests for the pure grid passes in helpers/pcd_to_gridmap.py.

The pose-connectivity filter, the poses.txt reader and the local floor
reference live here — the conversions through the service, with the recipe
parameters and the sidecar, are in test_maps_router.py. These are numpy/scipy-
only, so the file runs without ROS, FastAPI or opencv present.
"""

import numpy as np
import pytest
import yaml

from syncai_backend.helpers.pcd_to_gridmap import (
    LocalFloorError,
    convert_pcd_to_gridmap,
    flatten_to_local_floor,
    local_floor_levels,
    read_poses_xy,
    read_poses_xyz,
    revert_unreachable_free,
)
from syncai_backend.helpers.pgm import read_pgm_size


def _grid(width=40, height=20):
    """An all-unknown grid with two free islands, columns 2-9 and 25-32."""
    grid = np.full((height, width), 205, dtype=np.uint8)
    grid[5:15, 2:10] = 254
    grid[5:15, 25:33] = 254
    return grid


ORIGIN = np.array([0.0, 0.0])
RES = 0.05


def test_free_space_the_poses_cannot_reach_becomes_unknown(logger):
    grid = _grid()
    # One pose in the left island (cell col 5, row 10).
    pose_xy = np.array([[5 * RES, 10 * RES]])

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert stats["applied"] == 1
    assert stats["components_total"] == 2
    assert stats["components_kept"] == 1
    assert (out[5:15, 2:10] == 254).all(), "the seeded island lost free space"
    assert (out[5:15, 25:33] == 205).all(), "the unreachable island is still free"
    assert stats["reverted_free_cells"] == 10 * 8


def test_a_pose_on_an_occupied_cell_still_seeds_through_the_radius(logger):
    """The band-recentring measurements found 9-16 keyframe poses per map on
    occupied cells 5-14 cm from free space (wall brushes); a radius-0 seed would
    drop their component."""
    grid = _grid()
    grid[10, 10] = 0  # a wall cell hugging the left island's edge
    pose_xy = np.array([[10 * RES, 10 * RES]])  # the pose sits ON the wall

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES, seed_radius=3)

    assert stats["applied"] == 1
    assert (out[5:15, 2:10] == 254).all()


def test_out_of_bounds_poses_are_skipped_not_fatal(logger):
    grid = _grid()
    pose_xy = np.array([[5 * RES, 10 * RES], [-50.0, -50.0], [999.0, 999.0]])

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert stats["applied"] == 1
    assert stats["poses"] == 3
    assert stats["poses_in_grid"] == 1
    assert (out[5:15, 2:10] == 254).all()


def test_no_seeded_component_leaves_the_grid_unchanged(logger):
    """Poses that miss every free component mean mismatched inputs; reverting
    all free space to satisfy the filter would destroy the map."""
    grid = _grid()
    pose_xy = np.array([[39 * RES, 1 * RES]])  # unknown corner, no free nearby

    out, stats = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert stats["applied"] == 0
    assert stats["reverted_free_cells"] == 0
    assert (out == grid).all()


def test_occupied_and_unknown_cells_are_never_touched(logger):
    grid = _grid()
    grid[0, 0:5] = 0
    before_occ = (grid == 0).copy()
    pose_xy = np.array([[5 * RES, 10 * RES]])

    out, _ = revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert ((out == 0) == before_occ).all()
    # Nothing unknown became free: this pass only demotes.
    assert not ((grid == 205) & (out == 254)).any()


def test_the_input_grid_is_not_mutated(logger):
    grid = _grid()
    before = grid.copy()
    pose_xy = np.array([[5 * RES, 10 * RES]])

    revert_unreachable_free(logger, grid, pose_xy, ORIGIN, RES)

    assert (grid == before).all()


def test_diagonal_contact_counts_as_connected(logger):
    """8-connectivity, matching _despeckle: a diagonal touch is one component,
    so free space beyond it survives a seed on the other side."""
    grid = np.full((10, 10), 205, dtype=np.uint8)
    grid[2, 2] = 254
    grid[3, 3] = 254  # touches only diagonally

    out, stats = revert_unreachable_free(
        logger, grid, np.array([[2 * RES, 2 * RES]]), ORIGIN, RES, seed_radius=0
    )

    assert stats["components_total"] == 1
    assert out[3, 3] == 254


# --- read_poses_xy ------------------------------------------------------------


def test_read_poses_xy_parses_the_pgo_format(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text(
        "0.pcd 7.39105e-06 -4.7505e-06 0.00050289 0.990556 -0.00839693 0.136854 1.49619e-05\n"
        "1.pcd 0.253802 0.0762091 0.257022 0.985168 -0.0309628 0.126643 0.111568\n"
        "\n"
    )

    poses = read_poses_xy(str(path))

    assert poses.shape == (2, 2)
    assert poses[1, 0] == pytest.approx(0.253802)
    assert poses[1, 1] == pytest.approx(0.0762091)


def test_read_poses_xy_rejects_a_malformed_line(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("0.pcd 1.0 2.0 0.0 1 0 0 0\nnot a pose\n")

    with pytest.raises(ValueError, match="poses.txt:2"):
        read_poses_xy(str(path))


def test_read_poses_xy_rejects_unparseable_coordinates(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("0.pcd one two three 1 0 0 0\n")

    with pytest.raises(ValueError, match="could not parse"):
        read_poses_xy(str(path))


def test_read_poses_xy_rejects_an_empty_file(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("\n\n")

    with pytest.raises(ValueError, match="no poses"):
        read_poses_xy(str(path))


def test_read_poses_xyz_keeps_the_keyframe_height(tmp_path):
    """z is what the local floor reference measures against; xy alone was
    enough for the connectivity filter and is not any more."""
    path = tmp_path / "poses.txt"
    path.write_text("0.pcd 1.0 2.0 0.5 1 0 0 0\n1.pcd 3.0 4.0 -0.25 1 0 0 0\n")

    poses = read_poses_xyz(str(path))

    assert poses.shape == (2, 3)
    assert poses[:, 2].tolist() == [0.5, -0.25]


def test_read_poses_xyz_rejects_a_line_without_z(tmp_path):
    path = tmp_path / "poses.txt"
    path.write_text("0.pcd 1.0 2.0\n")

    with pytest.raises(ValueError, match="poses.txt:1"):
        read_poses_xyz(str(path))


# --- the local floor reference -------------------------------------------------
#
# The regression in miniature: a floor that rises steadily along x, the way a
# LIO map's z drifts across a large venue. One floor level cannot band it; the
# floor measured under each keyframe can.

LIDAR_HEIGHT = 0.5


def _drifting_floor(length=40.0, width=6.0, step=0.2, slope=0.02):
    """A floor sheet whose z rises ``slope`` per metre of x (0.8 m over 40 m)."""
    xs = np.arange(0.0, length, step)
    ys = np.arange(0.0, width, step)
    xx, yy = np.meshgrid(xs, ys)
    return np.column_stack([xx.ravel(), yy.ravel(), slope * xx.ravel()]).astype(np.float32)


def _poses_over(floor_of_x, xs, y=3.0):
    """Keyframes along the sheet's centreline, the lidar LIDAR_HEIGHT above the floor."""
    return np.array([[x, y, floor_of_x(x) + LIDAR_HEIGHT] for x in xs])


def test_local_floor_levels_follow_a_drifting_floor(logger):
    xyz = _drifting_floor()
    poses = _poses_over(lambda x: 0.02 * x, np.arange(2.0, 40.0, 2.0))

    floors, stats = local_floor_levels(logger, xyz, poses)

    # Within the 0.1 m histogram bin plus the slope across a 4 m neighbourhood.
    assert np.allclose(floors, 0.02 * poses[:, 0], atol=0.15)
    assert floors[-1] - floors[0] > 0.5, "the estimate did not follow the drift"
    assert stats["lidar_height"] == pytest.approx(LIDAR_HEIGHT, abs=0.1)
    assert stats["keyframes_measured"] == len(poses)
    assert stats["keyframes_filled"] == 0
    assert stats["pose_z_spread"] == pytest.approx(0.02 * 36 * 0.9, abs=0.1)


def test_local_floor_levels_fill_an_unmeasurable_keyframe_from_the_lidar_height(logger):
    """A keyframe with no cloud around it (a doorway dwell at the rim) still
    gets a floor: the robot's constant height above it, off its own pose z."""
    xyz = _drifting_floor()
    poses = _poses_over(lambda x: 0.02 * x, np.arange(2.0, 40.0, 2.0))
    stray = np.array([[200.0, 200.0, 1.2 + LIDAR_HEIGHT]])

    floors, stats = local_floor_levels(logger, xyz, np.vstack([poses, stray]))

    assert stats["keyframes_filled"] == 1
    assert floors[-1] == pytest.approx(1.2, abs=0.1)


def test_local_floor_levels_replace_a_measurement_far_from_the_robot(logger):
    """A pit under one keyframe — a lower level seen through a railing — must
    not become that keyframe's floor: the robot was not standing in it."""
    xyz = _drifting_floor()
    # A dense sheet 1.5 m below the floor around x=20, denser than the floor
    # there so the lowest substantial peak would otherwise pick it.
    pit = _drifting_floor(length=6.0, width=6.0, step=0.1)
    pit[:, 0] += 17.0
    pit[:, 2] = 0.02 * pit[:, 0] - 1.5
    poses = _poses_over(lambda x: 0.02 * x, np.arange(2.0, 40.0, 2.0))

    floors, stats = local_floor_levels(logger, np.vstack([xyz, pit]), poses)

    assert stats["keyframes_replaced"] >= 1
    assert np.allclose(floors, 0.02 * poses[:, 0], atol=0.15)


def test_local_floor_levels_refuse_poses_that_see_no_cloud(logger):
    xyz = _drifting_floor()
    poses = np.array([[500.0, 500.0, 1.0], [600.0, 600.0, 1.0]])

    with pytest.raises(LocalFloorError, match="mismatched"):
        local_floor_levels(logger, xyz, poses)


def test_flatten_blends_the_floors_of_the_nearest_keyframes():
    pose_xy = np.array([[0.0, 0.0], [10.0, 0.0]])
    floors = np.array([0.0, 1.0])
    xyz = np.array([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0], [10.0, 0.0, 2.0]], dtype=np.float32)

    z_rel = flatten_to_local_floor(xyz, pose_xy, floors)

    # On a keyframe: that keyframe's floor. Halfway: the mean — a ramp between
    # the two, not a seam.
    assert z_rel[0] == pytest.approx(2.0, abs=1e-3)
    assert z_rel[1] == pytest.approx(1.5, abs=1e-3)
    assert z_rel[2] == pytest.approx(1.0, abs=1e-3)
    assert z_rel.dtype == np.float32


BANDS = dict(floor_zmin=-0.29, floor_zmax=0.41, zmin=0.36, zmax=2.16)
RECIPE = dict(free_mode="floor", min_points=2, obstacle_close=1, free_close=5)


def _convert_drifting(logger, tmp_path, make_pcd, **extra):
    xyz = _drifting_floor()
    # 2 cm, not the grid's 5 cm: points spaced exactly one cell apart land on
    # cell boundaries and float32 rounding leaves the block dashed.
    block = np.arange(0.0, 1.0, 0.02)
    obstacle = np.array(
        [
            (1.0 + bx, 1.0 + by, 0.02 * (1.0 + bx) + h)
            for bx in block
            for by in block
            for h in (0.5, 1.0)
        ],
        dtype=np.float32,
    )
    make_pcd(tmp_path / "map.pcd", points=np.vstack([xyz, obstacle]).tolist())
    basename = str(tmp_path / "gridmap")
    stats = convert_pcd_to_gridmap(
        logger, str(tmp_path / "map.pcd"), basename, **RECIPE, **BANDS, **extra
    )
    width, height = read_pgm_size(basename + ".pgm")
    with open(basename + ".pgm", "rb") as handle:
        body = handle.read()[-width * height:]
    grid = np.frombuffer(body, np.uint8).reshape(height, width)
    with open(basename + ".yaml", "r", encoding="utf-8") as handle:
        meta = yaml.safe_load(handle)
    res, (ox, oy, _) = meta["resolution"], meta["origin"]

    def cell(x, y):
        return grid[grid.shape[0] - 1 - int((y - oy) / res), int((x - ox) / res)]

    return cell, stats


def test_a_drifting_floor_leaves_the_absolute_bands_half_way_along(logger, tmp_path, make_pcd):
    """The failure this exists for: banded against one floor level, the far end
    of the sheet (0.8 m up) is out of the floor band and inside the obstacle one."""
    cell, stats = _convert_drifting(logger, tmp_path, make_pcd)

    assert cell(2.0, 3.0) == 254, "the near end should be free under either reference"
    assert cell(38.0, 3.0) != 254, "the far end should have drifted out of the floor band"
    assert stats == {}


def test_the_local_floor_reference_bands_the_whole_drifting_floor_as_free(
    logger, tmp_path, make_pcd
):
    poses = _poses_over(lambda x: 0.02 * x, np.arange(2.0, 40.0, 2.0))

    cell, stats = _convert_drifting(logger, tmp_path, make_pcd, floor_reference_xyz=poses)

    assert cell(2.0, 3.0) == 254
    assert cell(38.0, 3.0) == 254, "the far end is still not free — the drift was not followed"
    assert cell(1.5, 1.5) == 0, "the obstacle on the floor was flattened away"
    assert set(stats) == {"local_floor"}
    assert stats["local_floor"]["keyframes_measured"] == len(poses)
