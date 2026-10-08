"""Unit tests for the point-cloud helpers (transform, downsample, pack, PCD)."""

import struct

import pytest

pytest.importorskip("numpy")

import numpy as np  # noqa: E402

from syncai_backend.helpers.pointcloud import (  # noqa: E402
    cap_points,
    pack_xyz_f32,
    quat_to_rotation_matrix,
    read_pcd_xyz,
    transform_points,
    voxel_downsample,
)


def test_identity_quat_is_identity_matrix():
    assert np.allclose(quat_to_rotation_matrix(0, 0, 0, 1), np.eye(3))


def test_quat_rotates_90deg_about_z():
    # +90 deg about z maps +x -> +y.
    rot = quat_to_rotation_matrix(0, 0, 0.7071068, 0.7071068)
    assert np.allclose(rot @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-6)


def test_transform_applies_rotation_then_translation():
    pts = np.array([[1.0, 0.0, 0.0]])
    out = transform_points(
        pts,
        translation=np.array([10.0, 0.0, 0.0]),
        quat_xyzw=np.array([0, 0, 0.7071068, 0.7071068]),
    )
    assert np.allclose(out[0], [10.0, 1.0, 0.0], atol=1e-6)


def test_voxel_downsample_collapses_nearby_points():
    pts = np.array([[1.0, 0, 0], [1.0, 0, 0], [1.01, 0, 0], [5.0, 5.0, 5.0]])
    out = voxel_downsample(pts, 0.5)
    assert out.shape[0] == 2


def test_cap_points_bounds_count():
    pts = np.arange(30).reshape(10, 3).astype(float)
    assert cap_points(pts, 4).shape[0] <= 4


def test_pack_xyz_f32_layout():
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    data = pack_xyz_f32(pts)
    assert len(data) == 2 * 3 * 4
    assert np.frombuffer(data, dtype="<f4").tolist() == [1, 2, 3, 4, 5, 6]


def test_read_pcd_binary_roundtrip(tmp_path):
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    intensity = np.array([10.0, 20.0], dtype=np.float32)

    header = (
        "# .PCD v0.7\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH 2\nHEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS 2\n"
        "DATA binary\n"
    )
    body = b"".join(
        struct.pack("<ffff", *pts[i], intensity[i]) for i in range(2)
    )
    path = tmp_path / "map.pcd"
    path.write_bytes(header.encode("ascii") + body)

    xyz = read_pcd_xyz(str(path))
    assert xyz.shape == (2, 3)
    assert np.allclose(xyz, pts)


def test_read_pcd_ascii_with_a_single_point(tmp_path, make_pcd):
    """One row must still come back as an (N, 3) array, not a bare vector.

    numpy.loadtxt collapses a single row of a structured dtype to a 0-d array;
    the catalogue's /pointcloud route used to answer 404 for such a map.
    """
    path = make_pcd(tmp_path / "map.pcd", points=((1.0, 2.0, 3.0),))

    xyz = read_pcd_xyz(str(path))

    assert xyz.shape == (1, 3)
    assert xyz.tolist() == [[1.0, 2.0, 3.0]]


def test_read_pcd_binary_as_pcl_writes_point_xyz(tmp_path, make_pcl_pcd):
    """The robot side's 3D map layers: ``pcl::PointXYZ`` through PCL's binary
    writer, which drops the struct's padding -- 12-byte rows, three fields."""
    pts = ((0.05, 0.15, -0.45), (1.25, -2.5, 0.35))
    path = make_pcl_pcd(tmp_path / "octomap_road.pcd", points=pts)

    xyz = read_pcd_xyz(str(path))

    assert xyz.shape == (2, 3)
    assert np.allclose(xyz, pts)


def test_read_pcd_steps_over_a_padding_field(tmp_path, make_pcl_pcd):
    """A writer that keeps PointXYZ's padding: 16-byte rows, a ``_`` column."""
    pts = ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0))
    path = make_pcl_pcd(tmp_path / "padded.pcd", points=pts, pad_fields=1)

    assert np.allclose(read_pcd_xyz(str(path)), pts)


def test_read_pcd_tolerates_repeated_padding_fields(tmp_path, make_pcl_pcd):
    """numpy refuses a structured dtype with a repeated field name, and every
    padding field is called ``_``."""
    pts = ((1.0, 2.0, 3.0),)
    path = make_pcl_pcd(tmp_path / "padded.pcd", points=pts, pad_fields=2)

    assert np.allclose(read_pcd_xyz(str(path)), pts)
