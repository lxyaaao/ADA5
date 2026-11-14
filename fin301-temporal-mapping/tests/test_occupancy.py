import numpy as np
import pytest
import yaml
from PIL import Image

from indoor_mapping.occupancy import build_occupancy, save_map, save_pcd


def index_at(map_data, x, y):
    col, row = np.floor(
        (np.array([x, y]) - np.asarray(map_data['origin'])[:2])
        / map_data['resolution']
    ).astype(int)
    return row, col


def test_valid_return_ray_clears_free_space_and_marks_endpoint():
    data = build_occupancy([np.array([[2.0, 0.0]])] * 6, np.zeros((6, 3)), resolution=0.1)
    assert data['occupancy'][index_at(data, 2.0, 0)] > 65
    assert data['occupancy'][index_at(data, 1.0, 0)] < 20
    assert data['occupancy'][index_at(data, 1.0, 0.3)] == -1


def test_max_range_endpoint_does_not_create_obstacle():
    data = build_occupancy([np.array([[2.0, 0.0]])] * 6, np.zeros((6, 3)), resolution=0.1, max_range=2)
    assert data['occupancy'][index_at(data, 1.8, 0)] < 20
    assert np.max(data['occupancy']) <= 50


def test_pose_rotates_scan_endpoint_into_world_coordinates():
    poses = np.repeat([[1.0, 2.0, np.pi / 2]], 6, axis=0)
    data = build_occupancy([np.array([[2.0, 0.0]])] * 6, poses, resolution=0.1)
    assert data['occupancy'][index_at(data, 1.0, 4.0)] > 65


def test_map_export_flips_rows_and_uses_relative_pgm_path(tmp_path):
    data = build_occupancy([np.array([[0.0, 2.0], [1.0, 0.0]])] * 6, np.zeros((6, 3)), resolution=0.1)
    paths = save_map(data, tmp_path, basename='floor')
    pgm = np.asarray(Image.open(paths['pgm']))
    png = np.asarray(Image.open(paths['png']))
    np.testing.assert_array_equal(pgm, data['grid'][::-1])
    np.testing.assert_array_equal(png, pgm)
    meta = yaml.safe_load(paths['yaml'].read_text())
    assert meta['image'] == 'floor.pgm'
    assert meta['origin'] == list(data['origin'])
    assert meta['resolution'] == 0.1


def test_map_rejects_mismatched_scan_pose_count():
    with pytest.raises(ValueError, match='poses'):
        build_occupancy([np.array([[1.0, 0]])], np.zeros((2, 3)))


def test_no_valid_returns_still_preserves_unknown_space():
    data = build_occupancy([np.empty((0, 2))], np.zeros((1, 3)), resolution=0.1)
    assert np.all(data['occupancy'] == -1)
    assert np.all(data['grid'] == 205)


def test_pcd_exports_transformed_metric_points_on_explicit_plane(tmp_path):
    scans = [np.array([[1, 0], [0, 2], [np.nan, 0], [20, 0]])]
    result = save_pcd(scans, np.array([[3, 4, np.pi / 2]]), tmp_path / 'map.pcd', voxel_size=0, z=0.25)
    text = result['path'].read_text()
    header, body = text.split('DATA ascii\n')
    assert 'FIELDS x y z\n' in header
    assert 'WIDTH 2\n' in header
    assert 'HEIGHT 1\n' in header
    assert 'POINTS 2\n' in header
    assert result['point_count'] == 2
    coords = np.array([[float(v) for v in line.split()] for line in body.splitlines()])
    np.testing.assert_allclose(coords, [[3, 5, 0.25], [1, 4, 0.25]], atol=1e-6)
