"""Filtered viewer artifacts must preserve the original map geometry and bytes."""
import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location('viewer_builder', Path(__file__).parents[1] / 'scripts/build_viewer.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def pcd(rows):
    header = (f'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(rows)}\nHEIGHT 1\nPOINTS {len(rows)}\nDATA ascii\n')
    return (header + '\n'.join(rows) + '\n').encode('ascii')


def test_originals_preserved_when_filtered_variant_added(tmp_path):
    original = pcd(['0.000000 0.000000 0.000000', '1.234567 0.000000 0.000000', '2.000000 1.000000 0.000000'])
    grid = b'original grid bytes'
    files = {'lidar_scan_map.pcd': original, 'map.pgm': grid}
    variant = tmp_path / 'lidar_scan_map_clean_conservative.pcd'
    variant.write_bytes(pcd(['1.234567 0.000000 0.000000', '2.000000 1.000000 0.000000']))
    metadata, validation = {}, {}
    builder.add_filtered_clouds(files, metadata, validation, [variant])
    assert files['lidar_scan_map.pcd'] == original
    assert files['map.pgm'] == grid
    assert files[variant.name] == variant.read_bytes()
    assert validation['original_grid_unchanged'] is True
    assert metadata['pointcloud_variants'][variant.name]['removed_points'] == 1


@pytest.mark.parametrize('row', ['1.234568 0 0', '1.234567 0 0.1', '100 100 0'])
def test_shifted_or_invented_geometry_is_rejected(tmp_path, row):
    files = {'lidar_scan_map.pcd': pcd(['0 0 0', '1.234567 0 0', '2 1 0'])}
    variant = tmp_path / 'shifted.pcd'
    variant.write_bytes(pcd([row]))
    with pytest.raises(ValueError, match='exactly match'):
        builder.add_filtered_clouds(files, {}, {}, [variant])


def test_filtered_cloud_cannot_replace_original(tmp_path):
    files = {'lidar_scan_map.pcd': pcd(['0 0 0', '1 0 0'])}
    variant = tmp_path / 'lidar_scan_map.pcd'
    variant.write_bytes(pcd(['0 0 0']))
    with pytest.raises(ValueError, match='separate PCD filename'):
        builder.add_filtered_clouds(files, {}, {}, [variant])
