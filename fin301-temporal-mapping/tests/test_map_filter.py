import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE = Path(__file__).parents[1] / 'indoor_mapping' / 'map_filter.py'


def filter_api():
    assert MODULE.exists(), 'map_filter module is not implemented'
    from indoor_mapping.map_filter import filter_points
    return filter_points


def xyz(xy):
    return np.column_stack([xy, np.zeros(len(xy))])


def test_isolated_returns_removed_but_sparse_wall_endpoints_are_preserved():
    fn = filter_api()
    wall = xyz(np.column_stack([np.arange(0, 2.01, .08), np.zeros(26)]))
    isolated = xyz(np.array([[8., 8.], [9., -4.]]))
    points = np.vstack([wall, isolated])
    retained, stats, mask = fn(points, 'strong')
    assert mask[:len(wall)].all()
    assert not mask[-2:].any()
    np.testing.assert_array_equal(retained, points[mask])
    assert stats['line_protected_points'] == len(wall)
    assert stats['removed_points'] == 2


def test_sparse_right_angle_corner_retains_both_branches():
    fn = filter_api()
    arm = np.arange(.08, .81, .08)
    corner = xyz(np.vstack([[0, 0], np.column_stack([arm, arm*0]),
                            np.column_stack([arm*0, arm])]))
    retained, stats, mask = fn(corner, 'strong')
    assert mask.all(), 'sparse wall corner must not be pruned as non-linear clutter'
    np.testing.assert_array_equal(retained, corner)


def test_all_presets_are_nested_exact_subsets_and_report_density():
    fn = filter_api()
    rng = np.random.default_rng(31)
    points = xyz(rng.uniform(-1, 1, (500, 2)))
    masks = []
    for preset in ('conservative', 'balanced', 'strong'):
        retained, stats, mask = fn(points, preset)
        assert mask.dtype == bool and mask.shape == (len(points),)
        np.testing.assert_array_equal(retained, points[mask])
        assert stats['input_points'] == len(points)
        assert stats['retained_points'] + stats['removed_points'] == len(points)
        assert stats['parameters']['density_radius_m'] == .12
        assert stats['input_representation']['production_voxel_m'] == .03
        masks.append(mask)
    assert np.all(~masks[2] | masks[1])
    assert np.all(~masks[1] | masks[0])


def test_filter_rejects_bad_input_and_never_projects_visual_geometry():
    fn = filter_api()
    with pytest.raises(ValueError, match='planar'):
        fn(np.array([[0., 0., .2]]), 'balanced')
    with pytest.raises(ValueError, match='finite'):
        fn(np.array([[np.nan, 0., 0.]]), 'balanced')
    with pytest.raises(ValueError, match='preset'):
        fn(np.zeros((3, 3)), 'unknown')
    result, stats, mask = fn(np.empty((0, 3)), 'balanced')
    assert result.shape == (0, 3) and len(mask) == 0 and stats['removed_points'] == 0


def test_plain_radius_comparison_disables_line_protection_without_moving_points():
    fn = filter_api()
    wall = xyz(np.column_stack([np.arange(0, 2.01, .08), np.zeros(26)]))
    # Deliberately sparse wall: protected preset retains it, plain radius does not.
    protected, _, _ = fn(wall, 'balanced')
    retained, stats, mask = fn(wall, 'radius_balanced')
    assert len(protected) == len(wall)
    assert len(retained) == 0
    assert not stats['parameters']['enable_line_protection']
    assert stats['line_protected_points'] == 0
    # Normally sampled 3cm wall retains its interior under min6-within12cm.
    dense_wall = xyz(np.column_stack([np.arange(0, 2.011, .03), np.zeros(68)]))
    kept, _, dense_mask = fn(dense_wall, 'radius_balanced')
    assert dense_mask[4:-4].all()
    np.testing.assert_array_equal(kept, dense_wall[dense_mask])


def test_pcd_subset_writer_preserves_exact_source_rows(tmp_path):
    assert MODULE.exists(), 'map_filter module is not implemented'
    from indoor_mapping.map_filter import read_ascii_pcd, write_pcd_subset
    source = tmp_path / 'source.pcd'
    rows = ['1.123456 2.654321 0.000000', '-2.000000 1.000001 0.000000', '9.000000 4.000000 0.000000']
    source.write_text('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n' + '\n'.join(rows) + '\n')
    points, header, data_rows = read_ascii_pcd(source)
    target = tmp_path / 'filtered.pcd'
    write_pcd_subset(target, header, data_rows, np.array([True, False, True]))
    kept, _, kept_rows = read_ascii_pcd(target)
    np.testing.assert_array_equal(kept, points[[0, 2]])
    assert kept_rows == [rows[0], rows[2]]
    assert 'WIDTH 2\n' in target.read_text() and 'POINTS 2\n' in target.read_text()
    assert source.read_text().endswith('\n'.join(rows) + '\n')
