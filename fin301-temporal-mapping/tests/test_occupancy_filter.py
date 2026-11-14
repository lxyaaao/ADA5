from pathlib import Path

import numpy as np
import pytest


def api():
    assert (Path(__file__).parents[1]/'indoor_mapping/occupancy_filter.py').exists()
    from indoor_mapping.occupancy_filter import filter_occupancy_supported
    return filter_occupancy_supported


def metadata(**changes):
    return dict(resolution=.05, origin=[0., 0., 0.], negate=0,
                occupied_thresh=.65, free_thresh=.196, mode='trinary', **changes)


def test_fixed_edt_uses_cell_center_distance_not_point_to_center_distance():
    fn = api()
    image = np.full((7, 7), 254, np.uint8)
    image[3, 3] = 0
    points = np.array([[.175, .175, 0.], [.299, .175, 0.], [.275, .225, 0.],
                       [.225, .225, 0.], [.325, .175, 0.]])
    # 2-cell x offset survives even when the actual point is >.10m from the
    # occupied center. sqrt(5)-cell offset fails; sqrt(2)-cell offset passes.
    kept, stats, mask = fn(points, image, metadata())
    np.testing.assert_array_equal(mask, [True, True, False, True, False])
    np.testing.assert_array_equal(kept, points[mask])
    assert stats['parameters']['maximum_cell_center_distance_m'] == .10
    assert stats['retention_by_original_cell_class']['occupied']['retained'] == 1
    assert stats['retention_by_original_cell_class']['free']['retained'] == 3-1


def test_rotated_yaml_origin_and_image_top_row_are_applied():
    fn = api()
    image = np.full((4, 4), 205, np.uint8)
    image[0, 0] = 0  # local x cell0, y cell3 (top row)
    meta = metadata()
    meta.update(resolution=1., origin=[10., 20., np.pi/2])
    # Rotate local [x=.5,y=3.5] by +90deg and translate.
    points = np.array([[6.5, 20.5, 0.], [9.5, 20.5, 0.], [10.1, 19.9, 0.]])
    _, stats, keep = fn(points, image, meta)
    np.testing.assert_array_equal(keep, [True, False, False])
    assert stats['retention_by_original_cell_class']['unknown']['removed'] == 1
    assert stats['retention_by_original_cell_class']['outside_grid']['removed'] == 1


def test_negate_and_trinary_thresholds_and_invalid_input():
    fn = api()
    image = np.array([[255, 0, 128]], np.uint8)
    meta = metadata()
    meta.update(resolution=1., negate=1)
    points = np.array([[.5, .5, 0.], [1.5, .5, 0.], [2.5, .5, 0.]])
    _, stats, keep = fn(points, image, meta)
    np.testing.assert_array_equal(keep, [True, False, False])
    assert stats['grid_cell_counts'] == {'occupied': 1, 'free': 1, 'unknown': 1}
    rejected, empty_stats, empty_keep = fn(points, np.full((1, 3), 254), metadata())
    assert not empty_keep.any() and len(rejected) == 0
    assert empty_stats['no_occupied_cells_all_rejected']
    with pytest.raises(ValueError, match='planar'):
        fn(np.array([[0., 0., .1]]), image, meta)


def test_exact_thresholds_are_unknown_not_occupied_or_free():
    fn = api()
    meta = metadata()
    meta.update(resolution=1., occupied_thresh=1-89/255., free_thresh=1-205/255.)
    image = np.array([[0, 89, 205, 254]], np.uint8)
    points = np.array([[i+.5, .5, 0.] for i in range(4)])
    _, stats, keep = fn(points, image, meta)
    assert stats['grid_cell_counts'] == {'occupied': 1, 'free': 1, 'unknown': 2}
    np.testing.assert_array_equal(keep, [True, False, False, False])
