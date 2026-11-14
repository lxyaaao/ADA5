import numpy as np
from scripts.export_temporal_map import companion_grid


def metadata():
    return dict(resolution=1., origin=[0.,0.,0.], negate=0, mode='trinary', occupied_thresh=.65, free_thresh=.196)


def test_removed_obstacle_becomes_unknown_never_free_and_other_pixels_preserved():
    grid = np.array([[0, 0, 254, 205]], np.uint8)
    result, stats = companion_grid(grid, metadata(), np.array([[.5,.5,0.]]))
    np.testing.assert_array_equal(result, [[0,205,254,205]])
    assert stats['new_free_cells'] == 0
    assert stats['occupied_cells_masked_to_unknown'] == 1


def test_rotated_origin_and_top_row_conversion():
    grid = np.zeros((2,2), np.uint8)
    meta = metadata();meta['origin']=[10.,20.,np.pi/2]
    result, _ = companion_grid(grid, meta, np.array([[8.5,20.5,0.]]))
    np.testing.assert_array_equal(result, [[0,205],[205,205]])


def test_negated_grid_uses_unknown_interval_and_keeps_free():
    grid = np.array([[255,255,0,128]], np.uint8)
    meta = metadata();meta['negate']=1
    result, stats = companion_grid(grid, meta, np.empty((0,3)))
    assert meta['free_thresh'] <= result[0,0]/255 <= meta['occupied_thresh']
    np.testing.assert_array_equal(result[0,2:], grid[0,2:])
    assert stats['occupied_cells_masked_to_unknown']==2
