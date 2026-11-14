import numpy as np
from indoor_mapping.localization_map import persistent_points


def test_repeated_beams_do_not_create_independent_scan_support():
    first = np.repeat([[.01, .01]], 100, axis=0)
    points = persistent_points([first, np.array([[1., 1.]])])
    assert points.shape == (0, 2)


def test_common_surface_retained_and_single_observations_excluded():
    points = persistent_points([np.array([[.01, .01], [.03, .03], [1., 1.]]),
                                np.array([[.04, .04], [2., 2.]])])
    np.testing.assert_allclose(points, [[.03, .03]])
