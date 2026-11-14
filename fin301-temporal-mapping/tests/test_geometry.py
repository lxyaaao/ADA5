import numpy as np
import pytest

from indoor_mapping.geometry import (
    pose_between, pose_compose, pose_inverse, register_scan,
    scan_to_points, transform_points, voxel_downsample,
)


def room_points():
    x = np.linspace(-3.0, 4.0, 280)
    y = np.linspace(-2.0, 3.0, 200)
    # An asymmetric room and a short interior wall avoid rotational ambiguity.
    return np.vstack([
        np.column_stack([x, np.full_like(x, -2.0)]),
        np.column_stack([x, np.full_like(x, 3.0)]),
        np.column_stack([np.full_like(y, -3.0), y]),
        np.column_stack([np.full_like(y, 4.0), y]),
        np.column_stack([np.full(70, 0.7), np.linspace(-0.8, 0.9, 70)]),
    ])


def test_se2_composition_inverse_and_between():
    a = np.array([1.0, -2.0, 0.7])
    b = np.array([-0.3, 0.4, -0.25])
    points = np.array([[0.1, 0.5], [2.0, -0.4]])
    np.testing.assert_allclose(
        transform_points(points, pose_compose(a, b)),
        transform_points(transform_points(points, b), a), atol=1e-12,
    )
    np.testing.assert_allclose(pose_compose(a, pose_inverse(a)), 0, atol=1e-12)
    np.testing.assert_allclose(pose_between(a, pose_compose(a, b)), b, atol=1e-12)


def test_registration_recovers_current_to_previous_transform_with_outliers():
    rng = np.random.default_rng(81)
    target = room_points()
    truth = np.array([0.19, -0.11, 0.065])
    source = transform_points(target, pose_inverse(truth))
    source += rng.normal(0, 0.004, source.shape)
    source = np.vstack([source, rng.uniform(-7, 7, (90, 2))])
    result = register_scan(source, target, max_distance=0.6)
    assert result['success'], result
    np.testing.assert_allclose(result['pose'], truth, atol=0.018)
    assert result['rmse'] < 0.03
    assert result['inlier_ratio'] > 0.75
    assert not result['degenerate']


def test_registration_uses_initial_guess_for_larger_motion():
    target = room_points()
    truth = np.array([0.85, -0.6, 0.21])
    source = transform_points(target, pose_inverse(truth))
    result = register_scan(source, target, initial=[0.75, -0.5, 0.17])
    assert result['success'], result
    np.testing.assert_allclose(result['pose'], truth, atol=0.02)


def test_parallel_corridor_is_reported_as_degenerate():
    x = np.linspace(-5, 5, 500)
    target = np.vstack([
        np.column_stack([x, np.full_like(x, -1.0)]),
        np.column_stack([x, np.full_like(x, 1.0)]),
    ])
    source = transform_points(target, [-0.15, 0.08, -0.02])
    result = register_scan(source, target)
    assert result['degenerate']
    assert not result['success']
    assert result['condition'] > 1e5


def test_insufficient_registration_returns_explicit_failure():
    result = register_scan(np.zeros((2, 2)), np.zeros((2, 2)))
    assert not result['success']
    assert result['inlier_ratio'] == 0
    assert np.isinf(result['rmse'])


def test_scan_filter_uses_valid_ranges_and_expected_angles():
    points = scan_to_points(
        [1.0, np.nan, np.inf, -1.0, 0.0, 2.0, 15.0],
        0.0, np.pi / 2, range_min=0.1, range_max=12,
        voxel_size=0, outlier_neighbors=0,
    )
    np.testing.assert_allclose(points, [[1, 0], [0, 2]], atol=1e-12)


def test_voxel_downsampling_retains_cell_centroids():
    points = np.array([[0.01, 0.01], [0.03, 0.03], [1.0, 1.0]])
    np.testing.assert_allclose(voxel_downsample(points, 0.1), [[0.02, 0.02], [1, 1]])


def test_scan_rejects_invalid_angular_increment():
    with pytest.raises(ValueError):
        scan_to_points([1, 2, 3], 0, 0)


def test_outlier_filter_preserves_distant_surface_when_near_returns_are_dense():
    # Constant angular sampling makes nearby arcs much denser in metric space.
    # A global neighbor threshold on raw rays must not erase the farther arc.
    ranges = np.full(1440, 0.3)
    ranges[1000:] = 5.0
    ranges[100] = 8.0  # A truly isolated return remains rejectable.
    points = scan_to_points(ranges, 0, 2 * np.pi / len(ranges), voxel_size=0.045)
    distances = np.linalg.norm(points, axis=1)
    assert np.count_nonzero(distances > 4.5) > 150
    assert np.all(distances < 7.0)
