import numpy as np

from indoor_mapping.geometry import pose_compose, pose_inverse, transform_points
from indoor_mapping.lidar_revisits import anchor_constraint_from_neighbor, verify_revisit


def room():
    x = np.linspace(-3, 4, 200)
    y = np.linspace(-2, 3, 160)
    return np.vstack([np.column_stack([x, np.full_like(x, -2)]),
                      np.column_stack([x, np.full_like(x, 3)]),
                      np.column_stack([np.full_like(y, -3), y]),
                      np.column_stack([np.full_like(y, 4), y])])


def test_revisit_recovers_nonzero_relative_pose_without_forcing_same_position():
    target = room()
    truth = np.array([.35, -.12, .06])
    source = transform_points(target, pose_inverse(truth))
    evidence = verify_revisit(source, target, [.45, -.18, .08])
    assert evidence['accepted'], evidence
    np.testing.assert_allclose(evidence['pose'], truth, atol=.015)
    assert len(evidence['initialization_trials']) >= 3


def test_parallel_corridor_cannot_supply_a_revisit_constraint():
    x = np.linspace(-5, 5, 400)
    target = np.vstack([np.column_stack([x, np.full_like(x, -1)]),
                        np.column_stack([x, np.full_like(x, 1)])])
    source = transform_points(target, [-.2, .1, -.01])
    evidence = verify_revisit(source, target, [.2, -.1, .01])
    assert not evidence['accepted']


def test_neighbor_measurement_is_transported_to_anchor_in_correct_direction():
    target_from_anchor = np.array([.3, -.2, .15])
    anchor_from_neighbor = np.array([.8, .1, -.2])
    target_from_neighbor = pose_compose(target_from_anchor, anchor_from_neighbor)
    recovered = anchor_constraint_from_neighbor(target_from_neighbor, anchor_from_neighbor)
    np.testing.assert_allclose(recovered, target_from_anchor, atol=1e-12)


def test_geometric_coverage_measures_counterpart_distance_without_normal_filter():
    from indoor_mapping.lidar_revisits import geometric_coverage
    points = np.random.default_rng(4).uniform(-2, 2, (200, 2))
    measured = geometric_coverage(points, points, np.zeros(3), tolerance=.03)
    assert measured['source_fraction'] == measured['target_fraction'] == 1.0
    separate = geometric_coverage(points, points, [10, 0, 0], tolerance=.03)
    assert separate['source_fraction'] == separate['target_fraction'] == 0.0
