import numpy as np
import pytest

from scripts.verify_delivery import check_loop_graph_consistency


def example():
    before = np.array([[0., 0., 0.], [1.1, 0., 0.]])
    after = np.array([[0., 0., 0.], [1.0, 0., 0.]])
    loop = {'i': 0, 'j': 1, 'pose': [1., 0., 0.], 'loop': True, 'sensor': 'lidar_revisit'}
    graph = {'edges': [{'i': 0, 'j': 1, 'pose': [1.1, 0., 0.], 'loop': False}, loop],
             'optimization': {'success': True}}
    metrics = {'accepted_camera_lidar_loops': 0, 'accepted_lidar_revisits': 1,
               'camera_constraints_applied': False, 'lidar_revisit_constraints_applied': True,
               'max_camera_graph_correction_m': 0., 'graph': {'success': True}}
    events = [{'i': 0, 'j': 1, 'pose': [1., 0., 0.], 'accepted': True}]
    return metrics, graph, [], events, before, after


def test_lidar_graph_edge_matches_evidence_and_reports_its_own_residual():
    result = check_loop_graph_consistency(*example())
    assert result['accepted_lidar_revisits'] == 1
    np.testing.assert_allclose(result['lidar_loop_translation_residual_before_m'], [.1])
    np.testing.assert_allclose(result['lidar_loop_translation_residual_after_m'], [0.])


def test_lidar_correction_cannot_be_reported_as_camera_pose_improvement():
    arguments = example()
    arguments[0]['max_camera_graph_correction_m'] = .1
    with pytest.raises(ValueError, match='camera'):
        check_loop_graph_consistency(*arguments)


def test_loop_counts_cannot_disagree_with_graph_edges():
    arguments = example()
    arguments[0]['accepted_lidar_revisits'] = 2
    with pytest.raises(ValueError, match='count'):
        check_loop_graph_consistency(*arguments)
