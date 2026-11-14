"""Holdout isolation and internal localization-agreement regressions."""

import numpy as np
import pytest


def _evaluator():
    from scripts import evaluate_localization
    return evaluate_localization


def test_protocol_excludes_all_odometry_scans_and_nearby_reference_origins():
    raw_times = 1_700_000_000_000_000_000 + np.arange(2401, dtype=np.int64) * 100_000_000
    ids = np.arange(0, len(raw_times), 5)
    theta = np.linspace(0, 4 * np.pi, len(ids))
    poses = np.column_stack([4 * np.sin(theta), 4 * np.cos(theta), theta])
    evaluator = _evaluator()
    protocol = evaluator.build_protocol(raw_times, raw_times[ids], poses, query_count=18)
    assert protocol == evaluator.build_protocol(raw_times, raw_times[ids], poses, query_count=18)
    assert protocol['queries']
    for query in protocol['queries']:
        query_id = query['scan_index']
        assert query_id not in set(ids)
        assert query_id not in protocol['training_scan_indices']
        for reference in query['reference_scan_indices']:
            assert abs(int(raw_times[query_id]) - int(raw_times[reference])) >= 5_000_000_000
            reference_pose = poses[np.flatnonzero(ids == reference)[0]]
            assert np.linalg.norm(reference_pose[:2] - np.asarray(query['seed_pose'])[:2]) >= .5 - 1e-10


def test_pose_spread_wraps_angles_and_reports_no_solution_honestly():
    evaluator = _evaluator()
    summary = evaluator.pose_agreement([[0, 0, np.deg2rad(179)], [.1, 0, np.deg2rad(-179)]], total_starts=2)
    assert summary['max_pairwise_translation_m'] == pytest.approx(.1)
    assert summary['max_pairwise_yaw_deg'] == pytest.approx(2.)
    assert summary['all_starts_agree']
    missing = evaluator.pose_agreement([], total_starts=8)
    assert missing['max_pairwise_translation_m'] is None
    assert not missing['all_starts_agree']


def test_localization_multiple_starts_agree_in_asymmetric_synthetic_room():
    evaluator = _evaluator()
    x, y = np.linspace(-3, 4, 180), np.linspace(-2, 3, 150)
    room = np.vstack([np.column_stack([x, np.full_like(x, -2)]),
                      np.column_stack([x, np.full_like(x, 3)]),
                      np.column_stack([np.full_like(y, -3), y]),
                      np.column_stack([np.full_like(y, 4), y]),
                      np.column_stack([np.full(70, .7), np.linspace(-.8, .9, 70)])])
    trials = evaluator.localize_query(room, room, evaluator.initial_perturbations())
    accepted = [trial['pose'] for trial in trials if trial['accepted']]
    agreement = evaluator.pose_agreement(accepted, total_starts=len(trials))
    assert len(trials) == 8
    assert agreement['all_starts_agree']
    assert agreement['max_pairwise_translation_m'] < .03


def test_timestamp_interpolation_uses_relative_integer_differences():
    evaluator = _evaluator()
    start = 1_793_312_142_514_555_067
    times = np.array([start, start + 200_000_000], dtype=np.int64)
    poses = np.array([[0, 0, np.deg2rad(179)], [2, 0, np.deg2rad(-179)]])
    middle = evaluator.interpolate_poses(times, poses, np.array([start + 100_000_000]))[0]
    assert middle[0] == pytest.approx(1.)
    assert abs(abs(middle[2]) - np.pi) < 1e-12
    with pytest.raises(ValueError, match='extrapolat'):
        evaluator.interpolate_poses(times, poses, np.array([start - 1]))
