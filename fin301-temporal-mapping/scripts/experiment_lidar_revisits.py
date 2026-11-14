"""Propose conservative LiDAR revisit constraints without changing final results.

Outputs are experimental internal consistency evidence, not absolute accuracy.
Independent held-out localization evaluation must decide whether to adopt them.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from indoor_mapping.geometry import pose_between, pose_compose, pose_inverse, scan_to_points
from indoor_mapping.graph import optimize_graph
from indoor_mapping.lidar_revisits import RevisitParameters, propose_revisits


def plain(value):
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, np.generic):
        return plain(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def save_json(path, value):
    Path(path).write_text(json.dumps(plain(value), indent=2, allow_nan=False), encoding='utf-8')


def run_experiment(cache, baseline, output, profile='strict'):
    cache, baseline, output = Path(cache), Path(baseline), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with np.load(cache / 'scans.npz') as loaded:
        data = {name: loaded[name] for name in loaded.files}
    with np.load(baseline / 'odometry.npz') as loaded:
        poses, scan_ids, keys = loaded['poses'], loaded['scan_indices'], loaded['key_ids']
    times = data['times_ns'][scan_ids]
    scans = [scan_to_points(data['ranges'][index], float(data['angle_min']), float(data['angle_increment']),
                           max(.12, float(data['range_min'])), min(12., float(data['range_max'])), voxel_size=.045) for index in scan_ids]
    parameters = RevisitParameters()
    if profile == 'partial-overlap':
        parameters = replace(parameters, min_geometric_coverage=.50, min_mean_geometric_coverage=.65,
                             max_odometry_correction_m=2.0, min_converged_initializations=3)
    loops, evidence, dense_training, parameters = propose_revisits(scans, poses, keys, times, parameters)
    key_poses = poses[keys]
    odometry_edges = [{'i': i, 'j': i+1, 'pose': pose_between(key_poses[i], key_poses[i+1]).tolist(), 'loop': False}
                      for i in range(len(keys) - 1)]
    corrected, graph = optimize_graph(key_poses, odometry_edges + loops)
    corrections = np.array([pose_compose(new, pose_inverse(old)) for old, new in zip(key_poses, corrected)])
    corrections[:, 2] = np.unwrap(corrections[:, 2])
    relative_times = (times - times[0]).astype(float) * 1e-9
    dense_corrections = np.column_stack([np.interp(relative_times, relative_times[keys], corrections[:, column]) for column in range(3)])
    proposed = np.array([pose_compose(correction, pose) for correction, pose in zip(dense_corrections, poses)])
    if not loops:
        proposed = poses.copy()
    before, after = [], []
    for edge in loops:
        before.append(pose_between(edge['pose'], pose_between(key_poses[edge['i']], key_poses[edge['j']])))
        after.append(pose_between(edge['pose'], pose_between(corrected[edge['i']], corrected[edge['j']])))
    before, after = np.asarray(before).reshape(-1, 3), np.asarray(after).reshape(-1, 3)
    summary = {'experiment': 'Conservative LiDAR revisit constraints', 'adopted_into_delivered_map': False,
               'independent_evaluation_required': True, 'absolute_accuracy_validated': False,
               'baseline': str(baseline), 'profile': profile, 'tested_candidates': len(evidence), 'accepted_constraints': len(loops),
               'candidate_reasons': dict(Counter(event['reason'] for event in evidence)), 'parameters': parameters,
               'graph': graph, 'loop_translation_residual_before_m': np.linalg.norm(before[:, :2], axis=1),
               'loop_translation_residual_after_m': np.linalg.norm(after[:, :2], axis=1),
               'loop_yaw_residual_before_rad': np.abs(before[:, 2]), 'loop_yaw_residual_after_rad': np.abs(after[:, 2]),
               'maximum_trajectory_correction_m': float(np.max(np.linalg.norm(proposed[:, :2] - poses[:, :2], axis=1))),
               'training_raw_scan_count': len(dense_training), 'elapsed_s': time.monotonic() - started,
               'diagnostic_rationale': ['Geometric point coverage is separated from the fraction supporting valid PCA normals.',
                   'The one bounded partial-overlap trial was declared after diagnostics found 0.621/0.840 counter-region coverage, 1.43 m drift hypothesis, 3 agreeing valid initializations and 4/22 mm neighboring-frame agreement.',
                   'Failed low-overlap optimizer basins are logged; any competing well-supported mode still rejects a constraint.'] if profile == 'partial-overlap' else [],
               'limitations': ['Constraint residual improvement is measured on fitted constraints, not independent evidence.',
                   'Repeated geometry and unobserved drift can remain despite reciprocal and temporal checks.',
                   'No endpoint is forced to coincide; all constraints preserve estimated nonzero relative poses.',
                   'Use raw scans excluded by training_scan_indices.json for independent localization evaluation.']}
    np.savez_compressed(output / 'proposed_trajectory.npz', times_ns=times, baseline=poses, proposed=proposed,
                        corrected=proposed, lidar_only=poses, key_ids=keys, scan_indices=scan_ids)
    with (output / 'proposed_trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp_ns', 'lidar_x', 'lidar_y', 'lidar_yaw'])
        writer.writerows([int(stamp), *pose] for stamp, pose in zip(times, proposed))
    save_json(output / 'candidate_evidence.json', evidence)
    save_json(output / 'proposed_pose_graph.json', {'edges': odometry_edges + loops, 'optimization': graph})
    save_json(output / 'training_scan_indices.json', {'raw_scan_indices': scan_ids[dense_training], 'dense_scan_indices': dense_training})
    sources = [cache / 'scans.npz', baseline / 'odometry.npz', baseline / 'metrics.json',
               ROOT / 'indoor_mapping/lidar_revisits.py', ROOT / 'indoor_mapping/geometry.py', ROOT / 'indoor_mapping/graph.py', Path(__file__)]
    save_json(output / 'provenance.json', {'source_sha256': {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
              'baseline_immutable': True, 'output_directory': str(output)})
    os.environ.setdefault('MPLCONFIGDIR', str(output / 'plot_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(poses[:, 0], poses[:, 1], label='Baseline LiDAR odometry', linewidth=1)
    axes[0].plot(proposed[:, 0], proposed[:, 1], label='Revisit proposal', linewidth=1)
    for edge in loops:
        nodes = corrected[[edge['i'], edge['j']]]
        axes[0].plot(nodes[:, 0], nodes[:, 1], 'g-', alpha=.5)
    axes[0].set_aspect('equal'); axes[0].set_xlabel('Map x (m)'); axes[0].set_ylabel('Map y (m)'); axes[0].legend(fontsize=8)
    axes[0].set_title('Experimental trajectory; no forced endpoint')
    if len(loops):
        positions = np.arange(len(loops))
        axes[1].bar(positions - .18, np.linalg.norm(before[:, :2], axis=1), width=.36, label='Before')
        axes[1].bar(positions + .18, np.linalg.norm(after[:, :2], axis=1), width=.36, label='After')
        axes[1].legend(); axes[1].set_xlabel('Accepted constraint')
    else:
        axes[1].text(.5, .5, 'No constraints passed the predeclared gates', ha='center', va='center', transform=axes[1].transAxes)
    axes[1].set_ylabel('Fitted constraint translation residual (m)')
    axes[1].set_title('Training residuals; independent evaluation required')
    figure.tight_layout(); figure.savefig(output / 'proposal_review.png', dpi=160); plt.close(figure)
    save_json(output / 'summary.json', summary)
    print(json.dumps(plain(summary), indent=2), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', default='outputs/fin301/cache')
    parser.add_argument('--baseline', default='outputs/fin301/baseline')
    parser.add_argument('--output', default='outputs/fin301/accuracy_experiment')
    parser.add_argument('--profile', choices=['strict', 'partial-overlap'], default='strict')
    args = parser.parse_args()
    run_experiment(args.cache, args.baseline, args.output, args.profile)


if __name__ == '__main__':
    main()
