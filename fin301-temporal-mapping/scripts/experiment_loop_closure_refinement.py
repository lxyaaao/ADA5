"""Bounded loop-refinement experiment; never overwrite the delivered pipeline."""
from __future__ import annotations
import argparse
from collections import Counter
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from indoor_mapping.geometry import pose_between, pose_compose, pose_inverse, scan_to_points
from indoor_mapping.graph import optimize_graph
from indoor_mapping.revisit_refinement import propose_refinement
from scripts.experiment_lidar_revisits import save_json


def run(args):
    output = args.output/args.profile; output.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    sources = [args.cache/'scans.npz', args.baseline/'odometry.npz', args.current/'trajectory_arrays.npz',
               args.current/'pose_graph.json', args.protocol, ROOT/'indoor_mapping/geometry.py',
               ROOT/'indoor_mapping/graph.py', ROOT/'indoor_mapping/lidar_revisits.py',
               ROOT/'indoor_mapping/revisit_refinement.py', Path(__file__)]
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    with np.load(args.cache/'scans.npz') as archive: data = {key: archive[key] for key in archive.files}
    with np.load(args.baseline/'odometry.npz') as archive:
        poses, scan_ids, keys = archive['poses'], archive['scan_indices'], archive['key_ids']
    with np.load(args.current/'trajectory_arrays.npz') as archive: current = archive['corrected']
    times = data['times_ns'][scan_ids]
    protocol = json.loads(args.protocol.read_text())
    heldout = {query['scan_index'] for group in ('queries', 'crosspass_queries') for query in protocol[group]}
    if heldout.intersection(scan_ids): raise ValueError('A frozen query occurs in accepted training scans')
    current_graph = json.loads((args.current/'pose_graph.json').read_text())
    existing = [edge for edge in current_graph['edges'] if edge['loop']]
    if any(edge.get('sensor') != 'lidar_revisit' for edge in existing):
        raise ValueError('Unexpected existing loop sensor; this experiment is LiDAR-only')
    scans = [scan_to_points(data['ranges'][i], float(data['angle_min']), float(data['angle_increment']),
                           max(.12, float(data['range_min'])), min(12., float(data['range_max'])), voxel_size=.045) for i in scan_ids]
    loops, events, used, parameters = propose_refinement(scans, poses, keys, times, existing, args.profile)
    key_poses = poses[keys]
    odometry = [{'i': i, 'j': i+1, 'pose': pose_between(key_poses[i], key_poses[i+1]).tolist(), 'loop': False}
                for i in range(len(keys)-1)]
    # Original odometry measurements, not relative poses from an already corrected graph.
    for a, b in zip(odometry, [edge for edge in current_graph['edges'] if not edge['loop']]):
        if a['i'] != b['i'] or a['j'] != b['j'] or not np.allclose(a['pose'], b['pose'], atol=1e-12, rtol=0):
            raise ValueError('Original odometry edges do not reproduce the delivered graph')
    corrected_keys, graph = optimize_graph(key_poses, odometry+loops)
    correction = np.array([pose_compose(new, pose_inverse(old)) for old, new in zip(key_poses, corrected_keys)])
    correction[:, 2] = np.unwrap(correction[:, 2]); seconds = (times-times[0]).astype(float)*1e-9
    dense = np.column_stack([np.interp(seconds, seconds[keys], correction[:, axis]) for axis in range(3)])
    proposed = np.array([pose_compose(delta, pose) for delta, pose in zip(dense, poses)])
    loop_residuals = []
    for edge in loops:
        i, j = edge['i'], edge['j']
        residuals = {name: pose_between(edge['pose'], pose_between(p[i], p[j])) for name, p in
                     [('original_odometry', key_poses), ('current_delivered_graph', current[keys]), ('proposed_graph', corrected_keys)]}
        loop_residuals.append({'i': i, 'j': j, 'sensor': edge['sensor'], 'measurement': edge['pose'],
                              'time_gap_s': float((times[keys[j]]-times[keys[i]])*1e-9),
                              'engineering_sigma_xy_yaw': [.08, .08, .035], 'residuals': residuals})
    graph_consistent = all(np.linalg.norm(e['residuals']['proposed_graph'][:2]) <= .12 and abs(e['residuals']['proposed_graph'][2]) <= .05 for e in loop_residuals)
    additional = len(loops)-len(existing)
    summary = {'experiment': 'All-heading loop refinement; development proposal only', 'profile': args.profile,
               'existing_loops': len(existing), 'additional_loops': additional, 'total_loops': len(loops),
               'tested_candidates': len(events), 'rejection_reasons': dict(Counter(e['reason'] for e in events)),
               'parameters': parameters, 'graph': graph, 'loop_residuals': loop_residuals,
               'graph_constraint_consistency_pass': graph_consistent,
               'maximum_change_from_delivered_trajectory_m': float(np.linalg.norm(proposed[:, :2]-current[:, :2], axis=1).max()),
               'maximum_change_from_original_odometry_m': float(np.linalg.norm(proposed[:, :2]-poses[:, :2], axis=1).max()),
               'training_raw_scan_count': len(used), 'heldout_query_count': len(heldout),
               'training_query_intersection': sorted(heldout.intersection(scan_ids[used])),
               'training_scan_ids_exclude_frozen_queries': True,
               'first_pose_fixed': bool(np.allclose(proposed[0], poses[0], atol=1e-12)),
               'absolute_accuracy_validated': False, 'adopted_into_primary_outputs': False,
               'limitations': ['Repeated structures can still alias despite geometric and two-pass temporal checks.',
                              'Neighbor validation uses disjoint observations but shares short odometry and submaps.',
                              'Engineering graph weights are unchanged and are not calibrated uncertainty.',
                              'Training residual reduction is not independent accuracy evidence.',
                              'The existing localization development protocol is reused for model comparison.'],
               'elapsed_s': time.monotonic()-started}
    if not graph['success'] or not graph_consistent: summary['eligible_for_export'] = False
    else: summary['eligible_for_export'] = additional > 0
    np.savez_compressed(output/'proposed_trajectory.npz', times_ns=times, baseline=poses, lidar_only=poses,
                        current_final=current, proposed=proposed, corrected=proposed, key_ids=keys, scan_indices=scan_ids)
    with (output/'proposed_trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream); writer.writerow(['timestamp_ns', 'lidar_x', 'lidar_y', 'lidar_yaw'])
        writer.writerows([int(stamp), *pose] for stamp, pose in zip(times, proposed))
    save_json(output/'candidate_evidence.json', events)
    save_json(output/'proposed_pose_graph.json', {'edges': odometry+loops, 'optimization': graph, 'loop_residuals': loop_residuals})
    save_json(output/'training_scan_indices.json', {'raw_scan_indices': scan_ids[used], 'dense_scan_indices': used,
              'heldout_raw_scan_indices': sorted(heldout), 'intersection': []})
    for path in sources:
        if hashlib.sha256(path.read_bytes()).hexdigest() != hashes[str(path)]: raise RuntimeError(f'Source changed: {path}')
    save_json(output/'provenance.json', {'source_sha256': hashes, 'primary_outputs_unchanged': True})
    os.environ.setdefault('MPLCONFIGDIR', str(output/'plot_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(current[:, 0], current[:, 1], label='Delivered one-loop trajectory', linewidth=1)
    ax.plot(proposed[:, 0], proposed[:, 1], label=f'{args.profile} proposal', linewidth=1)
    for edge in loops:
        pair = corrected_keys[[edge['i'], edge['j']]]
        ax.plot(pair[:, 0], pair[:, 1], '-', color='#33855f', linewidth=1)
        ax.text(pair[1, 0], pair[1, 1], f"{edge['i']}→{edge['j']}", fontsize=7)
    ax.set_aspect('equal'); ax.set(xlabel='Map x (m)', ylabel='Map y (m)', title='Experimental measured loops; no forced endpoint')
    ax.legend(); fig.tight_layout(); fig.savefig(output/'trajectory_comparison.png', dpi=170); plt.close(fig)
    save_json(output/'summary.json', summary)
    print(json.dumps({k: summary[k] for k in ('profile', 'additional_loops', 'total_loops', 'tested_candidates', 'eligible_for_export', 'maximum_change_from_delivered_trajectory_m', 'elapsed_s')}, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=Path('outputs/fin301/cache'))
    parser.add_argument('--baseline', type=Path, default=Path('outputs/fin301/baseline'))
    parser.add_argument('--current', type=Path, default=Path('outputs/fin301/result'))
    parser.add_argument('--protocol', type=Path, default=Path('outputs/fin301/verification/localization_protocol.json'))
    parser.add_argument('--output', type=Path, default=Path('outputs/fin301/loop_refinement'))
    parser.add_argument('--profile', choices=['all-heading', 'multikeyframe'], default='all-heading')
    run(parser.parse_args())


if __name__ == '__main__': main()
