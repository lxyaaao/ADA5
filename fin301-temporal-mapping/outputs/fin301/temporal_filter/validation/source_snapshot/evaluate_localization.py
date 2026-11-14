"""Held-out raw-scan localization stability; never an absolute-accuracy score.

Freeze --protocol on the baseline, then reuse it with candidate --trajectory.
Only LiDAR scan-plane returns form reference maps. Estimated poses define map
assembly and shared initialization, not ground-truth localization targets.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
from scipy.spatial.distance import pdist

from indoor_mapping.geometry import (pose_between, register_scan, scan_to_points,
                                     transform_points, voxel_downsample, wrap_angle)


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_trajectory(path):
    path = Path(path)
    if path.suffix.lower() == '.npz':
        with np.load(path) as data:
            times = np.asarray(data['times_ns'], dtype=np.int64)
            key = next((name for name in ('corrected', 'poses', 'lidar_only') if name in data), None)
            if key is None:
                raise ValueError('Trajectory NPZ requires corrected, poses, or lidar_only')
            poses = np.asarray(data[key], dtype=float)
    else:
        with path.open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            raise ValueError('Trajectory CSV is empty')
        columns = ('lidar_x', 'lidar_y', 'lidar_yaw') if 'lidar_x' in rows[0] else ('x', 'y', 'yaw')
        if columns[0] == 'x' and any(row.get('pose_frame') in ('base_link', 'base_footprint') for row in rows):
            raise ValueError('LiDAR reference points required; vehicle CSV must include lidar_x/lidar_y/lidar_yaw')
        times = np.array([int(row['timestamp_ns']) for row in rows], dtype=np.int64)
        poses = np.array([[float(row[name]) for name in columns] for row in rows])
    if len(times) < 2 or poses.shape != (len(times), 3) or not np.all(np.isfinite(poses)):
        raise ValueError('Trajectory must contain at least two finite SE(2) poses')
    if np.any(np.diff(times) <= 0):
        raise ValueError('Trajectory timestamps must be strictly increasing')
    return times, poses


def interpolate_poses(times, poses, queries):
    times, queries = np.asarray(times, np.int64), np.asarray(queries, np.int64)
    poses = np.asarray(poses, float)
    if not len(queries):
        return np.empty((0, 3))
    if queries.min() < times[0] or queries.max() > times[-1]:
        raise ValueError('Pose extrapolation is prohibited')
    # Subtract integer epochs before floating-point interpolation.
    seconds = (times - times[0]).astype(float) * 1e-9
    query_seconds = (queries - times[0]).astype(float) * 1e-9
    return np.column_stack([np.interp(query_seconds, seconds, poses[:, 0]),
                            np.interp(query_seconds, seconds, poses[:, 1]),
                            wrap_angle(np.interp(query_seconds, seconds, np.unwrap(poses[:, 2])))])


def initial_perturbations():
    perturbations = []
    for distance, yaw_deg in ((.25, 5.), (.5, 10.)):
        for dx, dy, sign in ((distance, 0, 1), (-distance, 0, -1), (0, distance, -1), (0, -distance, 1)):
            perturbations.append({'translation_m': distance, 'yaw_magnitude_deg': yaw_deg,
                                  'pose': [dx, dy, float(np.deg2rad(sign * yaw_deg))]})
    return perturbations


def build_protocol(raw_times, baseline_times, baseline_poses, query_count=100,
                   exclusion_seconds=5., min_reference_distance=.5,
                   reference_radius=8., max_reference_scans=18, crosspass_count=20):
    """Freeze held-out IDs, mapping IDs, reference subsets and baseline guesses."""
    raw_times = np.asarray(raw_times, np.int64)
    baseline_times, baseline_poses = np.asarray(baseline_times, np.int64), np.asarray(baseline_poses, float)
    if query_count < 1 or np.any(np.diff(raw_times) <= 0):
        raise ValueError('Positive query count and unique ordered raw timestamps required')
    odometry_ids = np.searchsorted(raw_times, baseline_times)
    if np.any(odometry_ids >= len(raw_times)) or not np.array_equal(raw_times[odometry_ids], baseline_times):
        raise ValueError('Every baseline pose must refer to an exact raw scan timestamp')
    insertion_positions = [0]
    for index in range(1, len(baseline_times)):
        previous = insertion_positions[-1]
        moved = np.linalg.norm(baseline_poses[index, :2] - baseline_poses[previous, :2]) > .06
        turned = abs(wrap_angle(baseline_poses[index, 2] - baseline_poses[previous, 2])) > .04
        if int(baseline_times[index]) - int(baseline_times[previous]) >= 400_000_000 and (moved or turned):
            insertion_positions.append(index)
    insertion_positions = np.asarray(insertion_positions)
    training_ids = odometry_ids[insertion_positions]
    training_poses = baseline_poses[insertion_positions]
    training_times = raw_times[training_ids]
    pool = np.setdiff1d(np.arange(odometry_ids[0] + 1, odometry_ids[-1]), odometry_ids)
    if not len(pool):
        raise ValueError('No held-out raw scans remain after excluding all odometry scans')
    query_ids = pool[np.unique(np.linspace(0, len(pool) - 1, min(query_count, len(pool)), dtype=int))]
    seed_poses = interpolate_poses(baseline_times, baseline_poses, raw_times[query_ids])
    queries, crosspass, excluded = [], [], []
    for query_id, seed in zip(query_ids, seed_poses):
        differences_s = (training_times - raw_times[query_id]).astype(float) * 1e-9
        distances = np.linalg.norm(training_poses[:, :2] - seed[:2], axis=1)
        eligible = np.flatnonzero((np.abs(differences_s) >= exclusion_seconds)
                                  & (distances >= min_reference_distance) & (distances <= reference_radius))
        if len(eligible) < 3:
            excluded.append({'scan_index': int(query_id), 'reason': 'fewer_than_three_temporally_and_spatially_separated_references'})
            continue
        order = np.lexsort((training_ids[eligible], np.abs(differences_s[eligible]), distances[eligible]))
        selected = eligible[order[:max_reference_scans]]
        item = {'scan_index': int(query_id), 'timestamp_ns': int(raw_times[query_id]),
                'seed_pose': seed.tolist(), 'reference_scan_indices': training_ids[selected].tolist()}
        queries.append(item)
        # An earlier-pass subset requires a nearby earlier origin and an actual
        # estimated departure >=3m before this query. This is still not truth.
        earlier = eligible[(differences_s[eligible] <= -30.) & (distances[eligible] <= 3.)]
        if len(earlier) >= 3:
            closest = earlier[np.argmin(distances[earlier])]
            interval = (baseline_times >= training_times[closest]) & (baseline_times <= raw_times[query_id])
            excursion = np.max(np.linalg.norm(baseline_poses[interval, :2] - training_poses[closest, :2], axis=1))
            if excursion >= 3.:
                ordered = earlier[np.lexsort((training_ids[earlier], distances[earlier]))][:max_reference_scans]
                crosspass.append({**item, 'reference_scan_indices': training_ids[ordered].tolist(),
                                  'estimated_intervening_excursion_m': float(excursion)})
    if crosspass_count <= 0:
        crosspass = []
    elif len(crosspass) > crosspass_count:
        crosspass = [crosspass[index] for index in np.linspace(0, len(crosspass) - 1, crosspass_count, dtype=int)]
    return {'schema': 'heldout_localization_v1', 'raw_timestamp_sha256': hashlib.sha256(raw_times.tobytes()).hexdigest(),
            'raw_scan_count': len(raw_times), 'all_odometry_scan_indices': odometry_ids.tolist(),
            'training_scan_indices': training_ids.tolist(), 'queries': queries, 'crosspass_queries': crosspass,
            'excluded_queries': excluded, 'perturbations': initial_perturbations(),
            'selection': {'requested_queries': query_count, 'temporal_exclusion_s': exclusion_seconds,
                          'minimum_reference_origin_distance_m': min_reference_distance,
                          'maximum_reference_origin_distance_m': reference_radius,
                          'maximum_reference_scans': max_reference_scans,
                          'crosspass_minimum_time_s': 30., 'crosspass_minimum_excursion_m': 3.,
                          'reference_insertion': 'Baseline: >=0.4s and >0.06m or >0.04rad from previous insertion',
                          'seed_policy': 'Identical frozen baseline pose and perturbations for every candidate; baseline is not ground truth'}}


def pose_agreement(poses, total_starts):
    poses = np.asarray(poses, float).reshape(-1, 3)
    count = len(poses)
    if count < 2:
        return {'accepted_starts': count, 'max_pairwise_translation_m': None, 'max_pairwise_yaw_deg': None,
                'median_pairwise_translation_m': None, 'largest_agreement_neighborhood': count,
                'all_starts_agree': False}
    distances = pdist(poses[:, :2])
    angle_matrix = np.abs(wrap_angle(poses[:, 2, None] - poses[None, :, 2]))
    angles = angle_matrix[np.triu_indices(count, 1)]
    neighborhoods = (np.linalg.norm(poses[:, None, :2] - poses[None, :, :2], axis=2) <= .15) & (angle_matrix <= np.deg2rad(3))
    return {'accepted_starts': count, 'max_pairwise_translation_m': float(distances.max()),
            'median_pairwise_translation_m': float(np.median(distances)),
            'max_pairwise_yaw_deg': float(np.rad2deg(angles.max())),
            'largest_agreement_neighborhood': int(neighborhoods.sum(axis=1).max()),
            'all_starts_agree': bool(count == total_starts and distances.max() <= .15 and angles.max() <= np.deg2rad(3))}


def localize_query(source, target, perturbations):
    trials = []
    for index, perturbation in enumerate(perturbations):
        result = register_scan(source, target, initial=perturbation['pose'], max_distance=.8, iterations=45)
        accepted = bool(result['success'] and result['inlier_ratio'] >= .5 and result['rmse'] <= .10
                        and result['condition'] <= 5000.)
        trial = {'initialization_id': index, **perturbation, 'initial_pose': perturbation['pose'],
                 'pose': np.asarray(result['pose']).tolist(), 'accepted': accepted,
                 'reason': result['reason'] if accepted or not result['success'] else 'evaluation_quality_gate',
                 'inlier_ratio': float(result['inlier_ratio']), 'iterations': int(result['iterations']),
                 'rmse_m': float(result['rmse']) if np.isfinite(result['rmse']) else None,
                 'euclidean_rmse_m': float(result['euclidean_rmse']) if np.isfinite(result['euclidean_rmse']) else None,
                 'condition': float(result['condition']) if np.isfinite(result['condition']) else None}
        trials.append(trial)
    return trials


def distribution(values):
    values = np.asarray([value for value in values if value is not None and np.isfinite(value)], float)
    return {'count': len(values), 'median': float(np.median(values)) if len(values) else None,
            'p95': float(np.percentile(values, 95)) if len(values) else None,
            'max': float(values.max()) if len(values) else None}


def summarize(queries):
    trials = [trial for query in queries for trial in query['trials']]
    accepted = [trial for trial in trials if trial['accepted']]
    stable = sum(query['agreement']['all_starts_agree'] for query in queries)
    return {'query_count': len(queries), 'attempted_initializations': len(trials),
            'accepted_registrations': len(accepted), 'acceptance_rate': len(accepted) / len(trials) if trials else None,
            'all_starts_agree_queries': stable, 'all_starts_agree_fraction': stable / len(queries) if queries else None,
            'heldout_point_to_line_rmse_m': distribution([trial['rmse_m'] for trial in accepted]),
            'heldout_euclidean_rmse_m': distribution([trial['euclidean_rmse_m'] for trial in accepted]),
            'accepted_overlap_fraction': distribution([trial['inlier_ratio'] for trial in accepted]),
            'query_max_pairwise_translation_m': distribution([query['agreement']['max_pairwise_translation_m'] for query in queries]),
            'query_max_pairwise_yaw_deg': distribution([query['agreement']['max_pairwise_yaw_deg'] for query in queries]),
            'by_perturbation': {str(level): {'attempted': sum(trial['translation_m'] == level for trial in trials),
                                            'accepted': sum(trial['translation_m'] == level for trial in accepted)}
                                for level in (.25, .5)}}


def evaluate(data, trajectory_times, trajectory_poses, protocol, reference_mode='all_returns'):
    raw_times = data['times_ns']
    all_reference_ids = sorted({index for name in ('queries', 'crosspass_queries')
                                for query in protocol[name] for index in query['reference_scan_indices']})
    reference_poses = interpolate_poses(trajectory_times, trajectory_poses, raw_times[all_reference_ids])
    pose_lookup = dict(zip(all_reference_ids, reference_poses))
    points_cache = {}
    def points(index):
        if index not in points_cache:
            points_cache[index] = scan_to_points(data['ranges'][index], float(data['angle_min']), float(data['angle_increment']),
                                                 max(.12, float(data['range_min'])), min(12., float(data['range_max'])), voxel_size=.06)
        return points_cache[index]
    output = {}
    for group in ('queries', 'crosspass_queries'):
        results = []
        for number, query in enumerate(protocol[group]):
            anchor = np.asarray(query['seed_pose'])
            if reference_mode == 'persistent':
                from indoor_mapping.localization_map import persistent_points
                world = persistent_points([transform_points(points(index), pose_lookup[index])
                                           for index in query['reference_scan_indices']], .10, 2)
                target = transform_points(world, pose_between(anchor, np.zeros(3)))
            else:
                target = np.vstack([transform_points(points(index), pose_between(anchor, pose_lookup[index]))
                                    for index in query['reference_scan_indices']])
            target = voxel_downsample(target[np.linalg.norm(target, axis=1) <= 14.], .07)
            source = points(query['scan_index'])
            trials = localize_query(source, target, protocol['perturbations'])
            agreement = pose_agreement([trial['pose'] for trial in trials if trial['accepted']], len(trials))
            results.append({**query, 'source_point_count': len(source), 'reference_point_count': len(target),
                            'trials': trials, 'agreement': agreement})
            if number % 20 == 0:
                print(f'{group}: {number+1}/{len(protocol[group])}, accepted={agreement["accepted_starts"]}/8', flush=True)
        output[group] = results
        output[group + '_summary'] = summarize(results)
    return output


def compare_results(candidate, baseline):
    if candidate['protocol_sha256'] != baseline['protocol_sha256']:
        raise ValueError('Comparison requires identical frozen held-out/reference IDs and initializations')
    output = {}
    for name in ('queries_summary', 'crosspass_queries_summary'):
        old, new = baseline[name], candidate[name]
        comparison = {}
        for key in ('acceptance_rate', 'all_starts_agree_fraction'):
            comparison[key + '_difference'] = None if old[key] is None or new[key] is None else new[key] - old[key]
        for key in ('heldout_point_to_line_rmse_m', 'query_max_pairwise_translation_m', 'query_max_pairwise_yaw_deg'):
            comparison[key] = {'baseline': old[key], 'candidate': new[key]}
        output[name] = comparison
    output['selection_rule'] = ('Prefer an experiment only if acceptance and eight-start agreement do not materially worsen '
                                'and spread or earlier-pass consistency improves across the same queries. Lower fitting residual alone '
                                'is insufficient; inspect ambiguous and rejected cases. These checks do not establish physical accuracy.')
    return output


def markdown_report(result):
    lines = ['# Held-out raw-scan localization stability', '',
             '**Internal evaluation only. No independent pose ground truth or absolute accuracy measurement.**', '',
             f"Trajectory: `{result['trajectory']}`. Frozen protocol SHA-256: `{result['protocol_sha256']}`.", '',
             f"Reference representation: `{result.get('reference_mode', 'all_returns')}`; parameters: `{result.get('reference_representation', {'voxel_m': .07})}`.", '',
             'Queries are raw scans excluded from every odometry pose. Reference points come only from baseline map-insertion '
             'scan IDs, with a five-second temporal exclusion and at least 0.5 m estimated sensor-origin separation. '
             'Queries and reference IDs are identical between methods; each method supplies its own map poses. '
             'Seeds remain the same baseline estimate plus deterministic perturbations; no deviation from that estimate is scored as truth.', '',
             '| Internal measure | Separated reference map | Earlier-pass reference only |', '|---|---:|---:|']
    a, b = result['queries_summary'], result['crosspass_queries_summary']
    for label, key in [('Queries', 'query_count'), ('Accepted registrations', 'accepted_registrations'),
                       ('Initialization attempts', 'attempted_initializations'), ('Acceptance fraction', 'acceptance_rate'),
                       ('All eight starts agree within 0.15 m / 3 deg', 'all_starts_agree_fraction')]:
        lines.append(f'| {label} | {a[key]} | {b[key]} |')
    for label, key in [('Held-out line residual (m)', 'heldout_point_to_line_rmse_m'),
                       ('Per-query maximum pairwise translation (m)', 'query_max_pairwise_translation_m'),
                       ('Per-query maximum pairwise yaw (deg)', 'query_max_pairwise_yaw_deg')]:
        lines.append(f"| {label}: median / p95 | {a[key]['median']} / {a[key]['p95']} | {b[key]['median']} / {b[key]['p95']} |")
    lines += ['', 'Each query uses eight starts: four signed XY/yaw directions at 0.25 m / 5 deg and four at 0.5 m / 10 deg. '
              'Pose spread is measured only between successful solutions for the same scan. A stable case requires all eight '
              'registrations to be accepted and agree. Rejections remain in the denominator; residual statistics alone use accepted runs.', '',
              'Earlier-pass tests use only references at least 30 seconds earlier, with an estimated intervening departure of at least '
              '3 m. This is retrospective consistency against an estimated earlier map, not independent first-pass ground truth. '
              'No camera 3D landmarks are projected into the 2D reference map.', '',
              'Selection: do not select a candidate solely for a lower residual or visually cleaner map. Require no material degradation '
              'in acceptance and multi-start agreement, together with improved spread or earlier-pass consistency; inspect multi-basin failures. '
              'Shared baseline seeds and baseline-based spatial selection are a declared prior and can favor that prior. '
              'External checks are still required before claiming absolute localization accuracy.', '']
    if result.get('comparison'):
        lines += ['## Paired comparison', '', '```json', json.dumps(result['comparison'], indent=2), '```', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=Path('outputs/fin301/cache'))
    parser.add_argument('--trajectory', type=Path, default=Path('outputs/fin301/result/trajectory.csv'))
    parser.add_argument('--baseline-trajectory', type=Path, default=Path('outputs/fin301/result/trajectory.csv'))
    parser.add_argument('--protocol', type=Path, help='Create once on baseline; reuse unchanged for every candidate')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--queries', type=int, default=100)
    parser.add_argument('--crosspass-queries', type=int, default=20)
    parser.add_argument('--compare', type=Path, help='Baseline localization_evaluation.json with the same frozen protocol')
    parser.add_argument('--reference-mode', choices=['all_returns','persistent'], default='all_returns',
                        help='Persistent: global 0.10 m voxels supported by at least two distinct reference scans')
    args = parser.parse_args()
    started = time.monotonic()
    args.output.mkdir(parents=True, exist_ok=True)
    cache_path = args.cache / 'scans.npz'
    cache_hash = file_hash(cache_path)
    with np.load(cache_path) as archive:
        data = {name: archive[name] for name in archive.files}
    protocol_path = args.protocol or args.output / 'localization_protocol.json'
    if protocol_path.exists():
        protocol = json.loads(protocol_path.read_text(encoding='utf-8'))
        if protocol.get('scan_cache_sha256') != cache_hash:
            raise ValueError('Raw scan cache differs from frozen evaluation protocol')
    else:
        baseline_times, baseline_poses = load_trajectory(args.baseline_trajectory)
        protocol = build_protocol(data['times_ns'], baseline_times, baseline_poses,
                                  query_count=args.queries, crosspass_count=args.crosspass_queries)
        protocol.update(scan_cache_sha256=cache_hash, baseline_trajectory=str(args.baseline_trajectory),
                        baseline_trajectory_sha256=file_hash(args.baseline_trajectory))
        protocol_path.parent.mkdir(parents=True, exist_ok=True)
        protocol_path.write_text(json.dumps(protocol, indent=2), encoding='utf-8')
    trajectory_times, trajectory_poses = load_trajectory(args.trajectory)
    result = evaluate(data, trajectory_times, trajectory_poses, protocol, args.reference_mode)
    result['reference_mode'] = args.reference_mode
    result['reference_representation'] = ({'voxel_m':.10, 'minimum_distinct_scans':2}
                                          if args.reference_mode == 'persistent' else {'voxel_m':.07})
    result.update(trajectory=str(args.trajectory), trajectory_sha256=file_hash(args.trajectory),
                  protocol=str(protocol_path), protocol_sha256=file_hash(protocol_path),
                  scan_cache_sha256=cache_hash, evaluator_sha256=file_hash(__file__),
                  registration_source_sha256=file_hash(Path(__file__).resolve().parents[1] / 'indoor_mapping' / 'geometry.py'),
                  excluded_query_count=len(protocol['excluded_queries']), elapsed_s=time.monotonic() - started,
                  absolute_accuracy='UNVALIDATED: no independent physical reference; estimated trajectory is not ground truth',
                  quality_gates={'minimum_overlap':.5,'maximum_point_to_line_rmse_m':.10,'maximum_condition':5000.,
                                 'agreement_translation_m':.15,'agreement_yaw_deg':3.,'icp_distance_m':.8,'icp_iterations':45})
    if args.compare:
        result['comparison'] = compare_results(result, json.loads(args.compare.read_text(encoding='utf-8')))
    (args.output / 'localization_evaluation.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    (args.output / 'localization_evaluation.md').write_text(markdown_report(result), encoding='utf-8')
    fields = ['group','scan_index','timestamp_ns','initialization_id','translation_m','yaw_magnitude_deg',
              'accepted','reason','rmse_m','euclidean_rmse_m','inlier_ratio','condition','x','y','yaw']
    with (args.output / 'localization_runs.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for group in ('queries','crosspass_queries'):
            for query in result[group]:
                for trial in query['trials']:
                    row = {key:trial.get(key) for key in fields}
                    row.update(group=group,scan_index=query['scan_index'],timestamp_ns=query['timestamp_ns'],
                               x=trial['pose'][0],y=trial['pose'][1],yaw=trial['pose'][2])
                    writer.writerow(row)
    print(json.dumps({key:result[key] for key in ('queries_summary','crosspass_queries_summary','elapsed_s')}, indent=2))


if __name__ == '__main__':
    main()
