"""Compare exact exported PCDs using full-map repeat-observation stability.

This deliberately does NOT implement temporal-exclusion held-out validation.
The full map contains nearby observations of the same scene. No cross-pass or
absolute-accuracy score is reported. Existing separated-reference tests remain
unchanged. Frozen query scans, baseline-frame seeds and ICP gates are reused.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))

import numpy as np
import evaluate_localization as evaluator
from indoor_mapping.geometry import pose_between, scan_to_points, transform_points, voxel_downsample, wrap_angle


SCOPE = 'full-map repeat-observation stability'
GATES = {'minimum_overlap': .5, 'maximum_point_to_line_rmse_m': .1,
         'maximum_condition': 5000., 'agreement_translation_m': .15,
         'agreement_yaw_deg': 3., 'icp_distance_m': .8, 'icp_iterations': 45}


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def read_exact_planar_pcd(path):
    """Read exported XYZ coordinates without geometry regeneration or filtering."""
    raw = Path(path).read_bytes()
    stream = io.StringIO(raw.decode('ascii'))
    header = {}
    for line in stream:
        parts = line.split()
        if not parts or parts[0].startswith('#'):
            continue
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == 'DATA':
            break
    if header.get('FIELDS') != ['x', 'y', 'z'] or header.get('DATA') != ['ascii']:
        raise ValueError(f'{path}: expected exported XYZ ASCII PCD')
    if header.get('SIZE') != ['4'] * 3 or header.get('TYPE') != ['F'] * 3 or header.get('COUNT') != ['1'] * 3:
        raise ValueError(f'{path}: expected scalar float32 XYZ export header')
    points = np.loadtxt(stream).reshape(-1, 3)
    if not len(points) or len(points) != int(header['POINTS'][0]):
        raise ValueError(f'{path}: empty PCD or point count mismatch')
    if len(points) != int(header['WIDTH'][0]) * int(header['HEIGHT'][0]):
        raise ValueError(f'{path}: PCD shape metadata mismatch')
    if not np.all(np.isfinite(points)) or np.any(points[:, 2] != 0):
        raise ValueError(f'{path}: finite planar LiDAR z=0 coordinates required')
    return points, hashlib.sha256(raw).hexdigest()


def verify_training_separation(data, protocol, trajectory, map_metadata, metrics):
    """Reproduce only the documented insertion-ID rule, never map coordinates."""
    times, poses = evaluator.load_trajectory(trajectory)
    raw_times = np.asarray(data['times_ns'], np.int64)
    if hashlib.sha256(raw_times.tobytes()).hexdigest() != protocol['raw_timestamp_sha256']:
        raise ValueError('Raw timestamp array differs from the frozen protocol')
    if len(raw_times) != protocol['raw_scan_count']:
        raise ValueError('Raw scan count differs from the frozen protocol')
    ids = np.searchsorted(raw_times, times)
    if np.any(ids >= len(raw_times)) or not np.array_equal(raw_times[ids], times):
        raise ValueError('Every map trajectory pose must have an exact raw timestamp')
    if not np.array_equal(ids, protocol['all_odometry_scan_indices']):
        raise ValueError('Map trajectory scan IDs differ from frozen odometry IDs')
    if int(times[0]) != int(map_metadata['origin_timestamp_ns']):
        raise ValueError('Map and trajectory origins do not match')
    if map_metadata.get('unit') != 'metre' or map_metadata.get('frame_id') != 'map':
        raise ValueError('Expected metre coordinates in the original map frame')
    positions = [0]
    for index in range(1, len(times)):
        previous = positions[-1]
        moved = np.linalg.norm(poses[index, :2] - poses[previous, :2]) > .06
        turned = abs(wrap_angle(poses[index, 2] - poses[previous, 2])) > .04
        if int(times[index]) - int(times[previous]) >= 400_000_000 and (moved or turned):
            positions.append(index)
    insertion_ids = ids[positions]
    if len(insertion_ids) != metrics['map_insertions']:
        raise ValueError('Recomputed map insertion count differs from original metrics')
    queries = protocol['queries']
    query_ids = np.array([query['scan_index'] for query in queries], dtype=int)
    if len(set(query_ids)) != len(query_ids) or len(queries) != 100 or len(protocol['perturbations']) != 8:
        raise ValueError('This comparison requires the frozen 100 distinct queries and eight starts')
    for query in queries:
        if int(raw_times[query['scan_index']]) != int(query['timestamp_ns']):
            raise ValueError('Frozen query index and timestamp disagree')
        if len(query['seed_pose']) != 3 or not np.all(np.isfinite(query['seed_pose'])):
            raise ValueError('Frozen query seed must be a finite SE(2) pose')
    overlap = np.intersect1d(query_ids, insertion_ids)
    if len(overlap) or len(np.intersect1d(query_ids, ids)):
        raise ValueError('A query scan occurs in map insertion or odometry input IDs')
    nearest_seconds = np.min(np.abs(raw_times[query_ids, None] - raw_times[insertion_ids][None, :]), axis=1) * 1e-9
    return {'passed': True, 'query_count': len(query_ids), 'odometry_scan_count': len(ids),
            'map_insertion_count': len(insertion_ids), 'query_insertion_overlap': overlap.tolist(),
            'actual_map_insertion_scan_indices': insertion_ids.tolist(),
            'insertion_provenance': 'Recomputed from final trajectory using the production insertion rule; count matches original metrics. No PCD geometry was regenerated.',
            'insertion_rule': 'At least0.4s since previous insertion and translation>0.06m or wrapped yaw change>0.04rad',
            'nearest_map_insertion_time_difference_s': evaluator.distribution(nearest_seconds),
            'queries_with_a_map_insertion_within_5s': int(np.sum(nearest_seconds < 5.)),
            'temporal_exclusion_applied': False,
            'limitation': 'Raw query IDs are absent from map insertions, but the full PCD contains temporally nearby observations. This is not the original separated-reference test.'}


def summary_without_heldout_labels(queries):
    summary = evaluator.summarize(queries)
    summary['point_to_line_fit_rmse_m'] = summary.pop('heldout_point_to_line_rmse_m')
    summary['euclidean_fit_rmse_m'] = summary.pop('heldout_euclidean_rmse_m')
    return summary


def evaluate_export(points, query_points, protocol, name):
    started = time.monotonic()
    results = []
    xy = points[:, :2]
    for number, query in enumerate(protocol['queries']):
        anchor = np.asarray(query['seed_pose'], dtype=float)
        # The exact exported cloud is transformed into the common frozen seed
        # frame, then cropped/aggregated identically for all PCD candidates.
        local = transform_points(xy, pose_between(anchor, np.zeros(3)))
        cropped = local[np.linalg.norm(local, axis=1) <= 14.]
        target = voxel_downsample(cropped, .07)
        source = query_points[query['scan_index']]
        trials = evaluator.localize_query(source, target, protocol['perturbations'])
        agreement = evaluator.pose_agreement([trial['pose'] for trial in trials if trial['accepted']], len(trials))
        results.append({'scan_index': query['scan_index'], 'timestamp_ns': query['timestamp_ns'],
                        'seed_pose': query['seed_pose'], 'source_point_count': len(source),
                        'full_pcd_crop_point_count': len(cropped), 'registration_target_point_count': len(target),
                        'trial_pose_frame': 'frozen baseline query-seed frame',
                        'trials': trials, 'agreement': agreement})
        if number % 20 == 0:
            print(f'{name}: {number + 1}/{len(protocol["queries"])} queries', flush=True)
    return {'scope': SCOPE, 'variant': name, 'queries': results,
            'queries_summary': summary_without_heldout_labels(results), 'elapsed_s': time.monotonic() - started}


def paired_changes(candidate, original):
    gained, lost, stable_gains, stable_losses, residual_changes = [], [], [], [], []
    for before, after in zip(original['queries'], candidate['queries']):
        if before['scan_index'] != after['scan_index'] or before['seed_pose'] != after['seed_pose']:
            raise ValueError('Paired query identities or frozen seeds changed')
        scan_id = before['scan_index']
        if after['agreement']['all_starts_agree'] and not before['agreement']['all_starts_agree']:
            stable_gains.append(scan_id)
        if before['agreement']['all_starts_agree'] and not after['agreement']['all_starts_agree']:
            stable_losses.append(scan_id)
        for one, two in zip(before['trials'], after['trials']):
            if one['initialization_id'] != two['initialization_id'] or one['initial_pose'] != two['initial_pose']:
                raise ValueError('Frozen initial perturbations changed')
            identity = [scan_id, one['initialization_id']]
            if two['accepted'] and not one['accepted']:
                gained.append(identity)
            if one['accepted'] and not two['accepted']:
                lost.append(identity)
            if one['accepted'] and two['accepted']:
                residual_changes.append(two['rmse_m'] - one['rmse_m'])
    return {'accepted_trial_gains': len(gained), 'accepted_trial_losses': len(lost),
            'accepted_gain_scan_initialization_ids': gained, 'accepted_loss_scan_initialization_ids': lost,
            'stable_query_gains': len(stable_gains), 'stable_query_losses': len(stable_losses),
            'stable_gain_scan_ids': stable_gains, 'stable_loss_scan_ids': stable_losses,
            'common_accepted_fit_rmse_delta_m': evaluator.distribution(residual_changes),
            'common_accepted_improved_more_than_1mm': sum(value < -.001 for value in residual_changes),
            'common_accepted_worsened_more_than_1mm': sum(value > .001 for value in residual_changes)}


def write_runs(path, result):
    with path.open('w', newline='', encoding='utf-8') as stream:
        fields = ['scan_index', 'initialization_id', 'accepted', 'reason', 'rmse_m', 'inlier_ratio',
                  'x_seed_frame', 'y_seed_frame', 'yaw_seed_frame', 'registration_target_point_count']
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for query in result['queries']:
            for trial in query['trials']:
                writer.writerow({'scan_index': query['scan_index'],
                    **{key: trial[key] for key in ('initialization_id', 'accepted', 'reason', 'rmse_m', 'inlier_ratio')},
                    'x_seed_frame': trial['pose'][0], 'y_seed_frame': trial['pose'][1], 'yaw_seed_frame': trial['pose'][2],
                    'registration_target_point_count': query['registration_target_point_count']})


def write_summary(output, results, provenance):
    summary = {'scope': SCOPE, 'schema': 'exported_pcd_repeat_observation_v1',
               'provenance': provenance, 'variants': {name: {key: value for key, value in result.items() if key != 'queries'}
                                                    for name, result in results.items()},
               'interpretation': 'Development diagnostic of the exported map artifacts. No five-second exclusion, no earlier-pass-only metric, no independent absolute accuracy claim.',
               'default_map_changed': False}
    write_json(output / 'exported_validation.json', summary)
    lines = ['# Exported PCDs: full-map repeat-observation stability', '',
             '**This is not the temporally excluded reference test and does not measure absolute localization accuracy.**', '',
             'The exact exported XYZ coordinates are used as targets. Every method uses the same frozen100 raw query scans and eight baseline-frame initial perturbations. '
             'Queries retain the original0.06m source preprocessing. The exported target is transformed into each frozen seed frame, cropped to14m and aggregated at0.07m, '
             'with unchanged ICP/acceptance gates. No map is regenerated and no camera geometry enters.', '',
             f"The reconstructed production insertion list contains{provenance['training_separation']['map_insertion_count']} scans and shares zero IDs with the queries. "
             f"However,{provenance['training_separation']['queries_with_a_map_insertion_within_5s']}/100 queries have a map insertion within5s. "
             'The full map includes nearby observations of the same scene; temporal/cross-pass claims from the separate evaluator do not apply here.', '',
             '| Exact exported artifact | Points | Accepted /800 | Stable /100 | Conditional translation spread p95(m) | Accepted line-fit median(m) |',
             '|---|---:|---:|---:|---:|---:|']
    with (output / 'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['variant', 'pcd_points', 'accepted_of_800', 'all_eight_stable_of_100', 'conditional_translation_spread_p95_m', 'accepted_line_fit_median_m'])
        for name, result in results.items():
            s = result['queries_summary']
            row = [name, result['pcd_points'], s['accepted_registrations'], s['all_starts_agree_queries'],
                   s['query_max_pairwise_translation_m']['p95'], s['point_to_line_fit_rmse_m']['median']]
            writer.writerow(row)
            lines.append('| ' + ' | '.join(str(value) for value in row) + ' |')
    lines += ['', 'Stable requires all eight registrations to pass and agree within0.15m and3degrees. '
              'Rejections stay in the denominator. Conditional spread includes only queries with at least two accepted solutions; '
              'fit residual is not pose error against truth.', '', '| Candidate vs original | Accepted gains /losses | Stable-query gains /losses |', '|---|---:|---:|']
    for name, result in results.items():
        if name == 'original':
            continue
        paired = result['paired_against_original']
        lines.append(f"| {name} | {paired['accepted_trial_gains']} /{paired['accepted_trial_losses']} | {paired['stable_query_gains']} /{paired['stable_query_losses']} |")
    lines += ['', 'This reused development diagnostic supports inspecting artifact-specific stability, not automatic promotion. '
              'The original map remains unchanged. Interpret alongside the separate temporally excluded reference-only tests and spatial feature loss; '
              'no independent physical-reference measurement was provided.', '']
    (output / 'exported_validation.md').write_text('\n'.join(lines), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=Path('outputs/fin301/cache/scans.npz'))
    parser.add_argument('--protocol', type=Path, default=Path('outputs/fin301/verification/localization_protocol.json'))
    parser.add_argument('--trajectory', type=Path, default=Path('outputs/fin301/result/trajectory_arrays.npz'))
    parser.add_argument('--map-metadata', type=Path, default=Path('outputs/fin301/result/map_metadata.json'))
    parser.add_argument('--metrics', type=Path, default=Path('outputs/fin301/result/metrics.json'))
    parser.add_argument('--output', type=Path, default=Path('outputs/fin301/filtering/exported_validation'))
    parser.add_argument('--pcd', action='append', metavar='NAME=PATH', help='Repeat for exact PCDs; requires original first')
    args = parser.parse_args()
    specs = args.pcd or ['original=outputs/fin301/result/lidar_scan_map.pcd',
                        'clean_conservative=outputs/fin301/filtering/lidar_scan_map_clean_conservative.pcd',
                        'plainradius_balanced=outputs/fin301/filtering/lidar_scan_map_clean_radius_balanced.pcd']
    paths = {}
    for spec in specs:
        name, separator, path = spec.partition('=')
        if not separator or not re.fullmatch('[A-Za-z0-9_-]+', name) or name in paths:
            raise ValueError('Each --pcd must be a distinct NAME=PATH')
        paths[name] = Path(path)
    if next(iter(paths)) != 'original':
        raise ValueError('The first exported PCD must be named original')
    started = time.monotonic()
    protocol = json.loads(args.protocol.read_text(encoding='utf-8'))
    metadata = json.loads(args.map_metadata.read_text(encoding='utf-8'))
    metrics = json.loads(args.metrics.read_text(encoding='utf-8'))
    sources = {'scan_cache': args.cache, 'protocol': args.protocol, 'trajectory': args.trajectory,
               'map_metadata': args.map_metadata, 'map_metrics': args.metrics,
               'registration': ROOT / 'indoor_mapping/geometry.py', 'original_evaluator': Path(evaluator.__file__),
               'production_pipeline': ROOT / 'indoor_mapping/pipeline.py', 'export_evaluator': Path(__file__),
               **{f'pcd_{name}': path for name, path in paths.items()}}
    hashes = {name: evaluator.file_hash(path) for name, path in sources.items()}
    if hashes['scan_cache'] != protocol['scan_cache_sha256']:
        raise ValueError('Scan cache differs from the frozen protocol')
    with np.load(args.cache) as archive:
        data = {key: archive[key] for key in archive.files}
    separation = verify_training_separation(data, protocol, args.trajectory, metadata, metrics)
    clouds = {}
    for name, path in paths.items():
        points, digest = read_exact_planar_pcd(path)
        if digest != hashes[f'pcd_{name}']:
            raise ValueError('PCD changed while being read')
        if name == 'original':
            if len(points) != metadata['lidar_point_count']:
                raise ValueError('Original PCD count differs from metadata')
            original_coordinates = {tuple(point) for point in points}
        else:
            if len(points) >= len(clouds['original']) or len({tuple(point) for point in points}) != len(points):
                raise ValueError('Filtered PCD must be a strict subset without duplicates')
            if any(tuple(point) not in original_coordinates for point in points):
                raise ValueError('Filtered PCD has moved or invented XYZ coordinates')
        clouds[name] = points
    query_points = {query['scan_index']: scan_to_points(data['ranges'][query['scan_index']],
                    float(data['angle_min']), float(data['angle_increment']), max(.12, float(data['range_min'])),
                    min(12., float(data['range_max'])), voxel_size=.06) for query in protocol['queries']}
    provenance = {'scope': SCOPE, 'source_paths': {name: str(path) for name, path in sources.items()},
                  'source_sha256': hashes, 'training_separation': separation,
                  'source_query_voxel_m': .06, 'target_crop_radius_m': 14., 'target_registration_voxel_m': .07,
                  'exact_exported_coordinates_used': True, 'map_geometry_regenerated': False,
                  'query_seed_policy': protocol['selection']['seed_policy'],
                  'quality_gates': GATES, 'temporal_exclusion_applied': False, 'crosspass_metrics_computed': False,
                  'absolute_accuracy_validated': False}
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output / 'provenance.json', provenance)
    results = {}
    for name, points in clouds.items():
        result = evaluate_export(points, query_points, protocol, name)
        result.update(pcd_path=str(paths[name]), pcd_sha256=hashes[f'pcd_{name}'], pcd_points=len(points),
                      protocol_sha256=hashes['protocol'], quality_gates=GATES,
                      temporal_exclusion_applied=False, crosspass_metrics_computed=False,
                      absolute_accuracy_validated=False)
        if name != 'original':
            result['paired_against_original'] = paired_changes(result, results['original'])
        results[name] = result
        directory = args.output / name
        write_json(directory / 'localization_evaluation.json', result)
        write_runs(directory / 'localization_runs.csv', result)
        print(json.dumps({'variant': name, 'summary': result['queries_summary'], 'elapsed_s': result['elapsed_s']}), flush=True)
    for name, path in sources.items():
        if evaluator.file_hash(path) != hashes[name]:
            raise ValueError(f'Source changed during evaluation: {path}')
    provenance['elapsed_s'] = time.monotonic() - started
    provenance['sources_unchanged_after_evaluation'] = True
    write_json(args.output / 'provenance.json', provenance)
    write_summary(args.output, results, provenance)
    print(f'Completed {SCOPE}: {args.output}', flush=True)


if __name__ == '__main__':
    main()
