"""Compare conservative map filters using only frozen, allowed reference scans.

The original evaluator and estimator files are imported unchanged. A temporary
target-voxelization hook inserts reference-only filtering; query scans, seeds,
registration and quality gates retain the original development protocol.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import csv
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import evaluate_localization as evaluator


def json_ready(value):
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(json_ready(value), indent=2, allow_nan=False), encoding='utf-8')


@contextmanager
def reference_target_hook(records, preset=None, input_voxel_m=.03):
    """Intercept only evaluate()'s assembled reference target, never raw scans.

    scan_to_points and register_scan keep their own geometry-module globals, so
    replacing this evaluator-local imported name cannot change query processing.
    Execution is sequential and the original callable is restored on all exits.
    """
    original = evaluator.voxel_downsample
    if preset:
        from indoor_mapping.map_filter import filter_points

    def filtered_target(points, voxel_size):
        if float(voxel_size) != .07:
            raise ValueError('Unexpected evaluator hook: only 0.07 m target voxelization may be intercepted')
        prepared = original(points, input_voxel_m)
        kept_xy = prepared
        stats = {}
        if preset:
            xyz = np.column_stack([prepared, np.zeros(len(prepared))])
            retained, stats, mask = filter_points(xyz, preset)
            mask = np.asarray(mask)
            retained = np.asarray(retained)
            if mask.dtype != bool or mask.shape != (len(xyz),):
                raise ValueError('The map filter must return one Boolean keep decision per input point')
            if retained.shape != (int(mask.sum()), 3) or not np.array_equal(retained, xyz[mask]):
                raise ValueError('The map filter must preserve an exact ordered subset without moving points')
            kept_xy = retained[:, :2]
        target = original(kept_xy, voxel_size)
        records.append({'assembled_point_count': len(points), 'input_003m_point_count': len(prepared),
                        'retained_003m_point_count': len(kept_xy), 'removed_003m_point_count': len(prepared)-len(kept_xy),
                        'removed_fraction': (len(prepared)-len(kept_xy))/len(prepared) if len(prepared) else 0.,
                        'registration_007m_point_count': len(target), 'filter_stats': json_ready(stats)})
        return target

    evaluator.voxel_downsample = filtered_target
    try:
        yield
    finally:
        evaluator.voxel_downsample = original


def evaluate_variant(data, trajectory_times, trajectory_poses, protocol, name):
    records = []
    start = time.monotonic()
    if name == 'original':
        result = evaluator.evaluate(data, trajectory_times, trajectory_poses, protocol, 'all_returns')
    else:
        with reference_target_hook(records, None if name == 'spacing_control' else name):
            result = evaluator.evaluate(data, trajectory_times, trajectory_poses, protocol, 'all_returns')
        expected_count = sum(len(result[group]) for group in ('queries', 'crosspass_queries'))
        if len(records) != expected_count:
            raise ValueError('Reference hook count differs from the frozen query count')
        for query, record in zip([query for group in ('queries', 'crosspass_queries') for query in result[group]], records):
            query['reference_filtering'] = record
    result['elapsed_s'] = time.monotonic() - start
    result['filter_variant'] = name
    result['reference_filtering'] = {
        'placement': ('none: original assembled targets voxelized directly to 0.07 m' if name == 'original' else
                      'allowed reference scans only; crop to 14 m, aggregate at 0.03 m, filter exact subset, aggregate at 0.07 m'),
        'source_scan_voxel_m': .06, 'registration_target_voxel_m': .07,
        'filter_input_voxel_m': None if name == 'original' else .03,
        'uses_full_map_membership': False, 'changes_query_scans': False,
        'removed_fraction': evaluator.distribution([record['removed_fraction'] for record in records]),
        'point_count_003m_before': sum(record['input_003m_point_count'] for record in records),
        'point_count_003m_after': sum(record['retained_003m_point_count'] for record in records),
        'caveat': ('Each target contains only its permitted reference observations, already reduced per scan to 0.06 m. '
                   'Its density and local voxel orientation differ from the full 0.03 m map. This tests the filter procedure '
                   'without temporal leakage; it is not a direct accuracy evaluation of the exported filtered PCD.')}
    return result


def paired_changes(candidate, control):
    comparisons = {}
    for group in ('queries', 'crosspass_queries'):
        new = {query['scan_index']: query for query in candidate[group]}
        old = {query['scan_index']: query for query in control[group]}
        if new.keys() != old.keys():
            raise ValueError('Paired comparison requires identical query IDs')
        gained_stability, lost_stability, gained_trials, lost_trials = [], [], [], []
        residual_changes, pose_changes = [], []
        for scan_id in old:
            before, after = old[scan_id], new[scan_id]
            if before['reference_scan_indices'] != after['reference_scan_indices'] or before['seed_pose'] != after['seed_pose']:
                raise ValueError('Reference IDs or frozen seed changed')
            stable_before = before['agreement']['all_starts_agree']
            stable_after = after['agreement']['all_starts_agree']
            if stable_after and not stable_before:
                gained_stability.append(scan_id)
            if stable_before and not stable_after:
                lost_stability.append(scan_id)
            if len(before['trials']) != len(after['trials']):
                raise ValueError('Initialization count changed')
            for old_trial, new_trial in zip(before['trials'], after['trials']):
                if old_trial['initialization_id'] != new_trial['initialization_id'] or old_trial['initial_pose'] != new_trial['initial_pose']:
                    raise ValueError('Frozen initialization changed')
                identity = [scan_id, old_trial['initialization_id']]
                if new_trial['accepted'] and not old_trial['accepted']:
                    gained_trials.append(identity)
                if old_trial['accepted'] and not new_trial['accepted']:
                    lost_trials.append(identity)
                if new_trial['accepted'] and old_trial['accepted']:
                    residual_changes.append(new_trial['rmse_m']-old_trial['rmse_m'])
                    pose_changes.append(float(np.linalg.norm(np.asarray(new_trial['pose'][:2])-old_trial['pose'][:2])))
        comparisons[group] = {
            'newly_stable_queries': len(gained_stability), 'lost_stable_queries': len(lost_stability),
            'newly_stable_scan_ids': gained_stability, 'lost_stable_scan_ids': lost_stability,
            'newly_accepted_trials': len(gained_trials), 'newly_rejected_trials': len(lost_trials),
            'newly_accepted_scan_and_initialization_ids': gained_trials,
            'newly_rejected_scan_and_initialization_ids': lost_trials,
            'common_accepted_count': len(residual_changes),
            'common_accepted_line_rmse_change_m': evaluator.distribution(residual_changes),
            'common_accepted_line_rmse_improved_over_1mm': sum(value < -.001 for value in residual_changes),
            'common_accepted_line_rmse_worsened_over_1mm': sum(value > .001 for value in residual_changes),
            'common_accepted_solution_shift_m': evaluator.distribution(pose_changes),
            'solution_shift_is_error_against_ground_truth': False}
    return comparisons


def original_reproduction(actual, expected):
    comparisons = paired_changes(actual, expected)
    summary_equal = all(actual[group + '_summary'] == expected[group + '_summary'] for group in ('queries', 'crosspass_queries'))
    maximum_pose_difference = 0.
    for group in ('queries', 'crosspass_queries'):
        for a, b in zip(actual[group], expected[group]):
            for one, two in zip(a['trials'], b['trials']):
                maximum_pose_difference = max(maximum_pose_difference, float(np.max(np.abs(np.asarray(one['pose'])-two['pose']))))
    report = {'summary_numbers_exactly_equal': summary_equal, 'maximum_trial_pose_difference': maximum_pose_difference,
              'paired_changes': comparisons}
    if not summary_equal or maximum_pose_difference != 0.:
        raise ValueError('Unfiltered reproduction differs from the frozen final evaluation; stop before interpreting filters')
    return report


def write_runs(path, result):
    fields = ['group', 'scan_index', 'initialization_id', 'accepted', 'reason', 'rmse_m', 'inlier_ratio',
              'x', 'y', 'yaw', 'reference_point_count', 'removed_fraction']
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for group in ('queries', 'crosspass_queries'):
            for query in result[group]:
                for trial in query['trials']:
                    writer.writerow({'group': group, 'scan_index': query['scan_index'],
                        **{key: trial[key] for key in ('initialization_id', 'accepted', 'reason', 'rmse_m', 'inlier_ratio')},
                        'x': trial['pose'][0], 'y': trial['pose'][1], 'yaw': trial['pose'][2],
                        'reference_point_count': query['reference_point_count'],
                        'removed_fraction': query.get('reference_filtering', {}).get('removed_fraction', 0.)})


def assess_candidates(results):
    assessments = {}
    for name, result in results.items():
        if name in ('original', 'spacing_control'):
            continue
        nondegrading = all(
            result[group+'_summary'][metric] >= results[control][group+'_summary'][metric]
            for control in ('original', 'spacing_control') for group in ('queries', 'crosspass_queries')
            for metric in ('accepted_registrations', 'all_starts_agree_queries'))
        stable_gain = any(
            result[group+'_summary']['all_starts_agree_queries'] > results[control][group+'_summary']['all_starts_agree_queries']
            for control in ('original', 'spacing_control') for group in ('queries', 'crosspass_queries'))
        assessments[name] = {'no_count_degradation_against_either_control': nondegrading,
                             'at_least_one_stability_count_gain': stable_gain,
                             'eligible_for_further_review': nondegrading and stable_gain}
    eligible = [name for name, assessment in assessments.items() if assessment['eligible_for_further_review']]
    return {'default_recommendation': 'retain_original', 'candidate_assessments': assessments,
            'eligible_for_further_review': eligible,
            'reason': ('No filter demonstrates nondegrading acceptance and stability against both controls plus a stability gain.'
                       if not eligible else 'Some filters qualify for further review, but this reused development set alone does not justify replacing the localization map.'),
            'selection_scope': 'Internal development validation used to inspect filtering; not an untouched test or physical accuracy evidence.'}


def markdown_summary(results, recommendation, reproduction):
    lines = ['# Reference-only map filtering: development validation', '',
             '**No independent pose truth is available. The original localization map remains the default.**', '',
             'All methods use the final LiDAR trajectory and the same frozen 100 queries, 20 earlier-pass queries, reference scan IDs and eight starts. '
             'The query scans are absent from odometry/map insertions; reference subsets preserve the 5 s temporal and 0.5 m origin exclusion. '
             'No full-map keep mask or camera points enter evaluation.', '',
             'The original control is the unchanged evaluator. The spacing control adds only 0.03 m aggregation before its original 0.07 m target aggregation. '
             'Filters act between those two stages. Existing per-scan 0.06 m preprocessing is unchanged. Reference-only density is not identical to the complete exported 0.03 m map.', '',
             f"Exact original reproduction: **{reproduction['summary_numbers_exactly_equal']}**; maximum trial-pose difference: {reproduction['maximum_trial_pose_difference']:.3g}.", '',
             '| Configuration | Accepted / starts | Stable / queries | Conditional spread p95 (m) | Earlier accepted / starts | Earlier stable / queries | Median filtered fraction |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for name, result in results.items():
        a, b = result['queries_summary'], result['crosspass_queries_summary']
        fraction = result['reference_filtering']['removed_fraction']['median']
        spread = a['query_max_pairwise_translation_m']['p95']
        lines.append(f"| {name} | {a['accepted_registrations']}/{a['attempted_initializations']} | {a['all_starts_agree_queries']}/{a['query_count']} | "
                     f"{spread:.6f} | {b['accepted_registrations']}/{b['attempted_initializations']} | {b['all_starts_agree_queries']}/{b['query_count']} | "
                     f"{100*(fraction or 0):.2f}% |")
    lines += ['', 'Stable requires all eight starts to pass the fixed gates and agree within 0.15 m and 3 degrees. '
              'Acceptance failures remain in the denominator. Spread uses only queries with at least two accepted solutions; '
              'a lower conditional spread can hide rejected or ambiguous cases. Accepted-fit residual is not absolute pose error.', '',
              '## Paired changes', '', '| Candidate / control | Main stable gains / losses | Main accepted gains / losses | Earlier stable gains / losses | Earlier accepted gains / losses |',
              '|---|---:|---:|---:|---:|']
    for name, result in results.items():
        for control, comparison in result.get('paired_against', {}).items():
            a, b = comparison['queries'], comparison['crosspass_queries']
            lines.append(f"| {name} / {control} | {a['newly_stable_queries']} / {a['lost_stable_queries']} | "
                         f"{a['newly_accepted_trials']} / {a['newly_rejected_trials']} | {b['newly_stable_queries']} / {b['lost_stable_queries']} | "
                         f"{b['newly_accepted_trials']} / {b['newly_rejected_trials']} |")
    lines += ['', '## Recommendation', '', recommendation['reason'], '',
              'Unsupported isolated returns can be clutter or useful sparse structure. Visual cleanup alone does not justify deleting localization evidence. '
              'The validation reuses the method-selection development set and baseline-derived spatial selection/initialization. '
              'It does not establish physical accuracy or performance on unseen recordings. Full per-query failures, acceptance changes, '
              'conditional residual distributions and source hashes are preserved in the JSON/CSV files.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=Path('outputs/fin301/cache'))
    parser.add_argument('--trajectory', type=Path, default=Path('outputs/fin301/result/trajectory.csv'))
    parser.add_argument('--protocol', type=Path, default=Path('outputs/fin301/verification/localization_protocol.json'))
    parser.add_argument('--original-evaluation', type=Path, default=Path('outputs/fin301/verification/localization_final/localization_evaluation.json'))
    parser.add_argument('--output', type=Path, default=Path('outputs/fin301/filtering/validation'))
    parser.add_argument('--presets', nargs='+', default=['conservative', 'balanced'])
    parser.add_argument('--reuse-controls', action='store_true', help='Reuse verified original/spacing controls already in the output directory')
    args = parser.parse_args()
    if len(args.presets) > 4 or len(set(args.presets)) != len(args.presets) or any(name in ('original', 'spacing_control') for name in args.presets):
        raise ValueError('Choose at most four distinct filter presets; controls are always included')
    started = time.monotonic()
    args.output.mkdir(parents=True, exist_ok=True)
    expected = json.loads(args.original_evaluation.read_text(encoding='utf-8'))
    protocol = json.loads(args.protocol.read_text(encoding='utf-8'))
    cache = args.cache/'scans.npz'
    sources = {'scan_cache': cache, 'trajectory': args.trajectory, 'protocol': args.protocol,
               'original_evaluation': args.original_evaluation, 'original_evaluator': Path(evaluator.__file__),
               'registration': ROOT/'indoor_mapping/geometry.py', 'filter_evaluator': Path(__file__)}
    hashes = {key: evaluator.file_hash(path) for key, path in sources.items()}
    for actual_key, expected_key in [('scan_cache', 'scan_cache_sha256'), ('trajectory', 'trajectory_sha256'),
                                     ('protocol', 'protocol_sha256'), ('original_evaluator', 'evaluator_sha256'),
                                     ('registration', 'registration_source_sha256')]:
        if hashes[actual_key] != expected[expected_key]:
            raise ValueError(f'{actual_key} differs from the frozen final evaluation')
    if hashes['scan_cache'] != protocol['scan_cache_sha256']:
        raise ValueError('Frozen protocol does not match the raw scan cache')
    with np.load(cache) as archive:
        data = {key: archive[key] for key in archive.files}
    if hashlib.sha256(data['times_ns'].tobytes()).hexdigest() != protocol['raw_timestamp_sha256']:
        raise ValueError('Raw timestamps differ from the frozen protocol')
    odometry_ids = set(protocol['all_odometry_scan_indices'])
    training_ids = set(protocol['training_scan_indices'])
    for group in ('queries', 'crosspass_queries'):
        for query in protocol[group]:
            if query['scan_index'] in odometry_ids or not set(query['reference_scan_indices']) <= training_ids:
                raise ValueError('Query/reference separation is invalid')
            if query['timestamp_ns'] != int(data['times_ns'][query['scan_index']]):
                raise ValueError('Query index does not match its frozen timestamp')
            differences = np.abs(data['times_ns'][query['reference_scan_indices']] - query['timestamp_ns'])
            if np.any(differences < 5_000_000_000):
                raise ValueError('Reference temporal exclusion was violated')
    times, poses = evaluator.load_trajectory(args.trajectory)
    results = {}
    reproduction = None
    for name in ['original', 'spacing_control'] + args.presets:
        directory = args.output/name
        directory.mkdir(parents=True, exist_ok=True)
        if args.reuse_controls and name in ('original', 'spacing_control'):
            result = json.loads((directory/'localization_evaluation.json').read_text(encoding='utf-8'))
            for key in ('scan_cache', 'trajectory', 'protocol', 'original_evaluation', 'original_evaluator', 'registration'):
                if result['source_sha256'][key] != hashes[key]:
                    raise ValueError(f'Reusable control has changed inputs: {name}/{key}')
            for key, snapshot_path in result.get('source_snapshots', {}).items():
                if evaluator.file_hash(snapshot_path) != result['source_sha256'][key]:
                    raise ValueError(f'Reusable control source snapshot differs: {snapshot_path}')
            if not result.get('source_snapshots'):
                raise ValueError('Reusable controls require their original source snapshots')
            if name == 'original':
                reproduction = original_reproduction(result, expected)
            results[name] = result
            print(f'Reused exact {name} control with preserved original source snapshots', flush=True)
            continue
        if name not in ('original', 'spacing_control'):
            sources['filter_module'] = ROOT/'indoor_mapping/map_filter.py'
            hashes['filter_module'] = evaluator.file_hash(sources['filter_module'])
        snapshots = {}
        snapshot_dir = directory/'source_snapshot'
        snapshot_dir.mkdir(exist_ok=True)
        for key, path in sources.items():
            if path.suffix == '.py':
                snapshot = snapshot_dir/(key+'.py')
                snapshot.write_bytes(path.read_bytes())
                if evaluator.file_hash(snapshot) != hashes[key]:
                    raise ValueError(f'Source changed before evaluation: {path}')
                snapshots[key] = str(snapshot)
        print(f'\nEvaluating {name}', flush=True)
        result = evaluate_variant(data, times, poses, protocol, name)
        result.update(trajectory=str(args.trajectory), trajectory_sha256=hashes['trajectory'],
                      protocol=str(args.protocol), protocol_sha256=hashes['protocol'],
                      scan_cache_sha256=hashes['scan_cache'], quality_gates=expected['quality_gates'],
                      absolute_accuracy='UNVALIDATED: development consistency only; no independent physical reference')
        if name == 'original':
            reproduction = original_reproduction(result, expected)
            result['reproduction_of_original_final_evaluation'] = reproduction
        elif name == 'spacing_control':
            result['paired_against'] = {'original': paired_changes(result, results['original'])}
        else:
            result['paired_against'] = {control: paired_changes(result, results[control]) for control in ('original', 'spacing_control')}
        result['source_sha256'] = dict(hashes)
        result['source_snapshots'] = snapshots
        results[name] = result
        write_json(directory/'localization_evaluation.json', result)
        write_runs(directory/'localization_runs.csv', result)
        print(json.dumps({'variant': name, 'accepted': result['queries_summary']['accepted_registrations'],
                          'stable': result['queries_summary']['all_starts_agree_queries'],
                          'earlier_stable': result['crosspass_queries_summary']['all_starts_agree_queries'],
                          'elapsed_s': result['elapsed_s']}, indent=2), flush=True)
    for key, path in sources.items():
        if evaluator.file_hash(path) != hashes[key]:
            raise ValueError(f'Source changed during evaluation: {path}')
    recommendation = assess_candidates(results)
    summary = {'schema': 'reference_only_filter_validation_v1', 'variants': {
        name: {key: result[key] for key in ('queries_summary', 'crosspass_queries_summary', 'reference_filtering', 'elapsed_s')}
        for name, result in results.items()}, 'recommendation': recommendation,
        'original_reproduction': reproduction, 'source_paths': {key: str(path) for key, path in sources.items()},
        'variant_source_snapshots': {name: result.get('source_snapshots', {}) for name, result in results.items()},
        'source_sha256': hashes, 'elapsed_s': time.monotonic()-started,
        'unchanged_primary_artifacts': True, 'absolute_accuracy_validated': False}
    write_json(args.output/'filtering_validation.json', summary)
    (args.output/'filtering_validation.md').write_text(markdown_summary(results, recommendation, reproduction), encoding='utf-8')
    print(json.dumps({'recommendation': recommendation, 'elapsed_s': summary['elapsed_s']}, indent=2))


if __name__ == '__main__':
    main()
