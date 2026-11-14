"""Evaluate a loop-refinement trajectory with frozen scans and matched map types.

The separated-reference development protocol is kept distinct from exact-PCD
repeat-observation diagnostics. Unfiltered targets are compared with unfiltered
controls, and occupancy-supported targets with the same occupancy procedure.
No existing evaluator, original result, or primary map is modified.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import evaluate_localization as separated
from scripts import evaluate_exported_pcd_filtering as exported
from scripts.evaluate_map_filtering import paired_changes as paired_reference
from indoor_mapping.geometry import pose_between, wrap_angle


def load_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def audit_current_loop(baseline):
    events_document = load_json(baseline/'lidar_revisit_events.json')
    events = events_document['events']
    graph = load_json(baseline/'pose_graph.json')
    with np.load(baseline/'trajectory_arrays.npz') as archive:
        before = archive['lidar_only'].copy()
        after = archive['corrected'].copy()
        keys, times = archive['key_ids'].copy(), archive['times_ns'].copy()
    accepted = []
    for event in events:
        if not event.get('accepted'):
            continue
        edge = next(edge for edge in graph['edges'] if edge.get('loop') and edge['i'] == event['i'] and edge['j'] == event['j'])
        residuals = {}
        for label, poses in [('before', before[keys]), ('after', after[keys])]:
            residual = pose_between(poses[edge['i']], poses[edge['j']])-edge['pose']
            residual[2] = wrap_angle(residual[2])
            residuals[label] = {'translation_m': float(np.linalg.norm(residual[:2])),
                                'yaw_deg': float(np.rad2deg(residual[2]))}
        cycle = event['reciprocal_cycle']
        accepted.append({'i': event['i'], 'j': event['j'], 'time_i_s': event['time_i_s'], 'time_j_s': event['time_j_s'],
                         'time_separation_s': (event['timestamp_j_ns']-event['timestamp_i_ns'])/1e9,
                         'measured_relative_pose': event['pose'], 'graph_residuals': residuals,
                         'forward_line_rmse_m': event['forward']['rmse'], 'forward_condition': event['forward']['condition'],
                         'geometric_coverage': event['geometric_coverage'],
                         'agreeing_initializations': event['qualifying_agreeing_initializations'],
                         'tested_initializations': len(event['initialization_trials']),
                         'reciprocal_translation_m': float(np.linalg.norm(cycle[:2])),
                         'reciprocal_yaw_deg': float(np.rad2deg(cycle[2])),
                         'temporal_support': event['temporal_support']})
    chain = {(edge['i'], edge['j']) for edge in graph['edges'] if not edge.get('loop')}
    connected_chain = all((index, index+1) in chain for index in range(len(keys)-1))
    shift = np.linalg.norm(after[:, :2]-before[:, :2], axis=1)
    late_rejected = []
    for event in events:
        if event['reason'] not in ('initial_fit', 'insufficient_geometric_coverage', 'accepted'):
            late_rejected.append({'i': event['i'], 'j': event['j'], 'reason': event['reason'],
                                  'odometry_correction': event.get('odometry_correction'),
                                  'geometric_coverage': event.get('geometric_coverage'),
                                  'temporal_support': event.get('temporal_support', [])})
    return {'baseline': str(baseline), 'accepted_constraints': accepted, 'tested_candidates': len(events),
            'rejection_counts': dict(Counter(event['reason'] for event in events)),
            'parameters': events_document['parameters'], 'late_rejected_candidates': late_rejected,
            'consecutive_odometry_chain_connected': connected_chain,
            'max_graph_correction_m': float(shift.max()),
            'max_correction_elapsed_s': float((times[np.argmax(shift)]-times[0])/1e9),
            'max_consecutive_translation_before_m': float(np.linalg.norm(np.diff(before[:, :2], axis=0), axis=1).max()),
            'max_consecutive_translation_after_m': float(np.linalg.norm(np.diff(after[:, :2], axis=0), axis=1).max()),
            'graph_optimization': graph['optimization'],
            'limitations': [
                'Residual after graph fitting measures agreement with imposed constraints, not independent pose accuracy.',
                'Nearby source-side temporal checks use the same target submap and are correlated evidence.',
                'Three of five initializations supported the current loop; other starts had poor unsupported fits.',
                'A connected pose graph does not establish the physical connectivity of rooms or mapped surfaces.',
                'Nearby rejected 255/258-to-422 pairs and accepted 265-to-423 belong to the same revisit neighborhood.',
                'Legacy metrics.loop_translation_residual_* lists cover camera loops; LiDAR residuals here are recomputed directly.']}


def verify_candidate(result_dir, filtering_dir, baseline, data, protocol, baseline_provenance):
    provenance = load_json(result_dir/'export_provenance.json')
    required = ['trajectory_arrays.npz', 'trajectory.csv', 'map_insertion_indices.json', 'lidar_scan_map.pcd',
                'map.pgm', 'map.yaml', 'map_metadata.json', 'metrics.json', 'pose_graph.json']
    for name in required:
        if separated.file_hash(result_dir/name) != provenance['output_sha256'].get(name):
            raise ValueError(f'Candidate export is incomplete or modified: {name}')
    insertion = load_json(result_dir/'map_insertion_indices.json')
    expected_ids = np.asarray(baseline_provenance['training_separation']['actual_map_insertion_scan_indices'], dtype=int)
    raw_ids = np.asarray(insertion['raw_scan_indices'], dtype=int)
    dense_ids = np.asarray(insertion['dense_indices'], dtype=int)
    if not np.array_equal(raw_ids, expected_ids) or insertion['map_insertions'] != len(expected_ids):
        raise ValueError('Candidate mapping scan IDs differ from the original final map')
    if insertion['baseline_trajectory_sha256'] != separated.file_hash(baseline/'trajectory_arrays.npz'):
        raise ValueError('Insertion manifest refers to a different original final trajectory')
    if insertion['baseline_odometry_sha256'] != separated.file_hash(baseline/'odometry.npz'):
        raise ValueError('Insertion manifest refers to different odometry scan IDs')
    times, poses = separated.load_trajectory(result_dir/'trajectory_arrays.npz')
    old_times, old_poses = separated.load_trajectory(baseline/'trajectory_arrays.npz')
    if not np.array_equal(times, old_times) or not np.allclose(poses[0], old_poses[0], atol=1e-10):
        raise ValueError('Candidate changes the tracked timestamps or anchored map frame')
    if not np.array_equal(times[dense_ids], data['times_ns'][raw_ids]):
        raise ValueError('Dense insertion positions do not match raw scan IDs')
    if not np.array_equal(insertion['timestamp_ns'], times[dense_ids]):
        raise ValueError('Insertion timestamps are inconsistent')
    query_ids = {query['scan_index'] for group in ('queries', 'crosspass_queries') for query in protocol[group]}
    if query_ids.intersection(raw_ids.tolist()) or query_ids.intersection(protocol['all_odometry_scan_indices']):
        raise ValueError('Query observations leaked into map or odometry inputs')
    metadata = load_json(result_dir/'map_metadata.json')
    old_metadata = load_json(baseline/'map_metadata.json')
    for key in ('origin_timestamp_ns', 'frame_id', 'unit'):
        if metadata[key] != old_metadata[key]:
            raise ValueError(f'Candidate map frame metadata differs: {key}')
    filter_report = load_json(filtering_dir/'occupancy_filter_report.json')
    old_filter = load_json(baseline.parent/'filtering/occupancy_filter_report.json')
    if filter_report['parameters'] != old_filter['parameters']:
        raise ValueError('Candidate occupancy filter parameters differ from the matched baseline control')
    for key in ('resolution', 'negate', 'mode', 'occupied_thresh', 'free_thresh'):
        if filter_report['grid_metadata'][key] != old_filter['grid_metadata'][key]:
            raise ValueError(f'Candidate occupancy interpretation differs: {key}')
    if filter_report['source_sha256']['filter_module'] != old_filter['source_sha256']['filter_module']:
        raise ValueError('Candidate occupancy filtering algorithm differs from baseline')
    for key, name in [('source_pcd', 'lidar_scan_map.pcd'), ('source_yaml', 'map.yaml'), ('source_image', 'map.pgm')]:
        if filter_report['source_sha256'][key] != separated.file_hash(result_dir/name):
            raise ValueError(f'Occupancy filter did not use the candidate map: {key}')
    filtered_path = filtering_dir/filter_report['file']
    if separated.file_hash(filtered_path) != filter_report['sha256']:
        raise ValueError('Filtered candidate PCD no longer matches its report')
    plain, _ = exported.read_exact_planar_pcd(result_dir/'lidar_scan_map.pcd')
    occupied, _ = exported.read_exact_planar_pcd(filtered_path)
    point_set = {tuple(point) for point in plain}
    if any(tuple(point) not in point_set for point in occupied) or len(occupied) != filter_report['retained_points']:
        raise ValueError('Occupancy-filtered candidate is not an exact subset of its own new PCD')
    return times, poses, plain, occupied, filtered_path, {
        'frozen_map_insertions': len(raw_ids), 'mapping_scan_ids_equal_original_final': True,
        'query_scan_ids_disjoint': True, 'same_occupancy_rule_as_baseline': True,
        'reference_temporal_exclusion_applies_only_to_separated_protocol': True,
        'full_pcd_temporal_exclusion_applied': False,
        'full_pcd_queries_with_map_insertion_within_5s': baseline_provenance['training_separation']['queries_with_a_map_insertion_within_5s'],
        'candidate_max_translation_from_original_final_m': float(np.linalg.norm(poses[:, :2]-old_poses[:, :2], axis=1).max()),
        'candidate_max_yaw_change_from_original_final_deg': float(np.rad2deg(np.abs(wrap_angle(poses[:, 2]-old_poses[:, 2])).max())),
        'export_provenance': str(result_dir/'export_provenance.json'), 'occupancy_filter_report': str(filtering_dir/'occupancy_filter_report.json')}


def comparison_markdown(comparisons, invariant_checks):
    lines = ['# Loop-refinement development validation', '',
             '**Each candidate uses the same scan observations and is compared with the same map representation. No independent physical accuracy is measured.**', '',
             '| Evaluation scope | Original final accepted / starts | Refined accepted / starts | Original final stable / queries | Refined stable / queries |',
             '|---|---:|---:|---:|---:|']
    for name, pair in comparisons.items():
        for label, key in [('main', 'queries_summary')] + ([('earlier pass', 'crosspass_queries_summary')] if name == 'separated_reference' else []):
            a, b = pair['baseline'][key], pair['candidate'][key]
            lines.append(f"| {name}: {label} | {a['accepted_registrations']}/{a['attempted_initializations']} | {b['accepted_registrations']}/{b['attempted_initializations']} | "
                         f"{a['all_starts_agree_queries']}/{a['query_count']} | {b['all_starts_agree_queries']}/{b['query_count']} |")
    lines += ['', 'The separated-reference protocol preserves 100 queries plus 20 earlier-pass queries, frozen reference IDs, the 5 s temporal exclusion, '
              'and the same eight initial perturbations. Its reference geometry changes only through the proposed trajectory. '
              'The other two rows use full exported maps and are repeat-observation diagnostics, with no temporal exclusion or earlier-pass score. '
              f"Their fixed map observations include nearby insertions for {invariant_checks['full_pcd_queries_with_map_insertion_within_5s']}/100 queries.", '',
              'Unfiltered maps are compared only with unfiltered controls. Occupancy-supported maps use the same 0.10 m support rule on each map\'s own regenerated grid. '
              'The original 780 mapping observations are frozen across pose variants. Camera XYZ is excluded from every localization target.', '',
              'Stability requires all eight fits accepted and agreeing within 0.15 m / 3 degrees. Failures stay in the denominator; residuals and solution spread are conditional '
              'on accepted fits and are not errors against ground truth. Inspect paired losses and ambiguous cases before accepting aggregate improvements. '
              'These reused development queries inform selection; they are not an untouched final test set.', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate-result', type=Path, required=True)
    parser.add_argument('--candidate-filtering', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, default=Path('outputs/fin301/result'))
    parser.add_argument('--cache', type=Path, default=Path('outputs/fin301/cache'))
    parser.add_argument('--protocol', type=Path, default=Path('outputs/fin301/verification/localization_protocol.json'))
    args = parser.parse_args()
    started = time.monotonic()
    args.output.mkdir(parents=True, exist_ok=True)
    baseline_root = args.baseline.parent
    baseline_reference_path = baseline_root/'verification/localization_final/localization_evaluation.json'
    baseline_plain_path = baseline_root/'filtering/exported_validation/original/localization_evaluation.json'
    baseline_occupied_path = baseline_root/'filtering/exported_validation/occupancy_supported/localization_evaluation.json'
    baseline_provenance_path = baseline_root/'filtering/exported_validation/provenance.json'
    baseline_occupied_provenance_path = baseline_root/'filtering/exported_validation/occupancy_supported/provenance.json'
    baseline_reference = load_json(baseline_reference_path)
    baseline_plain, baseline_occupied = load_json(baseline_plain_path), load_json(baseline_occupied_path)
    baseline_provenance = load_json(baseline_provenance_path)
    occupied_provenance = load_json(baseline_occupied_provenance_path)
    protocol = load_json(args.protocol)
    sources = {'scan_cache': args.cache/'scans.npz', 'protocol': args.protocol,
               'baseline_trajectory': args.baseline/'trajectory_arrays.npz',
               'baseline_graph': args.baseline/'pose_graph.json',
               'baseline_revisit_evidence': args.baseline/'lidar_revisit_events.json',
               'baseline_reference_result': baseline_reference_path, 'baseline_plain_result': baseline_plain_path,
               'baseline_occupancy_result': baseline_occupied_path, 'baseline_full_map_provenance': baseline_provenance_path,
               'baseline_occupancy_provenance': baseline_occupied_provenance_path,
               'separated_evaluator': Path(separated.__file__), 'exported_evaluator': Path(exported.__file__),
               'paired_reference_helper': ROOT/'scripts/evaluate_map_filtering.py',
               'registration': ROOT/'indoor_mapping/geometry.py', 'loop_evaluator': Path(__file__)}
    hashes = {key: separated.file_hash(path) for key, path in sources.items()}
    for key, expected in [('scan_cache', protocol['scan_cache_sha256']), ('protocol', baseline_reference['protocol_sha256']),
                          ('separated_evaluator', baseline_reference['evaluator_sha256']),
                          ('registration', baseline_reference['registration_source_sha256']),
                          ('exported_evaluator', baseline_provenance['source_sha256']['export_evaluator'])]:
        if hashes[key] != expected:
            raise ValueError(f'Frozen evaluation dependency changed: {key}')
    for saved, provenance, pcd_key in [(baseline_plain, baseline_provenance, 'pcd_original'),
                                      (baseline_occupied, occupied_provenance, 'candidate_pcd')]:
        if saved['protocol_sha256'] != hashes['protocol'] or saved['quality_gates'] != exported.GATES:
            raise ValueError('Saved exact-PCD control uses a different protocol or gates')
        if provenance['source_sha256']['registration'] != hashes['registration'] or provenance['source_sha256']['scan_cache'] != hashes['scan_cache']:
            raise ValueError('Saved exact-PCD control uses different scans or registration')
        if separated.file_hash(saved['pcd_path']) != saved['pcd_sha256'] or saved['pcd_sha256'] != provenance['source_sha256'][pcd_key]:
            raise ValueError('Saved exact-PCD control artifact changed')
    with np.load(sources['scan_cache']) as archive:
        data = {key: archive[key] for key in archive.files}
    times, poses, plain, occupied, occupancy_path, checks = verify_candidate(
        args.candidate_result, args.candidate_filtering, args.baseline, data, protocol, baseline_provenance)
    sources.update(candidate_trajectory=args.candidate_result/'trajectory_arrays.npz',
                   insertion_manifest=args.candidate_result/'map_insertion_indices.json',
                   candidate_export_provenance=args.candidate_result/'export_provenance.json',
                   candidate_unfiltered_pcd=args.candidate_result/'lidar_scan_map.pcd', candidate_occupancy_pcd=occupancy_path,
                   candidate_filter_report=args.candidate_filtering/'occupancy_filter_report.json',
                   candidate_graph=args.candidate_result/'pose_graph.json', candidate_metrics=args.candidate_result/'metrics.json')
    hashes.update({key: separated.file_hash(path) for key, path in sources.items()})
    snapshots = {}
    snapshot_dir = args.output/'source_snapshot'
    snapshot_dir.mkdir(exist_ok=True)
    for key, path in sources.items():
        if path.suffix == '.py':
            snapshot = snapshot_dir/(key+'.py')
            snapshot.write_bytes(path.read_bytes())
            snapshots[key] = str(snapshot)
    write_json(args.output/'current_loop_audit.json', audit_current_loop(args.baseline))
    print('Evaluating frozen separated-reference protocol: 100 + 20 queries', flush=True)
    reference = separated.evaluate(data, times, poses, protocol, 'all_returns')
    reference.update(protocol_sha256=hashes['protocol'], trajectory_sha256=hashes['candidate_trajectory'],
                     trajectory=str(sources['candidate_trajectory']), quality_gates=baseline_reference['quality_gates'],
                     paired_against_current_final=paired_reference(reference, baseline_reference),
                     comparison=separated.compare_results({**reference, 'protocol_sha256': hashes['protocol']}, baseline_reference),
                     absolute_accuracy='UNVALIDATED: reused development consistency, no independent pose truth')
    write_json(args.output/'separated_reference/localization_evaluation.json', reference)
    print(json.dumps({'scope': 'separated_reference', 'accepted': reference['queries_summary']['accepted_registrations'],
                      'stable': reference['queries_summary']['all_starts_agree_queries'],
                      'earlier_stable': reference['crosspass_queries_summary']['all_starts_agree_queries']}), flush=True)
    query_points = {query['scan_index']: exported.scan_to_points(data['ranges'][query['scan_index']],
                    float(data['angle_min']), float(data['angle_increment']), max(.12, float(data['range_min'])),
                    min(12., float(data['range_max'])), voxel_size=.06) for query in protocol['queries']}
    comparisons = {'separated_reference': {'baseline': baseline_reference, 'candidate': reference}}
    for name, points, control, artifact in [('full_map_unfiltered', plain, baseline_plain, sources['candidate_unfiltered_pcd']),
                                           ('full_map_occupancy', occupied, baseline_occupied, occupancy_path)]:
        print(f'Evaluating {name} against its matched saved control', flush=True)
        result = exported.evaluate_export(points, query_points, protocol, name)
        result.update(pcd_path=str(artifact), pcd_sha256=separated.file_hash(artifact), pcd_points=len(points),
                      protocol_sha256=hashes['protocol'], quality_gates=exported.GATES,
                      paired_against_current_final=exported.paired_changes(result, control),
                      temporal_exclusion_applied=False, crosspass_metrics_computed=False, absolute_accuracy_validated=False)
        write_json(args.output/name/'localization_evaluation.json', result)
        exported.write_runs(args.output/name/'localization_runs.csv', result)
        comparisons[name] = {'baseline': control, 'candidate': result}
        print(json.dumps({'scope': name, 'accepted': result['queries_summary']['accepted_registrations'],
                          'stable': result['queries_summary']['all_starts_agree_queries']}), flush=True)
    for key, path in sources.items():
        if separated.file_hash(path) != hashes[key]:
            raise ValueError(f'Source changed during loop evaluation: {path}')
    summary = {'schema': 'loop_refinement_validation_v1', 'invariants': checks,
               'comparisons': {name: {side: {key: value for key, value in result.items() if key not in ('queries', 'crosspass_queries')}
                                      for side, result in pair.items()} for name, pair in comparisons.items()},
               'source_paths': {key: str(path) for key, path in sources.items()}, 'source_sha256': hashes,
               'source_snapshots': snapshots, 'elapsed_s': time.monotonic()-started,
               'original_results_modified': False, 'absolute_accuracy_validated': False,
               'selection_note': 'Review all three matched comparisons, paired failures and loop-constraint support; lower graph cost alone is insufficient.'}
    write_json(args.output/'loop_refinement_validation.json', summary)
    (args.output/'loop_refinement_validation.md').write_text(comparison_markdown(comparisons, checks), encoding='utf-8')
    print(f'Completed loop refinement validation in {summary["elapsed_s"]:.1f} s', flush=True)


if __name__ == '__main__':
    main()
