"""Compare temporal-subset localization diagnostics with two unchanged controls.

This does not rerun, modify, or retune registration. It preserves per-query
regressions and distinguishes full-map repeat observations from physical truth.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import evaluate_exported_pcd_filtering as evaluator


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def assert_same_protocol(candidate, control):
    if candidate['protocol_sha256'] != control['protocol_sha256']:
        raise ValueError('Comparison protocol hash differs')
    if candidate['quality_gates'] != control['quality_gates']:
        raise ValueError('Comparison quality gates differ')
    if len(candidate['queries']) != len(control['queries']) or len(control['queries']) != 100:
        raise ValueError('Expected the same 100 query scans')
    for after, before in zip(candidate['queries'], control['queries']):
        for key in ('scan_index', 'timestamp_ns', 'seed_pose', 'source_point_count'):
            if after[key] != before[key]:
                raise ValueError(f'Paired query data differ: {key}')
        if len(after['trials']) != 8 or len(before['trials']) != 8:
            raise ValueError('Expected eight initializations per query')
        for trial_after, trial_before in zip(after['trials'], before['trials']):
            for key in ('initialization_id', 'initial_pose'):
                if trial_after[key] != trial_before[key]:
                    raise ValueError(f'Paired initializations differ: {key}')


def compare(candidate, control):
    assert_same_protocol(candidate, control)
    paired = evaluator.paired_changes(candidate, control)
    lost_all, recovered = [], []
    for after, before in zip(candidate['queries'], control['queries']):
        a = sum(trial['accepted'] for trial in after['trials'])
        b = sum(trial['accepted'] for trial in before['trials'])
        if a == 0 and b > 0:
            lost_all.append(after['scan_index'])
        if a > 0 and b == 0:
            recovered.append(after['scan_index'])
    paired['newly_zero_accepted_scan_ids'] = lost_all
    paired['recovered_nonzero_scan_ids'] = recovered
    return paired


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validation', type=Path,
                        default=Path('outputs/fin301/temporal_filter/validation'))
    parser.add_argument('--occupancy-control', type=Path,
                        default=Path('outputs/fin301/filtering/exported_validation/occupancy_supported/localization_evaluation.json'))
    parser.add_argument('--candidate', action='append', required=True,
                        help='Candidate directory name; repeat for both fixed presets')
    parser.add_argument('--filter-report', type=Path)
    args = parser.parse_args()
    sources = {'summary_script': Path(__file__),
               'original_control': args.validation/'original/localization_evaluation.json',
               'occupancy_control': args.occupancy_control,
               'exported_evaluator': Path(evaluator.__file__)}
    original, occupancy = load(sources['original_control']), load(sources['occupancy_control'])
    assert_same_protocol(original, occupancy)
    results = {'original': original, 'occupancy_supported': occupancy}
    pairs = {}
    for name in args.candidate:
        if not re.fullmatch('[A-Za-z0-9_-]+', name) or name in results:
            raise ValueError('Each candidate name must be distinct and simple')
        source = args.validation/name/'localization_evaluation.json'
        provenance_path = args.validation/name/'provenance.json'
        result, provenance = load(source), load(provenance_path)
        if result['scope'] != evaluator.SCOPE or not provenance['sources_unchanged_after_evaluation']:
            raise ValueError('Await completed unchanged-source exact-PCD evaluation')
        for key, path in provenance['source_paths'].items():
            if digest(path) != provenance['source_sha256'][key]:
                raise ValueError(f'Evaluation source changed: {path}')
        if digest(result['pcd_path']) != result['pcd_sha256']:
            raise ValueError('Candidate point cloud changed after evaluation')
        pairs[name] = {'against_original': compare(result, original),
                       'against_occupancy_supported': compare(result, occupancy)}
        results[name] = result
        sources[f'{name}_result'] = source
        sources[f'{name}_provenance'] = provenance_path
        sources[f'{name}_pcd'] = Path(result['pcd_path'])
    temporal = None
    if args.filter_report:
        sources['filter_report'] = args.filter_report
        temporal_report = load(args.filter_report)
        for key, path in temporal_report['source_paths'].items():
            if digest(path) != temporal_report['source_sha256'][key]:
                raise ValueError(f'Temporal filter source changed: {path}')
            if Path(path).suffix == '.py':
                sources[f'temporal_{key}'] = Path(path)
        evidence_ids = temporal_report['raw_evidence_scan_ids']
        query_ids = [query['scan_index'] for query in original['queries']]
        if len(set(evidence_ids)) != len(evidence_ids) or set(evidence_ids).intersection(query_ids):
            raise ValueError('Temporal evidence contains duplicate or evaluation query IDs')
        if temporal_report['source_sha256']['source_pcd'] != original['pcd_sha256']:
            raise ValueError('Temporal filter used a different original PCD')
        reported_candidates = {value['sha256']: value for value in temporal_report['candidates'].values()}
        for name in args.candidate:
            result = results[name]
            record = reported_candidates.get(result['pcd_sha256'])
            if record is None or record['retained_points'] != result['pcd_points']:
                raise ValueError('Evaluated candidate differs from temporal filter report')
        temporal = {key: temporal_report[key] for key in (
            'evidence_scans', 'epoch_count', 'static_supported_points', 'conflicted_points',
            'insufficient_points', 'query_nearest_evidence_time_s', 'selected_trajectory',
            'corrected_pose_array_sha256', 'parameters', 'query_exclusion_scope')}
    hashes = {key: digest(path) for key, path in sources.items()}
    report = {'schema': 'temporal_exact_pcd_validation_review_v1',
              'scope': evaluator.SCOPE,
              'variants': {name: {key: value for key, value in result.items() if key != 'queries'}
                           for name, result in results.items()},
              'paired_comparisons': pairs,
              'temporal_evidence': temporal,
              'source_paths': {key: str(path) for key, path in sources.items()},
              'source_sha256': hashes,
              'frozen_query_count': 100, 'initializations_per_query': 8,
              'same_query_sources_seeds_initializations_and_gates': True,
              'temporal_exclusion_applied': False, 'absolute_accuracy_validated': False,
              'interpretation': [
                  'Temporal evidence classifies observed persistence, not semantic people or permanent structures.',
                  'Consecutive support epochs still share observations and are not independent physical measurements.',
                  'The exact-PCD targets and temporal-filter evidence include nearby observations; query-ID exclusion is not temporal independence.',
                  'All-eight stability and zero-success regressions must accompany conditional residual and spread.',
                  'Only exact LiDAR PCD targets are tested here; matched occupancy-grid interpretation and free-space safety are not evaluated.',
                  'Raw query scans can include transient returns; fewer accepted fits can reflect lost support or appropriate rejection, which this test cannot distinguish semantically.',
                  'Static-inspection quality and localization support are distinct selection objectives.']}
    evaluator.write_json(args.validation/'temporal_localization_review.json', report)
    lines = ['# Temporal-subset localization review', '',
             'The same 100 raw query scans, eight initializations, source preprocessing and registration gates are used for every exact exported PCD. '
             'This is full-map repeat-observation stability, not a temporally separated test or an absolute-accuracy measurement. '
             'Temporal filtering uses accepted scan observations beyond the original map insertion subset; its evidence can be close in time to a query even when raw query IDs are excluded. '
             'See the independent temporal-evidence audit for the actual nearest-query timing.', '',
             '| Artifact | Points | Accepted /800 | All-eight stable /100 | Conditional spread p95 (m) | Eligible spread queries |',
             '|---|---:|---:|---:|---:|---:|']
    for name, result in results.items():
        s = result['queries_summary']
        spread = s['query_max_pairwise_translation_m']
        lines.append(f"| {name} | {result['pcd_points']} | {s['accepted_registrations']} | {s['all_starts_agree_queries']} | {spread['p95']} | {spread['count']} |")
    if temporal:
        timing = temporal['query_nearest_evidence_time_s']
        lines += ['', f"The filter uses {temporal['evidence_scans']} accepted scans in {temporal['epoch_count']} epochs. "
                  f"All 100 queries are excluded by raw ID, but each has a filter-evidence scan within {timing['maximum']:.5f} s "
                  f"(median {timing['median']:.5f} s). These observations are strongly correlated."]
    lines += ['', 'Stability requires all eight registrations accepted and agreeing within 0.15 m / 3 degrees. '
              'Failures remain in the denominator. Spread is conditional on at least two accepted starts; the eight starts on a scan are correlated trials.', '',
              '| Candidate / comparison | Accepted gains / losses | Stable-query gains / losses | Newly zero-success scan IDs |',
              '|---|---:|---:|---|']
    for name, comparisons in pairs.items():
        for label, paired in comparisons.items():
            lines.append(f"| {name} / {label} | {paired['accepted_trial_gains']} / {paired['accepted_trial_losses']} | "
                         f"{paired['stable_query_gains']} / {paired['stable_query_losses']} | {paired['newly_zero_accepted_scan_ids']} |")
    lines += ['', 'These two controls answer different questions: original measures the change from the unfiltered map; occupancy-supported measures the change from the previously available cleaned map. '
              'A cleaner static-inspection artifact may lose localization support. Removal due to insufficient temporal evidence does not establish free space, '
              'and stationary people or long-lived movable objects can remain. This test uses only exact LiDAR PCD targets; it does not validate the matched occupancy grid or free-space safety. '
              'Raw query scans may contain transient returns, so lower acceptance does not by itself establish that removed geometry was static or that the map is wrong. '
              'Camera landmarks do not enter these localization targets. See the filter report for evidence categories and their distinct meanings.', '',
              'Per-query trials, paired IDs, actual PCD hashes and complete evaluation provenance remain in each candidate directory. '
              'Neither localization controls nor primary outputs are overwritten.', '']
    (args.validation/'temporal_localization_review.md').write_text('\n'.join(lines), encoding='utf-8')
    snapshot = args.validation/'source_snapshot'
    snapshot.mkdir(exist_ok=True)
    for key, path in sources.items():
        if path.suffix == '.py':
            (snapshot/f'{key}.py').write_bytes(path.read_bytes())
    print(json.dumps({name: {'accepted': result['queries_summary']['accepted_registrations'],
                            'stable': result['queries_summary']['all_starts_agree_queries']}
                      for name, result in results.items()}, indent=2))


if __name__ == '__main__':
    main()
