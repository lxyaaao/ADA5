"""Evaluate one additional exact PCD subset against a saved full-map baseline.

Reuses the frozen exported-PCD evaluator without repeating its original run.
This remains full-map repeat-observation stability, not a temporal-exclusion
held-out test or an independent physical-accuracy evaluation.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))

import numpy as np
import evaluate_exported_pcd_filtering as runner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-dir', type=Path, default=Path('outputs/fin301/filtering/exported_validation'))
    parser.add_argument('--candidate', type=Path,
                        default=Path('outputs/fin301/filtering/lidar_scan_map_clean_occupancy_supported.pcd'))
    parser.add_argument('--name', default='occupancy_supported')
    args = parser.parse_args()
    if not re.fullmatch('[A-Za-z0-9_-]+', args.name) or args.name == 'original':
        raise ValueError('Candidate name must be a distinct simple name')
    started = time.monotonic()
    base_provenance_path = args.baseline_dir / 'provenance.json'
    original_path = args.baseline_dir / 'original/localization_evaluation.json'
    base = json.loads(base_provenance_path.read_text(encoding='utf-8'))
    if base.get('scope') != runner.SCOPE or not base.get('sources_unchanged_after_evaluation'):
        raise ValueError('Await completed, unchanged-source exported-PCD baseline evaluation')
    for key, path in base['source_paths'].items():
        if runner.evaluator.file_hash(path) != base['source_sha256'][key]:
            raise ValueError(f'Frozen baseline source changed: {path}')
    if base['source_paths']['export_evaluator'] != str(Path(runner.__file__)):
        # Relative and absolute representations may differ, so compare resolved paths.
        if Path(base['source_paths']['export_evaluator']).resolve() != Path(runner.__file__).resolve():
            raise ValueError('The imported evaluator differs from the original exported-PCD run')
    original = json.loads(original_path.read_text(encoding='utf-8'))
    if original['protocol_sha256'] != base['source_sha256']['protocol']:
        raise ValueError('Saved original result uses a different protocol')
    original_points, digest = runner.read_exact_planar_pcd(base['source_paths']['pcd_original'])
    if digest != original['pcd_sha256'] or len(original_points) != original['pcd_points']:
        raise ValueError('Saved original result and exported original PCD disagree')
    points, candidate_hash = runner.read_exact_planar_pcd(args.candidate)
    originals = {tuple(point) for point in original_points}
    if len(points) >= len(original_points) or len({tuple(point) for point in points}) != len(points):
        raise ValueError('Candidate must be a strict subset without duplicate points')
    if any(tuple(point) not in originals for point in points):
        raise ValueError('Candidate contains moved or invented XYZ coordinates')
    protocol = json.loads(Path(base['source_paths']['protocol']).read_text(encoding='utf-8'))
    if len(original['queries']) != len(protocol['queries']):
        raise ValueError('Saved baseline query count differs from protocol')
    for query, saved in zip(protocol['queries'], original['queries']):
        if query['scan_index'] != saved['scan_index'] or query['seed_pose'] != saved['seed_pose']:
            raise ValueError('Saved baseline query IDs or seeds changed')
        if len(saved['trials']) != len(protocol['perturbations']):
            raise ValueError('Saved baseline initialization count differs')
        for initial, trial in zip(protocol['perturbations'], saved['trials']):
            if initial['pose'] != trial['initial_pose']:
                raise ValueError('Saved baseline initial poses differ from protocol')
    with np.load(base['source_paths']['scan_cache']) as archive:
        data = {key: archive[key] for key in archive.files}
    query_points = {query['scan_index']: runner.scan_to_points(data['ranges'][query['scan_index']],
                    float(data['angle_min']), float(data['angle_increment']), max(.12, float(data['range_min'])),
                    min(12., float(data['range_max'])), voxel_size=.06) for query in protocol['queries']}
    extra_paths = {'saved_original_result': original_path, 'saved_original_provenance': base_provenance_path,
                   'candidate_pcd': args.candidate, 'extension_runner': Path(__file__)}
    extra_hashes = {key: runner.evaluator.file_hash(path) for key, path in extra_paths.items()}
    provenance = {**base, 'source_paths': {**base['source_paths'], **{key: str(path) for key, path in extra_paths.items()}},
                  'source_sha256': {**base['source_sha256'], **extra_hashes},
                  'saved_original_reused_without_rerun': True,
                  'candidate_is_exact_original_subset': True, 'candidate_pcd_sha256': candidate_hash,
                  'original_suite_elapsed_s': base.get('elapsed_s'), 'sources_unchanged_after_evaluation': False}
    output = args.baseline_dir / args.name
    runner.write_json(output / 'provenance.json', provenance)
    result = runner.evaluate_export(points, query_points, protocol, args.name)
    result.update(pcd_path=str(args.candidate), pcd_sha256=candidate_hash, pcd_points=len(points),
                  protocol_sha256=base['source_sha256']['protocol'], quality_gates=runner.GATES,
                  temporal_exclusion_applied=False, crosspass_metrics_computed=False,
                  absolute_accuracy_validated=False, saved_original_reused_without_rerun=True,
                  paired_against_original=runner.paired_changes(result, original))
    runner.write_json(output / 'localization_evaluation.json', result)
    runner.write_runs(output / 'localization_runs.csv', result)
    for key, path in provenance['source_paths'].items():
        if runner.evaluator.file_hash(path) != provenance['source_sha256'][key]:
            raise ValueError(f'Source changed during candidate evaluation: {path}')
    provenance['elapsed_s'] = time.monotonic() - started
    provenance['sources_unchanged_after_evaluation'] = True
    runner.write_json(output / 'provenance.json', provenance)
    runner.write_summary(output, {'original': original, args.name: result}, provenance)
    print(json.dumps({'variant': args.name, 'summary': result['queries_summary'],
                      'paired_against_original': result['paired_against_original'],
                      'elapsed_s': result['elapsed_s'], 'output': str(output)}, indent=2), flush=True)


if __name__ == '__main__':
    main()
