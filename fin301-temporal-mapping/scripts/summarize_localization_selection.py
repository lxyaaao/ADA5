"""Record the fin301 development comparison used to choose the localization map."""
from pathlib import Path
import hashlib
import json


def main():
    root = Path(__file__).resolve().parents[1]
    base = root / 'outputs/fin301'
    paths = {name: base / 'verification' / f'localization_{name}' / 'localization_evaluation.json'
             for name in ('baseline', 'final', 'persistent')}
    results = {name: json.loads(path.read_text(encoding='utf-8')) for name, path in paths.items()}
    baseline, final = results['baseline'], results['final']
    if baseline['protocol_sha256'] != final['protocol_sha256'] or baseline['evaluator_sha256'] != final['evaluator_sha256']:
        raise ValueError('Selected comparison requires identical protocol and evaluator source')
    for group in ('queries_summary', 'crosspass_queries_summary'):
        if final[group]['acceptance_rate'] < baseline[group]['acceptance_rate'] or final[group]['all_starts_agree_fraction'] < baseline[group]['all_starts_agree_fraction']:
            raise ValueError('Selected map fails the declared no-degradation criterion')
    if final['queries_summary']['query_max_pairwise_translation_m']['p95'] > baseline['queries_summary']['query_max_pairwise_translation_m']['p95']:
        raise ValueError('Selected map worsens main-query convergence spread')
    paired = []
    for old, new in zip(baseline['queries'], final['queries']):
        if old['scan_index'] != new['scan_index'] or old['reference_scan_indices'] != new['reference_scan_indices']:
            raise ValueError('Paired comparison uses different measurements')
        paired.append((old['agreement']['all_starts_agree'], new['agreement']['all_starts_agree']))
    map_path = base / 'result/lidar_scan_map.pcd'
    decision = {
        'selected_localization_map': str(map_path.relative_to(root)),
        'selected_map_sha256': hashlib.sha256(map_path.read_bytes()).hexdigest(),
        'selected_configuration': 'LiDAR odometry plus one validated partial-overlap revisit; unfiltered scan returns',
        'camera_map': 'Separate six-view upward-ray structural candidates; camera pose correction and accuracy improvement not demonstrated',
        'evaluation_role': 'Raw scans held out from mapping; development validation used for map selection, not an untouched final test set',
        'absolute_localization_accuracy': 'UNVALIDATED: no independent physical position reference',
        'stability_definition': 'All eight perturbed initializations accepted and mutually within 0.15 m and 3 degrees',
        'paired_queries_gained_stability': sum(not old and new for old, new in paired),
        'paired_queries_lost_stability': sum(old and not new for old, new in paired),
        'comparisons': {name: {group: result[group] for group in ('queries_summary', 'crosspass_queries_summary')}
                        for name, result in results.items()},
        'source_sha256': {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths.values()},
        'limitations': ['Reference maps and initial priors are estimated from the same recording, not physical truth.',
                        'Shared baseline initial poses and spatial selection may favor the baseline prior.',
                        'Aggregate improvement is not uniform; some queries lose stability.',
                        'Earlier-pass acceptance and stable-query counts are unchanged; paired residuals are mixed.',
                        'Camera XYZ must not be projected into the 2D localization scan map.']}
    output = base / 'verification/map_selection.json'
    output.write_text(json.dumps(decision, indent=2), encoding='utf-8')
    rows = []
    for name, value in results.items():
        main, earlier = value['queries_summary'], value['crosspass_queries_summary']
        rows.append(f"| {name} | {main['accepted_registrations']}/800 | {main['all_starts_agree_queries']}/100 | "
                    f"{main['query_max_pairwise_translation_m']['p95']:.4f} m | {earlier['all_starts_agree_queries']}/20 |")
    text = ('# fin301 map selection for localization\n\n'
            'Selected: `outputs/fin301/result/lidar_scan_map.pcd`, using one LiDAR revisit constraint. '
            'The combined `map.pcd` and separate camera landmarks are companion mapping outputs.\n\n'
            '| Configuration | Accepted starts | Stable queries | p95 within-query translation spread | Earlier-pass stable queries |\n'
            '|---|---:|---:|---:|---:|\n' + '\n'.join(rows) + '\n\n'
            'Stability requires all eight starts to pass and agree within 0.15 m / 3 degrees. These are development '
            'validation results used for selection, not ground-truth errors. Ten queries gain stability and six lose it; '
            'earlier-pass stable counts do not increase. No absolute localization accuracy or camera accuracy gain is claimed.\n\n'
            'The persistent-point variant is rejected because acceptance and stability worsen. '
            'The original baseline and all candidate evidence are retained for inspection.\n')
    (output.parent / 'map_selection.md').write_text(text, encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
