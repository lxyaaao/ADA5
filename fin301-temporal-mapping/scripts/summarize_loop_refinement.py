"""Apply the recorded non-degradation policy to the bounded loop experiments."""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
BASE = ROOT/'outputs/fin301'
EXPERIMENT = BASE/'loop_refinement'
PROFILES = ('all-heading', 'multikeyframe')
CONTROLS = {
    'separated_reference': BASE/'verification/localization_final/localization_evaluation.json',
    'full_map_unfiltered': BASE/'filtering/exported_validation/original/localization_evaluation.json',
    'full_map_occupancy': BASE/'filtering/exported_validation/occupancy_supported/localization_evaluation.json'}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def new_complete_failures(candidate, baseline, group):
    old = {query['scan_index']: query for query in baseline[group]}
    new = {query['scan_index']: query for query in candidate[group]}
    if old.keys() != new.keys():
        raise ValueError('Query identity changed')
    return [scan for scan in old if any(item['accepted'] for item in old[scan]['trials'])
            and not any(item['accepted'] for item in new[scan]['trials'])]


def plot_maps(profile):
    os.environ.setdefault('MPLCONFIGDIR', str(ROOT/'outputs/.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    from indoor_mapping.ros_node import load_ascii_pcd
    paths = [BASE/'result/lidar_scan_map.pcd', EXPERIMENT/profile/'result/lidar_scan_map.pcd',
             BASE/'filtering/lidar_scan_map_clean_occupancy_supported.pcd',
             EXPERIMENT/profile/'filtering/lidar_scan_map_clean_occupancy_supported.pcd']
    points = [load_ascii_pcd(path) for path in paths]
    minimum = np.min([array[:, :2].min(axis=0) for array in points], axis=0)-.5
    maximum = np.max([array[:, :2].max(axis=0) for array in points], axis=0)+.5
    labels = ['Current: one loop, raw PCD', f'{profile}: two loops, raw PCD',
              'Current: occupancy supported', f'{profile}: occupancy supported']
    figure, axes = plt.subplots(2, 2, figsize=(11, 11), sharex=True, sharey=True)
    for axis, cloud, label in zip(axes.ravel(), points, labels):
        axis.scatter(cloud[:, 0], cloud[:, 1], s=.7, c='#264b5b', edgecolors='none', linewidths=0, rasterized=True)
        axis.set(xlim=(minimum[0], maximum[0]), ylim=(minimum[1], maximum[1]),
                 xlabel='Map x (m)', ylabel='Map y (m)', title=f'{label}\n{len(cloud):,} observed points')
        axis.set_aspect('equal'); axis.grid(alpha=.13)
    figure.suptitle('Loop-refinement comparison | same mapping observations, frame and scale')
    figure.tight_layout(); figure.savefig(EXPERIMENT/profile/'map_comparison.png', dpi=170); plt.close(figure)


def main():
    policy = read(EXPERIMENT/'selection_policy.json')
    sources = {str(path.relative_to(ROOT)): sha(path) for path in CONTROLS.values()}
    sources['outputs/fin301/loop_refinement/selection_policy.json'] = sha(EXPERIMENT/'selection_policy.json')
    controls = {scope: read(path) for scope, path in CONTROLS.items()}
    results = {}
    lines = ['# fin301 loop-closure refinement', '',
             'Two measured-loop candidates were tested without changing the existing final map or filtering package. '
             'The previous proposal gate excluded heading differences of 1.2 rad or more despite the 360-degree LiDAR. '
             'The all-heading candidate removes that proposal restriction; the multikeyframe candidate also uses wider local scan support. '
             'The pair registration, reciprocal, geometric-coverage and multistart gates are retained. Both passes add disjoint temporal-neighbor checks.', '',
             '| Map | Reference stable /100 | Earlier-pass stable /20 | Raw PCD accepted /800 | Raw PCD stable /100 | Filtered PCD accepted /800 | Filtered PCD stable /100 |',
             '|---|---:|---:|---:|---:|---:|---:|']
    def row(label, suite):
        a = suite['separated_reference']; b = suite['full_map_unfiltered']['queries_summary']; c = suite['full_map_occupancy']['queries_summary']
        return f"| {label} | {a['queries_summary']['all_starts_agree_queries']} | {a['crosspass_queries_summary']['all_starts_agree_queries']} | {b['accepted_registrations']} | {b['all_starts_agree_queries']} | {c['accepted_registrations']} | {c['all_starts_agree_queries']} |"
    lines.append(row('Current one-loop map', controls))
    for profile in PROFILES:
        directory = EXPERIMENT/'validation'/profile
        validation_path = directory/'loop_refinement_validation.json'
        validation = read(validation_path)
        experiment = read(EXPERIMENT/profile/'summary.json')
        sources[str(validation_path.relative_to(ROOT))] = sha(validation_path)
        checks, suite = {}, {}
        gain = False
        for scope, control in controls.items():
            path = directory/scope/'localization_evaluation.json'
            suite[scope] = candidate = read(path)
            for group in (('queries', 'crosspass_queries') if scope == 'separated_reference' else ('queries',)):
                old, new = control[group+'_summary'], candidate[group+'_summary']
                gain |= new['all_starts_agree_queries'] > old['all_starts_agree_queries']
                checks[scope+'/'+group] = {
                    'accepted_before': old['accepted_registrations'], 'accepted_after': new['accepted_registrations'],
                    'stable_before': old['all_starts_agree_queries'], 'stable_after': new['all_starts_agree_queries'],
                    'no_count_decrease': all(new[key] >= old[key] for key in ('accepted_registrations', 'all_starts_agree_queries')),
                    'new_zero_acceptance_scan_ids': new_complete_failures(candidate, control, group)}
        eligible = (experiment['eligible_for_export'] and experiment['graph_constraint_consistency_pass'] and gain
                    and all(item['no_count_decrease'] and not item['new_zero_acceptance_scan_ids'] for item in checks.values()))
        score = [suite['full_map_occupancy']['queries_summary']['all_starts_agree_queries'],
                 suite['separated_reference']['queries_summary']['all_starts_agree_queries'],
                 suite['full_map_occupancy']['queries_summary']['accepted_registrations']]
        results[profile] = {'eligible_to_replace_default': bool(eligible), 'checks': checks,
                            'stable_count_gain_exists': bool(gain), 'score': score,
                            'new_measured_loops': experiment['additional_loops'],
                            'maximum_added_pose_shift_m': experiment['maximum_change_from_delivered_trajectory_m'],
                            'validation_invariants': validation['invariants']}
        lines.append(row(profile, suite))
        plot_maps(profile)
    eligible = [name for name in PROFILES if results[name]['eligible_to_replace_default']]
    selected = max(eligible, key=lambda name: results[name]['score']) if eligible else 'current-one-loop'
    decision = {'selected': selected, 'policy': policy, 'profiles': results, 'source_sha256': sources,
                'primary_artifacts_overwritten': False, 'absolute_accuracy_validated': False}
    (EXPERIMENT/'selection.json').write_text(json.dumps(decision, indent=2)+'\n', encoding='utf-8')
    lines += ['', f'**Selected default: {selected}.** The conservative post-review policy is not preregistered; it requires non-decreasing acceptance and stable-query counts in all matched scopes, '
              'no newly complete failures, and at least one stable-query count gain. Candidate maps are retained even when they do not meet that policy.', '',
              'All maps use the same 780 raw mapping scans, original odometry measurements, fixed map origin, camera calibration and timestamps. '
              'The existing measured loop 265→423 remains exactly once. All-heading adds 339→379; multikeyframe instead adds 340→375. '
              'They represent alternatives for the same revisit family and are not stacked together as independent observations. '
              'Each changes the current trajectory by at most about 3.5 cm. Camera landmarks are re-triangulated and the occupancy grid is rebuilt for each trajectory.', '',
              'The reference-only protocol excludes nearby reference scans. Full-PCD tests include nearby mapping observations and measure repeat-observation stability. '
              'There are 100 distinct query scans; 20 of those are additionally tested against earlier-pass references, not 20 extra independent locations. '
              'The two scopes must not be compared as one accuracy metric. Occupancy-filtered maps are compared with the same fixed 0.10 m filter applied to the old map. '
              'All evaluations reuse a development recording, not independent physical pose truth.', '',
              'The new loop checks include reciprocal registration and four disjoint temporal validation scans. No nearby additional inter-loop cycle was available; '
              'the post-optimization graph residual is an imposed-constraint fit, not independent confirmation. '
              'Fifteen audited camera pairs fail the unchanged visual verification gates. Opposite camera headings often lack common views. '
              'No camera pose edge or start/end equality was introduced. See visual_audit/visual_audit.md and the timestamped image pairs.', '',
              'A stable query requires all eight starts to pass and agree within 0.15 m / 3 degrees. '
              'Rejected starts stay in the denominator. Low fitting residual or visually aligned walls alone do not establish absolute accuracy.', '',
              '## Reproduction', '', 'Run from the original project root with its existing Python environment and sensor cache:', '',
              '```powershell', '.venv/Scripts/python.exe scripts/experiment_loop_closure_refinement.py --profile all-heading',
              '.venv/Scripts/python.exe scripts/experiment_loop_closure_refinement.py --profile multikeyframe', '```', '',
              'Then export each proposal with scripts/export_loop_map.py, rebuild its occupancy-supported candidate with scripts/filter_pcd_occupancy.py, '
              'and run scripts/evaluate_loop_refinement.py with that result/filtering pair. The scripts provide --help. '
              'scripts/summarize_loop_refinement.py applies selection_policy.json and draws the equal-scale map comparisons.', '']
    (EXPERIMENT/'loop_refinement_report.md').write_text('\n'.join(lines), encoding='utf-8')
    print(json.dumps({'selected': selected, 'profiles': results}, indent=2))


if __name__ == '__main__':
    main()
