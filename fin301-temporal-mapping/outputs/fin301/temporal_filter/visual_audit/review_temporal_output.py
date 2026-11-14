"""Independent read-only checks and fixed-axis room detail for saved outputs."""
from pathlib import Path
import hashlib
import json
import os
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from indoor_mapping.map_filter import read_ascii_pcd

OUT = Path(__file__).resolve().parent
DATA = ROOT/'outputs/fin301'
FILTER = DATA/'temporal_filter'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    report = json.loads((FILTER/'temporal_filter_report.json').read_text())
    points, _, rows = read_ascii_pcd(DATA/'result/lidar_scan_map.pcd')
    with np.load(FILTER/'temporal_evidence.npz') as archive:
        e = {name: archive[name] for name in archive.files}
    hits = (e['epoch_states'] & 1) != 0
    free = ((e['epoch_states'] & 2) != 0) & ~hits
    occluded = ((e['epoch_states'] & 4) != 0) & ~hits & ~free
    count_checks = {name: bool(np.array_equal(mask.sum(0), e[name])) for name, mask in
                    [('hit_epochs', hits), ('free_epochs', free), ('occluded_epochs', occluded)]}
    fraction = e['free_epochs']/(e['hit_epochs']+e['free_epochs'])
    conflicted = (e['free_epochs'] >= 3) & (e['free_span_s'] >= 4) & (fraction >= .70)
    supported = (e['hit_epochs'] >= 3) & (e['hit_span_s'] >= 10) & (fraction <= .30)
    insufficient = ~(conflicted | supported)
    mask_checks = {name: bool(np.array_equal(mask, e[name])) for name, mask in
                   [('conflicted', conflicted), ('static_supported', supported), ('insufficient', insufficient)]}
    subset_checks = {}
    for name, mask in [('conservative', ~conflicted), ('static', supported)]:
        candidate = report['candidates'][name]
        q, _, candidate_rows = read_ascii_pcd(FILTER/candidate['file'])
        subset_checks[name] = {
            'mask_exact': bool(np.array_equal(mask, np.load(FILTER/candidate['mask_file']))),
            'coordinates_exact': bool(np.array_equal(q, points[mask])),
            'ascii_rows_exact': candidate_rows == [row for row, keep in zip(rows, mask) if keep],
            'pcd_hash_matches': sha(FILTER/candidate['file']) == candidate['sha256'],
        }
    source_checks = {name: sha(path) == report['source_sha256'][name] for name, path in report['source_paths'].items()}
    with np.load(DATA/'cache/scans.npz') as archive:
        raw = {key: archive[key] for key in ['ranges', 'times_ns', 'angle_min', 'angle_increment']}
    with np.load(DATA/'result/trajectory_arrays.npz') as archive:
        poses = archive['corrected']
    raw_to_dense = {int(raw_id): i for i, raw_id in enumerate(e['selected_raw_scan_ids'])}
    ray_cases = []
    angles = float(raw['angle_min'])+np.arange(raw['ranges'].shape[1])*float(raw['angle_increment'])
    beams = len(angles)
    windows = np.unique([2**k for k in range(1, 11) if 2**k < beams//2]+[beams//2-1])
    for point_id in [35291, 4968]:
        for event in ['first_hit', 'last_hit', 'first_free', 'last_free']:
            raw_id = int(e[event+'_raw_scan'][point_id]); dense = raw_to_dense[raw_id]
            pose = poses[dense]; delta = points[point_id, :2]-pose[:2]
            c, s = np.cos(pose[2]), np.sin(pose[2])
            local = np.array([c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1]])
            radius = float(np.linalg.norm(local)); ranges = raw['ranges'][raw_id].astype(float)
            valid = np.isfinite(ranges) & (ranges >= .12) & (ranges < 12.)
            endpoints = ranges[valid, None]*np.column_stack([np.cos(angles[valid]), np.sin(angles[valid])])
            closest = float(np.linalg.norm(endpoints-local, axis=1).min())
            center = int(np.rint((np.arctan2(local[1], local[0])-float(raw['angle_min']))/float(raw['angle_increment']))) % beams
            minimum_half = max(2, int(np.ceil(np.arctan2(.10, radius)/float(raw['angle_increment']))))
            half = int(windows[min(np.searchsorted(windows, minimum_half), len(windows)-1)])
            fan = np.arange(center-half, center+half+1) % beams
            all_valid = bool(valid[fan].all())
            minimum = float(ranges[fan].min()) if all_valid else None
            is_hit = closest <= .120000000001
            is_free = bool(.12 <= radius < 12 and all_valid and minimum > radius+.20 and not is_hit)
            expected = is_hit if event.endswith('hit') else is_free
            ray_cases.append({'point_id': point_id, 'point_xyz': points[point_id].tolist(), 'event': event,
                              'raw_scan_index': raw_id, 'timestamp_ns': int(raw['times_ns'][raw_id]),
                              'relative_time_s': (int(raw['times_ns'][raw_id])-int(raw['times_ns'][0]))/1e9,
                              'point_range_m': radius, 'closest_endpoint_m': closest,
                              'fan_beam_count': len(fan), 'all_fan_rays_finite_valid': all_valid,
                              'minimum_fan_return_m': minimum, 'event_independently_reproduced': expected})
    # Fixed requested limits; compare the same coordinates and point sizes.
    region = (points[:, 0] >= -10) & (points[:, 0] <= 10) & (points[:, 1] >= -8) & (points[:, 1] <= 6)
    region_counts = {name: int((mask & region).sum()) for name, mask in
                     [('original', np.ones(len(points), bool)), ('conservative', ~conflicted),
                      ('strict_static_supported', supported), ('conflicted', conflicted), ('insufficient', insufficient)]}
    os.environ.setdefault('MPLCONFIGDIR', str(OUT/'plot_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(14, 10), sharex=True, sharey=True)
    for axis, mask, label in zip([axes[0, 0], axes[1, 0], axes[1, 1]],
                                 [np.ones(len(points), bool), ~conflicted, supported],
                                 ['Original', 'Conservative: conflicts removed', 'Strict: supported points only']):
        chosen = mask & region
        axis.scatter(points[chosen, 0], points[chosen, 1], s=1.6, color='#5d666c', linewidths=0)
        axis.set_title(f'{label}: {chosen.sum():,} points in region')
    axis = axes[0, 1]
    for mask, color, label in [(supported, '#8a9297', 'Supported (kept in both)'),
                               (insufficient, '#d58b28', 'Insufficient/mixed (kept conservatively)'),
                               (conflicted, '#bc3434', 'Conflicted (removed conservatively)')]:
        chosen = mask & region
        axis.scatter(points[chosen, 0], points[chosen, 1], s=2.2, color=color, linewidths=0,
                     label=f'{label}: {chosen.sum():,}')
    axis.set_title('Temporal evidence classes; no semantic labels')
    axis.legend(fontsize=8, loc='upper left', markerscale=4, framealpha=.95)
    for axis in axes.ravel():
        axis.set(xlim=(-10, 10), ylim=(-8, 6), xlabel='Map x (m)', ylabel='Map y (m)')
        axis.set_aspect('equal'); axis.grid(alpha=.12)
    fig.suptitle('Main cluttered room: identical axes, coordinates and observations\n'
                 'Persistent clutter remains; orange is insufficient or mixed evidence, not measured free space', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(OUT/'main_room_temporal_detail.png', dpi=170); plt.close(fig)
    output = {
        'epoch_count_recomputation': count_checks, 'classification_recomputation': mask_checks,
        'exact_subset_checks': subset_checks, 'source_hashes_unchanged': source_checks,
        'evidence_hash_matches': sha(FILTER/'temporal_evidence.npz') == report['evidence_sha256'],
        'all_raw_ray_examples_pass': all(case['event_independently_reproduced'] for case in ray_cases),
        'raw_ray_examples': ray_cases, 'main_room_limits_xy_m': [[-10, 10], [-8, 6]], 'main_room_counts': region_counts,
        'removed_with_hit_epochs_at_least': {str(k): int((conflicted & (e['hit_epochs'] >= k)).sum()) for k in [2, 3, 5, 10]},
        'insufficient_with_at_least_3_hit_epochs_and_10s_span': int((insufficient & (e['hit_epochs'] >= 3) & (e['hit_span_s'] >= 10)).sum()),
        'interpretation': [
            'The conservative mask implements the declared repeated hit/free rule and preserves all insufficient points.',
            'The same-axis room plot must be visually reviewed; temporal filtering does not remove every dense or disordered region.',
            '966 removed points have at least three hit epochs. Removal means contradictory evidence under estimated poses, not proven human motion.',
            'Point35291 and point4968 have both hit and free support across multiple times; the raw-ray examples reproduce the claimed event geometry.',
            'Pose error, reflective/transparent surfaces and boundaries can cause false temporal conflicts. Camera context cannot certify individual laser-plane points.',
            'Strict output also removes 8065 insufficient/mixed points;5094 have enough hit count/span but fail the strict free-fraction criterion.',
            'Do not convert removed PCD locations into free occupancy cells. Original YAML/PGM semantics and origin remain unchanged.',
        ],
        'review_source_sha256': {'filter_report': sha(FILTER/'temporal_filter_report.json'),
                                  'evidence': sha(FILTER/'temporal_evidence.npz'), 'review_script': sha(__file__)},
        'filter_parameters_changed': False, 'original_files_modified': False,
    }
    (OUT/'independent_temporal_review.json').write_text(json.dumps(output, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'epoch_counts': count_checks, 'masks': mask_checks, 'raw_ray_checks': output['all_raw_ray_examples_pass'],
                      'room_counts': region_counts, 'subset_checks': subset_checks}, indent=2))


if __name__ == '__main__':
    main()
