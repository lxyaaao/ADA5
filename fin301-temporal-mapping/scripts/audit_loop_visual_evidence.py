"""Read-only camera evidence for training-keyframe LiDAR revisit candidates.

Creates timestamped photographic comparisons and applies the existing ORB gates.
No metric constraint is fitted, no held-out query scans are read, and no map or
trajectory is modified. Opposite-heading LiDAR revisits may lack camera overlap.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from indoor_mapping.geometry import wrap_angle
from indoor_mapping.vision import image_features, verify_images


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def pair_plot(record, output, feature_cache):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.1), dpi=140)
    for side, axis in zip(('a', 'b'), axes):
        frame = record[side]
        rgb = cv2.cvtColor(cv2.imread(frame['path']), cv2.COLOR_BGR2RGB)
        axis.imshow(rgb)
        verified = record['orb'].get('points_' + side, [])
        if verified:
            p = np.asarray(verified)
            axis.scatter(p[:, 0], p[:, 1], s=24, facecolors='none', edgecolors='lime', linewidths=.7)
        axis.set_title(f"{side.upper()}: image {frame['image_id']}, t={frame['relative_time_s']:.3f}s\n"
                       f"{frame['timestamp_ns']} ns | ORB features {record['orb_features_' + side]}", fontsize=9)
        axis.set_axis_off()
    orb = record['orb']
    fig.suptitle(f"{record['label']} | ORB gate {'PASS' if orb['success'] else 'FAIL'}: "
                 f"{orb['matches']} matches, {orb['inliers']} inliers\n"
                 f"{record['camera_comparison_scope']}", fontsize=11)
    fig.text(.5, .02, 'Photographic/appearance evidence only. No camera metric loop or localization gain is inferred.',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .04, 1, .89))
    path = output / (record['id'] + '.png')
    fig.savefig(path)
    plt.close(fig)
    return path


def contact_pages(records, output):
    paths = []
    for offset in range(0, len(records), 4):
        selected = records[offset:offset + 4]
        fig, axes = plt.subplots(len(selected), 2, figsize=(10, 3.9 * len(selected)), dpi=140, squeeze=False)
        for record, row in zip(selected, axes):
            for side, axis in zip(('a', 'b'), row):
                item = record[side]
                image = cv2.cvtColor(cv2.imread(item['path']), cv2.COLOR_BGR2RGB)
                axis.imshow(image)
                axis.set_axis_off()
                axis.set_title(f"{record['label']} / {side.upper()} / image {item['image_id']} / {item['relative_time_s']:.2f}s\n"
                               f"ORB {'PASS' if record['orb']['success'] else 'FAIL'}; {record['orb']['inliers']} inliers", fontsize=9)
        fig.suptitle('Recorded camera views: training-keyframe candidate audit', fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, .975))
        path = output / f'contact_{offset // 4 + 1:02d}.png'
        fig.savefig(path)
        plt.close(fig)
        paths.append(str(path))
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('outputs/fin301'))
    parser.add_argument('--output', type=Path, default=Path('outputs/fin301/loop_refinement/visual_audit'))
    parser.add_argument('--pair', action='append', default=[], help='Additional training keyframe pair, e.g.335:387')
    args = parser.parse_args()
    dataset, output = args.dataset, args.output
    output.mkdir(parents=True, exist_ok=True)
    result = dataset / 'result'
    source_paths = [result / 'odometry.npz', result / 'trajectory_arrays.npz', result / 'image_associations.json',
                    result / 'loop_events.json', dataset / 'cache/images.csv', dataset / 'audit/calibration.json',
                    dataset / 'audit/audit.json', ROOT / 'indoor_mapping/vision.py', Path(__file__)]
    source_hashes = {str(path): digest(path) for path in source_paths}
    with np.load(result / 'odometry.npz') as odometry:
        raw_indices = odometry['scan_indices']
        key_positions = odometry['key_ids']
        baseline_dense = odometry['poses']
    with np.load(result / 'trajectory_arrays.npz') as archive:
        dense_times = archive['times_ns']
        current_dense = archive['corrected']
    baseline = baseline_dense[key_positions]
    current = current_dense[key_positions]
    times = dense_times[key_positions]
    start_ns = int(dense_times[0])
    association = json.loads((result / 'image_associations.json').read_text(encoding='utf-8'))
    manifest = list(csv.DictReader((dataset / 'cache/images.csv').open(encoding='utf-8')))
    by_stamp = {int(row['timestamp_ns']): row for row in manifest}
    audit = json.loads((dataset / 'audit/audit.json').read_text(encoding='utf-8'))
    bad_images = set(audit['camera']['decode_failure_image_ids'])
    calibration = json.loads((dataset / 'audit/calibration.json').read_text(encoding='utf-8'))
    camera_info = calibration['camera_info_first']
    intrinsic = np.asarray(camera_info['K'])
    width = int(camera_info['width'])
    hfov = float(np.degrees(np.arctan(intrinsic[0, 2] / intrinsic[0, 0])
                           + np.arctan((width - intrinsic[0, 2]) / intrinsic[0, 0])))

    def frame_record(row, key=None):
        return {'image_id': int(row['image_id']), 'timestamp_ns': int(row['timestamp_ns']),
                'relative_time_s': (int(row['timestamp_ns']) - start_ns) * 1e-9,
                'path': row['path'], 'sha256': digest(row['path']), 'keyframe': key}

    def key_image(key):
        item = association[key]
        if not item['valid'] or item['image_path'] is None:
            return None
        row = by_stamp[int(item['image_timestamp_ns'])]
        if int(row['image_id']) in bad_images:
            return None
        out = frame_record(row, key)
        out.update(scan_timestamp_ns=int(times[key]), raw_scan_index=int(raw_indices[key_positions[key]]),
                   image_scan_delta_ms=item['delta_ms'])
        return out

    valid_keys = {key: key_image(key) for key in range(len(times))}
    valid_keys = {key: value for key, value in valid_keys.items() if value is not None}
    spatial = []
    for j in range(len(times)):
        if j not in valid_keys:
            continue
        for i in range(j):
            if i not in valid_keys or int(times[j]) - int(times[i]) < 35_000_000_000:
                continue
            distance = float(np.linalg.norm(baseline[i, :2] - baseline[j, :2]))
            if distance > 2.:
                continue
            excursion = float(np.max(np.linalg.norm(baseline[i:j + 1, :2] - baseline[i, :2], axis=1)))
            if excursion < 3.:
                continue
            spatial.append({'i': i, 'j': j, 'baseline_distance_m': distance, 'excursion_m': excursion,
                            'yaw_difference_rad': float(abs(wrap_angle(baseline[j, 2] - baseline[i, 2])))})
    spatial.sort(key=lambda item: item['baseline_distance_m'])
    representative = []
    for item in spatial:
        if any(abs(int(times[item['i']]) - int(times[old['i']])) < 10_000_000_000
               and abs(int(times[item['j']]) - int(times[old['j']])) < 10_000_000_000 for old in representative):
            continue
        representative.append(item)
        if len(representative) == 6:
            break
    requested = [(265, 423), (262, 422), (104, 137), (335, 387), (334, 388), (329, 400),
                 (336, 386), (344, 373), (170, 224), (82, 146)]
    requested += [(item['i'], item['j']) for item in representative]
    requested += [tuple(map(int, value.split(':'))) for value in args.pair]
    requested = list(dict.fromkeys(tuple(sorted(pair)) for pair in requested))
    pairs, skipped, features = [], [], {}

    def compare(label, identity, first, second, geometry=None):
        for frame in (first, second):
            if frame['path'] not in features:
                features[frame['path']] = image_features(frame['path'])
        a, b = features[first['path']], features[second['path']]
        cv2.setRNGSeed(13)
        verified = verify_images(a, b)
        record = {'id': identity, 'label': label, 'a': first, 'b': second,
                  'orb': verified, 'orb_features_a': len(a['xy']), 'orb_features_b': len(b['xy']),
                  'appearance_cosine': float(a['global'] @ b['global']),
                  'camera_comparison_scope': 'No accepted metric visual loop; fixed original ORB/homography gates.'}
        if geometry:
            record.update(geometry)
            if np.degrees(geometry['yaw_difference_rad']) > hfov:
                record['camera_comparison_scope'] = 'Camera headings differ more than approximate horizontal FOV; low direct overlap expected.'
        record['figure'] = str(pair_plot(record, output, features))
        return record

    pairs.append(compare('First / last recorded camera view', 'start_end', frame_record(manifest[0]), frame_record(manifest[-1])))
    for i, j in requested:
        if i not in valid_keys or j not in valid_keys:
            skipped.append({'i': i, 'j': j, 'reason': 'No valid camera association within50ms, or known corrupt image'})
            continue
        geometry = {'keyframe_pair': [i, j], 'time_separation_s': (int(times[j]) - int(times[i])) * 1e-9,
                    'baseline_distance_m': float(np.linalg.norm(baseline[i, :2] - baseline[j, :2])),
                    'current_estimated_distance_m': float(np.linalg.norm(current[i, :2] - current[j, :2])),
                    'yaw_difference_rad': float(abs(wrap_angle(baseline[j, 2] - baseline[i, 2]))),
                    'baseline_excursion_m': float(np.max(np.linalg.norm(baseline[i:j + 1, :2] - baseline[i, :2], axis=1)))}
        pairs.append(compare(f'Keyframes {i} / {j}', f'key_{i:03d}_{j:03d}', valid_keys[i], valid_keys[j], geometry))
    previous_path = dataset / 'verification/vision_diagnostic_summary.json'
    previous = json.loads(previous_path.read_text(encoding='utf-8')) if previous_path.is_file() else None
    summary = {'scope': 'Training-keyframe photographic evidence, not loop-constraint fitting',
               'source_sha256': source_hashes, 'keyframe_count': len(times), 'valid_camera_keyframes': len(valid_keys),
               'camera_hfov_approx_deg_ignoring_distortion': hfov,
               'camera_info': camera_info, 'recorded_T_camera_lidar': calibration['T_camera_lidar'],
               'existing_accepted_camera_loop_count': len(json.loads((result / 'loop_events.json').read_text())),
               'endpoint_baseline_separation_m': float(np.linalg.norm(baseline_dense[-1, :2] - baseline_dense[0, :2])),
               'endpoint_current_estimated_separation_m': float(np.linalg.norm(current_dense[-1, :2] - current_dense[0, :2])),
               'endpoint_baseline_heading_difference_deg': float(np.degrees(abs(wrap_angle(baseline_dense[-1, 2] - baseline_dense[0, 2])))),
               'near_spatial_pairs_all_headings': len(spatial), 'representative_near_pairs': representative,
               'pairs_checked': len(pairs), 'pairs_passing_original_orb_gate': sum(bool(pair['orb']['success']) for pair in pairs),
               'fixed_orb_gates': {'nfeatures': 1000, 'ratio_test': .72, 'minimum_ratio_matches_before_H': 24,
                                   'homography_RANSAC_px': 3., 'minimum_inliers': 30,
                                   'minimum_inlier_fraction': .55, 'minimum_hull_fraction_both_images': .035},
               'pairs': pairs, 'skipped_pairs': skipped, 'contact_sheets': contact_pages(pairs, output),
               'uses_evaluation_query_scans_for_constraints': False, 'metric_camera_constraints_created': 0,
               'prior_broader_diagnostic': None if previous is None else {
                   'path': str(previous_path), 'sha256': digest(previous_path), 'unique_pairs_checked': previous['unique_pairs_checked'],
                   'groups': previous['groups'], 'note': 'Previously recorded audit; not rerun here.'},
               'review_status': 'NUMERIC_EVIDENCE_COMPLETE_PHOTOGRAPHIC_INTERPRETATION_PENDING',
               'limitations': ['Estimated trajectory proximity is a retrieval aid, not physical truth.',
                               'A forward camera can miss real360-degree LiDAR revisits when headings differ.',
                               'Glass, repeating panels, moving people and low camera height can produce ambiguous views.',
                               'Homography consistency alone is not a metric camera pose or an independent loop validation.',
                               'No endpoint closure is imposed; camera mapping contributions remain the existing separate landmarks.']}
    for path in source_paths:
        if digest(path) != source_hashes[str(path)]:
            raise ValueError(f'Source changed during audit: {path}')
    save_json(output / 'visual_audit.json', summary)
    print(json.dumps({key: value for key, value in summary.items() if key in (
        'keyframe_count', 'valid_camera_keyframes', 'camera_hfov_approx_deg_ignoring_distortion',
        'endpoint_baseline_separation_m', 'endpoint_current_estimated_separation_m',
        'near_spatial_pairs_all_headings', 'pairs_checked', 'pairs_passing_original_orb_gate', 'skipped_pairs')}, indent=2))


if __name__ == '__main__':
    main()
