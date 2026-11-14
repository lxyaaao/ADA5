"""Small timestamped photographic audit for temporal LiDAR-map filtering.

This does not detect people, classify LiDAR points, fit poses, or modify maps.
The three manually selected sequences illustrate scene changes and occlusion.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import textwrap

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

SEQUENCES = [
    {
        'id': 'near_stationary_people_and_stools',
        'title': 'People change posture; nearby stools persist',
        'image_ids': [0, 50, 100],
        'observation': 'People change arm and body posture among the white stools. '
                       'The stool legs, seats, window frames and curtain remain visually consistent.',
        'interpretation': 'Near-stationary camera evidence distinguishes changing human posture from persistent thin furniture. '
                          'It does not identify which individual LiDAR returns hit people.',
    },
    {
        'id': 'doorway_person_and_occlusion',
        'title': 'Changing person posture beside a doorway',
        'image_ids': [5370, 5380, 5391],
        'observation': 'The visible person changes arm posture and reaches toward the wall/doorway. '
                       'The vehicle is also translating and turning; the wall, door frame and floor reflections change viewpoint.',
        'interpretation': 'An image disappearance or missing laser return alone is not a free-space contradiction. '
                          'A nearer occluder, different viewing direction, or insufficient ray coverage must remain unknown.',
    },
    {
        'id': 'persistent_bar_stools_and_windows',
        'title': 'Furniture and window structure persist as the vehicle moves',
        'image_ids': [4766, 4776, 4786],
        'observation': 'The bar stools, counter and large window frames persist across the short sequence. '
                       'Thin supports and reflective surfaces are visible from a low camera height.',
        'interpretation': 'Sparse legs and edges can be real static structure. Density alone is not sufficient evidence for removal; '
                          'limited or occluded temporal observations should not be treated as repeated free-space measurements.',
    },
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def interpolate_pose(times, poses, stamp):
    """Use integer timestamp differences; no extrapolation or yaw-wrap jump."""
    j = int(np.searchsorted(times, stamp))
    if j == len(times) or (j == 0 and stamp != int(times[0])):
        raise ValueError('Photographic evidence stamp is outside the tracked interval')
    if j == 0 or stamp == int(times[j]):
        return poses[j].copy()
    fraction = (stamp - int(times[j-1])) / (int(times[j]) - int(times[j-1]))
    answer = poses[j-1] * (1-fraction) + poses[j] * fraction
    dyaw = (poses[j, 2]-poses[j-1, 2]+np.pi) % (2*np.pi)-np.pi
    answer[2] = poses[j-1, 2] + fraction*dyaw
    return answer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=Path('outputs/fin301'))
    parser.add_argument('--output', type=Path, default=Path('outputs/fin301/temporal_filter/visual_audit'))
    args = parser.parse_args()
    root, output = args.dataset, args.output
    output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR', str(output/'plot_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import cv2

    sources = {
        'image_manifest': root/'cache/images.csv',
        'trajectory': root/'result/trajectory_arrays.npz',
        'original_pcd': root/'result/lidar_scan_map.pcd',
        'original_grid': root/'result/map.pgm',
        'original_yaml': root/'result/map.yaml',
        'calibration': root/'audit/calibration.json',
        'audit': root/'audit/audit.json',
        'scan_cache': root/'cache/scans.npz',
        'audit_script': Path(__file__),
    }
    hashes = {name: digest(path) for name, path in sources.items()}
    with sources['image_manifest'].open(encoding='utf-8') as stream:
        rows = {int(row['image_id']): row for row in csv.DictReader(stream)}
    with np.load(sources['trajectory']) as archive:
        times, poses = archive['times_ns'].copy(), archive['corrected'].copy()
    with np.load(sources['scan_cache']) as archive:
        scan_times = archive['times_ns'].copy()
    known_bad = set(json.loads(sources['audit'].read_text(encoding='utf-8'))['camera']['decode_failure_image_ids'])
    records = []
    for selection in SEQUENCES:
        item = dict(selection)
        item['frames'] = []
        fig, axes = plt.subplots(1, 3, figsize=(15, 5.7), dpi=140)
        for frame_id, axis in zip(item['image_ids'], axes):
            if frame_id in known_bad:
                raise ValueError(f'Known undecodable source image selected: {frame_id}')
            row = rows[frame_id]
            path, stamp = Path(row['path']), int(row['timestamp_ns'])
            image = cv2.imread(str(path))
            if image is None:
                raise ValueError(f'Image decode failed: {path}')
            pose = interpolate_pose(times, poses, stamp)
            nearest = int(np.argmin(np.abs(scan_times-stamp)))
            frame = {'image_id': frame_id, 'timestamp_ns': stamp, 'relative_time_s': (stamp-int(times[0]))/1e9,
                     'path': str(path), 'sha256': digest(path), 'estimated_lidar_pose_xy_yaw': pose.tolist(),
                     'nearest_raw_scan_index': nearest, 'nearest_raw_scan_timestamp_ns': int(scan_times[nearest]),
                     'nearest_raw_scan_delta_ms': abs(int(scan_times[nearest])-stamp)/1e6}
            item['frames'].append(frame)
            axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            axis.set_axis_off()
            axis.set_title(f"Image {frame_id} | t={frame['relative_time_s']:.3f} s\n{stamp} ns\n"
                           f"Estimated lidar XY: ({pose[0]:.2f}, {pose[1]:.2f}) m", fontsize=9)
        first, last = np.array(item['frames'][0]['estimated_lidar_pose_xy_yaw']), np.array(item['frames'][-1]['estimated_lidar_pose_xy_yaw'])
        item['estimated_endpoint_translation_m'] = float(np.linalg.norm(last[:2]-first[:2]))
        item['estimated_endpoint_yaw_change_deg'] = float(np.degrees((last[2]-first[2]+np.pi)%(2*np.pi)-np.pi))
        item['figure'] = str(output/(item['id']+'.png'))
        fig.suptitle(item['title']+'\n'+f"Estimated vehicle change: {item['estimated_endpoint_translation_m']:.3f} m, "
                     f"{item['estimated_endpoint_yaw_change_deg']:.2f} deg (not measured ground truth)", fontsize=12)
        caption = item['observation']+' '+item['interpretation']
        fig.text(.5, .02, '\n'.join(textwrap.wrap(caption, 160)), ha='center', fontsize=9)
        fig.tight_layout(rect=(0, .15, 1, .89))
        fig.savefig(item['figure']); plt.close(fig)
        records.append(item)
    limitations = [
        'Qualitative human review of recorded photographs, not an automated person/object detector.',
        'Camera field of view and height differ from the 360-degree laser scan plane. No removed LiDAR point is semantically labelled here.',
        'Vehicle poses and interpolated image-time positions are estimates, not independent physical truth.',
        'A moving person may remain still long enough to pass persistence checks; an observed static object may later move.',
        'No image absence, occlusion, invalid range or unobserved direction should be counted as observed free space.',
        'This audit does not claim that temporal filtering improves localization; paired frozen-query evaluation remains necessary.',
    ]
    for name, path in sources.items():
        if digest(path) != hashes[name]:
            raise ValueError(f'Source changed during audit: {path}')
    document = {'dataset': 'fin301', 'status': 'RECORDED_PHOTOGRAPHIC_SEQUENCES_REQUIRING_HUMAN_INTERPRETATION',
                'scope': 'Scene persistence examples; no detector, no LiDAR semantic attribution, no map mutation',
                'source_paths': {name: str(path) for name, path in sources.items()}, 'source_sha256': hashes,
                'sequences': records, 'limitations': limitations, 'temporal_filter_review': 'Pending candidate artifacts'}
    (output/'visual_evidence.json').write_text(json.dumps(document, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    lines = ['# Photographic context for temporal filtering', '',
             'People and persistent furniture coexist in these recordings. The examples support using repeated observations and actual free-space contradictions, rather than treating low density as proof of motion.', '',
             '| Sequence | Image IDs | Elapsed times (s) | Estimated vehicle displacement |',
             '|---|---|---|---:|']
    for item in records:
        times_text = ' / '.join(format(frame['relative_time_s'], '.3f') for frame in item['frames'])
        lines.append(f"| {item['title']} | {' / '.join(map(str,item['image_ids']))} | "
                     f"{times_text} | {item['estimated_endpoint_translation_m']:.3f} m |")
    for item in records:
        lines += ['', f"**{item['title']}**", '', item['observation']+' '+item['interpretation'], '',
                  f"![Recorded sequence]({Path(item['figure']).name})"]
    lines += ['', 'Limits:', ''] + ['- '+line for line in limitations]
    lines += ['', 'The JSON records exact header timestamps, nearest raw-scan IDs/deltas, image hashes and original map/trajectory provenance. '
              'Source photographs and original map files remain unchanged.', '',
              'Reproduce with `.venv/Scripts/python.exe scripts/audit_temporal_evidence.py`.', '']
    (output/'visual_evidence.md').write_text('\n'.join(lines), encoding='utf-8')
    print(json.dumps({'output': str(output), 'sequences': len(records), 'image_count': sum(len(r['frames']) for r in records),
                      'source_hashes_unchanged': True}, indent=2))


if __name__ == '__main__':
    main()
