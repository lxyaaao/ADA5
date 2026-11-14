"""Rebuild fin301 maps from a proposed trajectory using frozen mapping scan IDs.

Uses the same original-final insertion scans for every loop-closure candidate.
Camera landmarks are recomputed whenever the trajectory changes. No original
map or manually specified environment geometry is used to construct candidates.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from indoor_mapping.geometry import pose_compose, pose_inverse, pose_between, scan_to_points, wrap_angle
from indoor_mapping.occupancy import build_occupancy, save_map, save_pcd
from indoor_mapping.pipeline import write_json
from indoor_mapping.ros_node import load_ascii_pcd
from indoor_mapping.visual_landmarks import build_visual_landmarks, save_xyz_pcd


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def insertion_positions(times, poses):
    selected = [0]
    for index in range(1, len(times)):
        previous = selected[-1]
        if int(times[index])-int(times[previous]) >= 400_000_000 and (
                np.linalg.norm(poses[index, :2]-poses[previous, :2]) > .06 or
                abs(wrap_angle(poses[index, 2]-poses[previous, 2])) > .04):
            selected.append(index)
    return np.array(selected, dtype=int)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal', type=Path, required=True)
    parser.add_argument('--graph', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, default=Path('outputs/fin301/result'))
    parser.add_argument('--cache', type=Path, default=Path('outputs/fin301/cache'))
    args = parser.parse_args()
    start = time.monotonic()
    output, baseline, cache = args.output, args.baseline, args.cache
    if output.resolve().is_relative_to(baseline.resolve()):
        raise ValueError('Candidate output must be outside the original result directory')
    output.mkdir(parents=True, exist_ok=True)
    sources = {'proposal': args.proposal, 'proposed_graph': args.graph,
               'baseline_trajectory': baseline/'trajectory_arrays.npz', 'baseline_odometry': baseline/'odometry.npz',
               'baseline_metadata': baseline/'map_metadata.json', 'baseline_metrics': baseline/'metrics.json',
               'scan_cache': cache/'scans.npz', 'calibration': cache.parent/'audit/calibration.json',
               'exporter': Path(__file__), 'geometry': ROOT/'indoor_mapping/geometry.py',
               'map_export': ROOT/'indoor_mapping/occupancy.py',
               'camera_reconstruction': ROOT/'indoor_mapping/visual_landmarks.py'}
    hashes = {name: digest(path) for name, path in sources.items()}
    with np.load(baseline/'trajectory_arrays.npz') as archive:
        times = archive['times_ns'].copy()
        original = archive['corrected'].copy()
        odometry = archive['lidar_only'].copy()
        keys = archive['key_ids'].copy()
    with np.load(baseline/'odometry.npz') as archive:
        scan_ids = archive['scan_indices'].copy()
    with np.load(args.proposal) as archive:
        if not np.array_equal(archive['times_ns'], times):
            raise ValueError('Proposal must preserve every original tracked timestamp')
        proposed = archive['corrected'].copy()
        if 'scan_indices' in archive and not np.array_equal(archive['scan_indices'], scan_ids):
            raise ValueError('Proposal scan IDs differ from source trajectory')
    if proposed.shape != original.shape or not np.all(np.isfinite(proposed)) or not np.allclose(proposed[0], 0., atol=1e-10):
        raise ValueError('Expected a finite anchored SE(2) trajectory in the original map frame')
    metadata = json.loads((baseline/'map_metadata.json').read_text(encoding='utf-8'))
    metrics = json.loads((baseline/'metrics.json').read_text(encoding='utf-8'))
    graph = json.loads(args.graph.read_text(encoding='utf-8'))
    positions = insertion_positions(times, original)
    if len(positions) != metrics['map_insertions']:
        raise ValueError('Original insertion rule does not reproduce the original map count')
    raw_ids = scan_ids[positions]
    with np.load(cache/'scans.npz') as archive:
        if not np.array_equal(archive['times_ns'][scan_ids], times):
            raise ValueError('Raw scan timestamps do not match the trajectory')
        ranges = archive['ranges']
        angle_min, angle_increment = float(archive['angle_min']), float(archive['angle_increment'])
        range_min, range_max = max(.12, float(archive['range_min'])), min(12., float(archive['range_max']))
    scans = [scan_to_points(ranges[index], angle_min, angle_increment, range_min, range_max, voxel_size=.045)
             for index in raw_ids]
    write_json(output/'map_insertion_indices.json', {
        'policy': 'Frozen original-final map insertion observations; only estimated poses change',
        'baseline_trajectory_sha256': hashes['baseline_trajectory'],
        'baseline_odometry_sha256': hashes['baseline_odometry'],
        'dense_indices': positions, 'raw_scan_indices': raw_ids,
        'timestamp_ns': times[positions], 'map_insertions': len(positions)})
    offset = np.asarray(metadata['base_to_lidar_se2'], float)
    vehicle = np.array([pose_compose(pose, pose_inverse(offset)) for pose in proposed])
    with (output/'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp_ns', 'timestamp', 'x', 'y', 'yaw', 'lidar_x', 'lidar_y', 'lidar_yaw', 'pose_frame'])
        for stamp, base_pose, lidar_pose in zip(times, vehicle, proposed):
            writer.writerow([int(stamp), f'{int(stamp)//10**9}.{int(stamp)%10**9:09d}', *base_pose, *lidar_pose, 'base_link'])
    np.savez_compressed(output/'trajectory_arrays.npz', times_ns=times, lidar_only=odometry, corrected=proposed, key_ids=keys)
    shutil.copyfile(baseline/'odometry.npz', output/'odometry.npz')
    write_json(output/'pose_graph.json', graph)
    print(f'Building PCD/grid from {len(positions)} frozen source scans', flush=True)
    save_pcd(scans, proposed[positions], output/'lidar_scan_map.pcd', voxel_size=.03, z=0., max_range=12.)
    occupancy = build_occupancy(scans, proposed[positions], resolution=.05, max_range=12.)
    save_map(occupancy, output)
    lidar = load_ascii_pcd(output/'lidar_scan_map.pcd')
    if np.array_equal(proposed, original):
        for name in ('visual_landmarks.pcd', 'visual_landmark_stats.json', 'visual_landmark_evidence.csv'):
            shutil.copyfile(baseline/name, output/name)
        visual = load_ascii_pcd(output/'visual_landmarks.pcd')
        visual_stats = json.loads((output/'visual_landmark_stats.json').read_text(encoding='utf-8'))
        camera_mode = 'Original reconstruction reused only because every trajectory value is identical'
    else:
        print('Re-triangulating camera landmarks using the proposed trajectory', flush=True)
        visual, visual_stats = build_visual_landmarks(cache, times, proposed, sources['calibration'], output)
        camera_mode = 'Recomputed six-view camera geometry from original images at measurement timestamps'
    combined = np.vstack([lidar, visual])
    save_xyz_pcd(combined, output/'map.pcd')
    metadata.update(points=len(combined), lidar_point_count=len(lidar), visual_point_count=len(visual),
                    experiment='Loop-closure refinement; fixed mapping observations',
                    map_insertion_provenance='map_insertion_indices.json', camera_reconstruction=camera_mode)
    if not len(visual):
        metadata.update(geometry='lidar_scan_plane', components=['lidar_scan_map.pcd'],
                        z_definition='All points are LiDAR returns in the z=0 scan plane',
                        point_order='LiDAR scan points only; no camera landmarks passed reconstruction')
    write_json(output/'map_metadata.json', metadata)
    loops = [edge for edge in graph['edges'] if edge.get('loop')]
    old_keys, new_keys = odometry[keys], proposed[keys]
    before = [pose_between(edge['pose'], pose_between(old_keys[edge['i']], old_keys[edge['j']])) for edge in loops]
    after = [pose_between(edge['pose'], pose_between(new_keys[edge['i']], new_keys[edge['j']])) for edge in loops]
    metrics.update(map_insertions=len(positions), pcd_points=len(combined), lidar_point_count=len(lidar),
                   visual_point_count=len(visual), visual_landmarks=visual_stats, camera_mapping_contribution=bool(len(visual)),
                   graph=graph.get('optimization', graph.get('graph', {})), accepted_lidar_revisits=len(loops),
                   lidar_revisit_constraints_applied=bool(loops), geometry=metadata['geometry'],
                   estimated_path_length_m=float(np.linalg.norm(np.diff(proposed[:, :2], axis=0), axis=1).sum()),
                   endpoint_separation_after_m=float(np.linalg.norm(proposed[-1, :2]-proposed[0, :2])),
                   max_graph_correction_m=float(np.linalg.norm(proposed[:, :2]-odometry[:, :2], axis=1).max()),
                   max_added_loop_correction_m=float(np.linalg.norm(proposed[:, :2]-original[:, :2], axis=1).max()),
                   all_loop_fit_translation_before_m=[float(np.linalg.norm(item[:2])) for item in before],
                   all_loop_fit_translation_after_m=[float(np.linalg.norm(item[:2])) for item in after],
                   loop_fit_note='Residuals of imposed graph constraints; not independent localization accuracy',
                   map_insertion_policy='Frozen original-final scan IDs across variants',
                   export_elapsed_s=time.monotonic()-start)
    write_json(output/'metrics.json', metrics)
    from indoor_mapping.figures import make_figures
    make_figures(output, occupancy, odometry, proposed, [], times,
                 visual_points=visual, lidar_point_count=len(lidar), lidar_loops=bool(loops))
    for name, path in sources.items():
        if digest(path) != hashes[name]:
            raise ValueError(f'Source changed during export: {path}')
    identical_control = np.array_equal(proposed, original)
    byte_checks = {name: digest(output/name) == digest(baseline/name)
                   for name in ('lidar_scan_map.pcd', 'map.pgm', 'map.yaml', 'visual_landmarks.pcd')}
    if identical_control and not all(byte_checks.values()):
        raise ValueError(f'Unchanged-pose export failed byte-for-byte reproduction: {byte_checks}')
    write_json(output/'export_provenance.json', {'source_paths': {name: str(path) for name, path in sources.items()},
        'source_sha256': hashes, 'unchanged_pose_control': identical_control,
        'original_artifact_byte_equality': byte_checks, 'camera_reconstruction': camera_mode,
        'output_sha256': {path.name: digest(path) for path in output.iterdir()
                          if path.is_file() and path.name != 'export_provenance.json'},
        'absolute_accuracy_validated': False})
    print(json.dumps({'output': str(output), 'points': len(lidar), 'camera_landmarks': len(visual),
                      'loop_constraints': len(loops), 'control': identical_control,
                      'elapsed_s': time.monotonic()-start}, indent=2), flush=True)


if __name__ == '__main__':
    main()
