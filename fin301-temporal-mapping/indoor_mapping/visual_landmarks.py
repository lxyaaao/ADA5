"""Calibrated sparse camera landmarks anchored to a metric LiDAR trajectory.

Camera measurements contribute XYZ landmarks, not pose corrections. Vehicle
roll/pitch and LiDAR height are held constant by the planar trajectory model.
All map coordinates refer to the first LiDAR frame; negative map z is valid.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np


def interpolate_lidar_poses(times_ns, poses, query_ns, max_gap_s=0.5):
    """Interpolate XY and unwrapped yaw at exact header times, without extrapolation."""
    times = np.asarray(times_ns, dtype=np.int64)
    queries = np.asarray(query_ns, dtype=np.int64)
    poses = np.asarray(poses, dtype=float)
    if len(times) < 2 or poses.shape != (len(times), 3) or not np.all(np.diff(times) > 0):
        raise ValueError('trajectory must contain ordered, distinct timestamps and (N,3) poses')
    if not np.all(np.isfinite(poses)):
        raise ValueError('trajectory poses must be finite')
    right = np.clip(np.searchsorted(times, queries, side='right'), 1, len(times) - 1)
    left = right - 1
    # Integer subtraction precedes float conversion, preserving nanosecond deltas.
    fraction = (queries - times[left]).astype(float) / (times[right] - times[left])
    valid = (queries >= times[0]) & (queries <= times[-1]) & ((times[right] - times[left]) <= max_gap_s * 1e9)
    unwrapped = poses.copy()
    unwrapped[:, 2] = np.unwrap(unwrapped[:, 2])
    interpolated = unwrapped[left] + fraction[:, None] * (unwrapped[right] - unwrapped[left])
    interpolated[:, 2] = np.arctan2(np.sin(interpolated[:, 2]), np.cos(interpolated[:, 2]))
    interpolated[~valid] = np.nan
    return interpolated, valid


def camera_world_matrices(lidar_poses, camera_from_lidar):
    """Return world-from-optical-camera = world-from-LiDAR @ LiDAR-from-camera."""
    poses = np.asarray(lidar_poses, dtype=float)
    transform = np.asarray(camera_from_lidar, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError('recorded T_camera_lidar must be a finite 4x4 transform')
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-5):
        raise ValueError('camera extrinsic rotation must be a proper rigid transform')
    matrices = np.repeat(np.eye(4)[None], len(poses), axis=0)
    cosine, sine = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    matrices[:, 0, 0] = cosine
    matrices[:, 0, 1] = -sine
    matrices[:, 1, 0] = sine
    matrices[:, 1, 1] = cosine
    matrices[:, :2, 3] = poses[:, :2]
    return matrices @ np.linalg.inv(transform)


def triangulate_tracks(pixels, world_from_camera, intrinsic, distortion,
                       min_baseline=0.12, max_baseline=1.5, min_parallax_deg=2.0,
                       min_depth=0.4, max_depth=12.0, max_reprojection_px=2.0,
                       require_upward_world_rays=False):
    """Triangulate endpoint observations; validate depth and reprojection in ALL views.

    ``pixels`` has shape (views, tracks, 2) in original distorted image pixels.
    Returns all candidate XYZ points, an acceptance mask, and per-track evidence.
    The optional bearing gate selects upper-view structural candidates without
    imposing a metric ground height or clipping negative world z coordinates.
    """
    pixels = np.asarray(pixels, dtype=float)
    cameras = np.asarray(world_from_camera, dtype=float)
    intrinsic = np.asarray(intrinsic, dtype=float)
    distortion = np.asarray(distortion, dtype=float)
    if pixels.ndim != 3 or pixels.shape[2] != 2 or len(pixels) < 2 or cameras.shape != (len(pixels), 4, 4):
        raise ValueError('tracks require (views, tracks, 2) pixels and matching camera transforms')
    count = pixels.shape[1]
    camera_from_world = np.linalg.inv(cameras)
    baseline = float(np.linalg.norm(cameras[-1, :3, 3] - cameras[0, :3, 3]))
    normalized = [cv2.undistortPoints(view.reshape(-1, 1, 2), intrinsic, distortion).reshape(-1, 2) for view in pixels]
    homogeneous = cv2.triangulatePoints(camera_from_world[0, :3], camera_from_world[-1, :3],
                                       normalized[0].T, normalized[-1].T).T
    finite = np.abs(homogeneous[:, 3]) > 1e-10
    points = np.full((count, 3), np.nan)
    points[finite] = homogeneous[finite, :3] / homogeneous[finite, 3, None]
    finite &= np.all(np.isfinite(points), axis=1)
    safe_points = np.where(finite[:, None], points, 0.0)
    rays = []
    for view in (0, -1):
        bearing = np.column_stack([normalized[view], np.ones(count)]) @ cameras[view, :3, :3].T
        rays.append(bearing / np.linalg.norm(bearing, axis=1)[:, None])
    parallax = np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i', *rays), -1, 1)))
    depths, errors = [], []
    for index, extrinsic in enumerate(camera_from_world):
        camera_points = safe_points @ extrinsic[:3, :3].T + extrinsic[:3, 3]
        depths.append(camera_points[:, 2])
        rotation_vector, _ = cv2.Rodrigues(extrinsic[:3, :3])
        projected, _ = cv2.projectPoints(safe_points, rotation_vector, extrinsic[:3, 3], intrinsic, distortion)
        errors.append(np.linalg.norm(projected[:, 0] - pixels[index], axis=1))
    depths, errors = np.array(depths), np.array(errors)
    depth_ok = np.all((depths >= min_depth) & (depths <= max_depth), axis=0)
    reprojection_ok = np.all(np.isfinite(errors) & (errors <= max_reprojection_px), axis=0)
    parallax_ok = parallax >= min_parallax_deg
    baseline_ok = min_baseline <= baseline <= max_baseline
    upward = np.ones(count, dtype=bool)
    for view, camera in zip(normalized, cameras):
        world_rays = np.column_stack([view, np.ones(count)]) @ camera[:3, :3].T
        upward &= np.isfinite(world_rays[:, 2]) & (world_rays[:, 2] > 0)
    valid = finite & depth_ok & reprojection_ok & parallax_ok & baseline_ok
    if require_upward_world_rays:
        valid &= upward
    diagnostic = {'baseline_m': baseline, 'parallax_deg': parallax,
                  'min_depth_m': depths.min(axis=0), 'max_depth_m': depths.max(axis=0),
                  'max_reprojection_px': errors.max(axis=0), 'reprojection_px_all_views': errors,
                  'finite': finite, 'depth_ok': depth_ok, 'reprojection_ok': reprojection_ok,
                  'parallax_ok': parallax_ok, 'baseline_ok': baseline_ok,
                  'upward_ray_ok': upward}
    return points, valid, diagnostic


def save_xyz_pcd(points, path):
    """Write finite metric XYZ in the common map frame as ASCII PCD 0.7."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not np.all(np.isfinite(points)):
        raise ValueError('PCD points must be finite')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ('# .PCD v0.7 - metric XYZ in initial LiDAR map frame\nVERSION 0.7\n'
              'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n')
    with path.open('w', encoding='ascii', newline='\n') as stream:
        stream.write(header)
        np.savetxt(stream, points, fmt='%.6f')
    return {'path': path, 'point_count': len(points)}


def _track_views(images, max_fb_error=0.8):
    """Track one corner set through every supplied image in chronological order."""
    view_count = len(images)
    if view_count < 2:
        raise ValueError('Tracking requires at least two image views')
    initial = cv2.goodFeaturesToTrack(images[0], maxCorners=800, qualityLevel=0.01, minDistance=8, blockSize=7)
    if initial is None:
        return np.empty((view_count, 0, 2)), np.empty(0), 0
    initial = cv2.cornerSubPix(images[0], initial, (5, 5), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    tracks = [initial[:, 0]]
    original_count = len(initial)
    maximum_fb = np.zeros(original_count)
    arguments = dict(winSize=(21, 21), maxLevel=3,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    for previous, current in zip(images[:-1], images[1:]):
        before = tracks[-1].astype(np.float32).reshape(-1, 1, 2)
        following, forward_status, _ = cv2.calcOpticalFlowPyrLK(previous, current, before, None, **arguments)
        if following is None:
            return np.empty((view_count, 0, 2)), np.empty(0), original_count
        reverse, reverse_status, _ = cv2.calcOpticalFlowPyrLK(current, previous, following, None, **arguments)
        if reverse is None:
            return np.empty((view_count, 0, 2)), np.empty(0), original_count
        fb = np.linalg.norm(reverse[:, 0] - before[:, 0], axis=1)
        points = following[:, 0]
        height, width = current.shape
        valid = (forward_status[:, 0] > 0) & (reverse_status[:, 0] > 0) & np.isfinite(fb) & (fb <= max_fb_error)
        valid &= np.all(np.isfinite(points), axis=1) & (points[:, 0] >= 6) & (points[:, 0] < width - 6) & (points[:, 1] >= 6) & (points[:, 1] < height - 6)
        tracks = [track[valid] for track in tracks] + [points[valid]]
        maximum_fb = np.maximum(maximum_fb[valid], fb[valid])
        if len(points[valid]) < 8:
            return np.empty((view_count, 0, 2)), np.empty(0), original_count
    return np.asarray(tracks), maximum_fb, original_count


def _summary(values):
    values = np.asarray(values)
    return {name: None for name in ('min', 'median', 'p95', 'max')} if not len(values) else {
        'min': float(np.min(values)), 'median': float(np.median(values)),
        'p95': float(np.percentile(values, 95)), 'max': float(np.max(values))}


def _complete_jpeg(content):
    """Permit transport zero padding after JPEG EOI, but reject truncated data."""
    return content.startswith(b'\xff\xd8') and content.rstrip(b'\x00').endswith(b'\xff\xd9')


def _evidence_figure(path, images, pixels, image_ids, baseline, errors):
    pair = []
    for image, view in zip((images[0], images[-1]), (pixels[0], pixels[-1])):
        color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        for index in np.linspace(0, len(view) - 1, min(80, len(view))).astype(int):
            xy = tuple(np.rint(view[index]).astype(int))
            cv2.circle(color, xy, 4, (40, 220, 40), 1, cv2.LINE_AA)
        pair.append(color)
    canvas = np.vstack([np.full((64, sum(image.shape[1] for image in pair), 3), 245, np.uint8), np.hstack(pair)])
    label = f'{len(image_ids)}-view structural candidates: frames {image_ids}; accepted {pixels.shape[1]}; baseline {baseline:.3f} m'
    cv2.putText(canvas, label, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, .56, (25, 25, 25), 1, cv2.LINE_AA)
    cv2.putText(canvas, f'First / last views shown; all {len(image_ids)} views validate in-sample <= 2 px. No localization accuracy gain claimed.',
                (12, 49), cv2.FONT_HERSHEY_SIMPLEX, .53, (25, 25, 25), 1, cv2.LINE_AA)
    if not cv2.imwrite(str(path), canvas):
        raise OSError(f'Could not write {path}')


def build_visual_landmarks(cache_dir, trajectory_times_ns, lidar_poses, calibration_path, output_dir,
                          views_per_group=6, upward_only=True):
    """Build and audit sparse visual landmarks; return (XYZ, statistics).

    By default uses six frames about 0.3 s apart, starts groups every 0.9 s, interpolates
    LiDAR motion at actual image header timestamps, and never modifies poses.
    First/last observations triangulate; every view participates in acceptance.
    This is in-sample validation, not held-out evaluation or localization gain.
    """
    cache, output = Path(cache_dir), Path(output_dir)
    if not isinstance(views_per_group, int) or views_per_group < 2:
        raise ValueError('views_per_group must be an integer of at least two')
    calibration_path = Path(calibration_path)
    calibration = json.loads(calibration_path.read_text(encoding='utf-8'))
    info = calibration['camera_info_first']
    if calibration.get('camera_info_unique_calibrations') != 1:
        raise ValueError('Visual reconstruction requires one consistent recorded camera calibration')
    if info['distortion_model'] not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('Unsupported recorded camera distortion model')
    if info['frame_id'] != calibration['camera_frame'] or calibration.get('T_camera_lidar') is None:
        raise ValueError('Recorded camera frame/extrinsic calibration is missing or inconsistent')
    intrinsic, distortion = np.asarray(info['K'], dtype=float), np.asarray(info['D'], dtype=float)
    if intrinsic.shape != (3, 3) or not np.all(np.isfinite(intrinsic)) or min(intrinsic[0, 0], intrinsic[1, 1]) <= 0:
        raise ValueError('Recorded camera intrinsics are invalid')
    extrinsic = np.asarray(calibration['T_camera_lidar'], dtype=float)
    output.mkdir(parents=True, exist_ok=True)
    rows = sorted(csv.DictReader((cache / 'images.csv').open(encoding='utf-8')), key=lambda row: int(row['timestamp_ns']))
    times = np.array([int(row['timestamp_ns']) for row in rows], dtype=np.int64)
    image_poses, pose_valid = interpolate_lidar_poses(trajectory_times_ns, lidar_poses, times)
    invalid_images = set()
    audit_path = calibration_path.parent / 'audit.json'
    if audit_path.exists():
        invalid_images = set(json.loads(audit_path.read_text(encoding='utf-8'))['camera'].get('decode_failure_image_ids', []))
    parameters = {'views_per_group': views_per_group, 'frame_interval_s': .3, 'group_stride_s': .9,
                  'timestamp_selection_tolerance_s': .08, 'max_pose_bracket_gap_s': .5,
                  'min_baseline_m': .12, 'max_baseline_m': 1.5, 'min_parallax_deg': 2.,
                  'min_depth_m': .4, 'max_depth_m': 12., 'max_reprojection_px': 2.,
                  'max_forward_backward_error_px': .8, 'voxel_size_m': .05,
                  'min_landmarks_per_group': 8, 'min_image_hull_fraction': .01,
                  'require_upward_world_rays': bool(upward_only),
                  'validation_role': 'All views participate in in-sample acceptance; not held-out accuracy'}
    view_count = parameters['views_per_group']
    rejections = {name: 0 for name in ('timestamp_selection', 'pose_bracket', 'baseline', 'image',
                  'tracking', 'invalid_triangulation', 'depth', 'parallax', 'reprojection', 'upward_rays', 'feature_support')}
    evidence, all_points = [], []
    groups = accepted_groups = detected = tracks_tested = 0
    previous_start = -10**30
    best = None
    for start, stamp in enumerate(times):
        if int(stamp) - previous_start < 900_000_000:
            continue
        previous_start = int(stamp)
        groups += 1
        desired = stamp + np.arange(view_count, dtype=np.int64) * 300_000_000
        right = np.clip(np.searchsorted(times, desired), 0, len(times) - 1)
        left = np.maximum(right - 1, 0)
        selected = np.where(np.abs(times[left] - desired) <= np.abs(times[right] - desired), left, right)
        if len(set(selected)) != view_count or np.any(np.abs(times[selected] - desired) > 80_000_000):
            rejections['timestamp_selection'] += 1
            continue
        if not pose_valid[selected].all():
            rejections['pose_bracket'] += 1
            continue
        cameras = camera_world_matrices(image_poses[selected], extrinsic)
        baseline = np.linalg.norm(cameras[-1, :3, 3] - cameras[0, :3, 3])
        if not .12 <= baseline <= 1.5:
            rejections['baseline'] += 1
            continue
        images = []
        selected_ids = [int(rows[index]['image_id']) for index in selected]
        for index, image_id in zip(selected, selected_ids):
            path = Path(rows[index]['path'])
            if not path.exists():
                path = cache / 'images' / path.name
            if image_id in invalid_images or not path.is_file():
                break
            content = path.read_bytes()
            if path.suffix.lower() in ('.jpg', '.jpeg') and not _complete_jpeg(content):
                break
            image = cv2.imdecode(np.frombuffer(content, np.uint8), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (info['height'], info['width']):
                break
            images.append(image)
        if len(images) != view_count:
            rejections['image'] += 1
            continue
        pixels, fb_errors, initial_count = _track_views(images)
        detected += initial_count
        tracks_tested += pixels.shape[1]
        rejections['tracking'] += initial_count - pixels.shape[1]
        if pixels.shape[1] < 8:
            continue
        points, valid, diagnostic = triangulate_tracks(
            pixels, cameras, intrinsic, distortion, require_upward_world_rays=upward_only)
        for reason, mask in [('invalid_triangulation', 'finite'), ('depth', 'depth_ok'), ('parallax', 'parallax_ok'), ('reprojection', 'reprojection_ok')]:
            rejections[reason] += int(np.count_nonzero(~diagnostic[mask]))
        if upward_only:
            rejections['upward_rays'] += int(np.count_nonzero(~diagnostic['upward_ray_ok']))
        if np.count_nonzero(valid) < 8:
            rejections['feature_support'] += 1
            continue
        coverage = min(cv2.contourArea(cv2.convexHull(view[valid].astype(np.float32))) / (info['width'] * info['height']) for view in pixels)
        if coverage < .01:
            rejections['feature_support'] += 1
            continue
        accepted_groups += 1
        for index in np.flatnonzero(valid):
            point = points[index]
            all_points.append(point)
            evidence.append({'x_m': float(point[0]), 'y_m': float(point[1]), 'z_m': float(point[2]),
                'image_ids': json.dumps(selected_ids), 'timestamps_ns': json.dumps(times[selected].tolist()),
                'pixels_uv': json.dumps(pixels[:, index].tolist()), 'start_timestamp_ns': int(times[selected[0]]),
                'end_timestamp_ns': int(times[selected[-1]]), 'baseline_m': float(baseline),
                'parallax_deg': float(diagnostic['parallax_deg'][index]),
                'min_depth_m': float(diagnostic['min_depth_m'][index]), 'max_depth_m': float(diagnostic['max_depth_m'][index]),
                'max_reprojection_px': float(diagnostic['max_reprojection_px'][index]),
                'reprojection_px_all_views': json.dumps(diagnostic['reprojection_px_all_views'][:, index].tolist()),
                'max_forward_backward_error_px': float(fb_errors[index]),
                'upward_world_rays_all_views': bool(diagnostic['upward_ray_ok'][index])})
        score = np.count_nonzero(valid) * coverage
        if best is None or score > best[0]:
            best = (score, [image.copy() for image in images], pixels[:, valid].copy(), selected_ids,
                    float(baseline), diagnostic['max_reprojection_px'][valid].copy())
        if groups % 100 == 0:
            print(f'Visual landmarks: {groups} groups, {accepted_groups} accepted, {len(all_points)} candidate points', flush=True)
    raw_count = len(all_points)
    points = np.asarray(all_points, dtype=float).reshape(-1, 3)
    if len(points):
        quality = np.array([row['max_reprojection_px'] for row in evidence])
        order = np.argsort(quality, kind='stable')
        cells = np.floor(points[order] / .05).astype(np.int64)
        _, chosen = np.unique(cells, axis=0, return_index=True)
        keep = np.sort(order[chosen])
        points = points[keep]
        evidence = [evidence[index] for index in keep]
    save_xyz_pcd(points, output / 'visual_landmarks.pcd')
    fieldnames = ['landmark_id', 'x_m', 'y_m', 'z_m', 'image_ids', 'timestamps_ns', 'pixels_uv',
                  'start_timestamp_ns', 'end_timestamp_ns', 'baseline_m', 'parallax_deg', 'min_depth_m',
                  'max_depth_m', 'max_reprojection_px', 'reprojection_px_all_views', 'max_forward_backward_error_px',
                  'upward_world_rays_all_views']
    with (output / 'visual_landmark_evidence.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows({'landmark_id': index, **row} for index, row in enumerate(evidence))
    if best is not None:
        _evidence_figure(output / 'camera_landmarks.png', *best[1:])
    reprojection = _summary([row['max_reprojection_px'] for row in evidence])
    view_ids = sorted({image_id for row in evidence for image_id in json.loads(row['image_ids'])})
    observation_times = [stamp for row in evidence for stamp in json.loads(row['timestamps_ns'])]
    statistics = {'accepted_landmarks': len(points), 'raw_accepted_landmarks': raw_count,
                  'tracks_tested': tracks_tested, 'groups_tested': groups, 'groups_accepted': accepted_groups,
                  'detected_features': detected, 'rejections': rejections,
                  'rejection_count_note': 'tracking is a track count; geometry rejection counts overlap; group selection/support reasons count groups',
                  'reprojection_error_median_px': reprojection['median'], 'reprojection_error_p95_px': reprojection['p95'],
                  'reprojection_error_max_px': reprojection['max'], 'reprojection_px': reprojection,
                  'parallax_deg': _summary([row['parallax_deg'] for row in evidence]),
                  'baseline_m': _summary([row['baseline_m'] for row in evidence]),
                  'distinct_camera_views': len(view_ids), 'camera_image_ids': view_ids,
                  'observation_time_ns': {'min': min(observation_times) if observation_times else None,
                                          'max': max(observation_times) if observation_times else None},
                  'parameters': parameters, 'camera_pose_corrections_applied': False,
                  'validation_role': f'{view_count}-view in-sample geometric acceptance; no independent accuracy validation',
                  'localization_gain_demonstrated': False,
                  'pose_interpolation_method': 'Integer nanosecond differences; linear XY and unwrapped yaw at image header timestamps; no extrapolation; maximum bracket gap 0.5 s',
                  'world_from_camera': 'embed_SE2(world_from_lidar, z=0) @ inverse(T_camera_lidar)',
                  'coordinate_frame': 'Initial LiDAR frame, metres; LiDAR scan plane z=0; negative z is valid',
                  'camera_origin_in_lidar_m': np.linalg.inv(extrinsic)[:3, 3].tolist(),
                  'calibration_path': str(calibration_path), 'intrinsic_matrix': intrinsic.tolist(), 'distortion_coefficients': distortion.tolist(),
                  'assumptions': ['Recorded calibration is used without independent physical validation.',
                      'Vehicle roll, pitch and LiDAR height are fixed by the planar trajectory model.',
                      'Geometrically consistent tracks are treated as static; some moving objects or reflections can remain.',
                      ('Only upward world rays are retained; this excludes direct foreground floor views but not every reflection.'
                       if upward_only else 'Downward world rays are permitted for baseline reproduction; foreground reflections can remain.'),
                      f'All {view_count} views participate in acceptance; their residuals are not held-out performance.',
                      'Sparse triangulated heights are estimates, not a dense reconstruction or independently validated height measurements.'],
                  'representative_image_ids': None if best is None else best[3],
                  'representative_prevoxel_landmarks': 0 if best is None else int(best[2].shape[1]),
                  'final_point_count': len(points), 'accepted_before_voxel': raw_count,
                  'windows_total': groups, 'windows_accepted': accepted_groups, 'tracks_surviving_all_views': tracks_tested}
    (output / 'visual_landmark_stats.json').write_text(json.dumps(statistics, indent=2, allow_nan=False), encoding='utf-8')
    return points, statistics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', required=True)
    parser.add_argument('--trajectory', required=True)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--views', type=int, default=6, help='Views per track group; default six')
    parser.add_argument('--allow-downward-rays', action='store_true',
                        help='Disable upward bearing selection, e.g. --views 4 --allow-downward-rays for the diagnostic baseline')
    args = parser.parse_args()
    with np.load(args.trajectory) as arrays:
        points, stats = build_visual_landmarks(args.cache, arrays['times_ns'], arrays['corrected'], args.calibration, args.output,
                                             views_per_group=args.views, upward_only=not args.allow_downward_rays)
    print(json.dumps({key: value for key, value in stats.items() if key != 'camera_image_ids'}, indent=2))


if __name__ == '__main__':
    main()
