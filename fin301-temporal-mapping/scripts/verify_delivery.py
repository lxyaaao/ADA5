"""Verify current mapping/report artifacts without dataset-specific counts.

Run after the final report: python scripts/verify_delivery.py --dataset fin301
This checks artifact consistency and provenance, not physical map accuracy.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from indoor_mapping.geometry import pose_between, pose_compose, pose_inverse, wrap_angle
from indoor_mapping.ros_node import load_ascii_pcd, load_map_artifact, load_trajectory
from scripts.package_delivery import dataset_prefix


def require(condition, message):
    if not condition:
        raise ValueError(message)


def parse_visual_track_evidence(row, view_count, max_reprojection_px):
    """Check every saved observation against the configured view-count contract."""
    require(isinstance(view_count, int) and view_count >= 2, 'invalid configured visual view count')
    stamps = tuple(json.loads(row['timestamps_ns']))
    image_ids = json.loads(row['image_ids'])
    pixels = np.asarray(json.loads(row['pixels_uv']), dtype=float)
    errors = np.asarray(json.loads(row['reprojection_px_all_views']), dtype=float)
    require(len(stamps) == len(image_ids) == view_count and pixels.shape == (view_count, 2),
            f'landmark lacks configured {view_count}-view evidence')
    require(len(set(image_ids)) == view_count and all(isinstance(stamp, int) for stamp in stamps)
            and all(b > a for a, b in zip(stamps, stamps[1:])),
            'landmark view IDs/timestamps are not distinct, integer and ordered')
    require(np.all(np.isfinite(pixels)), 'landmark image pixels are not finite')
    require(errors.shape == (view_count,) and np.all(np.isfinite(errors))
            and np.all((errors >= 0) & (errors <= max_reprojection_px)),
            'per-view reprojection evidence is incomplete or exceeds its gate')
    require(np.isclose(float(row['max_reprojection_px']), errors.max(), atol=1e-8, rtol=0),
            'maximum reprojection error disagrees with per-view evidence')
    return stamps, image_ids, pixels, errors


def check_hashes(root, hashes, required_prefix=None, allowed_paths=()):
    """Resolve portable manifest paths, verify hashes, and return normalized keys."""
    root = Path(root).resolve()
    prefix = None if required_prefix is None else (root / required_prefix).resolve()
    allowed = {(root / path).resolve() for path in allowed_paths}
    require(bool(hashes), 'source hash manifest is empty')
    verified = {}
    for name, expected in hashes.items():
        path = (root / str(name).replace('\\', '/')).resolve()
        require(path.is_relative_to(root), f'source path is outside the project: {name}')
        if prefix is not None:
            require(path.is_relative_to(prefix) or path in allowed, f'source belongs to a different dataset: {name}')
        require(path.is_file(), f'source file is missing: {name}')
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        require(actual == expected, f'source hash mismatch: {name}')
        verified[path.relative_to(root).as_posix()] = actual
    return verified


def check_point_cloud(points, metrics, metadata):
    """Check a loaded PCD against current metric/coordinate metadata."""
    points = np.asarray(points)
    require(points.ndim == 2 and points.shape[1] == 3 and len(points) > 0, 'PCD must contain XYZ points')
    require(np.all(np.isfinite(points)), 'PCD contains nonfinite coordinates')
    require(len(points) == metrics['pcd_points'] == metadata['points'], 'PCD point count differs from metrics or map metadata')
    require(metadata['fields'] == ['x', 'y', 'z'], 'map metadata must describe XYZ fields')
    require(metadata['unit'] in ('metre', 'meter', 'metres', 'meters', 'm'), 'map units must be metric')
    require(metadata['frame_id'] == 'map', 'PCD coordinate frame must be map')
    geometry = metadata.get('geometry', 'lidar_scan_plane')
    lidar_count, visual_count = len(points), 0
    if geometry == 'lidar_scan_plane_plus_triangulated_visual_landmarks':
        lidar_count, visual_count = metadata['lidar_point_count'], metadata['visual_point_count']
        require(lidar_count > 0 and visual_count > 0 and lidar_count + visual_count == len(points), 'mixed geometry component count is invalid')
        require(lidar_count == metrics['lidar_point_count'] and visual_count == metrics['visual_point_count'], 'mixed geometry component counts disagree with metrics')
    else:
        require(geometry == 'lidar_scan_plane', 'unknown map geometry interpretation')
    require(np.all(np.abs(points[:lidar_count, 2]) <= 1e-6), 'planar LiDAR prefix must remain in the z=0 plane')
    return {'pcd_points': len(points), 'pcd_xyz_min_m': points.min(axis=0).tolist(),
            'pcd_xyz_max_m': points.max(axis=0).tolist(), 'pcd_frame': metadata['frame_id'],
            'pcd_unit': metadata['unit'], 'lidar_z_plane_m': 0.0, 'geometry': geometry,
            'lidar_point_count': lidar_count, 'visual_point_count': visual_count}


def check_loop_graph_consistency(metrics, graph, camera_events, lidar_events, before, after):
    """Cross-check constraints, sensor attribution, and saved graph residuals."""
    before, after = np.asarray(before), np.asarray(after)
    require(before.shape == after.shape and before.ndim == 2 and before.shape[1] == 3, 'graph keyframe poses have incompatible shapes')
    require(np.allclose(before[0], after[0], atol=1e-10, rtol=0), 'pose-graph anchor moved')
    edges = graph['edges']
    odometry = [edge for edge in edges if not edge['loop']]
    lidar = [edge for edge in edges if edge['loop'] and edge.get('sensor') == 'lidar_revisit']
    camera = [edge for edge in edges if edge['loop'] and edge.get('sensor') != 'lidar_revisit']
    expected_odometry = {(index, index + 1) for index in range(len(before) - 1)}
    require(len(odometry) == len(expected_odometry) and {(edge['i'], edge['j']) for edge in odometry} == expected_odometry,
            'odometry graph edges do not match the keyframe chain')
    accepted_camera = [event for event in camera_events if event.get('accepted')]
    accepted_lidar = [event for event in lidar_events if event.get('accepted')]
    require(len(camera) == len(accepted_camera) == metrics['accepted_camera_lidar_loops'], 'camera loop count differs between graph, events and metrics')
    require(len(lidar) == len(accepted_lidar) == metrics.get('accepted_lidar_revisits', 0), 'LiDAR loop count differs between graph, events and metrics')
    for label, selected, events in [('camera', camera, accepted_camera), ('lidar', lidar, accepted_lidar)]:
        indexed = {(event['i'], event['j']): event for event in events}
        require(len(indexed) == len(events), f'duplicate {label} loop events')
        require(len({(edge['i'], edge['j']) for edge in selected}) == len(selected), f'duplicate {label} graph constraints')
        for edge in selected:
            pair = (edge['i'], edge['j'])
            require(pair in indexed and np.allclose(edge['pose'], indexed[pair]['pose'], atol=1e-10, rtol=0),
                    f'{label} graph measurement differs from accepted evidence')
    require(graph['optimization'] == metrics['graph'], 'graph optimization diagnostics disagree with metrics')
    optimized = bool(graph['optimization']['success'])
    require(metrics['camera_constraints_applied'] == bool(camera and optimized), 'camera constraint application flag is incorrect')
    require(metrics.get('lidar_revisit_constraints_applied', False) == bool(lidar and optimized), 'LiDAR constraint application flag is incorrect')
    if not camera:
        require(metrics.get('max_camera_graph_correction_m') == 0, 'camera pose correction asserted without camera constraints')
    elif lidar:
        require(metrics.get('max_camera_graph_correction_m') is None, 'joint graph correction cannot be attributed wholly to camera constraints')
    residual_before, residual_after = [], []
    for edge in edges:
        i, j = edge['i'], edge['j']
        require(0 <= i < j < len(before), 'graph constraint has invalid keyframe indices')
        measurement = np.asarray(edge['pose'])
        if not edge['loop']:
            require(np.allclose(measurement, pose_between(before[i], before[j]), atol=1e-10, rtol=0), 'odometry edge measurement differs from original keyframe motion')
        sigma = np.array([.08, .08, .035] if edge['loop'] else [.06, .06, .025])
        for poses, residuals in ((before, residual_before), (after, residual_after)):
            error = pose_between(poses[i], poses[j]) - measurement
            error[2] = wrap_angle(error[2])
            residuals.extend((error / sigma).tolist())
    rms_before = float(np.sqrt(np.mean(np.square(residual_before)))) if edges else 0.
    rms_after = float(np.sqrt(np.mean(np.square(residual_after)))) if edges else 0.
    for suffix, value in [('before', rms_before), ('after', rms_after)]:
        expected = graph['optimization'].get(f'weighted_rms_{suffix}')
        if expected is not None:
            require(np.isclose(value, expected, atol=1e-7, rtol=0), f'recomputed graph RMS {suffix} differs from saved optimization')
    lidar_before = [float(np.linalg.norm(pose_between(edge['pose'], pose_between(before[edge['i']], before[edge['j']]))[:2])) for edge in lidar]
    lidar_after = [float(np.linalg.norm(pose_between(edge['pose'], pose_between(after[edge['i']], after[edge['j']]))[:2])) for edge in lidar]
    return {'graph_edges_checked': len(edges), 'accepted_camera_lidar_loops': len(camera),
            'accepted_lidar_revisits': len(lidar), 'camera_pose_correction_attributed': bool(camera),
            'lidar_loop_translation_residual_before_m': lidar_before,
            'lidar_loop_translation_residual_after_m': lidar_after,
            'graph_weighted_rms_recomputed_before': rms_before, 'graph_weighted_rms_recomputed_after': rms_after}


def verify_artifacts(root=ROOT, dataset='legacy'):
    """Return structured checks; never infer physical accuracy or test execution."""
    root = Path(root).resolve()
    relative = dataset_prefix(dataset)
    prefix = root / relative
    result, report, audit_dir = (prefix / name for name in ('result', 'report', 'audit'))
    checks = []
    facts = {'dataset': dataset, 'verified_at_utc': datetime.now(timezone.utc).isoformat(),
             'absolute_accuracy_validated': False,
             'scope': 'Artifact consistency and source provenance; no external physical reference or new ROS runtime test is asserted.'}
    checked_files = set()
    required = [result / name for name in ('map.pcd', 'map_metadata.json', 'metrics.json', 'trajectory.csv',
                'trajectory_arrays.npz', 'run_config.json', 'loop_events.json', 'pose_graph.json')]
    required += [report / 'group_technical_report.pdf', report / 'report_manifest.json']
    required += [audit_dir / name for name in ('audit.md', 'audit.json', 'calibration.json', 'rosout.json')]

    def record(name, operation):
        try:
            detail = operation()
            if isinstance(detail, dict):
                facts.update(detail)
            checks.append({'name': name, 'passed': True})
        except Exception as exc:
            checks.append({'name': name, 'passed': False, 'error': f'{type(exc).__name__}: {exc}'})

    def finish():
        facts['passed'] = all(check['passed'] for check in checks)
        facts['checks'] = checks
        facts['checked_file_sha256'] = {
            path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(checked_files) if path.is_file()
        }
        return facts

    missing = [path.relative_to(root).as_posix() for path in required if not path.is_file()]
    checks.append({'name': 'required_artifacts', 'passed': not missing, 'missing': missing})
    checked_files.update(path for path in required if path.is_file())
    if missing:
        return finish()

    try:
        metrics = json.loads((result / 'metrics.json').read_text(encoding='utf-8'))
        metadata = json.loads((result / 'map_metadata.json').read_text(encoding='utf-8'))
        config = json.loads((result / 'run_config.json').read_text(encoding='utf-8'))
        audit = json.loads((audit_dir / 'audit.json').read_text(encoding='utf-8'))
        manifest = json.loads((report / 'report_manifest.json').read_text(encoding='utf-8'))
    except Exception as exc:
        checks.append({'name': 'json_artifacts', 'passed': False, 'error': str(exc)})
        return finish()

    record('pcd_structure_count_units_and_plane', lambda: check_point_cloud(load_ascii_pcd(result / 'map.pcd'), metrics, metadata))

    def check_trajectory():
        poses = load_trajectory(result / 'trajectory.csv')
        with (result / 'trajectory.csv').open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        stamps = np.array([pose.timestamp_ns for pose in poses], dtype=np.int64)
        require(len(poses) == metrics['tracked_scans'], 'trajectory count differs from tracked scan count')
        require(np.all(np.diff(stamps) > 0), 'accepted trajectory timestamps must strictly increase')
        require(int(metadata['origin_timestamp_ns']) == int(stamps[0]), 'map origin timestamp differs from first accepted pose')
        duration = float((stamps[-1] - stamps[0]) * 1e-9)
        require(np.isclose(duration, metrics['tracked_duration_s'], atol=1e-6, rtol=0), 'trajectory duration differs from metrics')
        expected_frame = 'base_link' if metrics['vehicle_transform_available'] else metadata['source_scan_frame']
        require(all(row['pose_frame'] == expected_frame for row in rows), 'trajectory vehicle/sensor frame labels are inconsistent')
        sensor = np.array([[float(row[key]) for key in ('lidar_x', 'lidar_y', 'lidar_yaw')] for row in rows])
        vehicle = np.array([[pose.x, pose.y, pose.yaw] for pose in poses])
        require(np.all(np.isfinite(sensor)), 'sensor trajectory contains nonfinite values')
        with np.load(result / 'trajectory_arrays.npz') as arrays:
            require(np.array_equal(arrays['times_ns'], stamps), 'trajectory array timestamps differ from CSV')
            require(arrays['corrected'].shape == sensor.shape and np.allclose(arrays['corrected'], sensor, atol=1e-10), 'sensor trajectory CSV and arrays differ')
            require(len(arrays['lidar_only']) == len(poses), 'LiDAR-only trajectory count differs')
            keys = arrays['key_ids']
            require(len(keys) == metrics['keyframes'] and len(keys) > 0, 'keyframe count differs from metrics')
            require(keys[0] == 0 and keys[-1] == len(poses) - 1 and np.all(np.diff(keys) > 0), 'keyframe indices are invalid')
        expected_vehicle = sensor
        if metrics['vehicle_transform_available']:
            offset = np.asarray(metrics['base_to_lidar_se2'])
            require(np.allclose(offset, metadata['base_to_lidar_se2']), 'mount offsets disagree')
            expected_vehicle = np.array([pose_compose(pose, pose_inverse(offset)) for pose in sensor])
        require(np.allclose(vehicle, expected_vehicle, atol=1e-8), 'base/sensor trajectory transform direction is inconsistent')
        length = float(np.linalg.norm(np.diff(sensor[:, :2], axis=0), axis=1).sum())
        require(np.isclose(length, metrics['estimated_path_length_m'], atol=1e-6), 'path length differs from metrics')
        return {'trajectory_poses': len(poses), 'trajectory_frame': expected_frame,
                'trajectory_duration_s': duration, 'estimated_path_length_m': length}

    record('trajectory_timestamps_frames_and_metrics', check_trajectory)

    if metadata.get('geometry') == 'lidar_scan_plane_plus_triangulated_visual_landmarks':
        def check_visual_evidence():
            import cv2
            from indoor_mapping.visual_landmarks import camera_world_matrices, interpolate_lidar_poses
            companion_files = [result / name for name in ('lidar_scan_map.pcd', 'visual_landmarks.pcd',
                               'visual_landmark_evidence.csv', 'visual_landmark_stats.json', 'camera_landmarks.png')]
            require(all(path.is_file() for path in companion_files), 'mixed map is missing visual/LiDAR companions or evidence')
            checked_files.update(companion_files)
            combined = load_ascii_pcd(result / 'map.pcd')
            lidar = load_ascii_pcd(result / 'lidar_scan_map.pcd')
            visual = load_ascii_pcd(result / 'visual_landmarks.pcd')
            require(len(lidar) == metadata['lidar_point_count'] and len(visual) == metadata['visual_point_count'], 'component PCD counts disagree with map metadata')
            require(np.array_equal(combined[:len(lidar)], lidar) and np.array_equal(combined[len(lidar):], visual), 'primary PCD does not contain the stated ordered components')
            stats = json.loads((result / 'visual_landmark_stats.json').read_text(encoding='utf-8'))
            with (result / 'visual_landmark_evidence.csv').open(newline='', encoding='utf-8') as stream:
                evidence = list(csv.DictReader(stream))
            require(len(evidence) == len(visual) == stats['accepted_landmarks'], 'visual landmark/evidence/statistics counts disagree')
            require(stats['raw_accepted_landmarks'] >= len(visual), 'visual voxel filtering increased landmark count')
            require(stats['camera_pose_corrections_applied'] is False, 'triangulation must not assert camera pose corrections')
            require(metrics.get('camera_mapping_contribution') is True, 'camera geometry contribution is missing from metrics')
            require(metrics['visual_landmarks'] == stats, 'visual statistics differ from metrics snapshot')
            parameters = stats['parameters']
            view_count = parameters['views_per_group']
            require(isinstance(view_count, int) and view_count >= 2, 'invalid visual views_per_group parameter')
            max_reprojection = float(parameters['max_reprojection_px'])
            xyz = np.array([[float(row[key]) for key in ('x_m', 'y_m', 'z_m')] for row in evidence])
            require(np.allclose(xyz, visual, atol=1e-5, rtol=0), 'visual point coordinates differ from their track evidence')
            require([int(row['landmark_id']) for row in evidence] == list(range(len(evidence))), 'visual evidence landmark IDs are not aligned with PCD rows')
            groups = {}
            all_view_ids = set()
            reprojection_evidence = []
            for index, row in enumerate(evidence):
                stamps, image_ids, pixels, per_view_errors = parse_visual_track_evidence(row, view_count, max_reprojection)
                require(int(row['start_timestamp_ns']) == stamps[0] and int(row['end_timestamp_ns']) == stamps[-1], 'landmark time range disagrees with its views')
                require(.12 <= float(row['baseline_m']) <= 1.5, 'visual baseline is outside the documented gate')
                require(float(row['parallax_deg']) >= 2., 'visual parallax is below the documented gate')
                require(float(row['min_depth_m']) >= .4 and float(row['max_depth_m']) <= 12., 'visual depth is outside the documented gate')
                require(float(row['max_forward_backward_error_px']) <= .8, 'visual optical-flow consistency gate was exceeded')
                error = float(row['max_reprojection_px'])
                require(error <= max_reprojection, 'visual reprojection gate was exceeded')
                if parameters.get('require_upward_world_rays', False):
                    require(row.get('upward_world_rays_all_views', '').lower() == 'true',
                            'visual structural-candidate bearing gate is missing from evidence')
                reprojection_evidence.append(error)
                all_view_ids.update(image_ids)
                groups.setdefault(stamps, []).append((index, pixels, per_view_errors))
            require(len(all_view_ids) == stats['distinct_camera_views'], 'distinct visual view count disagrees with evidence')
            for key, actual in [('median', np.median(reprojection_evidence)), ('p95', np.percentile(reprojection_evidence, 95)), ('max', np.max(reprojection_evidence))]:
                require(np.isclose(actual, stats[f'reprojection_error_{key}_px'], atol=1e-8), f'visual reprojection {key} differs from evidence')
            calibration = json.loads((audit_dir / 'calibration.json').read_text(encoding='utf-8'))
            intrinsic = np.array(calibration['camera_info_first']['K'])
            distortion = np.array(calibration['camera_info_first']['D'])
            extrinsic = np.array(calibration['T_camera_lidar'])
            with np.load(result / 'trajectory_arrays.npz') as arrays:
                times, trajectory = arrays['times_ns'], arrays['corrected']
            maximum_error = 0.0
            for stamps, group in groups.items():
                poses, valid = interpolate_lidar_poses(times, trajectory, np.asarray(stamps, dtype=np.int64))
                require(valid.all(), 'visual point uses an extrapolated or unsupported trajectory time')
                cameras = camera_world_matrices(poses, extrinsic)
                indices = [index for index, _, _ in group]
                measured = np.stack([pixels for _, pixels, _ in group], axis=1)
                saved_errors = np.stack([errors for _, _, errors in group], axis=1)
                for view, world_from_camera in enumerate(cameras):
                    camera_from_world = np.linalg.inv(world_from_camera)
                    points = xyz[indices]
                    depths = (points @ camera_from_world[:3, :3].T + camera_from_world[:3, 3])[:, 2]
                    require(np.all((depths >= .4) & (depths <= 12.)), 'recomputed visual camera depth fails an all-view gate')
                    rotation, _ = cv2.Rodrigues(camera_from_world[:3, :3])
                    projection, _ = cv2.projectPoints(points, rotation, camera_from_world[:3, 3], intrinsic, distortion)
                    errors = np.linalg.norm(projection[:, 0] - measured[view], axis=1)
                    maximum_error = max(maximum_error, float(errors.max()))
                    require(np.all(errors <= max_reprojection + 1e-6), 'recomputed distorted-pixel reprojection fails an all-view gate')
                    require(np.allclose(errors, saved_errors[view], atol=1e-6, rtol=0),
                            'recomputed reprojection differs from per-view evidence')
                    if parameters.get('require_upward_world_rays', False):
                        normalized = cv2.undistortPoints(measured[view].reshape(-1, 1, 2), intrinsic, distortion).reshape(-1, 2)
                        rays = np.column_stack([normalized, np.ones(len(normalized))]) @ world_from_camera[:3, :3].T
                        require(np.all(rays[:, 2] > 0), 'recomputed world ray is not upward in every accepted view')
            return {'visual_evidence_rows': len(evidence), 'visual_distinct_camera_views': len(all_view_ids),
                    'visual_views_per_group': view_count,
                    'visual_upward_world_ray_gate': bool(parameters.get('require_upward_world_rays', False)),
                    'visual_reprojection_recomputed_max_px': maximum_error,
                    'visual_component_evidence_matches': True}
        record('mixed_geometry_components_and_all_view_evidence', check_visual_evidence)

    def check_provenance():
        require(all(metrics.get(name) is False for name in ('uses_imu', 'uses_recorded_odometry', 'uses_recorded_map')), 'excluded sensor/recorded-map input flags are not false')
        allowed = {'/scan/raw', '/camera/camera/color/image_raw/compressed', '/tf_static', '/camera/camera/color/camera_info'}
        inputs = set(config['estimator_inputs'])
        require({'/scan/raw', '/camera/camera/color/image_raw/compressed'} <= inputs <= allowed, 'estimator topic provenance is inconsistent with camera and raw 2D LiDAR scope')
        raw = audit['streams']['/scan/raw']
        require(metrics['input_scan_count'] == raw['unique_header_timestamps'], 'input scan count disagrees with deduplicated raw-scan audit')
        require(np.isclose(metrics['bag_scan_duration_s'], raw['duration_s'], atol=1e-6), 'input duration disagrees with audit')
        hashes = {f'indoor_mapping/{name}': digest for name, digest in config['source_sha256'].items()}
        verified = check_hashes(root, hashes, required_prefix=Path('indoor_mapping'))
        checked_files.update(root / name for name in verified)
        events = json.loads((result / 'loop_events.json').read_text(encoding='utf-8'))
        accepted = [event for event in events if event.get('accepted')]
        require(len(accepted) == metrics['accepted_camera_lidar_loops'], 'accepted loop count disagrees with metrics')
        require(not metrics.get('camera_constraints_applied') or (accepted and metrics['graph']['success']), 'camera correction asserted without accepted constraints and a successful graph')
        return {'estimator_source_hashes_match': True, 'input_topics': sorted(inputs),
                'input_scan_count': metrics['input_scan_count'], 'accepted_camera_lidar_loops': len(accepted)}

    record('sensor_provenance_and_estimator_source_hashes', check_provenance)

    def check_graph():
        graph = json.loads((result / 'pose_graph.json').read_text(encoding='utf-8'))
        camera_events = json.loads((result / 'loop_events.json').read_text(encoding='utf-8'))
        lidar_path = result / 'lidar_revisit_events.json'
        lidar_events = []
        if config.get('lidar_revisits') or metrics.get('accepted_lidar_revisits', 0):
            require(lidar_path.is_file(), 'LiDAR revisit evidence is missing')
        if lidar_path.is_file():
            checked_files.add(lidar_path)
            lidar_events = json.loads(lidar_path.read_text(encoding='utf-8'))['events']
        with np.load(result / 'trajectory_arrays.npz') as arrays:
            keys = arrays['key_ids']
            before, after = arrays['lidar_only'][keys], arrays['corrected'][keys]
            correction = float(np.max(np.linalg.norm(arrays['corrected'][:, :2] - arrays['lidar_only'][:, :2], axis=1)))
        if 'max_graph_correction_m' in metrics:
            require(np.isclose(correction, metrics['max_graph_correction_m'], atol=1e-8, rtol=0), 'maximum graph correction differs from saved trajectory')
        return check_loop_graph_consistency(metrics, graph, camera_events, lidar_events, before, after)

    record('loop_counts_measurements_and_sensor_attribution', check_graph)

    def check_report():
        import pymupdf
        source_hashes = check_hashes(root, manifest['source_sha256'], required_prefix=relative,
                                   allowed_paths=[Path('scripts/build_report.py')])
        checked_files.update(root / name for name in source_hashes)
        require(manifest['metrics_snapshot'] == metrics, 'report metrics snapshot differs from current results')
        require(not manifest.get('snapshot_warnings'), 'report manifest contains unresolved snapshot warnings')
        expected_pdf = (root / manifest['pdf'].replace('\\', '/')).resolve()
        require(expected_pdf == (report / 'group_technical_report.pdf').resolve(), 'report manifest points to a different PDF')
        with pymupdf.open(expected_pdf) as document:
            require(document.page_count == manifest['layout']['page_count'] == 1, 'technical report must be one page and match its manifest')
            require(bool(document[0].get_text().strip()), 'report PDF contains no selectable text')
            pages = document.page_count
        return {'report_pages': pages, 'report_sources_match': True,
                'report_snapshot_warnings': manifest.get('snapshot_warnings', [])}

    record('report_pdf_and_manifest_sources', check_report)

    if (result / 'map.yaml').exists():
        def check_occupancy():
            grid = load_map_artifact(result / 'map.yaml')
            require(grid.width > 0 and grid.height > 0, 'occupancy companion is empty')
            checked_files.update(result / name for name in ('map.yaml', 'map.pgm', 'map.png') if (result / name).exists())
            return {'occupancy_dimensions': [grid.width, grid.height], 'occupancy_resolution_m': grid.resolution}
        record('occupancy_companion', check_occupancy)
    return finish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', default='legacy', help='Dataset folder under outputs; legacy uses outputs/result')
    args = parser.parse_args()
    checks = verify_artifacts(dataset=args.dataset)
    destination = ROOT / dataset_prefix(args.dataset) / 'verification' / 'artifact_checks.json'
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(checks, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({key: value for key, value in checks.items() if key != 'checked_file_sha256'}, indent=2, allow_nan=False))
    print(f'Artifact checks: {destination}')
    raise SystemExit(0 if checks['passed'] else 1)


if __name__ == '__main__':
    main()
