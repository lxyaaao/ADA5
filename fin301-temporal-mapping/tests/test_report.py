"""Report facts must follow the selected recording, not a previous dataset."""

import json
import os

from scripts.build_report import attach_evaluation_evidence, build_content, chinese_notes, markdown_report, report_command


def _snapshot(tmp_path):
    result = tmp_path / 'fin301' / 'result'
    result.mkdir(parents=True)
    metrics = dict(tracked_duration_s=42., bag_scan_duration_s=50., tracked_scans=100,
                   rejected_registrations=2, keyframes=10, associated_within_50ms=9,
                   association_p95_ms=40., accepted_camera_lidar_loops=0,
                   lidar_icp_rmse_median_m=.01, lidar_icp_rmse_p95_m=.02,
                   estimated_path_length_m=8., pcd_points=150,
                   endpoint_separation_before_m=2., endpoint_separation_after_m=2.,
                   max_camera_graph_correction_m=0., vehicle_transform_available=False,
                   graph={'success': True})
    (result / 'metrics.json').write_text(json.dumps(metrics), encoding='utf-8')
    raw = dict(count=120, median_frequency_hz=8., duplicate_header_timestamps=4, first_header_ns=1000000000,
               header_reversals_in_bag_order=1, geometries=[{'geometry': {'beam_count': 720}}])
    audit = dict(bag_path='data/fin301/fin301_0.db3', streams={
        '/scan/raw': raw,
        '/scan': dict(duplicate_header_timestamps=7, duplicate_timestamp_range_conflicts=3),
        '/camera/camera/color/image_raw/compressed': dict(count=60, median_frequency_hz=4., first_header_ns=4000000000)},
        camera={'decode_failure_image_ids': [], 'decoded_dimensions': {'1280x720': 60}},
        synchronization={'camera_to_nearest_raw_scan': dict(median=10., p95=20., max=60., fraction_within_50_ms=.9)},
        calibration={'tf_static': [], 'camera_frame': 'camera_optical', 'scan': {'beam_count':720,'frame':'scan_frame'}})
    audit_path = result.parent / 'audit.json'
    audit_path.write_text(json.dumps(audit), encoding='utf-8')
    return result, audit_path


def test_report_uses_selected_dataset_sensor_properties(tmp_path):
    result, audit = _snapshot(tmp_path)
    content = build_content(result, audit, dataset_name='fin301 / FinTech301')
    assert '720 rays' in content['inputs']
    assert '1280x720' in content['inputs']
    assert '4 duplicate' in content['inputs']
    assert '1 reversal' in content['inputs']
    assert '3.00 s after' in content['inputs']
    assert '1,440' not in content['inputs'] and '640 x 480' not in content['inputs']
    assert 'filtering failure' not in content['method']
    assert 'fin301 / FinTech301' in content['title']


def test_report_handles_missing_mount_without_claiming_vehicle_conversion(tmp_path):
    result, audit = _snapshot(tmp_path)
    content = build_content(result, audit)
    assert 'unavailable' in content['coordinates']
    assert 'not converted' in content['coordinates']
    assert 'identity rotation' not in content['coordinates']


def test_report_notes_follow_audit_and_images_are_portable(tmp_path):
    result, audit = _snapshot(tmp_path)
    content = build_content(result, audit, dataset_name='fin301 / FinTech301')
    notes = chinese_notes(content)
    assert '3' in notes and '0' in notes
    assert '681' not in notes and '两张' not in notes
    assert 'fin301' in notes
    markdown = markdown_report(content, result.parent / 'report')
    assert '../result/pcd_preview.png' in markdown
    assert 'fin301 / FinTech301' in markdown


def test_combined_map_separates_visual_geometry_from_pose_constraints(tmp_path):
    result, audit = _snapshot(tmp_path)
    path = result / 'metrics.json'
    metrics = json.loads(path.read_text())
    metrics.update(geometry='lidar_scan_plane_plus_triangulated_visual_landmarks',
                   lidar_point_count=100, visual_point_count=50,
                   camera_mapping_contribution=True, camera_constraints_applied=False,
                   visual_landmarks={'final_point_count':50, 'accepted_before_voxel':70,
                                     'reprojection_px':{'median':.4,'p95':1.2,'max':1.8}})
    path.write_text(json.dumps(metrics), encoding='utf-8')
    content = build_content(result, audit, dataset_name='fin301 / FinTech301')
    assert 'visual_landmarks.pcd' in content['export']
    assert 'All z values are zero' not in content['export']
    assert 'four frames' in content['method']
    assert 'no camera loop constraint' in content['method']
    assert 'planar vehicle motion' in content['coordinates']
    assert ('LiDAR / visual / combined points', '100 / 50 / 150') in content['table']
    assert '50' in chinese_notes(content)


def test_six_view_camera_method_follows_run_parameters(tmp_path):
    result, audit = _snapshot(tmp_path)
    path = result / 'metrics.json'
    metrics = json.loads(path.read_text())
    metrics.update(geometry='lidar_scan_plane_plus_triangulated_visual_landmarks',
                   visual_point_count=26, lidar_point_count=124, camera_constraints_applied=False,
                   visual_landmarks={'parameters': {'views_per_group': 6}})
    path.write_text(json.dumps(metrics), encoding='utf-8')
    content = build_content(result, audit)
    assert 'six frames' in content['method']
    assert 'all six views' in content['method']
    assert 'in-sample' in content['method']
    assert '6 帧' in chinese_notes(content)
    assert '四帧' not in chinese_notes(content)
    assert 'Do not project' in content['export']


def test_localization_evidence_is_internal_and_portable(tmp_path):
    result, audit = _snapshot(tmp_path)
    evidence_path = result.parent / 'verification' / 'selected' / 'localization_evaluation.json'
    evidence_path.parent.mkdir(parents=True)
    evidence = dict(queries_summary={
        'query_count': 100, 'attempted_initializations': 800, 'accepted_registrations': 707,
        'all_starts_agree_queries': 67,
        'heldout_point_to_line_rmse_m': {'median': .0186, 'p95': .078},
        'query_max_pairwise_translation_m': {'count': 91, 'p95': .5284},
        'query_max_pairwise_yaw_deg': {'count': 91, 'p95': 3.445}},
        crosspass_queries_summary={'query_count': 20, 'all_starts_agree_queries': 11},
        quality_gates={'agreement_translation_m': .15, 'agreement_yaw_deg': 3.},
        absolute_accuracy='UNVALIDATED')
    evidence_path.write_text(json.dumps(evidence), encoding='utf-8')
    (evidence_path.parent / 'localization_quality.png').write_bytes(b'figure placeholder')
    content = build_content(result, audit)
    attach_evaluation_evidence(content, evidence_path, None)
    assert ('Held-out accepted starts', '707 / 800') in content['table']
    assert ('All starts agree / queries', '67 / 100') in content['table']
    assert '0.528' in str(content['table'])
    assert 'not absolute' in content['localization_text']
    assert '91' in content['localization_text']
    assert content['map_figure'].name == 'localization_quality.png'
    markdown = markdown_report(content, result.parent / 'report')
    assert '../verification/selected/localization_quality.png' in markdown
    assert 'all eight' in markdown
    assert '--localization-evidence' in report_command(content)
    assert '707/800' in chinese_notes(content)


def test_prior_camera_diagnostic_is_not_claimed_as_selected_method_validation(tmp_path):
    result, audit = _snapshot(tmp_path)
    content = build_content(result, audit)
    content['view_count'] = 6
    diagnostic = result.parent/'camera_diagnostic.json'
    diagnostic.write_text(json.dumps({'subsets': {'all_points': {
        'training_point_count': 93, 'both_heldout_views_within_2px': 10}}}), encoding='utf-8')
    attach_evaluation_evidence(content, None, diagnostic)
    assert '10/93' in content['camera_validation_text']
    assert 'Earlier four-view diagnostic' in content['camera_validation_text']
    assert 'selected 6-view' in content['camera_validation_text']
    assert 'not a new held-out test' in content['camera_validation_text']
    assert '--camera-validation-evidence' in report_command(content)


def test_figure_freshness_uses_run_start_not_metrics_write_time(tmp_path):
    result, audit = _snapshot(tmp_path)
    run_config = result/'run_config.json'
    run_config.write_text('{}', encoding='utf-8')
    figure = result/'pcd_preview.png'
    figure.write_bytes(b'placeholder')
    os.utime(run_config, (100, 100))
    os.utime(figure, (110, 110))
    os.utime(result/'metrics.json', (120, 120))
    content = build_content(result, audit)
    assert not any('predates' in warning for warning in content['warnings'])
    os.utime(figure, (90, 90))
    content = build_content(result, audit)
    assert any('predates run_config.json' in warning for warning in content['warnings'])
