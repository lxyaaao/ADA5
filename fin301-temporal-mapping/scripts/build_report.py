"""Build the one-page English report from auditable current result artifacts.

Run after mapping finishes: .venv/Scripts/python.exe scripts/build_report.py --strict
The report always remains a draft until independent physical validation exists.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
import os
from pathlib import Path

import pymupdf as fitz
import numpy as np
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas
from reportlab.platypus import Paragraph


REFERENCES = [
    ('ROS 2 LaserScan / timestamp and axes', 'https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg'),
    ('OpenCV / camera geometry and triangulation', 'https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html'),
    ('SciPy / nonlinear least squares', 'https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html'),
    ('PCL / PCD file format', 'https://pointclouds.org/documentation/tutorials/pcd_file_format.html'),
]


def load_json(path: Path):
    return json.loads(path.read_text(encoding='utf-8'))


def build_content(result: Path, audit_path: Path, dataset_name: str | None = None) -> dict:
    metrics_path = result / 'metrics.json'
    metrics = load_json(metrics_path)
    audit = load_json(audit_path)
    dataset_name = dataset_name or Path(audit['bag_path']).parent.name
    combined = metrics.get('geometry') == 'lidar_scan_plane_plus_triangulated_visual_landmarks'
    visual_stats = metrics.get('visual_landmarks', {})
    visual_count = int(metrics.get('visual_point_count', visual_stats.get('final_point_count', 0)))
    view_count = int(visual_stats.get('parameters', {}).get('views_per_group', 4))
    view_word = {4: 'four', 6: 'six'}.get(view_count, str(view_count))
    events = load_json(result / 'loop_events.json') if (result / 'loop_events.json').exists() else []
    accepted = [event for event in events if event.get('accepted')]
    warnings = []
    if (metrics.get('accepted_camera_lidar_loops',0) or metrics.get('accepted_lidar_revisits',0)) and not metrics.get('graph',{}).get('success',False):
        warnings.append('Pose-graph optimization did not converge; accepted revisit constraints were not applied.')
    if len(accepted) != metrics.get('accepted_camera_lidar_loops', 0):
        warnings.append('Loop event count differs from metrics: a mapping run may still be writing outputs.')
    if metrics.get('uses_imu') or metrics.get('uses_recorded_odometry') or metrics.get('uses_recorded_map'):
        raise ValueError('Estimator input provenance violates this report scope')
    arrays_path = result / 'trajectory_arrays.npz'
    nonstationary = []
    if arrays_path.exists():
        with np.load(arrays_path) as arrays:
            poses = arrays['lidar_only']
            keys = arrays['key_ids']
            if len(poses) != metrics['tracked_scans']:
                warnings.append('Trajectory and metrics scan counts differ.')
            key_poses = poses[keys]
            for event in accepted:
                i, j = int(event['i']), int(event['j'])
                if 0 <= i < j < len(key_poses):
                    excursion = float(np.max(np.linalg.norm(key_poses[i:j+1, :2] - key_poses[i, :2], axis=1)))
                    if excursion > 3.0 and event['time_j_s'] - event['time_i_s'] > 35:
                        nonstationary.append((event, excursion))
    representative = max(nonstationary, key=lambda pair: pair[0]['visual']['inliers']) if nonstationary else None
    displayed_event = max(accepted, key=lambda event: event['visual']['inliers']) if accepted else None
    camera_figure = result / ('camera_landmarks.png' if combined else 'camera_revisit.png')
    map_figure = result / 'pcd_preview.png'
    camera_safe = (bool(visual_count > 0 and camera_figure.exists()) if combined else
                   bool(representative and displayed_event is representative[0] and camera_figure.exists()))
    if combined and visual_count > 0 and not camera_figure.exists():
        warnings.append('Visual landmarks exist but camera_landmarks.png evidence is missing.')
    # Camera evidence is emitted during construction, before end-of-run metrics.
    # A figure is stale only if it precedes the recorded run start.
    run_start = result/'run_config.json'
    for figure in [map_figure] + ([camera_figure] if camera_safe else []):
        if not figure.exists():
            warnings.append(f'Missing result figure: {figure.name}.')
        elif run_start.is_file() and figure.stat().st_mtime_ns < run_start.stat().st_mtime_ns:
            warnings.append(f'{figure.name} predates run_config.json; regenerate mapping figures.')
            if figure == camera_figure:
                camera_safe = False
    raw = audit['streams']['/scan/raw']
    camera = audit['streams']['/camera/camera/color/image_raw/compressed']
    sync = audit['synchronization']['camera_to_nearest_raw_scan']
    scan_filtered = audit['streams'].get('/scan', {})
    calibration = audit['calibration']
    scan_frame = calibration.get('scan', {}).get('frame', 'unknown scan frame')
    mounts = calibration.get('tf_static', [])
    lidar_mount = next((tf for tf in mounts if tf['parent'] == 'base_link' and tf['child'] == scan_frame), None)
    camera_mount = next((tf for tf in mounts if tf['parent'] == 'base_link' and tf['child'] == calibration.get('camera_frame')), None)
    beam_counts = sorted({int(item['geometry']['beam_count']) for item in raw.get('geometries', [])
                          if 'beam_count' in item.get('geometry', {})})
    if not beam_counts and 'beam_count' in calibration.get('scan', {}):
        beam_counts = [int(calibration['scan']['beam_count'])]
    beams = '/'.join(f'{number:,}' for number in beam_counts) or 'unknown'
    dimensions = ', '.join(audit.get('camera', {}).get('decoded_dimensions', {})) or 'dimensions unavailable'
    decode_failures = len(audit.get('camera', {}).get('decode_failure_image_ids', []))
    camera_start_delay = ((int(camera['first_header_ns']) - int(raw['first_header_ns'])) / 1e9
                          if 'first_header_ns' in camera and 'first_header_ns' in raw else None)
    start_note = (f'RGB starts {camera_start_delay:.2f} s after raw LiDAR; initial scans have no camera observation. '
                  if camera_start_delay is not None and camera_start_delay > 0.05 else '')
    duration = float(metrics['tracked_duration_s'])
    total_duration = float(metrics['bag_scan_duration_s'])
    coverage = 100 * duration / total_duration if total_duration > 0 else 0.
    partial = duration < total_duration - 1.0
    track_status = (f'Tracked prefix only: {duration:.1f} / {total_duration:.1f} s ({coverage:.1f}%); later motion is unestimated.'
                    if partial else f'Estimated duration: {duration:.1f} / {total_duration:.1f} s ({coverage:.1f}% of raw-scan span).')
    graph = metrics.get('graph', {})
    graph_text = ('Weighted graph RMS: '
                  f"{graph['weighted_rms_before']:.4f} to {graph['weighted_rms_after']:.4f}. "
                  if 'weighted_rms_before' in graph and 'weighted_rms_after' in graph else '')
    scope = ('Offline reconstruction uses the supplied physical-car ROS 2 bag: a monocular RGB camera and raw planar LiDAR. '
             'No IMU, GNSS, magnetometer, wheel/recorded odometry, recorded poses, dynamic TF or recorded map enters estimation. '
             'The supplied recording is used as received; this report makes no claim of new data collection.')
    scan_audit = (f"The separate /scan stream has {scan_filtered.get('duplicate_header_timestamps', 0):,} duplicate stamps and "
                  f"{scan_filtered.get('duplicate_timestamp_range_conflicts', 0)} conflicting range arrays; it is excluded. "
                  if scan_filtered else 'No separate /scan stream is reported. ')
    inputs = (f"The bag contains {raw['count']:,} raw scans ({raw['median_frequency_hz']:.2f} Hz, {beams} rays) and "
              f"{camera['count']:,} compressed RGB frames ({camera['median_frequency_hz']:.2f} Hz, {dimensions}); "
              f"{decode_failures} images fail decoding. Raw stamps have {raw.get('duplicate_header_timestamps', 0)} duplicates and "
              f"{raw.get('header_reversals_in_bag_order', 0)} reversals in recording order. " + scan_audit + start_note +
              'Integer header timestamps [1] support nearest-image association within 50 ms. '
              f"Across all RGB frames, nearest-raw-scan error median/p95/max is {sync['median']:.2f}/{sync['p95']:.2f}/{sync['max']:.2f} ms; "
              f"{100 * sync['fraction_within_50_ms']:.1f}% are within 50 ms. This is timestamp alignment, not hardware synchronization.")
    method = ('Finite, range-gated LiDAR points are spatially filtered and registered to a recent local submap using robust point-to-line ICP. '
              'Spatial filtering and adjacent-scan recovery support registration. '
              'Failed or degenerate matches are rejected. Camera appearance proposes revisits; ORB correspondences and homography RANSAC [2] '
              'screen candidates after intervening motion. Reciprocal LiDAR registration supplies and checks metric loop edges. '
              'Robust SE(2) graph optimization [3] fixes the first pose; corrected scans form the map. Monocular image matches supply no metric translation. '
              'The camera contribution is measured as the graph correction, not assumed accuracy improvement.')
    if combined:
        loop_status = ('This run accepts no camera loop constraint; camera does not update poses. '
                       if not metrics.get('camera_constraints_applied', False) else
                       'Accepted loop constraints update the anchored SE(2) graph [3]. ')
        method = ('Raw LiDAR uses robust point-to-line ICP against a local submap; invalid or degenerate matches are rejected. '
                  'Camera revisit proposals require ORB/RANSAC and reciprocal LiDAR verification. ' + loop_status +
                  f'Separate camera mapping tracks features through {view_word} frames, undistorts recorded K/D and triangulates the first/last pair '
                  'using LiDAR-derived metric camera poses and static extrinsics [2]. Depth/range, baseline, parallax, forward-backward '
                  f'flow and reprojection checks in all {view_word} views provide in-sample screening. The combined map retains both sources; camera points '
                  'do not independently validate the LiDAR trajectory.')
        if visual_stats.get('parameters', {}).get('require_upward_world_rays'):
            method += ' Upward world-ray filtering selects structural candidates; no arbitrary world-height cutoff is applied.'
    if metrics.get('lidar_revisit_constraints_applied'):
        method += (f" {metrics.get('accepted_lidar_revisits',0)} separately verified LiDAR revisit edge(s) correct the SE(2) trajectory [3]. "
                   f"Maximum graph correction is {metrics.get('max_graph_correction_m',0):.3f} m; it is not a measured absolute-error reduction. "
                   'An engineering problem was that valid-normal support did not measure actual scan overlap. '
                   'Separate geometric coverage, reciprocal registration and neighboring-scan checks support conservative partial-overlap revisits.')
    mount_text = []
    for label, transform in [('laser', lidar_mount), ('camera', camera_mount)]:
        if transform is None:
            mount_text.append(f'Direct base-to-{label} mounting is unavailable in the recorded static TF.')
            continue
        xyz = transform.get('translation_xyz_m')
        if xyz is None and 'T_parent_child' in transform:
            xyz = np.asarray(transform['T_parent_child'])[:3, 3]
        if xyz is not None:
            mount_text.append(f"Recorded base-to-{label} xyz=({xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}) m.")
    conversion = ('Recorded planar mounting converts laser poses to vehicle poses.' if metrics.get('vehicle_transform_available')
                  else 'Laser poses are not converted to vehicle poses: a suitable mounting transform is unavailable.')
    coords = ('The local origin is the first accepted LiDAR pose: x forward, y left, metres; z=0 is the scan-plane projection. '
              + ' '.join(mount_text) + ' ' + conversion +
              ' Full rotations and intrinsics are in calibration.json; physical calibration remains unverified.')
    if combined:
        coords += ' Visual heights are relative to the initial LiDAR scan plane under planar vehicle motion.'
    export = ('map.pcd uses PCD 0.7 ASCII, FIELDS x y z, SIZE 4 4 4, TYPE F F F, in metres [4]. '
              'All z values are zero; it is a planar point cloud, not a 3D reconstruction. '
              'View with pcl_viewer, CloudCompare with PCD/qPCL support, or the ROS 2 PointCloud2 adapter in RViz. '
              'PGM/YAML occupancy and trajectory CSV are companion outputs.')
    if combined:
        export = ('map.pcd is PCD 0.7 ASCII XYZ in metres [4]: LiDAR scan-plane points (z=0) plus triangulated camera XYZ. '
                  'Use lidar_scan_map.pcd or LiDAR occupancy for 2D localization; visual_landmarks.pcd is a separate experimental component. '
                  'Do not project visual XYZ into a 2D localization map. The combined map is for source inspection, not validated 3D navigation. '
                  'Open with PCL pcl_viewer or CloudCompare with PCD support.')
    limitations = ('ICP residuals, graph residuals, endpoint separation and camera-induced corrections are internal consistency measures, '
                   'not absolute localization errors. ' + graph_text +
                   'Rolling-scan motion distortion is uncompensated; calibration uncertainty, planar assumptions, repeated scene structure and rejected scans limit reliability. '
                   '<b>Physical-reference validation remains incomplete; no reference provided.</b> '
                   'A measured wall length or independent trajectory is needed to test scale and pose error; no centimetre-accuracy claim is supported.')
    if combined:
        limitations = ('Visual reprojection and ICP residuals are internal consistency measures, not independent 3D or pose accuracy. '
                       'Planar-motion, recorded calibration, moving objects, low texture and uncompensated scan motion limit reliability. '
                       '<b>Physical-reference validation remains incomplete; no reference provided.</b> '
                       'Measured walls, landmarks or an independent trajectory are needed; no centimetre-accuracy claim is supported.')
    if partial:
        limitations = '<b>' + track_status + '</b> ' + limitations
    table = [
        ('Tracked duration / bag span', f'{duration:.1f} / {total_duration:.1f} s'),
        ('Accepted scans / rejected matches', f"{metrics['tracked_scans']:,} / {metrics['rejected_registrations']:,}"),
        ('Keyframes / RGB within 50 ms', f"{metrics['keyframes']} / {metrics['associated_within_50ms']}"),
        ('Keyframe timing p95, all nearest', f"{metrics['association_p95_ms']:.2f} ms"),
        ('Camera-proposed, LiDAR-verified loops', str(metrics['accepted_camera_lidar_loops'])),
        ('ICP RMSE median / p95', f"{metrics['lidar_icp_rmse_median_m']*1000:.2f} / {metrics['lidar_icp_rmse_p95_m']*1000:.2f} mm"),
        ('Estimated travel / exported points', f"{metrics['estimated_path_length_m']:.2f} m / {metrics['pcd_points']:,}"),
        ('Endpoint gap before / after graph', f"{metrics['endpoint_separation_before_m']:.3f} / {metrics['endpoint_separation_after_m']:.3f} m"),
        ('Largest camera graph correction', f"{metrics['max_camera_graph_correction_m']:.3f} m"),
    ]
    if combined:
        table[6] = ('LiDAR / visual / combined points',
                    f"{metrics.get('lidar_point_count', 0):,} / {visual_count:,} / {metrics['pcd_points']:,}")
        reprojection = visual_stats.get('reprojection_px', {})
        if all(isinstance(reprojection.get(key), (int, float)) for key in ('median', 'p95')):
            table.append(('Visual reprojection median / p95', f"{reprojection['median']:.2f} / {reprojection['p95']:.2f} px"))
    if combined and camera_safe:
        camera_caption = (f"Camera candidate landmarks: {visual_count:,} XYZ points after {view_count}-view in-sample screening. "
                          'Not a loop closure, independent height reference or demonstrated localization improvement.')
    elif combined:
        camera_caption = 'No consistent camera-landmark illustration is available in this snapshot.'
    elif representative and camera_safe:
        event, excursion = representative
        camera_caption = (f"Camera revisit {event['time_i_s']:.1f} to {event['time_j_s']:.1f} s: "
                          f"{event['visual']['inliers']} ORB inliers; intervening excursion {excursion:.1f} m; "
                          f"LiDAR residual {event['lidar_rmse']*1000:.1f} mm. Only a nonstationary accepted pair is shown.")
    else:
        camera_caption = 'No consistent accepted nonstationary camera pair is available in this result snapshot; no revisit illustration is claimed.'
    return dict(metrics=metrics, audit=audit, warnings=warnings, scope=scope, inputs=inputs, method=method, coordinates=coords,
                dataset_name=dataset_name, title=f'{dataset_name} | Indoor Mapping', result_path=result, audit_path=audit_path,
                camera_start_delay_s=camera_start_delay,
                combined_map=combined, visual_count=visual_count, visual_stats=visual_stats, view_count=view_count,
                report_output=result.parent/'report', notes_output=Path('docs/report_notes_zh.md'), runtime_evidence=None,
                export=export, limitations=limitations, table=table, track_status=track_status, partial=partial,
                camera_safe=camera_safe, camera_figure=camera_figure, map_figure=map_figure,
                camera_caption=camera_caption, nonstationary_loop_count=len(nonstationary),
                map_caption=(f"Exported combined PCD: {metrics['pcd_points']:,} points; estimated path {metrics['estimated_path_length_m']:.2f} m. "
                             if combined else
                             f"Exact exported scan-plane PCD: {metrics['pcd_points']:,} XYZ points; orange line is the estimated trajectory. ")
                            + ('Coverage is limited to the tracked prefix.' if partial else 'No external reference map is used.'))


def attach_evaluation_evidence(content: dict, localization_path: Path | None,
                               camera_validation_path: Path | None) -> None:
    """Attach measured evaluation, preserving the distinction between stability and truth."""
    content.update(localization_evidence=None, camera_validation_evidence=None,
                   localization_text='', camera_validation_text='', comparison_text='', evaluation_sources=[])
    if localization_path:
        evaluation = load_json(localization_path)
        summary = evaluation['queries_summary']
        crosspass = evaluation['crosspass_queries_summary']
        gates = evaluation['quality_gates']
        spread = summary['query_max_pairwise_translation_m']
        yaw = summary['query_max_pairwise_yaw_deg']
        residual = summary['heldout_point_to_line_rmse_m']
        def value(number, places=3):
            return f'{number:.{places}f}' if isinstance(number, (float, int)) else 'N/A'
        content['localization_evidence'] = localization_path.as_posix()
        content['localization_snapshot'] = evaluation
        content['evaluation_sources'].append(localization_path)
        content['localization_text'] = (
            f"Development validation: {summary['query_count']} raw-scan queries are absent from all odometry/map insertion stamps. "
            'Each uses eight fixed initial perturbations (0.25/0.50 m and 5/10 degrees); '
            'references exclude scans within 5 s or 0.5 m and contain LiDAR returns only. '
            f"{summary['accepted_registrations']}/{summary['attempted_initializations']} registrations pass the fixed fit gates; "
            f"{summary['all_starts_agree_queries']}/{summary['query_count']} queries have all eight starts accepted and agree within "
            f"{gates['agreement_translation_m']:.2f} m / {gates['agreement_yaw_deg']:g} degrees. "
            f"The p95 maximum within-query pose spread is {value(spread.get('p95'))} m / {value(yaw.get('p95'),2)} degrees "
            f"among {spread.get('count',0)} queries with at least two accepted solutions; failures remain in the acceptance denominator. "
            f"Earlier-pass references (at least 30 s earlier) yield {crosspass['all_starts_agree_queries']}/{crosspass['query_count']} stable queries. "
            'Query/reference IDs and initial poses are frozen for comparisons. Estimated baseline poses define selection and starting priors, '
            'not ground truth. These are internal robustness measures, not absolute localization accuracy; '
            'the same queries informed method selection and are not an untouched final test set.')
        comparison = evaluation.get('comparison', {}).get('queries_summary')
        if comparison:
            base_accepted = summary['accepted_registrations'] - round(
                comparison['acceptance_rate_difference']*summary['attempted_initializations'])
            base_stable = summary['all_starts_agree_queries'] - round(
                comparison['all_starts_agree_fraction_difference']*summary['query_count'])
            base_spread = comparison['query_max_pairwise_translation_m']['baseline']
            content['comparison_text'] = (
                f"Frozen-protocol comparison: accepted starts {base_accepted} to {summary['accepted_registrations']} / "
                f"{summary['attempted_initializations']}; stable queries {base_stable} to {summary['all_starts_agree_queries']} / "
                f"{summary['query_count']}; position-spread p95 {value(base_spread.get('p95'))} to {value(spread.get('p95'))} m. "
                'Selection uses this same internal test; generalization and absolute accuracy remain unvalidated.')
            content['comparison_pdf'] = (
                f"Same internal test: stable queries {base_stable} to {summary['all_starts_agree_queries']}/{summary['query_count']}; "
                f"accepted starts {base_accepted} to {summary['accepted_registrations']}/{summary['attempted_initializations']}. "
                'This test informed selection; generalization remains untested.')
            content['localization_text'] += ' '+content['comparison_text']
        metrics = content['metrics']
        content['table'] = [
            ('Tracked duration / bag span', f"{metrics['tracked_duration_s']:.1f} / {metrics['bag_scan_duration_s']:.1f} s"),
            ('LiDAR / camera candidate points', f"{metrics.get('lidar_point_count', metrics['pcd_points']):,} / {content['visual_count']:,}"),
            ('Accepted LiDAR / camera revisits', f"{metrics.get('accepted_lidar_revisits',0)} / {metrics['accepted_camera_lidar_loops']}"),
            ('Held-out accepted starts', f"{summary['accepted_registrations']} / {summary['attempted_initializations']}"),
            ('All starts agree / queries', f"{summary['all_starts_agree_queries']} / {summary['query_count']}"),
            ('Within-query spread p95: m / deg', f"{value(spread.get('p95'))} / {value(yaw.get('p95'),2)}"),
            ('Accepted fit median / p95 (mm)', f"{value(residual.get('median',0)*1000,1)} / {value(residual.get('p95',0)*1000,1)}"),
            ('Earlier-pass stable / queries', f"{crosspass['all_starts_agree_queries']} / {crosspass['query_count']}"),
        ]
        content['evaluation_pdf'] = (
            f"Held-out raw scans use eight starts (0.25/0.50 m, 5/10 deg), with training scans excluded within 5 s or 0.5 m. "
            f"Stable means all eight fits pass and agree within {gates['agreement_translation_m']:.2f} m / {gates['agreement_yaw_deg']:g} deg. "
            f"Spread uses {spread.get('count',0)} queries with at least two accepted starts; failed starts count against acceptance. "
            'Fixed baseline seeds and reference IDs support internal comparison, not ground truth.')
        figure = localization_path.parent/'localization_quality.png'
        if figure.is_file():
            content['map_figure'] = figure
            content['map_caption'] = ('Development validation: green means all eight initializations agree; red means a rejected or inconsistent start. '
                                      'Queries are held out from odometry, but used for method selection. Positions are estimated.')
            content['evaluation_sources'].append(figure)
        else:
            content['warnings'].append(f'Missing localization evidence figure: {figure.as_posix()}.')
        trajectory = content['result_path']/'trajectory.csv'
        if trajectory.is_file() and evaluation.get('trajectory_sha256'):
            if hashlib.sha256(trajectory.read_bytes()).hexdigest() != evaluation['trajectory_sha256']:
                content['warnings'].append('Localization evaluation trajectory hash differs from the selected result trajectory.')
        protocol_path = Path(evaluation['protocol']) if evaluation.get('protocol') else None
        if protocol_path and protocol_path.is_file():
            content['evaluation_sources'].append(protocol_path)
            if evaluation.get('protocol_sha256') != hashlib.sha256(protocol_path.read_bytes()).hexdigest():
                content['warnings'].append('Localization frozen protocol hash differs from evaluation evidence.')
    if camera_validation_path:
        content['camera_validation_evidence'] = camera_validation_path.as_posix()
        diagnostic = load_json(camera_validation_path)
        content['camera_validation_snapshot'] = diagnostic
        content['evaluation_sources'].append(camera_validation_path)
        # The supplied diagnostic may refer to a prior method; never label it as final-map validation.
        content['camera_validation_text'] = (
            'The supplied camera diagnostic is retained separately from the selected map. '
            'Final camera landmark reprojection checks use construction views and do not establish independent pose or height accuracy.')
        subset = diagnostic.get('subsets', {}).get('all_points', {})
        if 'training_point_count' in subset and 'both_heldout_views_within_2px' in subset:
            content['camera_validation_text'] = (
                f"Earlier four-view diagnostic: only {subset['both_heldout_views_within_2px']}/{subset['training_point_count']} landmarks "
                'pass the 2 px prediction gate in both additional frames with frozen XYZ and trajectory. '
                f"The selected {content['view_count']}-view method uses its construction views for in-sample screening; "
                'those views are not a new held-out test. Camera landmarks do not correct poses or establish localization improvement.')


def markdown_report(content: dict, output: Path) -> str:
    strip = lambda value: value.replace('<b>', '**').replace('</b>', '**')
    lines = ['# '+content['title'], '', 'Group: [enter group ID]', '',
             'Source recording: `'+str(content['audit']['bag_path'])+'`.', '',
             '**DRAFT - Internal evaluation; physical-reference validation incomplete.**', '',
             content['track_status'], '', '## Scope', '', content['scope'], '',
             '## Inputs and time alignment', '', content['inputs'], '',
             '## Estimation pipeline', '', content['method'], '',
             '## Coordinate frame and calibration', '', content['coordinates'], '',
             '## Measured output: internal consistency only', '', '| Measure | Current result |', '|---|---:|']
    lines.extend(f'| {label} | {value} |' for label, value in content['table'])
    if content.get('localization_text'):
        lines.extend(['', '## Development validation with held-out scans', '', content['localization_text'], ''])
    if content.get('camera_validation_text'):
        lines.extend(['', '## Camera diagnostic and limits', '', content['camera_validation_text'], ''])
    lines.extend(['', '## Export and inspection', '', content['export'], '',
                  f"![{'Held-out localization stability' if content.get('localization_evidence') else 'Exact exported PCD'}]({Path(os.path.relpath(content['map_figure'],output)).as_posix()})", '', content['map_caption'], ''])
    if content['camera_safe']:
        label = 'Camera multiview landmark evidence' if content['combined_map'] else 'Accepted nonstationary camera revisit'
        lines.extend([f"![{label}]({Path(os.path.relpath(content['camera_figure'],output)).as_posix()})", ''])
    lines.extend([content['camera_caption'], '', '## Limits and missing experiment', '', strip(content['limitations']), '',
                  '## Reproducibility', '',
                  'Evidence: `'+content['audit_path'].as_posix()+'`; `'+content['result_path'].as_posix()+'/metrics.json`, '
                  '`loop_events.json`, `trajectory_arrays.npz`, and `map.pcd`.', '',
                  '```text', report_command(content, output), '```', '',
                  '## References', ''])
    if content.get('localization_evidence') or content.get('camera_validation_evidence'):
        lines.insert(lines.index('## References'), 'Evaluation evidence: '+ '; '.join(
            '`'+content[key]+'`' for key in ('localization_evidence', 'camera_validation_evidence') if content.get(key)) + '.\n')
    lines.extend(f'{i}. [{title}]({url})' for i, (title, url) in enumerate(REFERENCES, 1))
    if content['warnings']:
        lines.extend(['', '## Snapshot consistency warnings', ''] + ['- ' + warning for warning in content['warnings']])
    return '\n'.join(lines) + '\n'


def create_pdf(path: Path, content: dict) -> dict:
    page_w, page_h = A4
    c = canvas.Canvas(str(path), pagesize=A4, pageCompression=1)
    c.setTitle(content['title']+' - Group Technical Report Draft')
    c.setAuthor('Group: [enter group ID]')
    navy, teal, gray = colors.HexColor('#173345'), colors.HexColor('#117983'), colors.HexColor('#43515B')
    margin = 32
    width = page_w - 2*margin
    column_gap = 18
    column_width = (width-column_gap)/2
    right_x = margin+column_width+column_gap
    styles = {
        'body': ParagraphStyle('body', fontName='Helvetica', fontSize=8.25, leading=10.3, textColor=gray, spaceAfter=0),
        'caption': ParagraphStyle('caption', fontName='Helvetica', fontSize=6.9, leading=8.3, textColor=gray),
        'tiny': ParagraphStyle('tiny', fontName='Helvetica', fontSize=6.35, leading=7.6, textColor=gray),
        'banner': ParagraphStyle('banner', fontName='Helvetica-Bold', fontSize=8, leading=10, textColor=navy),
    }
    def para(text: str, x: float, top: float, w: float, style='body') -> float:
        p = Paragraph(text, styles[style])
        _, h = p.wrap(w, page_h)
        p.drawOn(c, x, top-h)
        return top-h
    def heading(text: str, x: float, top: float, w: float) -> float:
        c.setFillColor(teal)
        c.setFont('Helvetica-Bold', 9.1)
        c.drawString(x, top-9.1, text)
        return top-15
    def section(title: str, text: str, x: float, y: float, w: float) -> float:
        y = heading(title, x, y, w)
        return para(text, x, y, w)-9
    def figure(path: Path, x: float, top: float, w: float, height: float) -> None:
        c.setFillColor(colors.HexColor('#F5F8F9'))
        c.roundRect(x, top-height, w, height, 3, fill=1, stroke=0)
        if path.exists():
            image = ImageReader(str(path))
            iw, ih = image.getSize()
            scale = min((w-6)/iw, (height-6)/ih)
            rw, rh = iw*scale, ih*scale
            c.drawImage(image, x+(w-rw)/2, top-height+(height-rh)/2, width=rw, height=rh, mask='auto')
    c.setFillColor(navy)
    c.setFont('Helvetica-Bold', 19)
    c.drawString(margin, page_h-45, content['title'])
    c.setFont('Helvetica', 10)
    c.drawString(margin, page_h-61, 'Camera-assisted planar LiDAR | Group: [enter group ID]')
    y = page_h-74
    c.setFillColor(colors.HexColor('#EAF1F3'))
    c.roundRect(margin, y-32, width, 32, 4, fill=1, stroke=0)
    para('DRAFT - INTERNAL EVALUATION; PHYSICAL VALIDATION INCOMPLETE.<br/>' + html.escape(content['track_status']), margin+8, y-5, width-16, 'banner')
    y = para(html.escape(content['scope']), margin, y-41, width)-12
    columns_top = y
    inputs, method, coordinates = content['inputs'], content['method'], content['coordinates']
    if content.get('localization_evidence'):
        audit, metrics = content['audit'], content['metrics']
        raw = audit['streams']['/scan/raw']
        rgb = audit['streams']['/camera/camera/color/image_raw/compressed']
        timing = audit['synchronization']['camera_to_nearest_raw_scan']
        inputs = (f"{raw['count']:,} raw scans ({raw['median_frequency_hz']:.2f} Hz); {rgb['count']:,} RGB messages "
                  f"({rgb['median_frequency_hz']:.2f} Hz); {len(audit.get('camera',{}).get('decode_failure_image_ids',[]))} decode failures. "
                  'Integer header stamps [1] associate RGB to LiDAR within 50 ms. '
                  f"Nearest-scan timing median/p95: {timing['median']:.2f}/{timing['p95']:.2f} ms. "
                  + (f"RGB starts {content['camera_start_delay_s']:.3f} s later; initial poses use LiDAR alone. "
                     if content.get('camera_start_delay_s',0) and content['camera_start_delay_s'] > .05 else '') +
                  'Recorded intrinsics and static extrinsics are used; physical calibration is unverified.')
        method = ('Robust point-to-line ICP registers raw LiDAR to local submaps. Camera appearance proposes loops; '
                  f"{metrics['accepted_camera_lidar_loops']} pass visual/geometric checks. "
                  + ('No camera pose correction is applied. ' if not metrics.get('camera_constraints_applied') else '') +
                  (f"{metrics.get('accepted_lidar_revisits',0)} LiDAR revisit edge(s) correct the SE(2) graph [3]; "
                   f"maximum correction {metrics.get('max_graph_correction_m',0):.3f} m. "
                   'Engineering issue: valid-normal support did not measure overlap. Separate coverage and reciprocal/neighbor checks handle partial overlap. '
                   if metrics.get('lidar_revisit_constraints_applied') else '') +
                  (f"Camera features are triangulated from first/last views using metric LiDAR poses and checked in all {content['view_count']} "
                   'construction views [2]. These sparse candidate landmarks are experimental. ' if content['visual_count'] else '') +
                  content['evaluation_pdf'])
        coordinates = ('First accepted LiDAR pose defines local x forward, y left, metres; LiDAR z=0 is the scan plane. '
                       'Recorded static mounting converts laser to vehicle poses when available. '
                       'Visual heights assume planar motion. Full calibration and transforms remain in the audit.')
    left_y = section('Inputs and time alignment', html.escape(inputs), margin, y, column_width)
    left_y = section('Estimation and development validation' if content.get('localization_evidence') else 'Estimation pipeline', html.escape(method), margin, left_y, column_width)
    left_y = section('Coordinates and mounting', html.escape(coordinates), margin, left_y, column_width)
    right_y = heading('Internal development validation' if content.get('localization_evidence') else 'Measured output: internal consistency only', right_x, y, column_width)
    for i, (label, value) in enumerate(content['table']):
        if i % 2 == 0:
            c.setFillColor(colors.HexColor('#F0F5F6'))
            c.rect(right_x, right_y-24, column_width, 24, fill=1, stroke=0)
        para(html.escape(label), right_x+4, right_y-3, column_width*.63-6, 'caption')
        para('<b>'+html.escape(value)+'</b>', right_x+column_width*.63, right_y-3, column_width*.37-4, 'caption')
        right_y -= 24
    right_y -= 10
    right_y = section('Export and inspection', html.escape(content['export']), right_x, right_y, column_width)
    figure_top = min(left_y, right_y)-1
    figure_height = 132 if content.get('localization_evidence') else 138
    figure(content['map_figure'], margin, figure_top, column_width, figure_height)
    if content['camera_safe']:
        figure(content['camera_figure'], right_x, figure_top, column_width, figure_height)
    else:
        para('<b>Camera evidence unavailable for this snapshot.</b><br/>'+html.escape(content['camera_caption']), right_x+8, figure_top-15, column_width-16)
    caption_y = figure_top-figure_height-5
    map_bottom = para(html.escape(content['map_caption']), margin, caption_y, column_width, 'caption')
    camera_bottom = para(html.escape(content['camera_caption']), right_x, caption_y, column_width, 'caption')
    y = min(map_bottom, camera_bottom)-10
    limitations = content['limitations']
    if content.get('comparison_text'):
        limitations = html.escape(content.get('comparison_pdf', content['comparison_text']))+' '+limitations
    if content.get('camera_validation_text'):
        limitations = html.escape(content['camera_validation_text'])+' '+limitations
    y = section('Limits and missing experiment', limitations, margin, y, width)
    if content['warnings']:
        y = para('<b>Snapshot warning:</b> '+html.escape(' '.join(content['warnings'])), margin, y, width, 'caption')-5
    y = heading('References and reproducibility', margin, y, width)
    refs = ('[1] <link href="'+REFERENCES[0][1]+'" color="#117983">ROS 2 LaserScan</link> (github.com/ros2/common_interfaces); '
            '[2] <link href="'+REFERENCES[1][1]+'" color="#117983">OpenCV calib3d</link> (docs.opencv.org); '
            '[3] <link href="'+REFERENCES[2][1]+'" color="#117983">SciPy least_squares</link> (docs.scipy.org); '
            '[4] <link href="'+REFERENCES[3][1]+'" color="#117983">PCL PCD format</link> (pointclouds.org).<br/>'
            'Evidence: '+html.escape(content['audit_path'].as_posix())+'; '+html.escape(content['result_path'].as_posix())+
            '/metrics.json, loop_events.json, trajectory_arrays.npz, map.pcd. '
            'Evaluation JSON paths, reproduction command and editable text: group_technical_report.md; '
            'source hashes: report_manifest.json.')
    bottom = para(refs, margin, y, width, 'tiny')
    if bottom < 25:
        raise ValueError(f'Report content exceeds one-page layout: last baseline {bottom:.1f} pt; shorten text or figure area')
    c.setStrokeColor(colors.HexColor('#CAD8DD'))
    c.line(margin, 22, page_w-margin, 22)
    c.setFont('Helvetica', 6.6)
    c.setFillColor(gray)
    c.drawString(margin, 13, 'Draft for review. No independent physical reference supplied. Group identity and individual contributions remain unfilled.')
    c.drawRightString(page_w-margin, 13, '1 / 1')
    c.showPage()
    c.save()
    with fitz.open(path) as document:
        if len(document) != 1:
            raise ValueError(f'Report must contain exactly one page, got {len(document)}')
        page = document[0]
        text = page.get_text()
        if 'Physical-reference validation remains incomplete; no reference provided.' not in text.replace('\n', ' '):
            raise ValueError('Required physical-validation statement missing from PDF text')
        page.get_pixmap(matrix=fitz.Matrix(2, 2), alpha=False).save(path.parent/'report_preview.png')
    return {'page_count': 1, 'body_font_pt': styles['body'].fontSize, 'bottom_content_pt': bottom,
            'render_scale': 2, 'column_top_pt': columns_top}


def report_command(content: dict, output: Path | None = None) -> str:
    """A dataset-specific command; arguments are quoted for documented CLI use."""
    output = output or content['report_output']
    command = ('.venv/Scripts/python.exe scripts/build_report.py '
            f'--result "{content["result_path"].as_posix()}" --audit "{content["audit_path"].as_posix()}" '
            f'--output "{output.as_posix()}" --dataset-name "{content["dataset_name"]}" '
            f'--notes-output "{content["notes_output"].as_posix()}" --strict')
    if content.get('runtime_evidence'):
        command += f' --runtime-evidence "{content["runtime_evidence"]}"'
    for key in ('localization_evidence', 'camera_validation_evidence'):
        if content.get(key):
            command += f' --{key.replace("_", "-")} "{content[key]}"'
    return command


def chinese_notes(content: dict) -> str:
    metrics = content['metrics']
    audit = content['audit']
    scan = audit['streams'].get('/scan', {})
    decode_count = len(audit.get('camera', {}).get('decode_failure_image_ids', []))
    runtime = (f"本数据集的 ROS 验证证据路径：`{content['runtime_evidence']}`；以该日志明确检查的范围为准。"
               if content.get('runtime_evidence') else '本报告未收到本数据集的 ROS 2 运行验证证据；不沿用其他数据集的发布时长或测试结论。')
    initialization = (f"相机首帧比原始激光首帧晚 {content['camera_start_delay_s']:.3f} 秒；这段初始化时间没有相机观测，不能强行关联未来图像。"
                      if content.get('camera_start_delay_s') is not None and content['camera_start_delay_s'] > .05 else '')
    if content['combined_map']:
        camera_role = (f"相机采用 {content['view_count']} 帧 LK 跟踪、记录的 K/D 去畸变，以及 LiDAR 轨迹和相机外参提供的米制基线，对首末两帧进行三角化，"
                       f"再用全部 {content['view_count']} 帧做构建样本内筛选；没有进行所有视图的联合优化。"
                       f"深度、视差、基线、正反向光流和多帧重投影检查后保留 {content['visual_count']:,} 个稀疏视觉 XYZ 候选点。"
                       '这些点增加地图的视觉几何内容；它们共享 LiDAR 位姿和已记录外参，并不是独立定位或高度真值。')
        if content['visual_stats'].get('parameters', {}).get('require_upward_world_rays'):
            camera_role += '采用世界坐标向上视线筛选结构候选点，没有任意设置世界高度截断值。'
        geometry_text = ('主 map.pcd 合并 LiDAR 的 z=0 平面端点与相机三角化的稀疏三维地标。'
                         'lidar_scan_map.pcd 与 visual_landmarks.pcd 分别保留两种来源。视觉高度相对于初始 LiDAR 扫描平面，'
                         '依赖车辆平面运动及记录的相机外参；不是完整、稠密或独立验证的三维重建。'
                         '二维定位使用 lidar_scan_map.pcd 或 LiDAR 占据地图，禁止直接把视觉三维点投影到 XY 后混入定位地图。')
    else:
        camera_role = '相机仅提出并验证外观回访，米制回环约束来自 LiDAR。原地重复画面不应写成有效运动回环。'
        geometry_text = 'PCD 是米制 ASCII 0.7 XYZ 点云，z=0 为扫描平面投影；不是具有真实高度变化的三维重建。'
    localization_notes = ''
    if content.get('localization_snapshot'):
        evidence = content['localization_snapshot']
        main, earlier = evidence['queries_summary'], evidence['crosspass_queries_summary']
        spread = main['query_max_pairwise_translation_m']
        gates = evidence['quality_gates']
        localization_notes = (f"## 留出原始扫描定位的开发验证\n\n"
            f"{main['query_count']} 个查询扫描没有用于里程计和地图插入；训练参考排除查询前后 5 秒和 0.5 米以内的扫描，且只含二维 LiDAR 点。"
            '每个查询使用 8 组固定初值（平移 0.25/0.50 m，航向 5/10°），所有候选方法使用同一份冻结查询、参考扫描及初值。'
            f"通过固定配准门槛的次数为 {main['accepted_registrations']}/{main['attempted_initializations']}；"
            f"全部初值通过且两两平移差不超过 {gates['agreement_translation_m']:.2f} m、航向差不超过 {gates['agreement_yaw_deg']:g}° 的查询为 "
            f"{main['all_starts_agree_queries']}/{main['query_count']}。"
            f"至少两组初值被接受的 {spread['count']} 个查询中，最大两两位置分歧的 p95 为 {spread['p95']:.3f} m；这不是相对真实位置的误差。"
            f"仅用至少 30 秒之前的参考扫描时，稳定查询为 {earlier['all_starts_agree_queries']}/{earlier['query_count']}。"
            '失败初值没有从接受率分母剔除；拟合残差仅统计被接受配准，不能单独用来选优。'
            '查询位置和初值来自估计轨迹，协议也受基线轨迹影响；这些查询用于选择改进方案，属于开发验证，不是未参与选型的最终测试集，也不是独立定位精度实验。'
            f"证据：`{content['localization_evidence']}`。\n\n")
    if content.get('camera_validation_snapshot'):
        all_points = content['camera_validation_snapshot'].get('subsets',{}).get('all_points',{})
        if 'training_point_count' in all_points and 'both_heldout_views_within_2px' in all_points:
            localization_notes += (f"四视图旧方案的额外两帧预测诊断：{all_points['both_heldout_views_within_2px']}/{all_points['training_point_count']} 个点同时通过两帧 2 px 门槛。"
                f"当前 {content['view_count']} 视图方案已把构建视图用于筛选，因此不能将这些视图再次称作独立留出验证。"
                '相机点保留为实验性几何附件，没有证明提高定位精度。'
                f"诊断来源：`{content['camera_validation_evidence']}`。\n\n")
    return f'''# {content['dataset_name']} 报告使用说明与尚缺实验

英文小组技术报告由实际 JSON 指标与结果图片生成；PDF 严格为一页，Markdown 可编辑。小组编号保留为 `[enter group ID]`，未填写人员姓名，也未虚构个人贡献或宣称本组亲自完成原始采集。

数据来源：`{audit['bag_path']}`。本报告的指标目录为 `{content['result_path'].as_posix()}`，审计记录为 `{content['audit_path'].as_posix()}`。

{initialization}

当前记录范围：{metrics['tracked_duration_s']:.1f} / {metrics['bag_scan_duration_s']:.1f} 秒；已接受 {metrics['tracked_scans']:,} 帧用于轨迹估计，导出 {metrics['pcd_points']:,} 个 PCD 点。{'当前结果仅覆盖成功跟踪的前缀，后续轨迹不应拼接或宣称已经恢复。' if content['partial'] else '时长覆盖不能代替独立精度验证；应同时检查拒绝帧和时序间隔。'}

{localization_notes}
## 尚缺的实体参照验证

**Physical-reference validation remains incomplete; no reference provided.**

现有输入没有独立测得的墙长、测量控制点或真实车辆轨迹。ICP 残差、图优化残差、首尾距离及相机引起的修正都是内部一致性指标，不是绝对位置误差。不得据此声称厘米级精度或已经满足评分中的实体测量要求。

后续应选取可以在图中明确对应的实体墙段或标记点，独立测量长度/位置并记录量具、测量日期、不确定度和端点定义；比较地图坐标的欧氏距离，报告绝对与相对误差。若要验证定位，应记录独立参考轨迹或多个独立控制点；不能使用该 bag 内由同一 SLAM 系统生成的 /pose、/odom 或 /map 充当真值。提交时附测量表和原始证据，并把结果回填英文报告。

## 当前报告的边界

- 输入来自所提供的实体小车 bag；估计器仅使用原始二维 LiDAR 与单目 RGB，CameraInfo 和静态 TF 用于标定。没有使用 IMU、GNSS、磁力计、轮速/录制里程计或动态 TF。
- `/scan` 重复时间戳为 {scan.get('duplicate_header_timestamps', 0):,} 次、同时间戳量程冲突为 {scan.get('duplicate_timestamp_range_conflicts', 0):,} 次（该话题存在情况以审计清单为准）；实际使用 `/scan/raw`。图像解码失败为 {decode_count} 帧，记录在审计中。
- {camera_role}
- 安装变换和内参来自记录，未独立完成物理标定验证。扫描过程中的运动畸变尚未补偿。
- {geometry_text}
- 当前运行结果、图和图像对必须来自同一轮计算；报告生成器 `--strict` 会拒绝检测到的快照不一致。预览文件为 `{content['report_output'].as_posix()}/report_preview.png`。

相机提出并通过 LiDAR 验证的回访约束为 {metrics['accepted_camera_lidar_loops']} 条；最大图优化平面修正为 {metrics['max_camera_graph_correction_m']:.3f} m。零回访意味着相机未通过回环修正位姿；视觉地标建图是另一个独立记录的贡献。约束数量、点数和修正幅度都不证明相对真实位置误差一定降低。

另外，经几何验证的 LiDAR 回访约束为 {metrics.get('accepted_lidar_revisits',0)} 条，是否实际应用：{metrics.get('lidar_revisit_constraints_applied',False)}；全部图约束造成的最大平面修正为 {metrics.get('max_graph_correction_m',metrics['max_camera_graph_correction_m']):.3f} m。这项修正不能归因于相机，也不是相对真值误差降低的幅度。

{runtime}

重新生成本数据集报告：

```text
{report_command(content)}
```

## 个人总结模板（与小组报告分开填写）

姓名 / 学号：[由本人填写]

实际承担的工作：[只写本人完成的任务；列出相应代码、实验或记录]

关键决定及依据：[说明一个真实技术选择、替代方案和证据]

遇到的问题与改进：[记录复现步骤、诊断和最终处理，未解决的问题也应说明]

验证与局限：[本人实际运行过的检查；独立测量仍缺失时明确注明]

协作与引用：[说明合作范围、外部库/文档及辅助工具的使用，遵循课程规定]

这是一份供核对和继续补实验的草稿，不能据此声称已经完成所有评分要求。
'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, default=Path('outputs/result'))
    parser.add_argument('--audit', type=Path, default=Path('outputs/audit/audit.json'))
    parser.add_argument('--output', type=Path, default=Path('outputs/report'))
    parser.add_argument('--dataset-name', help='Dataset identity displayed in the report; defaults to the bag directory name')
    parser.add_argument('--notes-output', type=Path, default=Path('docs/report_notes_zh.md'))
    parser.add_argument('--runtime-evidence', type=Path, help='Optional dataset-specific ROS runtime log; never inferred from another dataset')
    parser.add_argument('--localization-evidence', type=Path, help='Held-out raw-scan evaluation JSON for the selected trajectory')
    parser.add_argument('--camera-validation-evidence', type=Path, help='Separate camera predictive diagnostic JSON, possibly for the prior method')
    parser.add_argument('--strict', action='store_true', help='Reject inconsistent result snapshots; run after mapping completes')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    content = build_content(args.result, args.audit, args.dataset_name)
    content['report_output'] = args.output
    content['notes_output'] = args.notes_output
    content['runtime_evidence'] = args.runtime_evidence.as_posix() if args.runtime_evidence else None
    if args.runtime_evidence and not args.runtime_evidence.is_file():
        raise FileNotFoundError(args.runtime_evidence)
    attach_evaluation_evidence(content, args.localization_evidence, args.camera_validation_evidence)
    if args.strict and content['warnings']:
        raise ValueError('Inconsistent snapshot: ' + '; '.join(content['warnings']))
    pdf = args.output/'group_technical_report.pdf'
    layout = create_pdf(pdf, content)
    (args.output/'group_technical_report.md').write_text(markdown_report(content,args.output), encoding='utf-8')
    args.notes_output.parent.mkdir(parents=True, exist_ok=True)
    args.notes_output.write_text(chinese_notes(content), encoding='utf-8')
    sources = [args.audit, args.result/'metrics.json', args.result/'loop_events.json', args.result/'trajectory_arrays.npz', args.result/'pcd_preview.png']
    if content['camera_safe']:
        sources.append(content['camera_figure'])
    for name in ('map.pcd', 'map_metadata.json', 'run_config.json', 'trajectory.csv', 'lidar_scan_map.pcd', 'visual_landmarks.pcd',
                 'visual_landmark_stats.json', 'visual_landmark_evidence.csv'):
        if (args.result/name).exists():
            sources.append(args.result/name)
    if args.runtime_evidence:
        sources.append(args.runtime_evidence)
    sources.extend(content['evaluation_sources'])
    for name in ('map_selection.json', 'visual_landmark_review_final.json', 'ros_processing_final_test.txt',
                 'ros_final_artifacts_test.txt'):
        evidence = args.result.parent/'verification'/name
        if evidence.is_file():
            sources.append(evidence)
    sources.append(Path(os.path.relpath(Path(__file__), Path.cwd())))
    provenance = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources if path.exists()}
    manifest = {'pdf': str(pdf), 'dataset_name': content['dataset_name'], 'notes': str(args.notes_output),
                'layout': layout, 'source_sha256': provenance, 'runtime_evidence': content['runtime_evidence'],
                'localization_evidence': content['localization_evidence'],
                'camera_validation_evidence': content['camera_validation_evidence'],
                'snapshot_warnings': content['warnings'], 'camera_evidence_included': content['camera_safe'],
                'camera_evidence_kind': 'multiview_landmarks' if content['combined_map'] else 'revisit',
                'accepted_nonstationary_loop_count': content['nonstationary_loop_count'],
                'tracked_prefix_only': content['partial'], 'metrics_snapshot': content['metrics'],
                'physical_reference_validation': 'incomplete; no reference provided'}
    (args.output/'report_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in manifest.items() if k not in ('source_sha256', 'metrics_snapshot')}, indent=2))


if __name__ == '__main__':
    main()
