#!/usr/bin/env python3
"""Inspect a Project 2.2 vehicle recording without using it for estimation.

Reads the sensor, calibration and transform streams of a rosbag2 recording and
writes a JSON summary plus a few sample images and arrays for figures. Odometry
and the recorded occupancy grid are summarized for context only.

Usage: python3 inspect_bag.py <bag_dir> <out_dir>
"""
import collections
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAMERA = '/camera/camera/color'
TOPICS = [
    '/scan', '/scan/raw', f'{CAMERA}/camera_info', f'{CAMERA}/image_raw',
    f'{CAMERA}/image_raw/compressed', '/tf', '/tf_static', '/odom', '/map',
    '/imu/data', '/rosout',
]
SAMPLE_FRACTIONS = [0.02, 0.2, 0.4, 0.6, 0.8, 0.98]


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def quat_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def interval_stats(times):
    if len(times) < 2:
        return None
    d = np.diff(np.asarray(times))
    return {
        'rate_hz': float(1.0 / np.median(d)),
        'median_dt_s': float(np.median(d)),
        'max_dt_s': float(d.max()),
        'gaps_over_3x_median': int((d > 3 * np.median(d)).sum()),
        'backward_steps': int((d < 0).sum()),
        'duplicates': int((d == 0).sum()),
    }


def main():
    bag, out = sys.argv[1], sys.argv[2]
    os.makedirs(os.path.join(out, 'samples'), exist_ok=True)

    info = rosbag2_py.Info().read_metadata(bag, 'sqlite3')
    t0 = info.starting_time.nanoseconds * 1e-9
    duration = info.duration.nanoseconds * 1e-9

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in TOPICS if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    msg_cls = {t: get_message(types[t]) for t in present}

    bag_times = collections.defaultdict(list)
    hdr_times = collections.defaultdict(list)
    frames = collections.defaultdict(collections.Counter)
    tf_static, tf_pairs = {}, collections.Counter()
    tf_first = {}
    scan_meta, scan_valid = {}, collections.defaultdict(list)
    scan_samples = {}
    cam_info, cam_info_changes = None, 0
    image_meta, image_samples = None, 0
    comp_formats = collections.Counter()
    sample_targets = [t0 + f * duration for f in SAMPLE_FRACTIONS]
    next_sample = 0
    odom_xy, odom_meta = [], None
    map_meta = None
    rosout_nodes = collections.Counter()
    rosout_hits = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        tb = t_ns * 1e-9
        bag_times[topic].append(tb)
        if topic == f'{CAMERA}/image_raw' and image_meta is not None:
            continue  # raw image payloads are large; header stats come from compressed
        msg = deserialize_message(data, msg_cls[topic])
        if hasattr(msg, 'header'):
            hdr_times[topic].append(stamp_s(msg.header))
            frames[topic][msg.header.frame_id] += 1

        if topic == '/tf_static':
            for tr in msg.transforms:
                tt = tr.transform.translation
                tf_static[f'{tr.header.frame_id} -> {tr.child_frame_id}'] = {
                    'translation_m': [tt.x, tt.y, tt.z],
                    'quaternion_xyzw': [tr.transform.rotation.x, tr.transform.rotation.y,
                                        tr.transform.rotation.z, tr.transform.rotation.w],
                    'rpy_deg': quat_to_rpy(tr.transform.rotation),
                }
        elif topic == '/tf':
            for tr in msg.transforms:
                key = f'{tr.header.frame_id} -> {tr.child_frame_id}'
                tf_pairs[key] += 1
                if key not in tf_first:
                    tt = tr.transform.translation
                    tf_first[key] = {'translation_m': [tt.x, tt.y, tt.z],
                                     'rpy_deg': quat_to_rpy(tr.transform.rotation)}
        elif topic in ('/scan', '/scan/raw'):
            r = np.asarray(msg.ranges, dtype=np.float64)
            ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
            scan_valid[topic].append(float(ok.mean()))
            if topic not in scan_meta:
                scan_meta[topic] = {
                    'frame_id': msg.header.frame_id,
                    'beams': int(r.size),
                    'angle_min_deg': math.degrees(msg.angle_min),
                    'angle_max_deg': math.degrees(msg.angle_max),
                    'angle_increment_deg': math.degrees(msg.angle_increment),
                    'range_min_m': msg.range_min, 'range_max_m': msg.range_max,
                    'time_increment_s': msg.time_increment, 'scan_time_s': msg.scan_time,
                    'has_intensities': len(msg.intensities) > 0,
                }
            if topic == '/scan':
                k = len(scan_valid[topic]) - 1
                if k in (100, 5000, 10000, 15000):
                    scan_samples[str(k)] = {'ranges': r.tolist(), 'angle_min': msg.angle_min,
                                            'angle_increment': msg.angle_increment}
        elif topic == f'{CAMERA}/camera_info':
            cur = {'width': msg.width, 'height': msg.height,
                   'distortion_model': msg.distortion_model, 'D': list(msg.d),
                   'K': list(msg.k), 'P': list(msg.p), 'R': list(msg.r),
                   'frame_id': msg.header.frame_id}
            if cam_info is None:
                cam_info = cur
            elif cur != cam_info:
                cam_info_changes += 1
        elif topic == f'{CAMERA}/image_raw':
            image_meta = {'encoding': msg.encoding, 'width': msg.width, 'height': msg.height,
                          'step': msg.step, 'frame_id': msg.header.frame_id}
        elif topic == f'{CAMERA}/image_raw/compressed':
            comp_formats[msg.format] += 1
            if next_sample < len(sample_targets) and tb >= sample_targets[next_sample]:
                ext = 'png' if 'png' in msg.format else 'jpg'
                name = f'image_{next_sample}_{tb - t0:07.2f}s.{ext}'
                with open(os.path.join(out, 'samples', name), 'wb') as f:
                    f.write(bytes(msg.data))
                next_sample += 1
        elif topic == '/odom':
            p = msg.pose.pose.position
            odom_xy.append((stamp_s(msg.header), p.x, p.y, quat_to_rpy(msg.pose.pose.orientation)[2]))
            if odom_meta is None:
                odom_meta = {'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
                             'pose_cov_diag': [msg.pose.covariance[i] for i in (0, 7, 35)]}
        elif topic == '/map':
            g = msg.info
            map_meta = {'frame_id': msg.header.frame_id, 'resolution_m': g.resolution,
                        'width': g.width, 'height': g.height,
                        'origin_xy_m': [g.origin.position.x, g.origin.position.y],
                        'origin_yaw_deg': quat_to_rpy(g.origin.orientation)[2],
                        'stamp_s': stamp_s(msg.header), 'bag_time_s': tb}
            grid = np.asarray(msg.data, dtype=np.int16).reshape(g.height, g.width)
            np.save(os.path.join(out, 'recorded_map.npy'), grid)
        elif topic == '/rosout':
            rosout_nodes[msg.name] += 1
            text = msg.msg.lower()
            if any(k in text for k in ('lidar', 'laser', 'camera', 'realsense', 'calib',
                                       'urg', 'hokuyo', 'rplidar', 'sick', 'vesc', 'odom')):
                if len(rosout_hits) < 40:
                    rosout_hits.append(f'{tb - t0:7.2f}s [{msg.name}] {msg.msg[:160]}')

    summary = {
        'bag': os.path.abspath(bag),
        'start_time_unix_s': t0, 'duration_s': duration,
        'message_count_all_topics': info.message_count,
        'topics': {},
        'tf_static': tf_static,
        'tf_dynamic_pairs': dict(tf_pairs), 'tf_dynamic_first': tf_first,
        'scan': scan_meta,
        'camera_info': cam_info, 'camera_info_changes': cam_info_changes,
        'image_raw': image_meta, 'compressed_formats': dict(comp_formats),
        'odom': odom_meta, 'recorded_map': map_meta,
        'rosout_nodes': dict(rosout_nodes.most_common(30)), 'rosout_sensor_lines': rosout_hits,
    }
    for t in present:
        entry = {'type': types[t], 'count': len(bag_times[t]),
                 'bag_time_intervals': interval_stats(bag_times[t])}
        if hdr_times[t]:
            h = np.asarray(hdr_times[t])
            b = np.asarray(bag_times[t][:len(h)]) if t != f'{CAMERA}/image_raw' else None
            entry['frame_ids'] = dict(frames[t])
            entry['header_intervals'] = interval_stats(h)
            entry['header_first_rel_s'] = float(h[0] - t0)
            entry['header_last_rel_s'] = float(h[-1] - t0)
            if b is not None and len(b) == len(h):
                lag = b - h
                entry['receive_minus_header_s'] = {
                    'median': float(np.median(lag)), 'p95': float(np.percentile(lag, 95)),
                    'min': float(lag.min()), 'max': float(lag.max())}
        if t in scan_valid:
            entry['valid_beam_fraction_mean'] = float(np.mean(scan_valid[t]))
        summary['topics'][t] = entry

    if odom_xy:
        a = np.asarray([(s, x, y, yaw) for s, x, y, yaw in odom_xy])
        np.save(os.path.join(out, 'odom_xy_context_only.npy'), a)
        step = np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2]))
        summary['odom_context_only'] = {
            'path_length_m': float(step.sum()),
            'x_range_m': [float(a[:, 1].min()), float(a[:, 1].max())],
            'y_range_m': [float(a[:, 2].min()), float(a[:, 2].max())],
            'start_end_gap_m': float(np.hypot(a[-1, 1] - a[0, 1], a[-1, 2] - a[0, 2])),
            'max_speed_mps': float((step / np.maximum(np.diff(a[:, 0]), 1e-6)).max()),
        }
    with open(os.path.join(out, 'scan_samples.json'), 'w') as f:
        json.dump(scan_samples, f)
    with open(os.path.join(out, 'bag_inspection.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps({k: summary[k] for k in ('duration_s', 'tf_static', 'scan', 'camera_info',
                                               'image_raw', 'odom', 'recorded_map')}, indent=1))


if __name__ == '__main__':
    main()
