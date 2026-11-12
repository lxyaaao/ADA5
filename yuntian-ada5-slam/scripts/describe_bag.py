#!/usr/bin/env python3
"""Describe what a Project 2.2 recording contains, over time.

For one rosbag2 recording this writes into <out_dir>:
  inventory.json     every topic: type, count, rate, bytes, group
  series.npz         per-message series (columns listed in states.json):
                     odometry and drive commands (context only), LiDAR and
                     camera quality, battery, RC receiver
  scans_every10.npz  every 10th /scan/raw, for figures
  scan_filter.json   which /scan/raw returns /scan removes
  states.json        mode and state strings, diagnostics, /rosout summary
  frames/            one camera frame every 30 s, plus the last frame

Odometry and drive commands only describe the drive; nothing here estimates
poses.

Usage: python3 -I describe_bag.py <bag_dir> <out_dir>
"""
import os
import sys

# -I drops PYTHONPATH; put back only the ROS installation's entries.
sys.path[:0] = [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep)
                if p.startswith('/opt/ros/')]

import collections
import glob
import json
import math
import re
import sqlite3
import struct

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAM = '/camera/camera/color/image_raw/compressed'
STRINGS = ['/control_center/main_state', '/control_center/selected_cmd',
           '/control_center/debug', '/behavior/state', '/foxglove/waypoints/status']
DECODED = ['/odom', '/scan/raw', '/scan', CAM, '/rosout', '/battery_state', '/rc_data',
           '/behavior/override_active', '/diagnostics', '/tf_static'] + STRINGS
ACKERMANN = ['/ackermann_cmd', '/teleop']  # ackermann_msgs is not installed; parsed by hand
FRAME_EVERY_S = 30.0
SCAN_KEEP_EVERY = 10
RANGE_BINS = [0, 0.1, 0.2, 0.3, 0.5, 1, 2, 5, 10, 20, 50, 100.01]
LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
DIAG_LEVELS = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
COLUMNS = {
    'odom': 't_s (header), x_m, y_m, yaw_rad, v_mps, yaw_rate_radps; context only',
    'scan_raw': 't_s (header), valid_frac, median_range_m, frac_valid_under_1m, '
                'nonfinite_frac, zero_frac',
    'scan': 't_s (header), valid_frac',
    'cam': 't_s (header), mean_gray, frac_ge250, frac_le15, laplacian_var, fast_corners',
    'ackermann_cmd': 't_s (bag), steering_rad, speed_mps',
    'teleop': 't_s (bag), steering_rad, speed_mps',
    'battery': 't_s (bag), voltage_v, percentage, current_a',
    'rc': 't_s (bag), channel values',
    'override': 't_s (bag), active',
}


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def group(name):
    if name.startswith('/scan'):
        return 'LiDAR'
    if name.startswith('/camera'):
        return 'camera'
    if name in ('/imu/data', '/magnetometer_data'):
        return 'inertial (not allowed in 2.2)'
    if name in ('/odom', '/tf', '/tf_static', '/battery_state'):
        return 'vehicle state'
    if name.startswith(('/control_center', '/ackermann_cmd', '/teleop', '/drive', '/rc_data',
                        '/cmd_vel', '/enable_autonomous_control')):
        return 'driving commands and mode'
    if name.startswith('/behavior/'):
        return 'behavior module'
    if name in ('/rosout', '/diagnostics', '/parameter_events', '/events/write_split'):
        return 'logging and diagnostics'
    return 'navigation stack and UI'


def ackermann(data):
    """(steering_angle, speed) from CDR little-endian AckermannDriveStamped bytes."""
    _, _, n = struct.unpack_from('<iII', data, 4)       # stamp sec, nanosec, frame_id length
    body = (12 + n + 3) & ~3                           # floats are 4-byte aligned in the body
    steer, _, speed, _, _ = struct.unpack_from('<5f', data, 4 + body)
    return steer, speed


def norm(text):
    """Collapse numbers so repeated log lines with changing values group together."""
    return re.sub(r'\d+(?:\.\d+)?', '#', text)


def runs(mask):
    out, i = [], 0
    while i < mask.size:
        if mask[i]:
            j = i
            while j + 1 < mask.size and mask[j + 1]:
                j += 1
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def as_array(rows):
    if not rows:
        return np.zeros((0, 0))
    a = np.full((len(rows), max(len(r) for r in rows)), np.nan)
    for i, r in enumerate(rows):
        a[i, :len(r)] = r
    return a


def inventory(bag):
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    duration = meta['duration']['nanoseconds'] * 1e-9
    sizes = collections.Counter()
    for path in sorted(glob.glob(os.path.join(bag, '*.db3'))):
        con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        q = ('SELECT t.name, SUM(LENGTH(m.data)) FROM messages m '
             'JOIN topics t ON t.id = m.topic_id GROUP BY t.name')
        for name, nbytes in con.execute(q):
            sizes[name] += nbytes or 0
        con.close()
    topics = []
    for t in meta['topics_with_message_count']:
        md, n = t['topic_metadata'], t['message_count']
        topics.append({'name': md['name'], 'type': md['type'], 'count': n,
                       'rate_hz': round(n / duration, 3),
                       'megabytes': round(sizes[md['name']] / 1e6, 3),
                       'group': group(md['name'])})
    topics.sort(key=lambda r: (r['group'], r['name']))
    return meta, duration, topics


def main():
    bag, out = sys.argv[1], sys.argv[2]
    os.makedirs(os.path.join(out, 'frames'), exist_ok=True)
    meta, duration, topics = inventory(bag)
    t0 = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=[t for t in DECODED + ACKERMANN if t in types]))
    cls = {t: get_message(types[t]) for t in DECODED if t in types}

    series = collections.defaultdict(list)
    kept_scans, kept_t, scan_geom = [], [], None
    raw_by_stamp, n_raw = {}, 0
    filt = {'pairs': 0, 'changed': 0, 'removed': None, 'raw_valid': None,
            'hist_removed': np.zeros(len(RANGE_BINS) - 1, np.int64),
            'hist_kept': np.zeros(len(RANGE_BINS) - 1, np.int64)}
    transitions = {t: [] for t in STRINGS}
    values = {t: collections.Counter() for t in STRINGS}
    last_value = {}
    logs, log_example = collections.Counter(), {}
    diag, diag_example = collections.Counter(), {}
    tf_static = {}
    next_frame, last_jpeg = 0.0, None
    fast = cv2.FastFeatureDetector_create(threshold=20)

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        tb = t_ns * 1e-9 - t0
        if topic in ACKERMANN:
            series[topic.strip('/')].append((tb, *ackermann(data)))
            continue
        msg = deserialize_message(data, cls[topic])

        if topic == '/odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            tw = msg.twist.twist
            series['odom'].append((stamp_s(msg.header) - t0, p.x, p.y, yaw,
                                   tw.linear.x, tw.angular.z))

        elif topic in ('/scan/raw', '/scan'):
            r = np.asarray(msg.ranges, dtype=np.float32)
            fin = np.isfinite(r)
            ok = fin & (r > max(msg.range_min, 0.02)) & (r <= msg.range_max)
            th = stamp_s(msg.header) - t0
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if topic == '/scan/raw':
                if scan_geom is None:
                    scan_geom = {'angle_min': msg.angle_min,
                                 'angle_increment': msg.angle_increment, 'beams': int(r.size)}
                v = r[ok]
                series['scan_raw'].append((
                    th, ok.mean(), np.median(v) if v.size else np.nan,
                    (v < 1.0).mean() if v.size else np.nan,
                    (~fin).mean(), (fin & (r <= 0.02)).mean()))
                if n_raw % SCAN_KEEP_EVERY == 0:
                    kept_scans.append(np.where(ok, r, np.nan).astype(np.float16))
                    kept_t.append(th)
                n_raw += 1
                raw_by_stamp[key] = (r, ok)
                if len(raw_by_stamp) > 50:
                    raw_by_stamp.pop(next(iter(raw_by_stamp)))
            else:
                series['scan'].append((th, ok.mean()))
                pair = raw_by_stamp.pop(key, None)
                if pair is not None and pair[0].size == r.size:
                    rr, rok = pair
                    if filt['removed'] is None:
                        filt['removed'] = np.zeros(r.size, np.int64)
                        filt['raw_valid'] = np.zeros(r.size, np.int64)
                    filt['pairs'] += 1
                    removed = rok & ~ok
                    filt['removed'] += removed
                    filt['raw_valid'] += rok
                    filt['hist_removed'] += np.histogram(rr[removed], RANGE_BINS)[0]
                    filt['hist_kept'] += np.histogram(rr[rok & ok], RANGE_BINS)[0]
                    both = rok & ok
                    filt['changed'] += int((np.abs(rr[both] - r[both]) > 1e-3).sum())

        elif topic == CAM:
            th = stamp_s(msg.header) - t0
            g = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            if g is None:
                series['cam'].append((th, np.nan, np.nan, np.nan, np.nan, np.nan))
                continue
            series['cam'].append((th, g.mean(), (g >= 250).mean(), (g <= 15).mean(),
                                  float(cv2.Laplacian(g, cv2.CV_32F).var()),
                                  len(fast.detect(g, None))))
            last_jpeg = bytes(msg.data)
            if tb >= next_frame:
                with open(os.path.join(out, 'frames', f'frame_{tb:06.1f}s.jpg'), 'wb') as f:
                    f.write(last_jpeg)
                next_frame += FRAME_EVERY_S

        elif topic == '/rosout':
            k = (msg.name, LOG_LEVELS.get(msg.level, str(msg.level)), norm(msg.msg[:200]))
            logs[k] += 1
            log_example.setdefault(k, (round(tb, 2), msg.msg[:200]))

        elif topic == '/battery_state':
            series['battery'].append((tb, msg.voltage, msg.percentage, msg.current))

        elif topic == '/rc_data':
            series['rc'].append((tb, *msg.data))

        elif topic == '/behavior/override_active':
            series['override'].append((tb, float(msg.data)))

        elif topic in STRINGS:
            v = msg.data[:200]
            values[topic][norm(v)] += 1
            if last_value.get(topic) != v:
                transitions[topic].append((round(tb, 2), v))
                last_value[topic] = v

        elif topic == '/diagnostics':
            for s in msg.status:
                lv = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
                k = (s.name, DIAG_LEVELS.get(lv, str(lv)), norm(s.message[:150]))
                diag[k] += 1
                diag_example.setdefault(k, (round(tb, 1), s.message[:150],
                                            {kv.key: kv.value[:60] for kv in s.values[:15]}))

        elif topic == '/tf_static':
            for tr in msg.transforms:
                tt, rq = tr.transform.translation, tr.transform.rotation
                tf_static[f'{tr.header.frame_id} -> {tr.child_frame_id}'] = [
                    tt.x, tt.y, tt.z, rq.x, rq.y, rq.z, rq.w]

    if last_jpeg is not None:
        with open(os.path.join(out, 'frames', 'frame_last.jpg'), 'wb') as f:
            f.write(last_jpeg)

    np.savez_compressed(os.path.join(out, 'series.npz'),
                        **{k: as_array(v) for k, v in series.items()})
    np.savez_compressed(os.path.join(out, 'scans_every10.npz'), ranges=np.stack(kept_scans),
                        t=np.asarray(kept_t), angle_min=scan_geom['angle_min'],
                        angle_increment=scan_geom['angle_increment'])

    scan_filter = {'pairs': filt['pairs']}
    if filt['pairs']:
        rem, val = filt['removed'], filt['raw_valid']
        rate = rem / np.maximum(val, 1)
        ang = np.degrees(scan_geom['angle_min'] + scan_geom['angle_increment'] * np.arange(rate.size))
        often = rate > 0.5
        scan_filter.update({
            'removed_share_of_raw_valid': float(rem.sum() / max(val.sum(), 1)),
            'sectors_removed_over_half_the_time_deg': [
                [round(float(ang[i]), 2), round(float(ang[j]), 2), round(float(rate[i:j + 1].mean()), 3)]
                for i, j in runs(often)],
            'share_of_removals_in_those_sectors': float(rem[often].sum() / max(rem.sum(), 1)),
            'values_changed_when_both_valid': filt['changed'],
            'range_bins_m': RANGE_BINS,
            'removed_by_range': filt['hist_removed'].tolist(),
            'kept_by_range': filt['hist_kept'].tolist(),
            'removal_rate_by_beam': np.round(rate, 4).tolist(),
            'beam_angles_deg': np.round(ang, 3).tolist(),
        })

    inv = {'bag': os.path.abspath(bag), 'start_unix_s': t0, 'duration_s': duration,
           'messages': meta['message_count'], 'storage': meta.get('storage_identifier'),
           'ros_distro': meta.get('ros_distro'), 'files': meta.get('relative_file_paths'),
           'topics': topics}
    states = {
        'series_columns': COLUMNS,
        'scan_geometry': scan_geom,
        'tf_static': tf_static,
        'strings': {t: {'n_transitions': len(transitions[t]),
                        'transitions': transitions[t][:300],
                        'values': values[t].most_common(20)} for t in STRINGS},
        'diagnostics': [{'name': n, 'level': lv, 'count': c, 'first_s': diag_example[(n, lv, m)][0],
                         'message': diag_example[(n, lv, m)][1],
                         'values': diag_example[(n, lv, m)][2]}
                        for (n, lv, m), c in diag.most_common()],
        'rosout': [{'node': n, 'level': lv, 'count': c, 'first_s': log_example[(n, lv, m)][0],
                    'example': log_example[(n, lv, m)][1]}
                   for (n, lv, m), c in logs.most_common()],
    }
    for name, obj in (('inventory.json', inv), ('states.json', states),
                      ('scan_filter.json', scan_filter)):
        with open(os.path.join(out, name), 'w') as f:
            json.dump(obj, f, indent=1)

    print(json.dumps({
        'duration_s': round(duration, 2), 'topics': len(topics),
        'topics_with_data': sum(1 for t in topics if t['count']),
        'ros_distro': meta.get('ros_distro'),
        'series_rows': {k: len(v) for k, v in series.items()},
        'scan_filter_pairs': filt['pairs'],
        'frames': sorted(os.listdir(os.path.join(out, 'frames'))),
    }, indent=1))


if __name__ == '__main__':
    main()
