"""Read-only ROS 2 SQLite bag access and reproducible sensor audit/extraction.

Only /scan/raw, compressed RGB, CameraInfo and /tf_static enter the exported
sensor cache. Recorded maps are exported separately as unverified baselines.
No wheel odometry, IMU, recorded poses or dynamic TF is decoded or exported.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import sqlite3
from collections import Counter
from pathlib import Path
from typing import Any, Iterator

import numpy as np
import yaml
from rosbags.typesys import Stores, get_typestore

RAW_SCAN = '/scan/raw'
SCAN = '/scan'
CAMERA = '/camera/camera/color/image_raw/compressed'
CAMERA_INFO = '/camera/camera/color/camera_info'


def stamp_ns(stamp: Any) -> int:
    """Integer arithmetic preserves nanoseconds at Unix-epoch magnitudes."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ReadOnlyBag:
    def __init__(self, path: str | Path):
        self.path = Path(path).resolve()
        self.connection = sqlite3.connect(self.path.as_uri() + '?mode=ro', uri=True)
        self.connection.execute('PRAGMA query_only = ON')
        self.types = get_typestore(Stores.ROS2_HUMBLE)
        self.topics = {
            name: {'id': int(tid), 'type': typ}
            for tid, name, typ in self.connection.execute('SELECT id, name, type FROM topics')
        }

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.connection.close()

    def inventory(self) -> list[dict]:
        # Do not aggregate BLOB length: SQLite can materialize large values during
        # GROUP BY and spill gigabytes to a temporary file. Only select scalars.
        stats = {
            tid: (count, first, last)
            for tid, count, first, last in self.connection.execute(
                'SELECT topic_id, COUNT(*), MIN(timestamp), MAX(timestamp) '
                'FROM messages GROUP BY topic_id'
            )
        }
        return [dict(name=name, type=topic['type'], count=stats.get(topic['id'], (0,))[0],
                     first_bag_ns=stats.get(topic['id'], (0, None, None, 0))[1],
                     last_bag_ns=stats.get(topic['id'], (0, None, None, 0))[2])
                for name, topic in self.topics.items()]

    def messages(self, names: list[str]) -> Iterator[tuple[int, str, int, Any]]:
        """Stream only requested BLOBs; never load the full bag into memory."""
        selected = {self.topics[name]['id']: name for name in names if name in self.topics}
        marks = ','.join('?' for _ in selected)
        if not marks:
            return
        cursor = self.connection.execute(
            f'SELECT id, topic_id, timestamp, data FROM messages '
            f'WHERE topic_id IN ({marks}) ORDER BY timestamp, id', tuple(selected)
        )
        for message_id, tid, timestamp, data in cursor:
            name = selected[tid]
            yield int(message_id), name, int(timestamp), self.types.deserialize_cdr(data, self.topics[name]['type'])


def distribution(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    if not len(values):
        return {'count': 0}
    return {'count': len(values), 'min': float(values.min()), 'median': float(np.median(values)),
            'mean': float(values.mean()), 'p95': float(np.quantile(values, .95)), 'max': float(values.max())}


def timestamp_audit(header_ns: list[int] | np.ndarray, bag_ns: list[int] | np.ndarray) -> dict:
    header = np.asarray(header_ns, dtype=np.int64)
    bag = np.asarray(bag_ns, dtype=np.int64)
    unique = np.unique(header)
    delta = np.diff(unique)
    med = float(np.median(delta)) if len(delta) else 0.
    gap_indices = np.flatnonzero(delta > 1.5 * med) if med > 0 else np.array([], dtype=int)
    gaps = [{'start_ns': int(unique[i]), 'end_ns': int(unique[i + 1]),
             'duration_s': float(delta[i] / 1e9),
             'estimated_missing_frames': max(0, int(round(delta[i] / med)) - 1)} for i in gap_indices]
    return {
        'count': len(header), 'unique_header_timestamps': len(unique),
        'duplicate_header_timestamps': len(header) - len(unique),
        'nonpositive_header_timestamps': int(np.sum(header <= 0)),
        'header_reversals_in_bag_order': int(np.sum(np.diff(header) < 0)),
        'first_header_ns': int(unique[0]) if len(unique) else None,
        'last_header_ns': int(unique[-1]) if len(unique) else None,
        'duration_s': float((unique[-1] - unique[0]) / 1e9) if len(unique) > 1 else 0.,
        'median_frequency_hz': 1e9 / med if med else None,
        'interval_ms': distribution(delta / 1e6),
        'bag_minus_header_ms': distribution((bag - header) / 1e6),
        'gap_definition': 'Adjacent unique header interval > 1.5 x median interval',
        'gap_count': len(gaps), 'estimated_missing_frames': sum(g['estimated_missing_frames'] for g in gaps),
        'gaps': gaps,
    }


def nearest_time_matches(source_ns: np.ndarray, target_ns: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return nearest indices and absolute int64 ns errors; ties choose earlier."""
    source = np.asarray(source_ns, dtype=np.int64)
    target = np.asarray(target_ns, dtype=np.int64)
    if not len(target) or np.any(np.diff(target) < 0):
        raise ValueError('Target timestamps must be nonempty and sorted')
    right = np.clip(np.searchsorted(target, source), 0, len(target) - 1)
    left = np.clip(right - 1, 0, len(target) - 1)
    use_left = np.abs(source - target[left]) <= np.abs(source - target[right])
    index = np.where(use_left, left, right)
    return index, np.abs(source - target[index])


def synchronization_audit(source: np.ndarray, target: np.ndarray) -> dict:
    _, delta = nearest_time_matches(np.unique(source), np.unique(target))
    return {**distribution(delta / 1e6), 'units': 'ms', 'fraction_within_50_ms': float(np.mean(delta <= 50_000_000)),
            'fraction_within_20_ms': float(np.mean(delta <= 20_000_000))}


def camera_info_dict(message: Any) -> dict:
    return {'timestamp_ns': stamp_ns(message.header.stamp), 'frame_id': message.header.frame_id,
            'width': int(message.width), 'height': int(message.height),
            'distortion_model': message.distortion_model, 'D': np.asarray(message.d).tolist(),
            'K': np.asarray(message.k).reshape(3, 3).tolist(),
            'R': np.asarray(message.r).reshape(3, 3).tolist(),
            'P': np.asarray(message.p).reshape(3, 4).tolist(),
            'binning_x': int(message.binning_x), 'binning_y': int(message.binning_y),
            'roi': {k: getattr(message.roi, k) for k in ('x_offset', 'y_offset', 'height', 'width', 'do_rectify')}}


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.asarray([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 0:
        raise ValueError('Invalid zero quaternion')
    x, y, z, w = q / norm
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_dict(tf: Any) -> dict:
    trans, rot = tf.transform.translation, tf.transform.rotation
    xyz = [float(trans.x), float(trans.y), float(trans.z)]
    xyzw = [float(rot.x), float(rot.y), float(rot.z), float(rot.w)]
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(*xyzw)
    matrix[:3, 3] = xyz
    return {'parent': tf.header.frame_id, 'child': tf.child_frame_id,
            'timestamp_ns': stamp_ns(tf.header.stamp), 'translation_xyz_m': xyz,
            'quaternion_xyzw': xyzw, 'T_parent_child': matrix.tolist()}


def static_transform_between(transforms: list[dict], target_frame: str, source_frame: str) -> np.ndarray | None:
    """Compose T_target_source using recorded static transforms only."""
    graph: dict[str, list] = {}
    for tf in transforms:
        matrix = np.array(tf['T_parent_child'])
        graph.setdefault(tf['child'], []).append((tf['parent'], matrix))
        graph.setdefault(tf['parent'], []).append((tf['child'], np.linalg.inv(matrix)))
    queue = [(source_frame, np.eye(4))]
    seen = {source_frame}
    for frame, matrix in queue:
        if frame == target_frame:
            return matrix
        for neighbor, edge in graph.get(frame, []):
            if neighbor not in seen:
                seen.add(neighbor)
                queue.append((neighbor, edge @ matrix))
    return None


def scan_geometry(message: Any) -> dict:
    return {**{k: float(getattr(message, k)) for k in
               ('angle_min', 'angle_max', 'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max')},
            'beam_count': len(message.ranges), 'frame': message.header.frame_id}


def export_recorded_map(message: Any, out_dir: Path, bag_ns: int) -> dict:
    import cv2
    info = message.info
    grid = np.asarray(message.data).reshape(int(info.height), int(info.width))
    # ROS map image convention: top image row is highest world y.
    pixels = np.full(grid.shape, 205, dtype=np.uint8)
    pixels[(grid >= 0) & (grid <= 19)] = 254
    pixels[grid >= 65] = 0
    pixels = np.flipud(pixels)
    pgm = out_dir / 'recorded_baseline_UNVERIFIED.pgm'
    pgm.write_bytes(f'P5\n# UNVERIFIED recorded /map baseline; not reconstructed project output\n{info.width} {info.height}\n255\n'.encode() + pixels.tobytes())
    cv2.imwrite(str(pgm.with_suffix('.png')), pixels)
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    origin = [float(info.origin.position.x), float(info.origin.position.y), yaw]
    map_yaml = {'image': pgm.name, 'mode': 'trinary', 'resolution': float(info.resolution),
                'origin': origin, 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196}
    pgm.with_suffix('.yaml').write_text('# UNVERIFIED recorded baseline; never used by the estimator.\n' + yaml.safe_dump(map_yaml, sort_keys=False), encoding='utf-8')
    return {'status': 'UNVERIFIED RECORDED BASELINE; NOT PROJECT FINAL MAP',
            'source_topic': '/map', 'header_timestamp_ns': stamp_ns(message.header.stamp),
            'bag_timestamp_ns': bag_ns, 'frame': message.header.frame_id,
            'width': int(info.width), 'height': int(info.height), 'resolution_m': float(info.resolution),
            'origin_xy_yaw': origin, 'known_fraction': float(np.mean(grid >= 0)),
            'occupied_fraction': float(np.mean(grid >= 65)), 'image': pgm.as_posix(),
            'original_occupancy_counts': {str(int(v)): int(n) for v, n in zip(*np.unique(grid, return_counts=True))}}


def audit_and_extract(bag_path: str | Path, output_root: str | Path = 'outputs') -> dict:
    bag_path = Path(bag_path)
    output_root = Path(output_root)
    audit_dir, cache_dir = output_root / 'audit', output_root / 'cache'
    image_dir = cache_dir / 'images'
    for folder in (audit_dir, cache_dir, image_dir):
        folder.mkdir(parents=True, exist_ok=True)
    streams = {name: {'header_ns': [], 'bag_ns': [], 'frames': Counter(), 'valid_fraction': [],
                      'nan_fraction': [], 'inf_fraction': [], 'geometry': Counter()} for name in (RAW_SCAN, SCAN, CAMERA)}
    raw_ranges, raw_times, raw_bag_times, raw_scan_times, raw_time_increments = [], [], [], [], []
    hashes: dict[str, dict[int, bytes]] = {RAW_SCAN: {}, SCAN: {}}
    duplicate_conflicts = Counter()
    camera_rows, camera_formats, camera_sizes = [], Counter(), Counter()
    camera_info_first = camera_info_last = None
    camera_info_signatures = Counter()
    static_tfs: dict[tuple[str, str], dict] = {}
    static_conflicts = []
    rosout = []
    last_map = None
    last_map_bag_ns = 0
    compressed_bytes = 0
    image_decode_failures = []
    print('Reading topic inventory (SQLite read-only)...', flush=True)
    with ReadOnlyBag(bag_path) as bag:
        inventory = bag.inventory()
        print('Streaming scans, compressed RGB, calibration, static TF and baseline map...', flush=True)
        names = [RAW_SCAN, SCAN, CAMERA, CAMERA_INFO, '/tf_static', '/map', '/rosout']
        raw_geometry = None
        for number, (message_id, topic, bag_time, message) in enumerate(bag.messages(names), 1):
            if number % 10000 == 0:
                print(f'Processed {number:,} selected messages; raw scans={len(raw_ranges):,}, images={len(camera_rows):,}', flush=True)
            if topic in streams:
                header_time = stamp_ns(message.header.stamp)
                stream = streams[topic]
                stream['header_ns'].append(header_time)
                stream['bag_ns'].append(bag_time)
                stream['frames'][message.header.frame_id] += 1
            if topic in (RAW_SCAN, SCAN):
                ranges = np.asarray(message.ranges, dtype=np.float32)
                valid = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
                stream['valid_fraction'].append(float(np.mean(valid)))
                stream['nan_fraction'].append(float(np.mean(np.isnan(ranges))))
                stream['inf_fraction'].append(float(np.mean(np.isinf(ranges))))
                geometry = scan_geometry(message)
                fixed_geometry = {k: v for k, v in geometry.items() if k not in ('scan_time', 'time_increment')}
                geometry_json = json.dumps(fixed_geometry, sort_keys=True)
                stream['geometry'][geometry_json] += 1
                signature = hashlib.blake2b(ranges.tobytes(), digest_size=16).digest()
                if header_time in hashes[topic]:
                    if hashes[topic][header_time] != signature:
                        duplicate_conflicts[topic] += 1
                else:
                    hashes[topic][header_time] = signature
                    if topic == RAW_SCAN:
                        if raw_geometry is None:
                            raw_geometry = geometry
                        if fixed_geometry != {k: v for k, v in raw_geometry.items() if k not in ('scan_time', 'time_increment')}:
                            raise ValueError('Raw scan geometry changed; fixed rectangular cache would be unsafe')
                        raw_ranges.append(ranges.copy())
                        raw_times.append(header_time)
                        raw_bag_times.append(bag_time)
                        raw_scan_times.append(geometry['scan_time'])
                        raw_time_increments.append(geometry['time_increment'])
            elif topic == CAMERA:
                payload = bytes(message.data)
                suffix = '.jpg' if payload.startswith(b'\xff\xd8') else '.png' if payload.startswith(b'\x89PNG') else '.bin'
                image_id = len(camera_rows)
                image_path = image_dir / f'frame_{image_id:06d}{suffix}'
                image_path.write_bytes(payload)
                camera_rows.append({'image_id': image_id, 'timestamp_ns': header_time, 'path': image_path.as_posix()})
                compressed_bytes += len(payload)
                camera_formats[message.format] += 1
                # Decode every image to verify actual usable RGB coverage.
                import cv2
                decoded = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                if decoded is None:
                    image_decode_failures.append(image_id)
                else:
                    camera_sizes[f'{decoded.shape[1]}x{decoded.shape[0]}'] += 1
            elif topic == CAMERA_INFO:
                info = camera_info_dict(message)
                if camera_info_first is None:
                    camera_info_first = info
                camera_info_last = info
                camera_info_signatures[json.dumps({k: v for k, v in info.items() if k != 'timestamp_ns'}, sort_keys=True)] += 1
            elif topic == '/tf_static':
                for tf in message.transforms:
                    item = transform_dict(tf)
                    key = (item['parent'], item['child'])
                    if key in static_tfs and not np.allclose(static_tfs[key]['T_parent_child'], item['T_parent_child'], atol=1e-9):
                        static_conflicts.append({'earlier': static_tfs[key], 'later': item})
                    static_tfs[key] = item
            elif topic == '/map':
                last_map, last_map_bag_ns = message, bag_time
            elif topic == '/rosout':
                rosout.append({'timestamp_ns': stamp_ns(message.stamp), 'level': int(message.level),
                               'name': message.name, 'message': message.msg})

    if raw_geometry is None or not raw_ranges or not camera_rows:
        raise ValueError('Required raw scans or compressed camera images are absent')
    order = np.argsort(np.asarray(raw_times, np.int64), kind='stable')
    raw_times_array = np.asarray(raw_times, np.int64)[order]
    ranges_array = np.stack(raw_ranges)[order]
    scan_cache = cache_dir / 'scans.npz'
    np.savez_compressed(scan_cache, times_ns=raw_times_array, ranges=ranges_array,
                        bag_times_ns=np.asarray(raw_bag_times, np.int64)[order],
                        scan_times_s=np.asarray(raw_scan_times, np.float32)[order],
                        time_increments_s=np.asarray(raw_time_increments, np.float32)[order],
                        **{k: np.asarray(v) for k, v in raw_geometry.items()})
    with (cache_dir / 'images.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=['image_id', 'timestamp_ns', 'path'])
        writer.writeheader()
        writer.writerows(camera_rows)
    for name, stream in streams.items():
        filename = 'raw_scan' if name == RAW_SCAN else 'scan' if name == SCAN else 'camera'
        with (audit_dir / f'{filename}_timestamps.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['message_index', 'header_timestamp_ns', 'bag_timestamp_ns'])
            writer.writerows((i, int(h), int(b)) for i, (h, b) in enumerate(zip(stream['header_ns'], stream['bag_ns'])))
    camera_times = np.asarray(streams[CAMERA]['header_ns'], np.int64)
    camera_frame = next(iter(streams[CAMERA]['frames']))
    t_camera_lidar = static_transform_between(list(static_tfs.values()), camera_frame, raw_geometry['frame'])
    calibration = {'camera_info_first': camera_info_first, 'camera_info_last': camera_info_last,
                   'camera_info_unique_calibrations': len(camera_info_signatures),
                   'camera_info_calibration_counts': list(camera_info_signatures.values()),
                   'tf_static': list(static_tfs.values()), 'tf_static_conflicts': static_conflicts,
                   'scan': raw_geometry, 'camera_frame': camera_frame,
                   'T_camera_lidar': t_camera_lidar.tolist() if t_camera_lidar is not None else None,
                   'transform_convention': 'T_target_source maps a point from source coordinates into target coordinates; column vectors',
                   'extrinsics_status': 'Recorded static TF only; physical calibration accuracy is unverified'}
    (audit_dir / 'calibration.json').write_text(json.dumps(calibration, indent=2), encoding='utf-8')
    (audit_dir / 'rosout.json').write_text(json.dumps(rosout, indent=2), encoding='utf-8')
    shared = hashes[RAW_SCAN].keys() & hashes[SCAN].keys()
    identical = sum(hashes[RAW_SCAN][t] == hashes[SCAN][t] for t in shared)
    audit = {'bag_path': bag_path.as_posix(), 'bag_size_bytes': bag_path.stat().st_size,
             'topic_inventory': inventory, 'estimator_input_topics': [RAW_SCAN, CAMERA, CAMERA_INFO, '/tf_static'],
             'excluded_estimator_topics': ['/odom', '/pose', '/tf', '/imu/data', '/map', '/scan'],
             'streams': {}, 'scan_topic_overlap': {
                 'shared_unique_timestamps': len(shared), 'identical_range_arrays': identical,
                 'different_range_arrays': len(shared) - identical,
                 'raw_only_unique_timestamps': len(hashes[RAW_SCAN].keys() - hashes[SCAN].keys()),
                 'scan_only_unique_timestamps': len(hashes[SCAN].keys() - hashes[RAW_SCAN].keys())},
             'synchronization': {
                 'camera_to_nearest_raw_scan': synchronization_audit(camera_times, raw_times_array),
                 'raw_scan_to_nearest_camera': synchronization_audit(raw_times_array, camera_times)},
             'camera': {'formats': dict(camera_formats), 'decoded_dimensions': dict(camera_sizes),
                        'compressed_payload_bytes': compressed_bytes, 'decode_failure_image_ids': image_decode_failures,
                        'exported_image_count': len(camera_rows), 'manifest': (cache_dir / 'images.csv').as_posix()},
             'calibration': calibration, 'scan_cache': scan_cache.as_posix(),
             'rosout_provenance': rosout_provenance(rosout),
             'recorded_baseline': export_recorded_map(last_map, audit_dir, last_map_bag_ns) if last_map is not None else None,
             'assumptions': ['All synchronization uses message header timestamps, not bag receipt time.',
                             'Duplicate scan timestamps retain the first record; conflicts are counted.',
                             'Missing-frame estimates use median cadence and are diagnostic, not ground truth.',
                             'The recorded map is unverified and excluded from all estimation inputs.',
                             'Timestamp alignment does not establish hardware synchronization.']}
    for topic, stream in streams.items():
        item = timestamp_audit(stream['header_ns'], stream['bag_ns'])
        item['frames'] = dict(stream['frames'])
        if topic != CAMERA:
            item['valid_range_fraction'] = distribution(stream['valid_fraction'])
            item['nan_range_fraction'] = distribution(stream['nan_fraction'])
            item['inf_range_fraction'] = distribution(stream['inf_fraction'])
            item['duplicate_timestamp_range_conflicts'] = duplicate_conflicts[topic]
            item['geometries'] = [{'geometry': json.loads(k), 'count': n} for k, n in stream['geometry'].items()]
        audit['streams'][topic] = item
    metadata_path = bag_path.parent / 'metadata.yaml'
    if metadata_path.exists():
        metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))['rosbag2_bagfile_information']
        expected = {x['topic_metadata']['name']: x['message_count'] for x in metadata['topics_with_message_count']}
        audit['metadata_consistency'] = {'metadata_total_messages': int(metadata['message_count']),
                                         'sqlite_total_messages': sum(t['count'] for t in inventory),
                                         'count_mismatches': [t['name'] for t in inventory if expected.get(t['name']) != t['count']]}
    (audit_dir / 'audit.json').write_text(json.dumps(audit, indent=2), encoding='utf-8')
    (audit_dir / 'audit.md').write_text(audit_markdown(audit), encoding='utf-8')
    print(f'Finished: {scan_cache}, {cache_dir / "images.csv"}, {audit_dir / "audit.json"}', flush=True)
    return audit


def rosout_provenance(logs: list[dict]) -> dict:
    drops = [row for row in logs if row['name'] == 'slam_toolbox' and 'queue is full' in row['message']]
    return {'logger_counts': dict(Counter(row['name'] for row in logs)),
            'slam_toolbox_queue_full_log_count': len(drops),
            'first_queue_full_log_ns': drops[0]['timestamp_ns'] if drops else None,
            'last_queue_full_log_ns': drops[-1]['timestamp_ns'] if drops else None,
            'interpretation': 'Recorded online SLAM logged queue drops; this does not imply raw scans are missing from this bag and does not establish map accuracy.'}


def audit_markdown(audit: dict) -> str:
    lines = ['# Sensor bag audit', '', f"Bag: `{audit['bag_path']}` ({audit['bag_size_bytes'] / 1e9:.3f} GB).",
             '', 'Only raw laser scans and RGB images are motion/loop-estimation inputs. CameraInfo and static TF supply calibration. Recorded odometry, poses, dynamic TF, IMU, and maps are excluded.', '',
             '| Stream | Messages | Unique stamps | Duplicate stamps | Hz (median) | Header reversals | Gaps > 1.5 cadence |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for name, s in audit['streams'].items():
        lines.append(f"| `{name}` | {s['count']} | {s['unique_header_timestamps']} | {s['duplicate_header_timestamps']} | {s['median_frequency_hz']:.3f} | {s['header_reversals_in_bag_order']} | {s['gap_count']} |")
    lines += ['', '## Time alignment', '', '| Direction | Median (ms) | p95 (ms) | Maximum (ms) | Within 50 ms |', '|---|---:|---:|---:|---:|']
    for name, s in audit['synchronization'].items():
        lines.append(f"| {name} | {s['median']:.3f} | {s['p95']:.3f} | {s['max']:.3f} | {100*s['fraction_within_50_ms']:.3f}% |")
    lines += ['', '## Scan integrity', '']
    overlap = audit['scan_topic_overlap']
    lines.append(f"The two scan topics share {overlap['shared_unique_timestamps']} unique stamps; {overlap['identical_range_arrays']} have byte-identical ranges and {overlap['different_range_arrays']} differ. The cache contains only deduplicated `/scan/raw`.")
    for name in (RAW_SCAN, SCAN):
        s = audit['streams'][name]
        valid = s['valid_range_fraction']
        lines.append(f"- `{name}`: valid finite in-range fraction mean {100*valid['mean']:.2f}%, minimum {100*valid['min']:.2f}%; duplicate-stamp range conflicts {s['duplicate_timestamp_range_conflicts']}; frames {s['frames']}.")
    lines += ['', '## Camera and calibration', '',
              f"Exported all {audit['camera']['exported_image_count']} compressed frames ({audit['camera']['compressed_payload_bytes']/1e6:.1f} MB); decode failures: {len(audit['camera']['decode_failure_image_ids'])}. Dimensions: {audit['camera']['decoded_dimensions']}.",
              f"Failed-decode image IDs: {audit['camera']['decode_failure_image_ids']}. Original compressed bytes are retained; downstream image processing must skip these frames.",
              f"CameraInfo has {audit['calibration']['camera_info_unique_calibrations']} unique calibrations. Recorded static TF contains {len(audit['calibration']['tf_static'])} edges and {len(audit['calibration']['tf_static_conflicts'])} conflicting changes. Camera-to-laser static path available: {audit['calibration']['T_camera_lidar'] is not None}.",
              'Camera intrinsics and first/last CameraInfo, full static transforms, and transform conventions are in `calibration.json`. Static extrinsics are recorded values; physical accuracy has not been independently established.',
              '', '## Timing gaps and receipt delays', '']
    for name, s in audit['streams'].items():
        lag = s['bag_minus_header_ms']
        lines.append(f"- `{name}`: duration {s['duration_s']:.3f} s; largest unique-stamp interval {s['interval_ms']['max']:.3f} ms; estimated missing frames {s['estimated_missing_frames']}; receipt minus header median {lag['median']:.3f} ms, p95 {lag['p95']:.3f} ms, range [{lag['min']:.3f}, {lag['max']:.3f}] ms.")
        for gap in sorted(s['gaps'], key=lambda x: x['duration_s'], reverse=True)[:5]:
            lines.append(f"  - Gap {gap['start_ns']}–{gap['end_ns']}: {gap['duration_s']:.3f} s (estimated {gap['estimated_missing_frames']} frames).")
    lines += ['', '## Recorded map baseline', '',
              '**`recorded_baseline_UNVERIFIED.{pgm,png,yaml}` is the last recorded `/map`, exported for provenance only. It is not a validated map, a ground-truth map, or the project final output. It is never used by the estimator.**',
              f"Recorded `slam_toolbox` logs contain {audit.get('rosout_provenance', {}).get('slam_toolbox_queue_full_log_count', 0)} queue-full message-filter drops. These logs concern the online mapping process; the raw scan stream audited here has its own independent completeness checks.",
              '', '## Reproducibility', '',
              'Full per-message header and bag timestamps are in `raw_scan_timestamps.csv`, `scan_timestamps.csv`, and `camera_timestamps.csv`. Full gap lists, topic counts, range checks, and synchronization statistics are in `audit.json`. `rosout.json` preserves recorded logging provenance.',
              f"Sensor cache: `{audit['scan_cache']}`; image manifest: `{audit['camera']['manifest']}`.", '',
              'Timestamp matching measures numerical alignment only; it does not establish hardware synchronization. Gap counts are based on a cadence heuristic. No external ground-truth trajectory is present in this audit.', '']
    return '\n'.join(lines)
