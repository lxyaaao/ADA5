"""Scans, compressed images, camera calibration and static mounts from a rosbag2 recording.

Only the streams the method uses are read: the filtered LiDAR scan, the
compressed colour images, camera_info and /tf_static. The first load can
write a cache (npz + json) so later runs do not reread the 5.5 GB bag.
"""
import json
import os

import numpy as np

from .geometry import quat_matrix, yaw_of

SCAN = '/scan'
IMAGE = '/camera/camera/color/image_raw/compressed'
CAMERA_INFO = '/camera/camera/color/camera_info'
TF_STATIC = '/tf_static'


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Recording:
    def __init__(self, scan_t, ranges, scan_meta, image_t, images, camera_info, tf_static):
        self.scan_t = scan_t            # header stamps (s), sorted
        self.ranges = ranges            # N x beams, float32
        self.scan_meta = scan_meta      # angle_min, angle_increment, time_increment, ...
        self.image_t = image_t          # header stamps (s), sorted
        self.images = images            # JPEG bytes
        self.camera_info = camera_info  # K (3x3), D, width, height, frame_id
        self.tf_static = tf_static      # {'parent child': (t (3,), q (4,) xyzw)}
        self.t0 = float(min(scan_t[0], image_t[0]))

    @property
    def angles(self):
        m = self.scan_meta
        return m['angle_min'] + m['angle_increment'] * np.arange(self.ranges.shape[1])

    def mount(self, parent, child):
        """(R 3x3, t 3) of child in parent, from /tf_static."""
        t, q = self.tf_static[f'{parent} {child}']
        return quat_matrix(q), np.asarray(t, dtype=float)

    def laser_pose2d(self, base='base_link'):
        R, t = self.mount(base, self.scan_meta['frame_id'])
        return np.array([t[0], t[1], yaw_of(R)])

    @classmethod
    def load(cls, bag, cache_dir=None, scan_topic=SCAN, image_topic=IMAGE, info_topic=CAMERA_INFO):
        if cache_dir and os.path.exists(os.path.join(cache_dir, 'recording.npz')):
            return cls._from_cache(cache_dir)
        rec = cls._from_bag(bag, scan_topic, image_topic, info_topic)
        if cache_dir:
            rec._save_cache(cache_dir)
        return rec

    @classmethod
    def _from_bag(cls, bag, scan_topic, image_topic, info_topic):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        topics = [t for t in (scan_topic, image_topic, info_topic, TF_STATIC) if t in types]
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        msg_cls = {t: get_message(types[t]) for t in topics}
        scan_t, ranges, image_t, images = [], [], [], []
        scan_meta, camera_info, tf_static = None, None, {}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            msg = deserialize_message(data, msg_cls[topic])
            if topic == image_topic:
                image_t.append(_stamp(msg.header))
                images.append(bytes(msg.data))
            elif topic == scan_topic:
                scan_t.append(_stamp(msg.header))
                ranges.append(np.asarray(msg.ranges, dtype=np.float32))
                if scan_meta is None:
                    scan_meta = {'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                                 'time_increment': msg.time_increment, 'scan_time': msg.scan_time,
                                 'range_min': msg.range_min, 'range_max': msg.range_max,
                                 'frame_id': msg.header.frame_id}
            elif topic == info_topic and camera_info is None:
                camera_info = {'K': np.array(msg.k, dtype=float).reshape(3, 3),
                               'D': np.array(msg.d, dtype=float), 'width': msg.width,
                               'height': msg.height, 'frame_id': msg.header.frame_id,
                               'model': msg.distortion_model}
            elif topic == TF_STATIC:
                for tr in msg.transforms:
                    tt, q = tr.transform.translation, tr.transform.rotation
                    tf_static[f'{tr.header.frame_id} {tr.child_frame_id}'] = (
                        [tt.x, tt.y, tt.z], [q.x, q.y, q.z, q.w])
        o = np.argsort(scan_t, kind='stable')
        p = np.argsort(image_t, kind='stable')
        return cls(np.asarray(scan_t)[o], np.stack(ranges)[o], scan_meta,
                   np.asarray(image_t)[p], [images[i] for i in p], camera_info, tf_static)

    def _save_cache(self, d):
        os.makedirs(d, exist_ok=True)
        sizes = np.array([len(b) for b in self.images], dtype=np.int64)
        np.savez(os.path.join(d, 'recording.npz'), scan_t=self.scan_t, ranges=self.ranges,
                 image_t=self.image_t, image_offsets=np.r_[0, np.cumsum(sizes)],
                 image_blob=np.frombuffer(b''.join(self.images), dtype=np.uint8))
        info = dict(self.camera_info, K=self.camera_info['K'].tolist(), D=self.camera_info['D'].tolist())
        with open(os.path.join(d, 'recording.json'), 'w') as f:
            json.dump({'scan_meta': self.scan_meta, 'camera_info': info,
                       'tf_static': self.tf_static}, f, indent=1)

    @classmethod
    def _from_cache(cls, d):
        z = np.load(os.path.join(d, 'recording.npz'))
        meta = json.load(open(os.path.join(d, 'recording.json')))
        blob, off = z['image_blob'], z['image_offsets']
        images = [blob[off[i]:off[i + 1]].tobytes() for i in range(len(off) - 1)]
        info = meta['camera_info']
        info['K'], info['D'] = np.asarray(info['K']), np.asarray(info['D'])
        tf_static = {k: (v[0], v[1]) for k, v in meta['tf_static'].items()}
        return cls(z['scan_t'], z['ranges'], meta['scan_meta'], z['image_t'], images, info, tf_static)
