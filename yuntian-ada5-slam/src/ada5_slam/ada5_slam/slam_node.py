"""ROS 2 node running the same method on live or replayed topics.

Subscribes: LaserScan (default /scan), CompressedImage and CameraInfo of the
forward camera; reads the laser and camera mounts from /tf_static.
Publishes: /ada5/pose (PoseStamped), /ada5/path (Path), /ada5/map
(OccupancyGrid, every map_period s) and the transform map -> base_link.
While running it does LiDAR odometry with the camera heading prior and
builds the occupancy grid from keyframes. On shutdown it runs camera loop
closure and the pose graph over the collected keyframes and writes the map
(PGM + YAML), the trajectory and the loop list to output_dir.
"""
import array
import collections
import json
import os
import threading
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')   # threaded BLAS is far slower on the course VM
os.environ.setdefault('OMP_NUM_THREADS', '1')

import numpy as np  # noqa: E402
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid as GridMsg
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
import tf2_ros

from .geometry import between, compose, quat_matrix, yaw_of
from .lidar_odometry import LidarOdometry
from .loop_closure import find_loops
from .occupancy import OccupancyGrid
from .pipeline import DEFAULTS
from .pose_graph import optimize
from .visual import Camera, LiveYawTrack


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _quat(yaw):
    return 0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2))


class SlamNode(Node):
    def __init__(self):
        super().__init__('ada5_slam')
        dp = {'scan_topic': '/scan', 'image_topic': '/camera/camera/color/image_raw/compressed',
              'info_topic': '/camera/camera/color/camera_info', 'base_frame': 'base_link',
              'map_frame': 'map', 'process_every': 3, 'use_camera_prior': True,
              'map_period': 5.0, 'map_size_m': 120.0, 'output_dir': 'ada5_slam_out',
              'run_loop_closure_on_exit': True, 'max_prior_wait_s': 0.8}
        for k, v in dp.items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.cfg = {k: g(k) for k in dp}
        self.p = dict(DEFAULTS, process_every=self.cfg['process_every'])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_pub = tf2_ros.TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/ada5/pose', 10)
        self.path_pub = self.create_publisher(Path, '/ada5/path', 10)
        self.map_pub = self.create_publisher(
            GridMsg, '/ada5/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Scans and images run in separate threads (MultiThreadedExecutor) so the slow image
        # processing does not delay scan matching; shared state is guarded by self.lock.
        self.lock = threading.Lock()
        g_scan, g_img, g_map = (MutuallyExclusiveCallbackGroup() for _ in range(3))
        self.create_subscription(LaserScan, self.cfg['scan_topic'], self.on_scan, qos_profile_sensor_data,
                                 callback_group=g_scan)
        self.create_subscription(CompressedImage, self.cfg['image_topic'], self.on_image,
                                 qos_profile_sensor_data, callback_group=g_img)
        self.create_subscription(CameraInfo, self.cfg['info_topic'], self.on_info, qos_profile_sensor_data,
                                 callback_group=g_img)
        self.create_timer(self.cfg['map_period'], self.publish_map, callback_group=g_map)
        # Scans wait (up to max_prior_wait_s) until the camera heading track covers their time.
        self.scan_queue = collections.deque()
        self.create_timer(0.02, self.process_queue, callback_group=g_scan)

        self.cam, self.cam_loop, self.odo, self.grid = None, None, None, None
        self.image_count = 0
        self.track, self.prev_feat = LiveYawTrack(), None
        self.recent_images = []          # (t, jpeg) of the last few seconds
        self.kf_images, self.kf_image_t = [], []
        self.path = Path()
        self.path.header.frame_id = self.cfg['map_frame']
        self.scan_count, self.last_t, self.laser_xy = 0, None, None
        self.trajectory, self.stats = [], {'scans': 0, 'lost': 0, 'prior_used': 0, 'step_ms': []}

    def _mount(self, child):
        try:
            tr = self.tf_buffer.lookup_transform(self.cfg['base_frame'], child, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        return quat_matrix([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])

    def on_info(self, msg):
        if self.cam is not None:
            return
        m = self._mount(msg.header.frame_id)
        if m is None:
            return
        args = (np.array(msg.k).reshape(3, 3), np.array(msg.d), m[0])
        kw = {'ransac_px': self.p['camera_ransac_px'], 'ransac_iters': self.p['camera_ransac_iters']}
        self.cam = Camera(*args, n_features=self.p['orb_features_heading'], **kw)
        self.cam_loop = Camera(*args, n_features=self.p['orb_features_loop'], **kw)
        self.get_logger().info(f'camera model ready ({msg.width}x{msg.height}, frame {msg.header.frame_id})')

    def on_image(self, msg):
        if self.cam is None:
            return
        t, jpeg = _stamp(msg.header), bytes(msg.data)
        with self.lock:
            self.recent_images.append((t, jpeg))
            del self.recent_images[:-30]
        self.image_count += 1
        if self.image_count % self.p['camera_heading_every']:
            return
        feat = self.cam.features(jpeg)
        inc = None
        if self.prev_feat is not None:
            inc, _, _ = self.cam.relative_motion(self.prev_feat, feat)
        with self.lock:
            self.track.add(t, inc)
        self.prev_feat = feat

    def on_scan(self, msg):
        self.scan_count += 1
        if self.scan_count % self.cfg['process_every']:
            return
        self.scan_queue.append((time.monotonic(), msg))
        while len(self.scan_queue) > 60:
            self.scan_queue.popleft()
            self.stats['dropped'] = self.stats.get('dropped', 0) + 1

    def process_queue(self):
        while self.scan_queue:
            arrived, msg = self.scan_queue[0]
            t = _stamp(msg.header) + len(msg.ranges) / 2 * msg.time_increment
            with self.lock:
                covered = bool(self.track.t) and self.track.t[-1] >= t
            if (self.cfg['use_camera_prior'] and self.cam is not None and not covered
                    and time.monotonic() - arrived < self.cfg['max_prior_wait_s']):
                return
            self.scan_queue.popleft()
            self._process_scan(msg)

    def _process_scan(self, msg):
        n = len(msg.ranges)
        if self.odo is None:
            m = self._mount(msg.header.frame_id)
            if m is None:
                return
            R, t = m
            self.laser_xy = t[:2]
            angles = msg.angle_min + msg.angle_increment * np.arange(n)
            beam_dt = (np.arange(n) - n / 2) * msg.time_increment
            self.odo = LidarOdometry(angles, beam_dt, np.array([t[0], t[1], yaw_of(R)]), self.p)
            self.grid = OccupancyGrid(self.p['map_resolution']).allocate_square((0, 0), self.cfg['map_size_m'])
            self.get_logger().info(f'LiDAR odometry started ({n} beams, frame {msg.header.frame_id})')
        t = _stamp(msg.header) + n / 2 * msg.time_increment
        inc = None
        if self.cfg['use_camera_prior'] and self.last_t is not None:
            with self.lock:
                inc = self.track.increment(self.last_t, t)
        c0 = time.perf_counter()
        r = self.odo.step(t, np.asarray(msg.ranges, dtype=np.float32), inc)
        self.stats['step_ms'].append(1e3 * (time.perf_counter() - c0))
        self.stats['scans'] += 1
        self.stats['lost'] += r['status'] == 'lost'
        self.stats['prior_used'] += inc is not None
        self.last_t = t
        self.trajectory.append((t, *r['pose'], r['status']))
        if r['keyframe']:
            with self.lock:
                self.grid.integrate(r['pose'], r['pts'], self.laser_xy, self.p['map_max_free'])
                recent = list(self.recent_images)
            if recent:
                k = int(np.argmin([abs(ti - t) for ti, _ in recent]))
                self.kf_image_t.append(recent[k][0])
                self.kf_images.append(recent[k][1])
            else:
                self.kf_image_t.append(np.nan)
                self.kf_images.append(b'')
        self._publish_pose(msg.header.stamp, r['pose'])

    def _publish_pose(self, stamp, pose):
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, self.cfg['map_frame']
        ps.pose.position.x, ps.pose.position.y = float(pose[0]), float(pose[1])
        (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
         ps.pose.orientation.w) = _quat(pose[2])
        self.pose_pub.publish(ps)
        self.path.header.stamp = stamp
        self.path.poses.append(ps)
        self.path_pub.publish(self.path)
        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = self.cfg['base_frame']
        tf.transform.translation.x, tf.transform.translation.y = ps.pose.position.x, ps.pose.position.y
        tf.transform.rotation = ps.pose.orientation
        self.tf_pub.sendTransform(tf)

    def publish_map(self):
        """Publish the observed part of the grid (a full 120 m grid as a Python list stalled the node)."""
        if self.grid is None:
            return
        with self.lock:
            seen = (self.grid.n_occ + self.grid.n_free) > 0
            if not seen.any():
                return
            rows, cols = np.where(seen.any(1))[0], np.where(seen.any(0))[0]
            r0, r1, c0, c1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
            st = self.grid.states()[r0:r1, c0:c1]
        msg = GridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['map_frame']
        msg.info.resolution = float(self.grid.res)
        msg.info.height, msg.info.width = st.shape
        msg.info.origin.position.x = float(self.grid.origin[0] + c0 * self.grid.res)
        msg.info.origin.position.y = float(self.grid.origin[1] + r0 * self.grid.res)
        msg.info.origin.orientation.w = 1.0
        data = np.where(st == 1, 100, np.where(st == 0, 0, -1)).astype(np.int8)
        msg.data = array.array('b', data.tobytes())
        self.map_pub.publish(msg)

    def finish(self):
        """Loop closure, pose graph and map files for everything collected."""
        out = self.cfg['output_dir']
        os.makedirs(out, exist_ok=True)
        log = self.get_logger().info
        if self.odo is None or len(self.odo.keyframes) < 2:
            log('no keyframes collected; nothing to save')
            return
        kfs = self.odo.keyframes
        kf_front = np.array([k['pose'] for k in kfs])
        loops = []
        if self.cfg['run_loop_closure_on_exit'] and self.cam is not None:
            index = [k if self.kf_images[k] else -1 for k in range(len(kfs))]
            loops, _, _ = find_loops(kfs, self.kf_images, None, self.cam_loop, self.p, log=log,
                                     kf_image_index=index)
        edges = []
        for m in range(1, len(kfs)):
            z = between(kf_front[m - 1], kf_front[m])
            s_xy = self.p['odom_sigma_xy'] + self.p['odom_sigma_xy_per_m'] * np.hypot(z[0], z[1])
            s_th = np.radians(self.p['odom_sigma_deg'] + self.p['odom_sigma_deg_per_deg'] * abs(np.degrees(z[2])))
            edges.append((m - 1, m, z, np.diag([1 / s_xy, 1 / s_xy, 1 / s_th])))
        L = np.diag([1 / self.p['loop_sigma_xy']] * 2 + [1 / np.radians(self.p['loop_sigma_deg'])])
        loop_edges = [(l['j'], l['i'], np.array(l['z']), L) for l in loops]
        kf_opt = optimize(kf_front, edges + loop_edges, f_scale=self.p['graph_f_scale'])[0] if loop_edges else kf_front
        grid = OccupancyGrid(self.p['map_resolution']).build(kf_opt, [k['pts'] for k in kfs], self.laser_xy,
                                                             max_free=self.p['map_max_free'])
        grid.save(out, 'live_map')
        with open(os.path.join(out, 'live_trajectory.csv'), 'w') as f:
            f.write('t_s,x_m,y_m,yaw_rad,status\n')
            for row in self.trajectory:
                f.write('{:.4f},{:.4f},{:.4f},{:.5f},{}\n'.format(*row))
        ms = np.array(self.stats['step_ms'])
        summary = {'scans_processed': self.stats['scans'], 'scans_dropped': self.stats.get('dropped', 0),
                   'lost': self.stats['lost'],
                   'prior_used_share': self.stats['prior_used'] / max(self.stats['scans'], 1),
                   'keyframes': len(kfs), 'loops': len(loops),
                   'step_ms_p50_p95_max': [float(np.median(ms)), float(np.percentile(ms, 95)), float(ms.max())],
                   'first_last_scan_time': [self.trajectory[0][0], self.trajectory[-1][0]]}
        with open(os.path.join(out, 'live_summary.json'), 'w') as f:
            json.dump(summary, f, indent=1)
        log(f'saved live_map.pgm/.yaml and live_summary.json in {out}: {summary}')


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
