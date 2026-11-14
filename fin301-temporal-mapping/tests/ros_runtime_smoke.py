"""Actual ROS 2 smoke check; run explicitly on a sourced ROS host, not pytest.

The publisher sends artifacts before subscribers start. Its repeat interval is
one hour, so successful delivery here checks transient-local durability.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from indoor_mapping.ros_node import load_map_artifact, load_trajectory, load_ascii_pcd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--publisher-mode', choices=('module', 'installed'), default='module')
    parser.add_argument('--process-cache', action='store_true')
    parser.add_argument('--lidar-revisits', action='store_true')
    parser.add_argument('--cache')
    parser.add_argument('--log-dir', default='outputs/verification')
    parser.add_argument('--startup-timeout', type=float, default=180.0,
                        help='Seconds allowed for processing before publication')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    if args.process_cache and not args.cache:
        parser.error('--process-cache requires --cache')
    log_path = Path(args.log_dir) / 'ros_publisher.log'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command = (['ros2', 'run', 'indoor_mapping', 'batch_mapping']
               if args.publisher_mode == 'installed' else
               [sys.executable, '-m', 'indoor_mapping.ros_node'])
    command += ['--ros-args', '-r', '__ns:=/indoor_mapping_smoke',
                '-p', f'process_cache:={str(args.process_cache).lower()}', '-p', f'output_dir:={output}',
                '-p', 'publish_period:=3600.0']
    if args.cache:
        command += ['-p', f'cache_dir:={Path(args.cache).resolve()}']
    if args.lidar_revisits:
        command += ['-p', 'lidar_revisits:=true']
    print('Publisher mode:', args.publisher_mode, flush=True)
    print('ROS_DOMAIN_ID:', os.environ.get('ROS_DOMAIN_ID'), flush=True)
    process = None
    node = None
    rclpy.init()
    try:
        with log_path.open('w', encoding='utf-8') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + (args.startup_timeout if args.process_cache else 25)
            while time.monotonic() < deadline:
                contents = log_path.read_text(encoding='utf-8')
                if 'Published recorded map' in contents:
                    print('Publisher completed its initial send before subscriber creation', flush=True)
                    break
                if process.poll() is not None:
                    raise RuntimeError(f'Publisher exited with {process.returncode}:\n{contents}')
                time.sleep(0.1)
            else:
                raise RuntimeError('Publisher did not finish its initial send:\n' + contents)
            grid = load_map_artifact(output / 'map.yaml')
            poses = load_trajectory(output / 'trajectory.csv')
            points = load_ascii_pcd(output / 'map.pcd')
            print('Expected artifacts:', grid.width, 'x', grid.height, 'grid,', len(points), 'points,', len(poses), 'poses', flush=True)
            node = rclpy.create_node('artifact_smoke_subscriber', namespace='/indoor_mapping_smoke')
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            received = {}
            subscriptions = [
                node.create_subscription(OccupancyGrid, 'map', lambda msg: received.setdefault('map', msg), qos),
                node.create_subscription(PointCloud2, 'map_points', lambda msg: received.setdefault('points', msg), qos),
                node.create_subscription(RosPath, 'trajectory', lambda msg: received.setdefault('path', msg), qos),
            ]
            deadline = time.monotonic() + 20
            while len(received) < 3 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
                if process.poll() is not None:
                    raise RuntimeError('Publisher stopped unexpectedly')
            assert len(received) == 3, f'Missing actual ROS messages: {set(("map", "points", "path")) - received.keys()}'
            actual_grid = received['map']
            assert (actual_grid.info.width, actual_grid.info.height) == (grid.width, grid.height)
            assert abs(actual_grid.info.resolution - grid.resolution) < 1e-7
            assert list(actual_grid.data) == grid.cells
            assert abs(actual_grid.info.origin.position.x - grid.origin[0]) < 1e-9
            assert abs(actual_grid.info.origin.position.y - grid.origin[1]) < 1e-9
            print('PASS /map: dimensions, resolution, origin, every occupancy cell', flush=True)
            cloud = received['points']
            assert cloud.width == len(points) and cloud.height == 1
            assert cloud.point_step == 12 and cloud.row_step == 12 * len(points)
            assert not cloud.is_bigendian and cloud.is_dense
            assert [(field.name, field.offset, field.datatype, field.count) for field in cloud.fields] == [
                ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1)]
            actual_points = np.frombuffer(bytes(cloud.data), dtype='<f4').reshape(-1, 3)
            np.testing.assert_array_equal(actual_points, points)
            print('PASS /map_points: PointCloud2 fields, dimensions, every XYZ point', flush=True)
            path = received['path']
            assert len(path.poses) == len(poses)
            for message, pose in zip(path.poses, poses):
                assert message.header.stamp.sec * 10**9 + message.header.stamp.nanosec == pose.timestamp_ns
                assert message.pose.position.x == pose.x and message.pose.position.y == pose.y
                assert abs(message.pose.orientation.z - math.sin(pose.yaw / 2)) < 1e-12
                assert abs(message.pose.orientation.w - math.cos(pose.yaw / 2)) < 1e-12
                assert message.header.frame_id == 'map'
            for message in (actual_grid, cloud, path):
                assert message.header.frame_id == 'map'
                assert message.header.stamp.sec * 10**9 + message.header.stamp.nanosec == poses[-1].timestamp_ns
            print('PASS /trajectory: every historical pose, yaw quaternion, exact acquisition timestamp', flush=True)
            print('PASS late subscription: all messages arrived after initial send with repeat interval 3600 seconds', flush=True)
            print('ROS runtime smoke test PASSED; ' + ('measurement cache processed inside ROS node and published' if args.process_cache else 'historical artifact publication'), flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            print('Publisher process group stopped', flush=True)


if __name__ == '__main__':
    main()
