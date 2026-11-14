"""ROS 2 batch adapter for historical mapping artifacts.

Pure artifact readers remain importable without ROS. The ROS runtime is imported
only by ``main``. This node publishes no live odometry or TF transforms.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from decimal import Decimal
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


@dataclass(frozen=True)
class MapArtifact:
    resolution: float
    origin: tuple[float, float, float]
    width: int
    height: int
    cells: list[int]


@dataclass(frozen=True)
class HistoricalPose:
    timestamp_ns: int
    x: float
    y: float
    yaw: float


def load_map_artifact(yaml_path: str | Path) -> MapArtifact:
    """Read a trinary ROS map and change image rows to OccupancyGrid order."""
    yaml_path = Path(yaml_path)
    config = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("Map YAML must contain a mapping")
    if config.get("mode", "trinary") != "trinary":
        raise ValueError("Only trinary occupancy maps are supported")
    resolution = float(config["resolution"])
    origin = tuple(float(value) for value in config["origin"])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Map resolution must be finite and positive")
    if len(origin) != 3 or not all(math.isfinite(value) for value in origin):
        raise ValueError("Map origin must contain finite x, y, yaw")
    free_threshold = float(config["free_thresh"])
    occupied_threshold = float(config["occupied_thresh"])
    if not 0 <= free_threshold < occupied_threshold <= 1:
        raise ValueError("Map thresholds must satisfy 0 <= free < occupied <= 1")
    negate = int(config.get("negate", 0))
    if negate not in (0, 1):
        raise ValueError("Map negate must be 0 or 1")
    image_path = Path(config["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    with Image.open(image_path) as image:
        pixels = np.asarray(image.convert("L"), dtype=np.float64)
    probability = pixels / 255.0 if negate else (255.0 - pixels) / 255.0
    cells = np.full(pixels.shape, -1, dtype=np.int8)
    cells[probability < free_threshold] = 0
    cells[probability > occupied_threshold] = 100
    height, width = cells.shape
    return MapArtifact(resolution, origin, width, height, np.flipud(cells).ravel().tolist())


def split_timestamp(timestamp_ns: int) -> tuple[int, int]:
    """Convert exact acquisition nanoseconds to a ROS builtin_interfaces/Time."""
    seconds, nanoseconds = divmod(int(timestamp_ns), 10**9)
    if seconds < 0 or seconds > 2147483647:
        raise ValueError("Acquisition timestamp is outside the supported ROS Time range")
    return seconds, nanoseconds


def load_trajectory(csv_path: str | Path) -> list[HistoricalPose]:
    """Read historical poses; prefer integer nanoseconds over rounded seconds."""
    poses = []
    with Path(csv_path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("timestamp_ns"):
                timestamp_ns = int(row["timestamp_ns"])
            elif row.get("timestamp"):
                timestamp_ns = int(Decimal(row["timestamp"]) * 10**9)
            else:
                raise ValueError("Trajectory requires timestamp_ns or timestamp")
            split_timestamp(timestamp_ns)
            pose = HistoricalPose(timestamp_ns, float(row["x"]), float(row["y"]), float(row["yaw"]))
            if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
                raise ValueError("Trajectory poses must be finite")
            if poses and pose.timestamp_ns < poses[-1].timestamp_ns:
                raise ValueError("Trajectory timestamps must be nondecreasing")
            poses.append(pose)
    if not poses:
        raise ValueError("Trajectory contains no poses")
    return poses


def load_ascii_pcd(pcd_path: str | Path) -> np.ndarray:
    """Load scalar XYZ from an ASCII PCD without inventing unmeasured height."""
    header = {}
    with Path(pcd_path).open(encoding="ascii") as stream:
        for line in stream:
            tokens = line.strip().split()
            if not tokens or tokens[0].startswith("#"):
                continue
            key, values = tokens[0].upper(), tokens[1:]
            header[key] = values
            if key == "DATA":
                if values != ["ascii"]:
                    raise ValueError("Only ASCII PCD is supported by the ROS batch adapter")
                break
        else:
            raise ValueError("PCD is missing a DATA header")
        fields = header.get("FIELDS", [])
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        if len(counts) != len(fields) or any(count <= 0 for count in counts):
            raise ValueError("PCD COUNT does not match FIELDS")
        if any(name not in fields or counts[fields.index(name)] != 1 for name in ("x", "y", "z")):
            raise ValueError("PCD requires scalar x, y and z fields")
        rows = np.loadtxt(stream, dtype=np.float64, ndmin=2)
    expected_points = int(header["POINTS"][0])
    if rows.shape[0] != expected_points:
        raise ValueError("PCD POINTS disagrees with the number of data rows")
    if "WIDTH" in header and "HEIGHT" in header:
        if int(header["WIDTH"][0]) * int(header["HEIGHT"][0]) != expected_points:
            raise ValueError("PCD WIDTH * HEIGHT disagrees with POINTS")
    if rows.shape[1] != sum(counts):
        raise ValueError("PCD data columns disagree with COUNT")
    offsets = np.cumsum([0] + counts[:-1])
    xyz = rows[:, [offsets[fields.index(name)] for name in ("x", "y", "z")]]
    if not np.all(np.isfinite(xyz)):
        raise ValueError("PCD coordinates must be finite")
    points = np.asarray(xyz, dtype="<f4", order="C")
    if not np.all(np.isfinite(points)):
        raise ValueError("PCD coordinates exceed PointCloud2 float32 range")
    return points


def main(args=None):
    """Run offline mapping once, then publish its immutable historical results."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import OccupancyGrid, Path as RosPath
        from sensor_msgs.msg import PointCloud2, PointField
    except ImportError as exc:
        raise SystemExit("ROS 2 Python packages are required. Source the ROS installation and workspace first.") from exc

    class BatchMappingNode(Node):
        def __init__(self):
            super().__init__("batch_mapping")
            self.declare_parameter("cache_dir", "")
            self.declare_parameter("output_dir", "")
            self.declare_parameter("map_frame", "map")
            self.declare_parameter("process_cache", True)
            self.declare_parameter("lidar_revisits", False)
            self.declare_parameter("publish_period", 2.0)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.map_publisher = self.create_publisher(OccupancyGrid, "map", qos)
            self.path_publisher = self.create_publisher(RosPath, "trajectory", qos)
            self.points_publisher = self.create_publisher(PointCloud2, "map_points", qos)
            self.start_timer = self.create_timer(0.1, self.process)
            self.publish_timer = None

        def process(self):
            self.start_timer.cancel()
            output_value = self.get_parameter("output_dir").value
            cache_value = self.get_parameter("cache_dir").value
            frame = self.get_parameter("map_frame").value
            period = float(self.get_parameter("publish_period").value)
            if not output_value or not frame or not math.isfinite(period) or period <= 0:
                raise ValueError("Set output_dir, a nonempty map_frame, and a positive publish_period")
            output = Path(output_value).expanduser().resolve()
            if self.get_parameter("process_cache").value:
                if not cache_value:
                    raise ValueError("Set cache_dir to an extracted measurement cache")
                from indoor_mapping.pipeline import run_mapping
                cache = Path(cache_value).expanduser().resolve()
                self.get_logger().info(f"Processing recorded measurements from {cache}; this is batch processing.")
                run_mapping(cache, output, lidar_revisits=self.get_parameter("lidar_revisits").value)
            artifact = load_map_artifact(output / "map.yaml")
            poses = load_trajectory(output / "trajectory.csv")
            points = load_ascii_pcd(output / "map.pcd")
            seconds, nanoseconds = split_timestamp(poses[-1].timestamp_ns)
            self.grid = OccupancyGrid()
            self.grid.header.frame_id = frame
            self.grid.header.stamp.sec, self.grid.header.stamp.nanosec = seconds, nanoseconds
            self.grid.info.map_load_time = self.get_clock().now().to_msg()
            self.grid.info.resolution = artifact.resolution
            self.grid.info.width, self.grid.info.height = artifact.width, artifact.height
            self.grid.info.origin.position.x, self.grid.info.origin.position.y = artifact.origin[:2]
            self.grid.info.origin.orientation.z = math.sin(artifact.origin[2] / 2)
            self.grid.info.origin.orientation.w = math.cos(artifact.origin[2] / 2)
            self.grid.data = artifact.cells
            self.path = RosPath()
            self.path.header = self.grid.header
            for pose in poses:
                message = PoseStamped()
                message.header.frame_id = frame
                message.header.stamp.sec, message.header.stamp.nanosec = split_timestamp(pose.timestamp_ns)
                message.pose.position.x, message.pose.position.y = pose.x, pose.y
                message.pose.orientation.z, message.pose.orientation.w = math.sin(pose.yaw / 2), math.cos(pose.yaw / 2)
                self.path.poses.append(message)
            self.cloud = PointCloud2()
            self.cloud.header = self.grid.header
            self.cloud.height, self.cloud.width = 1, len(points)
            self.cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                                 for name, offset in (("x", 0), ("y", 4), ("z", 8))]
            self.cloud.is_bigendian = False
            self.cloud.point_step = 12
            self.cloud.row_step = 12 * len(points)
            self.cloud.is_dense = True
            self.cloud.data = points.tobytes(order="C")
            self.publish_results()
            self.publish_timer = self.create_timer(period, self.publish_results)
            self.get_logger().info(
                f"Published recorded map, {len(points)} map points, and {len(poses)} historical poses. "
                "Acquisition timestamps are preserved. No live TF or vehicle-control output is published."
            )

        def publish_results(self):
            self.map_publisher.publish(self.grid)
            self.path_publisher.publish(self.path)
            self.points_publisher.publish(self.cloud)

    rclpy.init(args=args)
    node = None
    try:
        node = BatchMappingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"Batch mapping failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
