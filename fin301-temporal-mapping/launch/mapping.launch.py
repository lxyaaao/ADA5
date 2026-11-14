"""Batch processing of an extracted recording; output stays historical."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("indoor_mapping")) / "config" / "mapping.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("cache_dir", default_value="", description="Absolute extracted measurement cache path"),
        DeclareLaunchArgument("output_dir", description="Absolute output artifact directory"),
        DeclareLaunchArgument("process_cache", default_value="true", description="False publishes existing artifacts only"),
        DeclareLaunchArgument("lidar_revisits", default_value="false", description="Enable the fin301 evaluated LiDAR revisit configuration"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        Node(
            package="indoor_mapping", executable="batch_mapping", name="batch_mapping", output="screen",
            parameters=[str(config), {
                "cache_dir": ParameterValue(LaunchConfiguration("cache_dir"), value_type=str),
                "output_dir": ParameterValue(LaunchConfiguration("output_dir"), value_type=str),
                "process_cache": ParameterValue(LaunchConfiguration("process_cache"), value_type=bool),
                "lidar_revisits": ParameterValue(LaunchConfiguration("lidar_revisits"), value_type=bool),
                "map_frame": ParameterValue(LaunchConfiguration("map_frame"), value_type=str),
            }],
        ),
    ])
