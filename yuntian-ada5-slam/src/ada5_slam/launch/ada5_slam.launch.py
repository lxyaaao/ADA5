"""Start the Ada5 SLAM node. Example with a recording:

  ros2 launch ada5_slam ada5_slam.launch.py output_dir:=/tmp/ada5_live
  ros2 bag play <bag> --topics /scan /camera/camera/color/image_raw/compressed \
      /camera/camera/color/camera_info /tf_static
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='ada5_slam_out'),
        DeclareLaunchArgument('use_camera_prior', default_value='true'),
        DeclareLaunchArgument('process_every', default_value='3'),
        Node(package='ada5_slam', executable='slam_node', name='ada5_slam', output='screen',
             parameters=[{
                 'output_dir': LaunchConfiguration('output_dir'),
                 'use_camera_prior': ParameterValue(LaunchConfiguration('use_camera_prior'), value_type=bool),
                 'process_every': ParameterValue(LaunchConfiguration('process_every'), value_type=int),
             }]),
    ])
