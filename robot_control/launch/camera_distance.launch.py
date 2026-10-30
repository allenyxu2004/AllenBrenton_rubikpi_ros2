"""Start camera, Foxglove, AprilTag detection, and distance reporting."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('distance_reporting', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_vision_camera'), 'launch',
                'robot_vision_camera.launch.py'])),
            launch_arguments={'image_rectify': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('apriltag_ros'), 'launch',
                'apriltag_launch.py']))),
        Node(package='robot_control', executable='tag_distance', output='screen',
             condition=IfCondition(LaunchConfiguration('distance_reporting'))),
    ])
