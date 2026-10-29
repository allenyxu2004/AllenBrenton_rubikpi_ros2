"""Start vision, motor bridge, and the explicit route controller."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_control'), 'launch', 'camera_distance.launch.py']))),
        Node(package='robot_control', executable='motor_control', output='screen'),
        Node(package='robot_control', executable='tag_route', output='screen'),
    ])
