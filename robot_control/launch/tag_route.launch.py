"""Start rectified vision, tag detection, the motor bridge and the route."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_vision_camera'), 'launch',
                'robot_vision_camera.launch.py'])),
            launch_arguments={'image_rectify': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('apriltag_ros'), 'launch', 'apriltag_launch.py']))),
        Node(package='robot_control', executable='motor_control', output='screen'),
        Node(package='robot_control', executable='tag_route', output='screen'),
    ])
