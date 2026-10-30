"""Start vision, motor bridge, and the explicit route controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('distance_reporting', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_control'), 'launch', 'camera_distance.launch.py'])),
            launch_arguments={
                'distance_reporting': LaunchConfiguration('distance_reporting')}.items()),
        Node(package='robot_control', executable='motor_control', output='screen'),
        Node(package='robot_control', executable='tag_route', output='screen'),
    ])
