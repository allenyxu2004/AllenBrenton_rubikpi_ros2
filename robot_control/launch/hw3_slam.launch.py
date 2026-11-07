"""HW3 vision, unrestricted tag detection, bridge and optional autonomous SLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_slam(context):
    """Load YAML first; override only arguments explicitly supplied by the user."""
    overrides = {}
    for name, value_type in [('drive', bool), ('shape', str),
                             ('side_m', float), ('laps', int)]:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = ParameterValue(value, value_type=value_type)
    return [Node(package='robot_control', executable='hw3_slam', output='screen',
                 parameters=[LaunchConfiguration('config'), overrides])]


def generate_launch_description():
    share = FindPackageShare('robot_control')
    return LaunchDescription([
        # Empty means use the YAML value, rather than overriding it with a
        # second set of launch defaults.
        DeclareLaunchArgument('drive', default_value='', description='Override YAML drive'),
        DeclareLaunchArgument('shape', default_value='', description='Override YAML shape'),
        DeclareLaunchArgument('side_m', default_value='', description='Override YAML side_m'),
        DeclareLaunchArgument('laps', default_value='', description='Override YAML laps'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            share, 'config', 'hw3_slam.yaml'])),
        DeclareLaunchArgument('tags_config', default_value=PathJoinSubstitution([
            share, 'config', 'hw3_tags.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_vision_camera'), 'launch',
            'robot_vision_camera.launch.py'])),
            launch_arguments={'image_rectify': 'true'}.items()),
        Node(package='apriltag_ros', executable='apriltag_node',
             name='apriltag_node', output='screen',
             parameters=[LaunchConfiguration('tags_config')],
             remappings=[('image_rect', '/camera/image_raw'),
                         ('camera_info', '/camera/camera_info')]),
        Node(package='robot_control', executable='motor_control', output='screen'),
        OpaqueFunction(function=launch_slam),
    ])
