"""Launch the camera, detector, and known-map HW4 controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from robot_control.hw4.course import read_course


def launch_course(context):
    course_path = LaunchConfiguration('course').perform(context)
    objective = LaunchConfiguration('objective').perform(context)
    course = read_course(course_path)
    if objective not in ('safety', 'distance'):
        raise ValueError('objective must be safety or distance')
    drive = LaunchConfiguration('drive')
    drive_value = drive.perform(context)
    if drive_value not in ('true', 'false'):
        raise ValueError('drive must be true or false')

    share = FindPackageShare('robot_control')
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_vision_camera'), 'launch', 'robot_vision_camera.launch.py'
        ])), launch_arguments={'image_rectify': 'true'}.items()),
        Node(package='apriltag_ros', executable='apriltag_node', name='apriltag_node',
             output='screen', parameters=[
                PathJoinSubstitution([share, 'config', 'hw3_tags.yaml']),
                {'size': course.tag_size},
            ], remappings=[('image_rect', '/camera/image_raw'),
                           ('camera_info', '/camera/camera_info')]),
        Node(package='robot_control', executable='hw4_navigation', output='screen',
             parameters=[LaunchConfiguration('controller'),
                         {'course': course_path, 'objective': objective,
                          'drive': ParameterValue(drive, value_type=bool)}]),
    ]
    if drive_value == 'true':
        actions.append(Node(package='robot_control', executable='motor_control', output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('course', default_value=PathJoinSubstitution([
            FindPackageShare('robot_control'), 'config', 'hw4_course.yaml'])),
        DeclareLaunchArgument('controller', default_value=PathJoinSubstitution([
            FindPackageShare('robot_control'), 'config', 'hw4_navigation.yaml'])),
        DeclareLaunchArgument('objective', default_value='safety'),
        DeclareLaunchArgument('drive', default_value='false'),
        OpaqueFunction(function=launch_course),
    ])
