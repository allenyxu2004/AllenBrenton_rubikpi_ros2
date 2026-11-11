from pathlib import Path
from runpy import run_path

from setuptools import find_packages, setup

package_name = 'robot_control'
package_version = run_path(str(
    Path(__file__).parent / package_name / 'version.py'))['__version__']

setup(
    name=package_name,
    version=package_version,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_teleop_launch.py',
            'launch/tag_route.launch.py',
            'launch/hw3_slam.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/waypoints.txt', 'config/hw3_slam.yaml', 'config/hw3_tags.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
        'pynput>=1.7.0',
        'numpy>=1.20.0',
        'PyYAML>=5.3',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='chl235@ucsd.edu',
    description='Robot motor controller with keyboard input for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tag_route = robot_control.tag_route:main',
            'hw3_slam = robot_control.slam_node:main',
            'hw3_slam_plot = robot_control.slam.evaluate:main',
            'camera_noise = robot_control.slam.camera_noise:main',
            'translation_noise = robot_control.slam.translation_noise:main',
            'motor_control = robot_control.motor_control:main',
            'keyboard_control = robot_control.keyboard_control:main',
            'velocity_control = robot_control.velocity_control:main',
            'move_forward = robot_control.move_forward:main',
        ],
    },
)
