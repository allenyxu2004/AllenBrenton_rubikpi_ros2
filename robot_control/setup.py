from setuptools import find_packages, setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_teleop_launch.py',
            'launch/tag_route.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/waypoints.txt']),
    ],
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
        'pynput>=1.7.0',
        'numpy>=1.20.0',
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='chl235@ucsd.edu',
    description='Robot motor controller with keyboard input for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_route = robot_control.tag_route:main',
            'motor_control = robot_control.motor_control:main',
            'keyboard_control = robot_control.keyboard_control:main',
            'velocity_control = robot_control.velocity_control:main',
            'move_forward = robot_control.move_forward:main',
        ],
    },
)
