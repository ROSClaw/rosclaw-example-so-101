import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so101_follower_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcodotio',
    maintainer_email='marcodotio@example.com',
    description='Standard ROS2 bringup and example clients for the SO-101 follower arm.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_trajectory_example = so101_follower_control.joint_trajectory_example:main',
            'gripper_command_example = so101_follower_control.gripper_command_example:main',
            'joint_state_echo = so101_follower_control.joint_state_echo:main',
            'cartesian_goal_node = so101_follower_control.cartesian_goal_node:main',
            'agent_bridge_node = so101_follower_control.agent_bridge_node:main',
        ],
    },
)
