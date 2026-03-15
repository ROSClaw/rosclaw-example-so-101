import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so101_pose_registry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='marcodotio',
    maintainer_email='marcodotio@example.com',
    description='Named pose registry and CLI utilities for the SO-101 follower arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_registry_node = so101_pose_registry.pose_registry_node:main',
            'so101_pose_cli = so101_pose_registry.pose_cli:main',
        ],
    },
)
