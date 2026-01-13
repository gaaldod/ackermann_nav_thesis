from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ackermann_thesis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Gaál Dominik',
    maintainer_email='gaaldod@users.noreply.github.com',
    description='ROS 2 navigation system for Ackermann steering robot - Thesis project',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_nav_node = ackermann_thesis.test_nav_node:main',
            'mock_lidar_publisher = ackermann_thesis.mock_lidar_publisher:main',
            'manual_control_node = ackermann_thesis.manual_control_node:main',
        ],
    },
)
