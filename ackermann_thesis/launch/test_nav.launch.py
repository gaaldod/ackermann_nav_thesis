"""
Launch file for test navigation node.

This launch file starts the test navigation node.
Note: The simulator (robotverseny_gazebo24) should be started separately.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    test_nav_node = Node(
        package='ackermann_thesis',
        executable='test_nav_node',
        name='test_nav_node',
        output='screen',
        parameters=[{
            'wheelbase': 0.3187,  # meters
            'car_length': 0.445,  # meters
            'max_velocity': 1.0,  # m/s
            'max_angular_velocity': 1.0,  # rad/s
            'safety_distance': 0.5,  # meters
            'lookahead_distance': 2.0,  # meters
            'grid_size': 20,  # 20x20 grid
            'grid_resolution': 0.1,  # 10cm per cell
            'grid_range': 2.0,  # 2m range
            'base_frame': 'base_link',
            'laser_frame': 'laser',
            'odom_frame': 'odom_combined',
            'cmd_vel_topic': 'cmd_vel',  # Change to 'roboworks/cmd_vel' if needed
        }]
    )

    return LaunchDescription([
        LogInfo(msg='Starting test navigation node...'),
        LogInfo(msg='Note: This node requires /scan topic and TF transforms.'),
        LogInfo(msg='Make sure the simulator is running before starting this node.'),
        LogInfo(msg='The simulator should provide /scan topic and TF transforms.'),
        test_nav_node,
    ])
