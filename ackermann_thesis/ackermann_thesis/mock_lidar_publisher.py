#!/usr/bin/env python3
"""
Mock LiDAR Publisher for Testing

This node publishes fake LaserScan data so you can test your navigation node
without needing Gazebo running. Useful for development and testing in WSL.

Author: Gaál Dominik
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np


class MockLidarPublisher(Node):
    """
    Publishes mock LaserScan data and TF transforms for testing.
    """
    
    def __init__(self):
        super().__init__('mock_lidar_publisher')
        
        # Parameters
        self.declare_parameter('scan_rate', 10.0)  # Hz
        self.declare_parameter('range_min', 0.1)  # meters
        self.declare_parameter('range_max', 10.0)  # meters
        self.declare_parameter('angle_min', -math.pi)  # radians
        self.declare_parameter('angle_max', math.pi)  # radians
        self.declare_parameter('angle_increment', math.pi / 180.0)  # 1 degree
        self.declare_parameter('obstacle_distance', 2.0)  # meters - distance to fake obstacle
        
        scan_rate = self.get_parameter('scan_rate').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self.obstacle_distance = self.get_parameter('obstacle_distance').get_parameter_value().double_value
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing scans
        timer_period = 1.0 / scan_rate
        self.timer = self.create_timer(timer_period, self.publish_scan)
        self.tf_timer = self.create_timer(0.1, self.publish_tf)  # 10 Hz for TF
        
        # State
        self.scan_count = 0
        
        self.get_logger().info('Mock LiDAR Publisher started')
        self.get_logger().info(f'Publishing /scan at {scan_rate} Hz')
        self.get_logger().info(f'Fake obstacle at {self.obstacle_distance} m in front')
    
    def publish_scan(self):
        """Publish a mock LaserScan message."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 10.0  # 10 Hz
        
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Generate ranges - create a simple obstacle pattern
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        scan.ranges = []
        scan.intensities = []
        
        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment
            
            # Create a fake obstacle in front (within ±30 degrees)
            if abs(angle) < math.radians(30):
                # Obstacle in front
                range_val = self.obstacle_distance + 0.1 * math.sin(self.scan_count * 0.1)  # Slight variation
            elif abs(angle) < math.radians(60):
                # Gradual transition
                range_val = self.obstacle_distance + (abs(angle) - math.radians(30)) * 5.0
            else:
                # Clear on sides
                range_val = self.range_max
            
            # Add some noise
            range_val += np.random.normal(0, 0.02)  # 2cm noise
            
            # Clamp to valid range
            range_val = max(self.range_min, min(self.range_max, range_val))
            
            scan.ranges.append(float(range_val))
            scan.intensities.append(1.0)
        
        self.scan_pub.publish(scan)
        self.scan_count += 1
        
        if self.scan_count % 100 == 0:
            self.get_logger().info(f'Published {self.scan_count} scans')
    
    def publish_tf(self):
        """Publish TF transforms."""
        # Publish odom_combined -> base_link transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom_combined'
        t.child_frame_id = 'base_link'
        
        # Static transform (robot at origin)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish base_link -> laser transform
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'laser'
        
        # Laser is slightly forward and up from base_link
        t2.transform.translation.x = 0.26  # Forward
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.228  # Up
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.000796326710733
        t2.transform.rotation.w = 0.999999682932
        
        self.tf_broadcaster.sendTransform(t2)


def main(args=None):
    rclpy.init(args=args)
    node = MockLidarPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down mock LiDAR publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
