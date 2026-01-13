#!/usr/bin/env python3
"""
Manual Control Node for Ackermann Robot

This node allows manual control of the robot by publishing commands to a topic.
Supports forward/backward movement and steering.

Usage:
  ros2 topic pub /manual_control std_msgs/msg/String "data: 'forward'"
  ros2 topic pub /manual_control std_msgs/msg/String "data: 'backward'"
  ros2 topic pub /manual_control std_msgs/msg/String "data: 'stop'"
  ros2 topic pub /manual_control std_msgs/msg/String "data: 'left'"
  ros2 topic pub /manual_control std_msgs/msg/String "data: 'right'"

Author: Gaál Dominik
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class ManualControlNode(Node):
    """
    Manual control node that accepts string commands and publishes Twist messages.
    """
    
    def __init__(self):
        super().__init__('manual_control_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)  # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s (increased for better turning)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('control_topic', 'manual_control')
        
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Subscriber for manual control commands
        self.control_sub = self.create_subscription(
            String,
            control_topic,
            self.control_callback,
            10
        )
        
        # State
        self.current_cmd = Twist()
        
        # Status publisher
        self.status_pub = self.create_publisher(String, 'manual_control_status', 10)
        
        # Timer to keep publishing current command (for continuous movement)
        self.timer = self.create_timer(0.1, self.publish_current_command)  # 10 Hz
        
        self.get_logger().info('Manual Control Node initialized')
        self.get_logger().info(f'Listening on topic: {control_topic}')
        self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
        self.get_logger().info('Commands: forward, backward, left, right, stop')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s, Angular speed: {self.angular_speed} rad/s')
    
    def control_callback(self, msg: String):
        """Process manual control commands."""
        command = msg.data.lower().strip()
        
        cmd = Twist()
        
        if command == 'forward' or command == 'előre' or command == 'f':
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('Command: FORWARD')
            
        elif command == 'backward' or command == 'hátra' or command == 'b' or command == 'back':
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0
            self.get_logger().info('Command: BACKWARD')
            
        elif command == 'left' or command == 'bal' or command == 'l':
            # For Ackermann: need forward motion + angular velocity to turn
            cmd.linear.x = self.linear_speed * 0.7  # Forward motion while turning
            cmd.angular.z = self.angular_speed  # Positive = turn left
            self.get_logger().info(f'Command: LEFT (linear: {cmd.linear.x:.2f}, angular: {cmd.angular.z:.2f})')
            
        elif command == 'right' or command == 'jobbra' or command == 'r':
            # For Ackermann: need forward motion + angular velocity to turn
            cmd.linear.x = self.linear_speed * 0.7  # Forward motion while turning
            cmd.angular.z = -self.angular_speed  # Negative = turn right
            self.get_logger().info(f'Command: RIGHT (linear: {cmd.linear.x:.2f}, angular: {cmd.angular.z:.2f})')
            
        elif command == 'stop' or command == 'megáll' or command == 's':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Command: STOP')
            
        elif command.startswith('speed:'):
            # Custom speed command: "speed:0.3" or "speed:0.5,0.3"
            try:
                parts = command.split(':')[1].split(',')
                if len(parts) == 1:
                    self.linear_speed = float(parts[0])
                    self.get_logger().info(f'Linear speed set to: {self.linear_speed} m/s')
                elif len(parts) == 2:
                    self.linear_speed = float(parts[0])
                    self.angular_speed = float(parts[1])
                    self.get_logger().info(f'Speed set to: linear={self.linear_speed} m/s, angular={self.angular_speed} rad/s')
            except ValueError:
                self.get_logger().warn(f'Invalid speed command: {command}')
            return
            
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            self.get_logger().info('Valid commands: forward, backward, left, right, stop, speed:<value>')
            return
        
        # Update current command
        self.current_cmd = cmd
        
        # Publish immediately
        self.cmd_vel_pub.publish(cmd)
        
        # Publish status
        status = String()
        status.data = f"Manual Control: {command.upper()}\nLinear: {cmd.linear.x:.2f} m/s\nAngular: {cmd.angular.z:.2f} rad/s"
        self.status_pub.publish(status)
    
    def publish_current_command(self):
        """Continuously publish current command to maintain movement."""
        # Only publish if there's a non-zero command
        if abs(self.current_cmd.linear.x) > 0.001 or abs(self.current_cmd.angular.z) > 0.001:
            self.cmd_vel_pub.publish(self.current_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down manual control node...')
        # Publish stop command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
