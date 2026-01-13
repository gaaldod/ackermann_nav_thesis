#!/usr/bin/env python3
"""
Test Navigation Node for Ackermann Steering Robot

This node implements a basic navigation system with:
- Sensor integration (LiDAR)
- Local board model (occupancy grid)
- Simple obstacle avoidance
- Ackermann-compatible control

Author: Gaál Dominik
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String, ColorRGBA
import tf2_ros
from tf2_ros import TransformException


class TestNavNode(Node):
    """
    Test navigation node implementing basic obstacle avoidance for Ackermann robot.
    """
    
    def __init__(self):
        super().__init__('test_nav_node')
        
        # Robot parameters (Ackermann-specific)
        self.declare_parameter('wheelbase', 0.3187)  # meters
        self.declare_parameter('car_length', 0.445)  # meters
        self.declare_parameter('max_velocity', 1.0)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('safety_distance', 0.5)  # meters
        self.declare_parameter('lookahead_distance', 2.0)  # meters
        
        # Board model parameters (local occupancy grid)
        self.declare_parameter('grid_size', 20)  # 20x20 grid
        self.declare_parameter('grid_resolution', 0.1)  # 10cm per cell
        self.declare_parameter('grid_range', 2.0)  # 2m range
        
        # Frame names
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('odom_frame', 'odom_combined')
        
        # Control topic (can be cmd_vel or roboworks/cmd_vel)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        
        # Load parameters
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.car_length = self.get_parameter('car_length').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        
        self.grid_size = self.get_parameter('grid_size').get_parameter_value().integer_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.grid_range = self.get_parameter('grid_range').get_parameter_value().double_value
        
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.laser_frame = self.get_parameter('laser_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        
        # Initialize board model (occupancy grid)
        # Grid is centered on robot, positive X is forward
        self.grid_size_px = self.grid_size
        self.grid_center = self.grid_size_px // 2
        self.board_model = np.zeros((self.grid_size_px, self.grid_size_px), dtype=np.float32)
        
        # State variables
        self.latest_scan = None
        self.robot_pose = None  # Will store (x, y, theta) in odom frame
        
        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_marker_pub = self.create_publisher(MarkerArray, '/debug_marker', 10)
        self.state_pub = self.create_publisher(String, 'control_state', 10)
        
        # Initialize subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Initialize TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Control timer (runs at ~10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Debug visualization timer (runs at ~5 Hz)
        self.viz_timer = self.create_timer(0.2, self.publish_debug_markers)
        
        self.get_logger().info('Test Navigation Node initialized')
        self.get_logger().info(f'Grid: {self.grid_size_px}x{self.grid_size_px}, '
                              f'Resolution: {self.grid_resolution}m, Range: {self.grid_range}m')
    
    def scan_callback(self, msg: LaserScan):
        """Process incoming LiDAR scan and update board model."""
        self.latest_scan = msg
        self.update_board_model(msg)
    
    def update_board_model(self, scan: LaserScan):
        """
        Update the local occupancy grid (board model) from LiDAR data.
        
        The grid is in the robot's base_link frame:
        - X axis: forward (positive)
        - Y axis: left (positive)
        - Origin: center of grid (robot position)
        """
        # Clear previous grid (decay factor for smoother updates)
        self.board_model *= 0.5
        
        # Convert scan to grid coordinates
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        for i, range_val in enumerate(scan.ranges):
            # Skip invalid readings
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            
            # Limit range
            if range_val > scan.range_max or range_val < scan.range_min:
                continue
            
            # Calculate angle
            angle = angle_min + i * angle_increment
            
            # Convert to Cartesian (in laser frame: X forward, Y left)
            x_laser = range_val * math.cos(angle)
            y_laser = range_val * math.sin(angle)
            
            # Transform to grid coordinates (assuming laser is close to base_link)
            # For simplicity, we'll use laser frame directly
            # In a full implementation, you'd transform via TF2
            x_grid = x_laser / self.grid_resolution
            y_grid = y_laser / self.grid_resolution
            
            # Convert to grid indices (center is robot position)
            grid_x = int(self.grid_center + x_grid)
            grid_y = int(self.grid_center + y_grid)
            
            # Mark obstacle cell
            if 0 <= grid_x < self.grid_size_px and 0 <= grid_y < self.grid_size_px:
                # Mark obstacle with high value
                self.board_model[grid_y, grid_x] = 1.0
                
                # Also mark nearby cells (robot size inflation)
                inflation_radius = int(self.car_length / self.grid_resolution / 2)
                for dx in range(-inflation_radius, inflation_radius + 1):
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        gx = grid_x + dx
                        gy = grid_y + dy
                        if 0 <= gx < self.grid_size_px and 0 <= gy < self.grid_size_px:
                            dist = math.sqrt(dx*dx + dy*dy)
                            if dist <= inflation_radius:
                                self.board_model[gy, gx] = max(
                                    self.board_model[gy, gx],
                                    0.8 * (1.0 - dist / inflation_radius)
                                )
    
    def get_robot_pose(self):
        """Get current robot pose from TF2."""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # Extract position and orientation
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
            return (x, y, yaw)
        except TransformException as ex:
            self.get_logger().warn(f'Could not get robot pose: {ex}')
            return None
    
    def check_path_clear(self, forward_distance: float, angle_range: float = math.pi / 6):
        """
        Check if path is clear in front of robot.
        
        Args:
            forward_distance: How far ahead to check (meters)
            angle_range: Angular range to check (radians, symmetric around forward)
        
        Returns:
            (is_clear, min_distance)
        """
        if self.latest_scan is None:
            return False, 0.0
        
        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        min_distance = float('inf')
        
        # Check scan points in the forward sector
        for i, range_val in enumerate(scan.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            
            angle = angle_min + i * angle_increment
            
            # Check if angle is in forward sector
            if abs(angle) <= angle_range:
                # Project forward distance
                forward_projection = range_val * math.cos(angle)
                
                if forward_projection > 0 and forward_projection < forward_distance:
                    min_distance = min(min_distance, forward_projection)
        
        is_clear = min_distance > self.safety_distance
        
        return is_clear, min_distance if min_distance != float('inf') else scan.range_max
    
    def simple_obstacle_avoidance(self):
        """
        Simple obstacle avoidance algorithm.
        
        Returns:
            (linear_velocity, angular_velocity)
        """
        # Check forward path
        is_clear, min_dist = self.check_path_clear(self.lookahead_distance)
        
        if not is_clear:
            # Obstacle detected - stop and turn
            # Check left and right sectors
            left_clear, left_dist = self.check_path_clear(
                self.lookahead_distance * 0.7,
                math.pi / 4
            )
            right_clear, right_dist = self.check_path_clear(
                self.lookahead_distance * 0.7,
                math.pi / 4
            )
            
            # Choose direction with more clearance
            if left_dist > right_dist:
                # Turn left
                linear_vel = 0.2  # Slow forward
                angular_vel = self.max_angular_velocity * 0.5
            else:
                # Turn right
                linear_vel = 0.2  # Slow forward
                angular_vel = -self.max_angular_velocity * 0.5
            
            # If too close, stop completely
            if min_dist < self.safety_distance * 0.5:
                linear_vel = 0.0
            
            return linear_vel, angular_vel
        else:
            # Path clear - drive forward
            # Adjust speed based on distance to nearest obstacle
            speed_factor = min(1.0, (min_dist - self.safety_distance) / self.lookahead_distance)
            linear_vel = self.max_velocity * speed_factor
            angular_vel = 0.0
            
            return linear_vel, angular_vel
    
    def control_loop(self):
        """Main control loop - runs periodically."""
        if self.latest_scan is None:
            return
        
        # Get robot pose (for future use)
        self.robot_pose = self.get_robot_pose()
        
        # Compute control commands
        linear_vel, angular_vel = self.simple_obstacle_avoidance()
        
        # Publish Twist command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        # Publish state info
        state_msg = String()
        is_clear, min_dist = self.check_path_clear(self.lookahead_distance)
        state_msg.data = (
            f"Test Navigation Node\n"
            f"Linear Vel: {linear_vel:.2f} m/s\n"
            f"Angular Vel: {angular_vel:.2f} rad/s\n"
            f"Path Clear: {is_clear}\n"
            f"Min Distance: {min_dist:.2f} m\n"
            f"Board Model: {self.grid_size_px}x{self.grid_size_px} grid"
        )
        self.state_pub.publish(state_msg)
    
    def publish_debug_markers(self):
        """Publish debug visualization markers for RViz."""
        if self.latest_scan is None:
            return
        
        marker_array = MarkerArray()
        
        # Create marker for board model visualization
        marker = Marker()
        marker.header.frame_id = self.laser_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "board_model"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.grid_resolution * 0.8
        marker.scale.y = self.grid_resolution * 0.8
        marker.scale.z = 0.05
        
        # Add occupied cells as cubes
        from geometry_msgs.msg import Point
        
        for y in range(self.grid_size_px):
            for x in range(self.grid_size_px):
                occupancy = self.board_model[y, x]
                if occupancy > 0.1:  # Threshold for visualization
                    # Convert grid coordinates to laser frame
                    x_laser = (x - self.grid_center) * self.grid_resolution
                    y_laser = (y - self.grid_center) * self.grid_resolution
                    
                    point = Point()
                    point.x = x_laser
                    point.y = y_laser
                    point.z = 0.0
                    marker.points.append(point)
                    
                    # Color based on occupancy
                    color = ColorRGBA()
                    color.r = 1.0
                    color.g = 0.0
                    color.b = 0.0
                    color.a = occupancy
                    marker.colors.append(color)
        
        marker_array.markers.append(marker)
        
        # Create marker for safety distance circle
        safety_marker = Marker()
        safety_marker.header.frame_id = self.laser_frame
        safety_marker.header.stamp = self.get_clock().now().to_msg()
        safety_marker.ns = "safety_zone"
        safety_marker.id = 1
        safety_marker.type = Marker.CYLINDER
        safety_marker.action = Marker.ADD
        safety_marker.pose.position.x = self.safety_distance
        safety_marker.pose.position.y = 0.0
        safety_marker.pose.position.z = 0.0
        safety_marker.pose.orientation.w = 1.0
        safety_marker.scale.x = self.safety_distance * 2
        safety_marker.scale.y = self.safety_distance * 2
        safety_marker.scale.z = 0.1
        safety_marker.color.r = 1.0
        safety_marker.color.g = 1.0
        safety_marker.color.b = 0.0
        safety_marker.color.a = 0.3
        marker_array.markers.append(safety_marker)
        
        self.debug_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TestNavNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down test navigation node...')
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
