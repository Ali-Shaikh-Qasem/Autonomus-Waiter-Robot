#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ReactiveUltrasonicAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_ultrasonic_avoidance')
        
        # Subscribe to front ultrasonic sensors
        self.front_up_sub = self.create_subscription(
            LaserScan, '/ultrasonic_scan_front_up',
            self.front_up_callback, 10)
        self.front_down_sub = self.create_subscription(
            LaserScan, '/ultrasonic_scan_front_down',
            self.front_down_callback, 10)
        
        # Subscribe to left and right sensors for deciding turn direction
        self.left_up_sub = self.create_subscription(
            LaserScan, '/ultrasonic_scan_left_up',
            self.left_callback, 10)
        self.left_down_sub = self.create_subscription(
            LaserScan, '/ultrasonic_scan_left_down',
            self.left_callback, 10)
        self.right_up_sub = self.create_subscription(
            LaserScan, '/ultrasonic_scan_right_up',
            self.right_callback, 10)
        self.right_down_sub = self.create_subscription(
            LaserScan, '/ultrasonic_scan_right_down',
            self.right_callback, 10)
        
        # Subscribe to navigation cmd_vel from Nav2
        self.nav_cmd_sub = self.create_subscription(
            Twist, '/nav_cmd_vel', 
            self.nav_cmd_callback, 10)
        
        # Publisher for robot cmd_vel (final output to robot)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Configuration - tuned for fast, decisive action
        self.obstacle_threshold = 1.0  # meters - react earlier
        self.safe_distance = 1.2  # meters - require more clearance before resuming
        self.turn_speed = 0.8  # rad/s - faster turning
        
        # Avoidance state
        self.avoiding = False
        self.turn_direction = 1  # 1 for left (counter-clockwise), -1 for right
        self.clear_counter = 0  # Count how many cycles path has been clear
        self.required_clear_cycles = 3  # Need path clear for 3 cycles (0.15s) before resuming
        self.turn_time = 0.0  # Track how long we've been turning
        self.min_turn_time = 0.5  # Minimum turn duration in seconds
        
        # Timer for publishing - faster update rate
        self.timer = self.create_timer(0.05, self.control_loop)
        self.last_time = self.get_clock().now()
        
        # Store latest nav command
        self.nav_cmd = Twist()
        
        self.get_logger().info('Reactive Ultrasonic Avoidance Started')
        self.get_logger().info(f'Obstacle threshold: {self.obstacle_threshold}m, Safe distance: {self.safe_distance}m')
    
    def front_up_callback(self, msg):
        """Get minimum distance from front upper sensor"""
        min_dist = self.safe_distance
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)
        self.front_distance = min(self.front_distance, min_dist)
    
    def front_down_callback(self, msg):
        """Get minimum distance from front lower sensor"""
        min_dist = self.safe_distance
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)
        self.front_distance = min(self.front_distance, min_dist)
    
    def left_callback(self, msg):
        """Get minimum distance from left sensors"""
        min_dist = self.safe_distance
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)
        self.left_distance = min(self.left_distance, min_dist)
    
    def right_callback(self, msg):
        """Get minimum distance from right sensors"""
        min_dist = self.safe_distance
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)
        self.right_distance = min(self.right_distance, min_dist)
    
    def nav_cmd_callback(self, msg):
        """Store the navigation command"""
        self.nav_cmd = msg
    
    def control_loop(self):
        """Main control loop - decide whether to avoid or follow navigation"""
        cmd = Twist()
        
        # Calculate delta time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Check if obstacle in front
        if self.front_distance < self.obstacle_threshold:
            if not self.avoiding:
                # Start avoidance - decide turn direction ONCE
                self.avoiding = True
                self.clear_counter = 0
                self.turn_time = 0.0
                
                # Turn towards the side with more space
                # Add bias: if distances similar, prefer left (convention)
                left_space = self.left_distance
                right_space = self.right_distance
                
                if left_space > right_space * 1.2:  # Left has 20% more space
                    self.turn_direction = 1  # Turn left
                    self.get_logger().info(f'Obstacle at {self.front_distance:.2f}m! Turning LEFT (L:{left_space:.2f}m R:{right_space:.2f}m)')
                elif right_space > left_space * 1.2:  # Right has 20% more space
                    self.turn_direction = -1  # Turn right
                    self.get_logger().info(f'Obstacle at {self.front_distance:.2f}m! Turning RIGHT (L:{left_space:.2f}m R:{right_space:.2f}m)')
                else:
                    # Similar space - prefer left
                    self.turn_direction = 1
                    self.get_logger().info(f'Obstacle at {self.front_distance:.2f}m! Similar space, turning LEFT')
            else:
                # Continue tracking turn time
                self.turn_time += dt
                # Reset clear counter if obstacle still there
                self.clear_counter = 0
        
        # Avoidance behavior
        if self.avoiding:
            # Check if path is clear AND we've turned long enough
            if self.front_distance >= self.safe_distance and self.turn_time >= self.min_turn_time:
                self.clear_counter += 1
                
                # Only resume if path has been clear for required cycles
                if self.clear_counter >= self.required_clear_cycles:
                    self.avoiding = False
                    self.clear_counter = 0
                    self.turn_time = 0.0
                    self.get_logger().info(f'Path CLEAR ({self.front_distance:.2f}m)! Resuming navigation')
                    # Follow navigation command
                    cmd = self.nav_cmd
                else:
                    # Path clear but waiting for stability - keep turning slowly
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.turn_direction * self.turn_speed * 0.3  # Slow turn while verifying
            else:
                # Path blocked or haven't turned enough - keep turning
                self.clear_counter = 0
                self.turn_time += dt
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = self.turn_direction * self.turn_speed
        else:
            # No obstacle - follow navigation command
            cmd = self.nav_cmd
        
        # Publish command
        self.cmd_pub.publish(cmd)
        
        # Reset distances for next cycle
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveUltrasonicAvoidance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
