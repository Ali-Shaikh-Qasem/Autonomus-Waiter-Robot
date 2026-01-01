#!/usr/bin/env python3
"""
Ultrasonic Detour Supervisor - Simple Obstacle Avoidance

Simple algorithm:
1. Detect obstacle at 0.5m -> STOP
2. Turn 90° (left or right based on which has more space)
3. Move 2m forward
4. Resume original goal

Author: ROS2 Navigation Assistant
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
from enum import Enum
from collections import deque


class State(Enum):
    IDLE = 0
    NAVIGATING = 1
    AVOID_STOP = 2
    AVOID_TURN = 3
    AVOID_STEP = 4
    RESUME_GOAL = 5


class UltrasonicDetourSupervisor(Node):
    def __init__(self):
        super().__init__('ultrasonic_detour_supervisor')
        
        # Parameters
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('turn_angle_deg', 70.0)
        self.declare_parameter('step_distance', 1.0)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('filter_window', 5)
        
        self.stop_dist = self.get_parameter('stop_distance').value
        self.turn_angle_deg = self.get_parameter('turn_angle_deg').value
        self.step_dist = self.get_parameter('step_distance').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.filter_window = self.get_parameter('filter_window').value
        
        # State
        self.state = State.IDLE
        self.original_goal = None
        self.nav_goal_handle = None
        self.is_intermediate_goal = False
        
        # Sensor buffers
        self.sensor_buffers = {
            'front_up': deque(maxlen=self.filter_window),
            'front_down': deque(maxlen=self.filter_window),
            'left_up': deque(maxlen=self.filter_window),
            'left_down': deque(maxlen=self.filter_window),
            'right_up': deque(maxlen=self.filter_window),
            'right_down': deque(maxlen=self.filter_window),
        }
        
        # Turn state
        self.turn_direction = 1
        self.turn_start_time = None
        self.turn_duration = 0.0
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action Client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Subscribers
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        for sensor in self.sensor_buffers.keys():
            self.create_subscription(Range, f'/us/{sensor}', 
                lambda msg, s=sensor: self.sensor_callback(msg, s), 10)
        self.nav_cmd_sub = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Storage
        self.nav_cmd = Twist()
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Ultrasonic Detour Supervisor initialized (SIMPLE MODE)")
    
    def sensor_callback(self, msg: Range, sensor_name: str):
        """Store sensor readings in buffers"""
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.sensor_buffers[sensor_name].append(msg.range)
    
    def nav_cmd_callback(self, msg: Twist):
        """Capture Nav2 commands"""
        self.nav_cmd = msg
    
    def goal_callback(self, msg: PoseStamped):
        """Handle new goal requests"""
        if self.state == State.IDLE:
            self.get_logger().info("New goal received - Starting navigation")
            self.original_goal = msg
            self.is_intermediate_goal = False
            self.send_nav_goal(msg)
    
    def send_nav_goal(self, goal: PoseStamped):
        """Send goal to Nav2"""
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.state = State.IDLE
            return
        
        self.state = State.NAVIGATING
        self.get_logger().info("Goal accepted, state → NAVIGATING")
        
        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle Nav2 goal completion"""
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            if self.is_intermediate_goal:
                self.get_logger().info("✓ Intermediate waypoint reached - Resuming original goal")
                self.is_intermediate_goal = False
                self.state = State.RESUME_GOAL
            else:
                self.get_logger().info("✓ GOAL REACHED!")
                self.state = State.IDLE
                self.original_goal = None
        elif status == 5:  # CANCELED
            self.get_logger().info("Goal canceled (expected during avoidance)")
        else:
            self.get_logger().warn(f"Goal failed with status {status}")
            if self.is_intermediate_goal:
                self.is_intermediate_goal = False
                self.state = State.RESUME_GOAL
    
    def get_aggregate_ranges(self):
        """Get median filtered front/left/right distances"""
        def get_median(buffer):
            if len(buffer) == 0:
                return 4.0
            sorted_vals = sorted(buffer)
            return sorted_vals[len(sorted_vals) // 2]
        
        front = min(get_median(self.sensor_buffers['front_up']), 
                   get_median(self.sensor_buffers['front_down']))
        left = min(get_median(self.sensor_buffers['left_up']),
                  get_median(self.sensor_buffers['left_down']))
        right = min(get_median(self.sensor_buffers['right_up']),
                   get_median(self.sensor_buffers['right_down']))
        
        return front, left, right
    
    def get_current_pose(self):
        """Get robot's current pose in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = transform.transform.rotation
            
            return pose
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw to quaternion"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def control_loop(self):
        """Main state machine"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        front, left, right = self.get_aggregate_ranges()
        
        # === IDLE ===
        if self.state == State.IDLE:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            return
        
        # === NAVIGATING ===
        if self.state == State.NAVIGATING:
            min_dist = min(front, left, right)
            
            if min_dist < self.stop_dist:
                self.get_logger().warn(f"⚠ Obstacle at {min_dist:.2f}m - STOPPING")
                self.state = State.AVOID_STOP
                return
            
            # Forward Nav2 commands
            self.cmd_pub.publish(self.nav_cmd)
            return
        
        # === AVOID_STOP ===
        if self.state == State.AVOID_STOP:
            # Stop robot
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            
            # Cancel current goal
            if self.nav_goal_handle is not None:
                self.nav_goal_handle.cancel_goal_async()
                self.nav_goal_handle = None
            
            # Decide turn direction
            self.turn_direction = 1 if left > right else -1
            direction_str = "LEFT" if self.turn_direction > 0 else "RIGHT"
            
            # Calculate turn duration
            turn_angle_rad = self.turn_angle_deg * (math.pi / 180.0)
            self.turn_duration = turn_angle_rad / self.turn_speed
            self.turn_start_time = current_time
            
            self.get_logger().info(f"Turning {direction_str} by {self.turn_angle_deg}°")
            self.state = State.AVOID_TURN
            return
        
        # === AVOID_TURN ===
        if self.state == State.AVOID_TURN:
            time_elapsed = current_time - self.turn_start_time
            
            if time_elapsed >= self.turn_duration:
                # Turn complete
                self.get_logger().info("Turn complete - Moving forward")
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                self.state = State.AVOID_STEP
                return
            
            # Continue turning
            cmd = Twist()
            cmd.angular.z = self.turn_direction * self.turn_speed
            self.cmd_pub.publish(cmd)
            
            remaining = self.turn_duration - time_elapsed
            self.get_logger().info(f"Turning... {remaining:.1f}s remaining", throttle_duration_sec=0.5)
            return
        
        # === AVOID_STEP ===
        if self.state == State.AVOID_STEP:
            self.get_logger().info(f"Moving {self.step_dist}m forward")
            
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().error("Cannot get pose - Resuming goal")
                self.state = State.RESUME_GOAL
                return
            
            # Compute goal straight ahead
            yaw = self.quaternion_to_yaw(current_pose.pose.orientation)
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.pose.position.x = current_pose.pose.position.x + self.step_dist * math.cos(yaw)
            waypoint.pose.position.y = current_pose.pose.position.y + self.step_dist * math.sin(yaw)
            waypoint.pose.orientation = self.yaw_to_quaternion(yaw)
            
            self.get_logger().info(f"Waypoint: ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})")
            
            self.is_intermediate_goal = True
            self.send_nav_goal(waypoint)
            
            # Immediately go back to navigating - Nav2 will handle the waypoint then we'll send original goal
            self.state = State.NAVIGATING
            return
        
        # === RESUME_GOAL ===
        if self.state == State.RESUME_GOAL:
            if self.original_goal is not None:
                self.get_logger().info("Resuming original goal")
                self.is_intermediate_goal = False
                self.send_nav_goal(self.original_goal)
            else:
                self.get_logger().warn("No original goal to resume")
                self.state = State.IDLE
            return


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicDetourSupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
