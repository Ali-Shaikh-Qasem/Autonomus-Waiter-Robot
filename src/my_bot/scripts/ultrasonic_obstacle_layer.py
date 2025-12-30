#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class UltrasonicObstacleLayer(Node):
    def __init__(self):
        super().__init__('ultrasonic_obstacle_layer')
        
        # Subscribe to all 8 ultrasonic sensors
        self.ultrasonic_subs = []
        self.ultrasonic_data = {}
        
        sensor_names = ['front_up', 'front_down', 'right_up', 'right_down', 
                       'left_up', 'left_down', 'rear_up', 'rear_down']
        
        for name in sensor_names:
            topic = f'/ultrasonic_scan_{name}'
            sub = self.create_subscription(
                LaserScan,
                topic,
                lambda msg, n=name: self.ultrasonic_callback(msg, n),
                10
            )
            self.ultrasonic_subs.append(sub)
            self.ultrasonic_data[name] = None
        
        # Publisher for merged ultrasonic scan (for costmap)
        self.merged_pub = self.create_publisher(LaserScan, '/ultrasonic_obstacles', 10)
        
        # Timer to publish merged data at 20 Hz for faster response
        self.timer = self.create_timer(0.05, self.publish_merged_scan)
        
        # Safety margin - make obstacles appear closer for early detection
        self.safety_margin = 0.3  # meters
        self.detection_threshold = 2.0  # Only consider obstacles within 2m
        
        self.get_logger().info('Ultrasonic Obstacle Layer Node Started')
    
    def ultrasonic_callback(self, msg, sensor_name):
        """Store the latest ultrasonic reading"""
        self.ultrasonic_data[sensor_name] = msg
    
    def publish_merged_scan(self):
        """Merge all ultrasonic readings into a single LaserScan for obstacle avoidance"""
        
        # Create a 360-degree scan message
        merged_scan = LaserScan()
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        merged_scan.header.frame_id = 'base_link'
        
        # Configure scan parameters
        merged_scan.angle_min = -math.pi
        merged_scan.angle_max = math.pi
        merged_scan.angle_increment = math.pi / 180.0  # 1 degree resolution
        merged_scan.range_min = 0.02
        merged_scan.range_max = 4.0
        merged_scan.time_increment = 0.0
        merged_scan.scan_time = 0.1
        
        # Initialize ranges with max range (no obstacle)
        num_readings = 360
        merged_scan.ranges = [merged_scan.range_max] * num_readings
        
        # Define sensor positions and angles (based on your URDF)
        sensor_configs = {
            'front_up': 0.0,           # 0 degrees
            'front_down': 0.0,         # 0 degrees
            'right_up': -45.0,         # -45 degrees
            'right_down': -45.0,       # -45 degrees
            'left_up': 45.0,           # 45 degrees
            'left_down': 45.0,         # 45 degrees
            'rear_up': 180.0,          # 180 degrees
            'rear_down': 180.0         # 180 degrees
        }
        
        # Process each sensor
        for sensor_name, angle_deg in sensor_configs.items():
            if self.ultrasonic_data[sensor_name] is None:
                continue
            
            scan = self.ultrasonic_data[sensor_name]
            
            # Get minimum range from this sensor (closest obstacle)
            min_range = merged_scan.range_max
            for r in scan.ranges:
                if scan.range_min < r < scan.range_max:
                    min_range = min(min_range, r)
            
            # If obstacle detected within threshold, mark it with safety margin
            if min_range < self.detection_threshold:
                # Apply safety margin - make obstacle appear closer
                safe_range = max(scan.range_min, min_range - self.safety_margin)
                
                # Convert angle to index
                angle_rad = math.radians(angle_deg)
                index = int((angle_rad - merged_scan.angle_min) / merged_scan.angle_increment)
                
                # Spread the obstacle across wider angular range (±20 degrees) for better detection
                spread = 20
                for offset in range(-spread, spread + 1):
                    idx = (index + offset) % num_readings
                    if 0 <= idx < num_readings:
                        # Use distance-based spreading - closer obstacles get wider spread
                        spread_factor = 1.0 - (abs(offset) / spread)
                        adjusted_range = safe_range / max(0.5, spread_factor)
                        merged_scan.ranges[idx] = min(merged_scan.ranges[idx], adjusted_range)
        
        # Publish merged scan
        self.merged_pub.publish(merged_scan)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicObstacleLayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
