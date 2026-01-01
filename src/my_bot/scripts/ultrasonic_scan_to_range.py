#!/usr/bin/env python3
"""
Ultrasonic LaserScan to Range Converter Node

This node converts LaserScan messages from ultrasonic sensors to Range messages
for use with Nav2's RangeSensorLayer in the local costmap.

Each ultrasonic sensor publishes a LaserScan with multiple rays spanning a small FOV.
This node extracts the minimum valid range and publishes it as a single Range message.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Range
import math
from typing import Dict, Optional
from dataclasses import dataclass
from datetime import datetime


@dataclass
class SensorData:
    """Holds the latest data for a sensor."""
    last_range: float = float('inf')
    last_stamp: Optional[datetime] = None
    frame_id: str = ""


class UltrasonicScanToRange(Node):
    """
    ROS2 Node that converts ultrasonic LaserScan messages to Range messages.
    
    Subscribes to 6 ultrasonic LaserScan topics and publishes corresponding
    Range messages suitable for Nav2 RangeSensorLayer.
    """

    # Mapping from input topic suffix to output topic name
    SENSOR_MAPPING = {
        'front_up': '/us/front_up',
        'front_down': '/us/front_down',
        'left_up': '/us/left_up',
        'left_down': '/us/left_down',
        'right_up': '/us/right_up',
        'right_down': '/us/right_down',
    }

    def __init__(self):
        super().__init__('ultrasonic_scan_to_range')

        # Declare parameters
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('fov_rad', 0.24)  # ~0.12 to +0.12 rad = 0.24 rad total
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('log_rate_sec', 1.0)

        # Get parameters
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.fov_rad = self.get_parameter('fov_rad').get_parameter_value().double_value
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value
        self.log_rate_sec = self.get_parameter('log_rate_sec').get_parameter_value().double_value

        self.get_logger().info(
            f"Ultrasonic converter initialized: min_range={self.min_range}, "
            f"max_range={self.max_range}, fov_rad={self.fov_rad:.3f}"
        )

        # Sensor QoS (best effort, depth 1) for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Storage for sensor data
        self.sensor_data: Dict[str, SensorData] = {}
        
        # Publishers dictionary
        self.range_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        
        # Create subscriptions and publishers for each sensor
        for sensor_name, output_topic in self.SENSOR_MAPPING.items():
            input_topic = f'/ultrasonic_scan_{sensor_name}'
            
            # Initialize sensor data
            self.sensor_data[sensor_name] = SensorData()
            
            # Create publisher for Range message
            self.range_publishers[sensor_name] = self.create_publisher(
                Range,
                output_topic,
                10
            )
            
            # Create subscription for LaserScan
            # Using a lambda with default argument to capture sensor_name correctly
            self.create_subscription(
                LaserScan,
                input_topic,
                lambda msg, name=sensor_name: self.scan_callback(msg, name),
                sensor_qos
            )
            
            self.get_logger().info(f"Mapping: {input_topic} -> {output_topic}")

        # Timer for periodic logging
        self.last_log_time = self.get_clock().now()
        self.create_timer(self.log_rate_sec, self.log_status)

    def scan_callback(self, msg: LaserScan, sensor_name: str) -> None:
        """
        Process incoming LaserScan and publish corresponding Range message.
        
        Args:
            msg: The incoming LaserScan message
            sensor_name: Name of the sensor (e.g., 'front_up')
        """
        # Extract minimum valid range from the scan
        min_range_value = self.extract_min_range(msg)
        
        # Update sensor data for logging
        self.sensor_data[sensor_name].last_stamp = datetime.now()
        self.sensor_data[sensor_name].frame_id = msg.header.frame_id
        self.sensor_data[sensor_name].last_range = min_range_value
        
        # Create and publish Range message
        range_msg = Range()
        
        # Use the same timestamp from LaserScan for proper TF lookup
        range_msg.header.stamp = msg.header.stamp
        # CRITICAL: Use exact same frame_id for proper costmap placement
        range_msg.header.frame_id = msg.header.frame_id
        
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = self.fov_rad
        range_msg.min_range = self.min_range
        range_msg.max_range = self.max_range
        
        # Clamp range to valid bounds
        if math.isfinite(min_range_value):
            range_msg.range = max(self.min_range, min(min_range_value, self.max_range))
        else:
            # No valid reading - publish max_range (indicates clear)
            range_msg.range = self.max_range
        
        self.range_publishers[sensor_name].publish(range_msg)

    def extract_min_range(self, msg: LaserScan) -> float:
        """
        Extract the minimum valid range from a LaserScan message.
        
        Args:
            msg: LaserScan message to process
            
        Returns:
            Minimum valid range value, or inf if no valid readings
        """
        min_val = float('inf')
        
        for r in msg.ranges:
            # Check for valid reading:
            # - Must be finite (not nan, not inf)
            # - Must be greater than minimum threshold (0.01m to filter noise)
            # - Must be within sensor's reported range
            if (math.isfinite(r) and 
                r > 0.01 and 
                r >= msg.range_min and 
                r <= msg.range_max):
                min_val = min(min_val, r)
        
        return min_val

    def log_status(self) -> None:
        """
        Periodic logging of sensor status.
        Logs min values and warns about stale sensors.
        """
        now = datetime.now()
        
        # Group sensors by position
        groups = {
            'front': ['front_up', 'front_down'],
            'left': ['left_up', 'left_down'],
            'right': ['right_up', 'right_down'],
        }
        
        status_parts = []
        stale_sensors = []
        
        for group_name, sensors in groups.items():
            min_range = float('inf')
            for sensor in sensors:
                data = self.sensor_data[sensor]
                if data.last_stamp is not None:
                    age = (now - data.last_stamp).total_seconds()
                    if age > self.timeout_sec:
                        stale_sensors.append(sensor)
                    if math.isfinite(data.last_range):
                        min_range = min(min_range, data.last_range)
                else:
                    stale_sensors.append(sensor)
            
            if math.isfinite(min_range):
                status_parts.append(f"{group_name}={min_range:.2f}m")
            else:
                status_parts.append(f"{group_name}=--")
        
        # Log status (throttled by timer)
        self.get_logger().info(f"Ultrasonic ranges: {', '.join(status_parts)}")
        
        # Warn about stale sensors
        if stale_sensors:
            self.get_logger().warn(
                f"No data from sensors (>{self.timeout_sec}s): {', '.join(stale_sensors)}"
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = UltrasonicScanToRange()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
