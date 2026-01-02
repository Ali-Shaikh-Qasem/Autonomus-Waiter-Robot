#!/usr/bin/env python3
"""
ultrasonic_scan_to_range.py

ROS2 (Humble) node that converts Gazebo ultrasonic LaserScan topics into Range topics.

Behavior:
- Subscribe to /ultrasonic_scan_* (LaserScan)
- Publish /us/* (Range) with one reading per sensor (min valid range)
- If scan has no valid readings: publish max_range (meaning "clear")
- Optional stale publishing of max_range if sensor stops publishing
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, Range


@dataclass
class SensorState:
    last_value: float = None
    last_stamp_sec: float = None
    last_frame_id: str = None


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class UltrasonicScanToRange(Node):
    def __init__(self):
        super().__init__("ultrasonic_scan_to_range")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("min_range", 0.02)
        self.declare_parameter("max_range", 4.0)
        self.declare_parameter("fov_rad", 0.24)          # matches -0.12..+0.12 in your Gazebo ray scan
        self.declare_parameter("timeout_sec", 0.5)       # stale sensor timeout
        self.declare_parameter("include_rear", False)    # enable if you use rear sensors too
        self.declare_parameter("publish_stale_clears", True)

        # Optional smoothing (EMA)
        self.declare_parameter("ema_alpha", 0.0)         # 0 disables EMA; typical 0.3..0.6

        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.fov_rad = float(self.get_parameter("fov_rad").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.include_rear = bool(self.get_parameter("include_rear").value)
        self.publish_stale_clears = bool(self.get_parameter("publish_stale_clears").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.ema_alpha = clamp(self.ema_alpha, 0.0, 1.0)

        # -------------------------
        # QoS (sensor data)
        # -------------------------
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -------------------------
        # Topic mapping
        # -------------------------
        self.sensor_configs: List[Tuple[str, str, str]] = [
            ("/ultrasonic_scan_front_up",   "/us/front_up",   "front_up"),
            ("/ultrasonic_scan_front_down", "/us/front_down", "front_down"),
            ("/ultrasonic_scan_left_up",    "/us/left_up",    "left_up"),
            ("/ultrasonic_scan_left_down",  "/us/left_down",  "left_down"),
            ("/ultrasonic_scan_right_up",   "/us/right_up",   "right_up"),
            ("/ultrasonic_scan_right_down", "/us/right_down", "right_down"),
        ]
        if self.include_rear:
            self.sensor_configs += [
                ("/ultrasonic_scan_rear_up",   "/us/rear_up",   "rear_up"),
                ("/ultrasonic_scan_rear_down", "/us/rear_down", "rear_down"),
            ]

        # -------------------------
        # Publishers/Subscribers
        # IMPORTANT: do NOT name it self.publishers
        # -------------------------
        self.range_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.states: Dict[str, SensorState] = {}

        for scan_topic, range_topic, name in self.sensor_configs:
            self.range_pubs[range_topic] = self.create_publisher(Range, range_topic, self.sensor_qos)
            self.states[name] = SensorState()

            self.create_subscription(
                LaserScan,
                scan_topic,
                lambda msg, n=name, rt=range_topic: self.scan_cb(msg, n, rt),
                self.sensor_qos,
            )

            self.get_logger().info(f"Configured: {scan_topic} -> {range_topic}")

        # Timer for stale sensors
        self.stale_timer = self.create_timer(0.1, self.stale_check_cb)

        self.get_logger().info(
            f"UltrasonicScanToRange ready | min={self.min_range} max={self.max_range} fov={self.fov_rad} "
            f"ema_alpha={self.ema_alpha}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def scan_cb(self, msg: LaserScan, sensor_name: str, range_topic: str):
        # Collect valid ranges
        valid: List[float] = []
        scan_rmin = msg.range_min if math.isfinite(msg.range_min) else self.min_range
        scan_rmax = msg.range_max if math.isfinite(msg.range_max) else self.max_range
        scan_rmin = max(scan_rmin, 0.0)
        scan_rmax = max(scan_rmax, scan_rmin)

        for r in msg.ranges:
            if not math.isfinite(r) or math.isnan(r):
                continue
            if r <= 0.0:
                continue
            if r < scan_rmin or r > scan_rmax:
                continue
            valid.append(r)

        # Reduce to one value (min valid)
        if valid:
            r_out = min(valid)
        else:
            r_out = self.max_range  # no hit => clear

        # Clamp
        r_out = clamp(r_out, self.min_range, self.max_range)

        # Optional EMA
        st = self.states[sensor_name]
        if self.ema_alpha > 0.0 and st.last_value is not None:
            r_out = self.ema_alpha * r_out + (1.0 - self.ema_alpha) * st.last_value

        # Update state
        now = self.now_sec()
        st.last_value = r_out
        st.last_stamp_sec = now
        st.last_frame_id = msg.header.frame_id if msg.header.frame_id else st.last_frame_id

        # Publish Range
        self.publish_range(range_topic, st.last_frame_id, msg.header.stamp, r_out)

    def publish_range(self, range_topic: str, frame_id: str, stamp, value: float):
        rmsg = Range()
        rmsg.header.stamp = stamp
        rmsg.header.frame_id = frame_id if frame_id else ""
        rmsg.radiation_type = Range.ULTRASOUND
        rmsg.field_of_view = float(self.fov_rad)
        rmsg.min_range = float(self.min_range)
        rmsg.max_range = float(self.max_range)
        rmsg.range = float(value)

        pub = self.range_pubs.get(range_topic)
        if pub is not None:
            pub.publish(rmsg)

    def stale_check_cb(self):
        if not self.publish_stale_clears:
            return

        now = self.now_sec()
        for _, range_topic, name in self.sensor_configs:
            st = self.states.get(name)
            if st is None or st.last_stamp_sec is None:
                continue

            if (now - st.last_stamp_sec) > self.timeout_sec:
                # publish clear reading to avoid stale "stuck obstacle"
                stamp = self.get_clock().now().to_msg()
                self.publish_range(range_topic, st.last_frame_id, stamp, self.max_range)


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


if __name__ == "__main__":
    main()
