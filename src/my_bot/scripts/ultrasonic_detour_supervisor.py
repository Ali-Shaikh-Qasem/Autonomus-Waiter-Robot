#!/usr/bin/env python3
"""
Ultrasonic Detour Supervisor (v3 - fixes over-rotation)

Key changes vs v2:
1) ARC exit uses DETOUR-side clearance (the side we want to pass through),
   not "obstacle-side must be huge", which caused long arcs and near-180 turns.
2) Adds max_arc_yaw_deg cap to prevent rotating too far.
3) Forward pass uses adaptive wall-avoid steering (nudges away only when needed).

Nav2 -> /cmd_vel_nav -> supervisor -> /cmd_vel
Sensors: /us/front_*, /us/left_*, /us/right_* (Range)
Odom: /odom
"""

import math
from enum import Enum
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


class Side(Enum):
    LEFT = 1
    RIGHT = -1


class Mode(Enum):
    IDLE = 0
    NAVIGATING = 1
    AVOID_START = 2
    AVOID_ARC = 3
    AVOID_BACKUP = 4
    AVOID_FORWARD_PASS = 5
    RESUME_GOAL = 6


@dataclass
class FilteredRange:
    value: float
    stamp_sec: float


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def quat_to_yaw(q):
    # yaw from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def ang_wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class UltrasonicDetourSupervisor(Node):
    def __init__(self):
        super().__init__('ultrasonic_detour_supervisor')

        # -------------------------
        # Params
        # -------------------------
        self.declare_parameter('control_rate_hz', 20.0)

        self.declare_parameter('robot_radius_m', 0.20)
        self.declare_parameter('safety_margin_m', 0.10)

        # Trigger distances
        self.declare_parameter('avoid_extra_m', 0.25)  # D_avoid = D_stop + avoid_extra

        # Exit criteria (IMPORTANT)
        self.declare_parameter('clear_front_m', 0.90)      # front must be this clear
        self.declare_parameter('detour_clear_m', 0.80)     # detour side must be this clear
        self.declare_parameter('obstacle_min_m', 0.38)     # obstacle side only needs to be above "safe min"

        # Hysteresis
        self.declare_parameter('persistence_ms', 250)
        self.declare_parameter('stable_ms', 400)
        self.declare_parameter('cool_down_sec', 1.2)

        # Sensor filtering
        self.declare_parameter('ema_alpha', 0.45)
        self.declare_parameter('sensor_timeout_sec', 0.6)
        self.declare_parameter('default_max_range', 4.0)

        # Arc motion
        self.declare_parameter('v_max_detour', 0.12)
        self.declare_parameter('w_min', 0.20)
        self.declare_parameter('w_max', 0.70)
        self.declare_parameter('k_corr', 0.12)
        self.declare_parameter('detour_max_time_sec', 8.0)

        # Anti-over-rotation
        self.declare_parameter('max_arc_yaw_deg', 75.0)     # cap total yaw change during ARC

        # Forward pass distance-based
        self.declare_parameter('commit_distance_m', 1.10)
        self.declare_parameter('v_pass', 0.12)

        # Forward pass steering (adaptive)
        self.declare_parameter('wall_target_m', 0.55)       # keep obstacle-side around/above this
        self.declare_parameter('k_wall_pass', 1.2)          # steering gain away from obstacle-side
        self.declare_parameter('w_pass_max', 0.35)          # max pass yaw rate

        # Backup
        self.declare_parameter('backup_duration_sec', 0.6)
        self.declare_parameter('v_backup', 0.10)

        # Rate limits
        self.declare_parameter('v_rate_limit', 0.60)
        self.declare_parameter('w_rate_limit', 2.20)

        # NAVIGATING side nudge (optional)
        self.declare_parameter('side_nudge_enable', True)
        self.declare_parameter('side_warn_m', 0.55)
        self.declare_parameter('side_nudge_gain', 0.30)
        self.declare_parameter('side_slow_min', 0.40)

        # Topics
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('nav_cmd_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        # -------------------------
        # Read params
        # -------------------------
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.robot_radius = float(self.get_parameter('robot_radius_m').value)
        self.safety_margin = float(self.get_parameter('safety_margin_m').value)
        self.avoid_extra = float(self.get_parameter('avoid_extra_m').value)

        self.clear_front = float(self.get_parameter('clear_front_m').value)
        self.detour_clear = float(self.get_parameter('detour_clear_m').value)
        self.obstacle_min = float(self.get_parameter('obstacle_min_m').value)

        self.persistence = float(self.get_parameter('persistence_ms').value) / 1000.0
        self.stable_time = float(self.get_parameter('stable_ms').value) / 1000.0
        self.cool_down_sec = float(self.get_parameter('cool_down_sec').value)

        self.alpha = clamp(float(self.get_parameter('ema_alpha').value), 0.0, 1.0)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout_sec').value)
        self.default_max_range = float(self.get_parameter('default_max_range').value)

        self.v_max_detour = float(self.get_parameter('v_max_detour').value)
        self.w_min = float(self.get_parameter('w_min').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.k_corr = float(self.get_parameter('k_corr').value)
        self.detour_max_time = float(self.get_parameter('detour_max_time_sec').value)

        self.max_arc_yaw = math.radians(float(self.get_parameter('max_arc_yaw_deg').value))

        self.commit_distance = float(self.get_parameter('commit_distance_m').value)
        self.v_pass = float(self.get_parameter('v_pass').value)

        self.wall_target = float(self.get_parameter('wall_target_m').value)
        self.k_wall_pass = float(self.get_parameter('k_wall_pass').value)
        self.w_pass_max = float(self.get_parameter('w_pass_max').value)

        self.backup_duration = float(self.get_parameter('backup_duration_sec').value)
        self.v_backup = float(self.get_parameter('v_backup').value)

        self.v_rate_limit = float(self.get_parameter('v_rate_limit').value)
        self.w_rate_limit = float(self.get_parameter('w_rate_limit').value)

        self.side_nudge_enable = bool(self.get_parameter('side_nudge_enable').value)
        self.side_warn = float(self.get_parameter('side_warn_m').value)
        self.side_nudge_gain = float(self.get_parameter('side_nudge_gain').value)
        self.side_slow_min = float(self.get_parameter('side_slow_min').value)

        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.nav_cmd_topic = str(self.get_parameter('nav_cmd_topic').value)
        self.cmd_out_topic = str(self.get_parameter('cmd_out_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        # Derived
        self.D_stop = self.robot_radius + self.safety_margin  # ~0.30
        self.D_avoid = self.D_stop + self.avoid_extra         # ~0.55

        # -------------------------
        # QoS
        # -------------------------
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.cmd_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # -------------------------
        # State
        # -------------------------
        self.mode = Mode.IDLE
        self.original_goal = None
        self.nav_goal_handle = None

        self.nav_cmd = Twist()
        self.last_nav_cmd_time = 0.0

        self.obstacle_side = Side.LEFT
        self.detour_side = Side.RIGHT

        self.obstacle_seen_since = None
        self.clear_seen_since = None

        self.avoid_started_at = 0.0
        self.backup_end_at = 0.0
        self.cooldown_until = 0.0

        # odom tracking
        self.odom_ready = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.pass_start_x = 0.0
        self.pass_start_y = 0.0

        # yaw cap for arc
        self.arc_yaw0 = 0.0

        # cmd smoothing
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0

        self._last_status_log = 0.0
        self._rx_counter = 0

        self.ranges = {
            'front_up': None,
            'front_down': None,
            'left_up': None,
            'left_down': None,
            'right_up': None,
            'right_down': None,
        }

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.create_subscription(Twist, self.nav_cmd_topic, self.nav_cmd_callback, self.cmd_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

        for key in self.ranges.keys():
            self.create_subscription(Range, f'/us/{key}',
                                     lambda msg, k=key: self.range_callback(msg, k),
                                     self.sensor_qos)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_out_topic, 10)

        dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info(
            f"Supervisor v3: D_stop={self.D_stop:.2f}, D_avoid={self.D_avoid:.2f}, "
            f"clear_front={self.clear_front:.2f}, detour_clear={self.detour_clear:.2f}, "
            f"obstacle_min={self.obstacle_min:.2f}, max_arc_yaw={math.degrees(self.max_arc_yaw):.0f}deg"
        )

    # ---------- Basic helpers ----------
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.odom_ready = True

    def publish_cmd(self, v: float, w: float, dt: float):
        dv = clamp(v - self.last_cmd_v, -self.v_rate_limit * dt, self.v_rate_limit * dt)
        dw = clamp(w - self.last_cmd_w, -self.w_rate_limit * dt, self.w_rate_limit * dt)
        self.last_cmd_v += dv
        self.last_cmd_w += dw

        cmd = Twist()
        cmd.linear.x = self.last_cmd_v
        cmd.angular.z = self.last_cmd_w
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.cmd_pub.publish(Twist())

    # ---------- Sensors ----------
    def range_callback(self, msg: Range, key: str):
        now = self.now_sec()
        r = msg.range
        if not math.isfinite(r):
            return
        r = clamp(r, msg.min_range, msg.max_range)

        prev = self.ranges.get(key)
        if prev is None:
            self.ranges[key] = FilteredRange(value=r, stamp_sec=now)
        else:
            filt = self.alpha * r + (1.0 - self.alpha) * prev.value
            self.ranges[key] = FilteredRange(value=filt, stamp_sec=now)

        self._rx_counter += 1

    def get_dir_dists(self):
        now = self.now_sec()

        def v(name: str):
            fr = self.ranges.get(name)
            if fr is None:
                return self.default_max_range
            if (now - fr.stamp_sec) > self.sensor_timeout:
                return self.default_max_range
            return clamp(fr.value, 0.02, self.default_max_range)

        dF = min(v('front_up'), v('front_down'))
        dL = min(v('left_up'), v('left_down'))
        dR = min(v('right_up'), v('right_down'))
        return dF, dL, dR

    def decide_sides_at_trigger(self, dL: float, dR: float):
        self.obstacle_side = Side.LEFT if dL < dR else Side.RIGHT
        self.detour_side = Side.RIGHT if self.obstacle_side == Side.LEFT else Side.LEFT

    # ---------- Nav2 ----------
    def nav_cmd_callback(self, msg: Twist):
        self.nav_cmd = msg
        self.last_nav_cmd_time = self.now_sec()

    def goal_callback(self, msg: PoseStamped):
        self.original_goal = msg
        self.get_logger().info(
            f"New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) frame={msg.header.frame_id}"
        )
        self.obstacle_seen_since = None
        self.clear_seen_since = None
        self.cooldown_until = self.now_sec() + self.cool_down_sec

        self.cancel_nav_goal()
        self.send_nav_goal(msg)
        self.mode = Mode.NAVIGATING

    def send_nav_goal(self, goal_pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose server not available!")
            self.mode = Mode.IDLE
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected goal.")
            self.mode = Mode.IDLE
            return
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        self.get_logger().info("Nav2 goal accepted.")

    def goal_result_callback(self, future):
        status = future.result().status
        if status == 4 and self.mode == Mode.NAVIGATING:
            self.get_logger().info("✓ Goal reached.")
            self.mode = Mode.IDLE
            self.original_goal = None

    def cancel_nav_goal(self):
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav_goal_handle = None

    # ---------- Control helpers ----------
    def nav_side_nudge(self, v_in: float, w_in: float, dL: float, dR: float):
        if not self.side_nudge_enable:
            return v_in, w_in

        dMin = min(dL, dR)
        if dMin >= self.side_warn:
            return v_in, w_in

        u = clamp((self.side_warn - dMin) / max(1e-3, (self.side_warn - self.D_stop)), 0.0, 1.0)

        scale = max(self.side_slow_min, 1.0 - 0.65 * u)
        v_out = v_in * scale

        if dL < dR:
            w_out = w_in - self.side_nudge_gain * u  # obstacle left -> turn right
        else:
            w_out = w_in + self.side_nudge_gain * u  # obstacle right -> turn left

        return v_out, w_out

    def compute_arc_cmd(self, dF: float, dL: float, dR: float):
        denom = max(1e-3, (self.D_avoid - self.D_stop))
        uF = clamp((self.D_avoid - dF) / denom, 0.0, 1.0)

        # slow down when close
        v = self.v_max_detour * (1.0 - 0.80 * uF)

        dObs = dL if self.obstacle_side == Side.LEFT else dR
        if dF <= self.D_stop or dObs <= self.D_stop:
            v = 0.0

        sign = self.detour_side.value
        w_base = sign * (self.w_min + (self.w_max - self.w_min) * uF)

        # gentle centering
        w_corr = self.k_corr * ((1.0 / max(dR, 0.10)) - (1.0 / max(dL, 0.10)))
        w = w_base + w_corr

        # enforce direction
        if self.detour_side == Side.LEFT:
            w = max(w, 0.08)
        else:
            w = min(w, -0.08)

        return v, w

    def should_exit_arc(self, dF, dL, dR):
        dObs = dL if self.obstacle_side == Side.LEFT else dR
        dDet = dR if self.detour_side == Side.RIGHT else dL

        # IMPORTANT: detour-side must be open. obstacle-side only needs to be safe-min.
        return (dF > self.clear_front) and (dDet > self.detour_clear) and (dObs > self.obstacle_min)

    def start_forward_pass(self):
        if not self.odom_ready:
            self.get_logger().warn("No odom yet -> RESUME_GOAL directly")
            self.mode = Mode.RESUME_GOAL
            return

        self.pass_start_x = self.odom_x
        self.pass_start_y = self.odom_y
        self.mode = Mode.AVOID_FORWARD_PASS
        self.get_logger().info("Clear stable -> FORWARD_PASS")

    def forward_pass_cmd(self, dL, dR):
        # steer away from obstacle-side only when too close
        dObs = dL if self.obstacle_side == Side.LEFT else dR
        sign = self.detour_side.value

        err = self.wall_target - dObs  # positive => too close
        if err <= 0.0:
            w = 0.0
        else:
            w = sign * clamp(self.k_wall_pass * (err / max(self.wall_target, 1e-3)), 0.0, self.w_pass_max)

        return self.v_pass, w

    # ---------- Main loop ----------
    def control_loop(self):
        now = self.now_sec()
        dt = 1.0 / max(1.0, self.rate_hz)

        dF, dL, dR = self.get_dir_dists()

        if (now - self._last_status_log) > 1.0:
            self._last_status_log = now
            self.get_logger().info(
                f"mode={self.mode.name} dF={dF:.2f} dL={dL:.2f} dR={dR:.2f} "
                f"obs={self.obstacle_side.name} detour={self.detour_side.name} rx={self._rx_counter}"
            )

        if self.mode == Mode.IDLE:
            self.stop_robot()
            return

        if self.mode == Mode.NAVIGATING:
            # Trigger avoidance mainly from FRONT
            if now >= self.cooldown_until:
                if dF < self.D_avoid:
                    if self.obstacle_seen_since is None:
                        self.obstacle_seen_since = now
                    elif (now - self.obstacle_seen_since) >= self.persistence:
                        self.get_logger().warn(
                            f"Trigger: dF={dF:.2f} < D_avoid={self.D_avoid:.2f} -> AVOID"
                        )
                        self.mode = Mode.AVOID_START
                        return
                else:
                    self.obstacle_seen_since = None
            else:
                if dF < self.D_stop:
                    self.get_logger().warn("Emergency trigger during cooldown -> AVOID")
                    self.mode = Mode.AVOID_START
                    return

            # forward nav2 cmd with small side safety
            if (now - self.last_nav_cmd_time) > 0.5:
                v_cmd, w_cmd = 0.0, 0.0
            else:
                v_cmd, w_cmd = self.nav_cmd.linear.x, self.nav_cmd.angular.z

            v_cmd, w_cmd = self.nav_side_nudge(v_cmd, w_cmd, dL, dR)
            self.publish_cmd(v_cmd, w_cmd, dt)
            return

        if self.mode == Mode.AVOID_START:
            self.stop_robot()
            self.cancel_nav_goal()

            self.decide_sides_at_trigger(dL, dR)
            self.avoid_started_at = now
            self.clear_seen_since = None
            self.obstacle_seen_since = None

            # record yaw start for cap
            if self.odom_ready:
                self.arc_yaw0 = self.odom_yaw
            else:
                self.arc_yaw0 = 0.0

            self.get_logger().info(
                f"Avoidance started. obstacle_side={self.obstacle_side.name} detour_side={self.detour_side.name}"
            )
            self.mode = Mode.AVOID_ARC
            return

        if self.mode == Mode.AVOID_ARC:
            # hard timeout
            if (now - self.avoid_started_at) > self.detour_max_time:
                self.get_logger().warn("Detour timeout -> BACKUP")
                self.backup_end_at = now + self.backup_duration
                self.mode = Mode.AVOID_BACKUP
                return

            # yaw cap to prevent “turning to rear”
            if self.odom_ready:
                dyaw = ang_wrap(self.odom_yaw - self.arc_yaw0)
                if abs(dyaw) > self.max_arc_yaw:
                    # If front is not terrible, stop arcing and do forward pass
                    if dF > (self.D_stop + 0.08):
                        self.get_logger().warn(
                            f"Arc yaw cap hit ({math.degrees(abs(dyaw)):.0f}deg) -> FORWARD_PASS"
                        )
                        self.start_forward_pass()
                        return
                    # otherwise backup/flip
                    self.get_logger().warn(
                        f"Arc yaw cap hit but front still tight -> BACKUP"
                    )
                    self.backup_end_at = now + self.backup_duration
                    self.mode = Mode.AVOID_BACKUP
                    return

            v, w = self.compute_arc_cmd(dF, dL, dR)
            self.publish_cmd(v, w, dt)

            # exit condition based on DETOUR side, stable for stable_ms
            if self.should_exit_arc(dF, dL, dR):
                if self.clear_seen_since is None:
                    self.clear_seen_since = now
                elif (now - self.clear_seen_since) >= self.stable_time:
                    self.start_forward_pass()
                    return
            else:
                self.clear_seen_since = None

            return

        if self.mode == Mode.AVOID_BACKUP:
            if now >= self.backup_end_at:
                # flip detour direction
                self.detour_side = Side.LEFT if self.detour_side == Side.RIGHT else Side.RIGHT
                self.avoid_started_at = now
                self.clear_seen_since = None
                if self.odom_ready:
                    self.arc_yaw0 = self.odom_yaw
                self.get_logger().info(f"Backup done. Switch detour_side -> {self.detour_side.name}")
                self.mode = Mode.AVOID_ARC
                return

            self.publish_cmd(-abs(self.v_backup), 0.0, dt)
            return

        if self.mode == Mode.AVOID_FORWARD_PASS:
            # distance traveled
            dx = self.odom_x - self.pass_start_x
            dy = self.odom_y - self.pass_start_y
            dist = math.sqrt(dx*dx + dy*dy)

            dObs = dL if self.obstacle_side == Side.LEFT else dR

            # If we get dangerously close again, return to ARC
            if dF < self.D_avoid or dObs < self.obstacle_min:
                self.get_logger().warn("Too close during forward pass -> ARC")
                self.clear_seen_since = None
                # reset yaw cap reference for a new arc attempt
                if self.odom_ready:
                    self.arc_yaw0 = self.odom_yaw
                self.mode = Mode.AVOID_ARC
                return

            if dist >= self.commit_distance:
                self.stop_robot()
                self.mode = Mode.RESUME_GOAL
                return

            v, w = self.forward_pass_cmd(dL, dR)
            # also apply mild generic side safety
            v, w = self.nav_side_nudge(v, w, dL, dR)
            self.publish_cmd(v, w, dt)
            return

        if self.mode == Mode.RESUME_GOAL:
            self.stop_robot()
            if self.original_goal is not None:
                self.get_logger().info("Resuming original goal with Nav2.")
                self.send_nav_goal(self.original_goal)
                self.cooldown_until = now + self.cool_down_sec
                self.mode = Mode.NAVIGATING
            else:
                self.mode = Mode.IDLE
            return


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicDetourSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
