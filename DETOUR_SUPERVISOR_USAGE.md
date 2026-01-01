# Ultrasonic Detour Supervisor - Usage Guide

## Overview

Goal-preserving reactive navigation wrapper that detours around low obstacles while maintaining the original navigation goal.

### How It Works

1. **User sends goal** → Supervisor stores it and forwards to Nav2
2. **Nav2 navigates** → Supervisor monitors ultrasonic sensors
3. **Obstacle detected** → Supervisor takes over:
   - Cancels Nav2 goal
   - Rotates to find clear direction
   - Steps forward into clear space
   - Resends original goal
4. **Resume navigation** → Nav2 continues toward original goal

## Quick Start

### 1. Build and Source
```bash
cd ~/Autonomous-Waiter-Robot-Simulations/dev_ws
colcon build --packages-select my_bot --symlink-install
source install/setup.bash
```

### 2. Launch Navigation
```bash
ros2 launch my_bot nav_with_detour_supervisor.launch.py slam:=True
```

### 3. Send Goals

**Option A: Using CLI**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 3.0, z: 0.0}}}"
```

**Option B: Using Python script**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = Node('goal_sender')
pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

goal = PoseStamped()
goal.header.frame_id = 'map'
goal.header.stamp = node.get_clock().now().to_msg()
goal.pose.position.x = 2.0
goal.pose.position.y = 3.0
goal.pose.position.z = 0.0

pub.publish(goal)
node.get_logger().info('Goal sent!')
```

**Option C: From RViz2** (requires custom plugin or topic remapping)

You can remap RViz's goal topic to `/goal_pose` by modifying RViz config.

## State Machine

```
IDLE
  ↓ (goal received)
NAVIGATING
  ↓ (obstacle detected: front < 0.5m for 250ms)
AVOID_CANCEL
  ↓ (goal canceled, check if backup needed)
AVOID_SCAN
  ↓ (rotate ±90° to find clear path > 0.8m)
AVOID_STEP
  ↓ (move forward 0.5m in clear direction)
RESUME_GOAL
  ↓ (resend original goal)
NAVIGATING
```

## Parameters

### Launch Arguments
```bash
ros2 launch my_bot nav_with_detour_supervisor.launch.py \
  slam:=True \
  stop_distance:=0.60 \
  clear_distance:=1.00
```

### All Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stop_distance` | 0.50 | Trigger avoidance (m) |
| `clear_distance` | 0.80 | Consider path clear (m) |
| `persistence_ms` | 250 | Obstacle must persist (ms) |
| `stable_ms` | 300 | Clear path must be stable (ms) |
| `turn_speed` | 0.5 | Rotation speed during scan (rad/s) |
| `scan_step_deg` | 15.0 | Rotation increment (degrees) |
| `max_scan_deg` | 90.0 | Max scan angle each side (degrees) |
| `d_step` | 0.5 | Intermediate step distance (m) |
| `intermediate_timeout` | 5.0 | Timeout for intermediate goal (s) |
| `cool_down_sec` | 2.0 | Cooldown between triggers (s) |
| `filter_window` | 5 | Median filter window size |
| `very_close_dist` | 0.30 | Backup threshold (m) |
| `backup_dist` | 0.20 | Backup distance (m) |

## Monitoring

### Check Topics
```bash
# Goals sent to supervisor
ros2 topic echo /goal_pose

# Nav2's velocity commands (intercepted)
ros2 topic echo /cmd_vel_nav

# Final robot commands (from supervisor)
ros2 topic echo /cmd_vel

# Ultrasonic sensors
ros2 topic echo /us/front_up
```

### Watch Logs
The supervisor logs all state transitions:
```
[INFO] Received goal: (2.00, 3.00)
[INFO] Sending goal to Nav2
[INFO] Goal accepted by Nav2, state → NAVIGATING
[WARN] OBSTACLE DETECTED: front=0.42m - Starting avoidance
[INFO] State: AVOID_CANCEL - Canceling Nav2 goal
[INFO] Starting scan LEFT
[INFO] Clear path found at 45° - Moving to AVOID_STEP
[INFO] Intermediate goal: (1.85, 2.65)
[INFO] State: RESUME_GOAL - Resending original goal
```

## Tuning Tips

### Robot too cautious?
- Decrease `stop_distance` (e.g., 0.40m)
- Decrease `persistence_ms` (e.g., 150ms)

### Robot hits obstacles?
- Increase `stop_distance` (e.g., 0.60m)
- Increase `clear_distance` (e.g., 1.00m)
- Increase `filter_window` for more smoothing

### Scanning too slow?
- Increase `turn_speed` (e.g., 0.8 rad/s)
- Increase `scan_step_deg` (e.g., 20°)

### False triggers?
- Increase `persistence_ms` (e.g., 400ms)
- Increase `filter_window` (e.g., 7)
- Increase `cool_down_sec` (e.g., 3.0s)

### Steps too short?
- Increase `d_step` (e.g., 0.8m)
- Increase `intermediate_timeout` (e.g., 8.0s)

## Troubleshooting

### Supervisor not receiving goals
**Check**: Is `/goal_pose` topic active?
```bash
ros2 topic info /goal_pose
```

### Robot doesn't avoid obstacles
**Check**: Are ultrasonic topics publishing?
```bash
ros2 topic hz /us/front_up
```
**Check**: Sensor values during approach
```bash
ros2 topic echo /us/front_up
```

### Robot keeps rotating
**Symptom**: Stuck in AVOID_SCAN state
**Cause**: No clear path found
**Fix**: Lower `clear_distance` or increase `max_scan_deg`

### Robot doesn't resume goal
**Check**: TF transform availability
```bash
ros2 run tf2_ros tf2_echo map base_link
```

### Nav2 ignores remapping
**Check**: Is controller publishing to `/cmd_vel_nav`?
```bash
ros2 topic hz /cmd_vel_nav
```
**Fix**: Verify launch file remapping is active

## Architecture

```
┌─────────┐
│  User   │ publishes /goal_pose
└────┬────┘
     │
     ▼
┌─────────────────────────────┐
│  Detour Supervisor          │
│  - Stores original goal     │
│  - Monitors ultrasonics     │
│  - State machine control    │
└─┬──────────────────────┬────┘
  │ sends goals          │ publishes /cmd_vel
  │                      │
  ▼                      ▼
┌─────────────┐      ┌──────┐
│   Nav2      │─────→│Robot │
│ /navigate   │ /cmd │      │
│ _to_pose    │ _vel │      │
└─────────────┘  _nav└──────┘
```

## Benefits vs Other Approaches

| Feature | Costmap | Reflex | Detour Supervisor |
|---------|---------|--------|-------------------|
| Preserves goal | ✅ | ⚠️ | ✅ |
| No oscillation | ❌ | ⚠️ | ✅ |
| Predictable | ❌ | ✅ | ✅ |
| Complex tuning | ✅ | ❌ | ⚠️ |
| True detour | ❌ | ❌ | ✅ |

