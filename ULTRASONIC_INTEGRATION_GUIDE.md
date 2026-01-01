# Ultrasonic Range Sensor Integration for Nav2

This package integrates ultrasonic sensors with Nav2's local costmap using the `RangeSensorLayer` plugin. This enables proper sonar-based obstacle detection for low obstacles that might be missed by the LiDAR.

## Overview

The integration consists of three main components:

1. **Converter Node** (`ultrasonic_scan_to_range.py`): Converts ultrasonic LaserScan messages to Range messages
2. **Nav2 Configuration**: Modified `nav2_params.yaml` with RangeSensorLayer for local costmap
3. **Launch File**: Integrated navigation launch that starts both Nav2 and the converter node

## Architecture

```
Gazebo Ultrasonic Sensors (6x)
    ↓ (sensor_msgs/LaserScan)
    ↓ Topics: /ultrasonic_scan_{front,left,right}_{up,down}
    ↓
ultrasonic_scan_to_range.py (Converter Node)
    ↓ (sensor_msgs/Range)
    ↓ Topics: /us/{front,left,right}_{up,down}
    ↓
Nav2 RangeSensorLayer (local_costmap only)
    ↓
Controller Server (DWB) steers around obstacles
```

## Sensor Configuration

### Ultrasonic Sensors (6 total)

The robot has 6 ultrasonic sensors defined in `description/ultrasonic.xacr`:

- **Front Up**: `ultrasonic_frame_front_up` at x=0.35, z=0.20
- **Front Down**: `ultrasonic_frame_front_down` at x=0.35, z=0.0
- **Left Up**: `ultrasonic_frame_left_up` at x=0.29, y=0.13, z=0.20 (yaw=0.7854 rad ≈ 45°)
- **Left Down**: `ultrasonic_frame_left_down` at x=0.29, y=0.13, z=0.0 (yaw=0.7854 rad)
- **Right Up**: `ultrasonic_frame_right_up` at x=0.29, y=-0.13, z=0.20 (yaw=-0.7854 rad ≈ -45°)
- **Right Down**: `ultrasonic_frame_right_down` at x=0.29, y=-0.13, z=0.0 (yaw=-0.7854 rad)

### Sensor Specifications

- **Range**: 0.02m to 4.0m
- **Field of View**: ~0.24 radians (~14°)
- **Update Rate**: 20 Hz (configured in Gazebo)
- **Scan Samples**: 80 samples per scan (-0.12 to +0.12 rad)

## Usage

### 1. Build the Package

```bash
cd ~/Autonomous-Waiter-Robot-Simulations/dev_ws
colcon build --packages-select my_bot --symlink-install
source install/setup.bash
```

### 2. Launch Simulation with Navigation

You have two options:

#### Option A: Use the Integrated Launch File (Recommended)

This starts Nav2 with ultrasonic support in one command:

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch my_bot launch_sim.launch.py

# Terminal 2: Launch navigation with ultrasonic integration
ros2 launch my_bot nav_with_ultrasonic_range_layer.launch.py map:=/path/to/your/map.yaml

# Terminal 3: Launch RViz
ros2 launch my_bot launch_rviz.launch.py
```

#### Option B: Manual Launch (for debugging)

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch my_bot launch_sim.launch.py

# Terminal 2: Start converter node
ros2 run my_bot ultrasonic_scan_to_range.py --ros-args -p use_sim_time:=true

# Terminal 3: Start Nav2 with custom params
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/your/dev_ws/src/my_bot/config/nav2_params.yaml

# Terminal 4: Launch RViz
ros2 launch my_bot launch_rviz.launch.py
```

### 3. Verify in RViz

1. Add display: **Map** → Topic: `/local_costmap/costmap`
2. Add display: **Range** → Topic: `/us/front_up` (repeat for all 6 sensors)
3. Check that:
   - Range cones are visible from each ultrasonic sensor
   - Local costmap shows obstacles detected by ultrasonics
   - Robot avoids low obstacles that ultrasonics detect

## Monitoring and Debugging

### Check Converter Node Status

The converter node logs sensor readings every second:

```bash
ros2 run my_bot ultrasonic_scan_to_range.py --ros-args -p use_sim_time:=true
```

Expected output:
```
[INFO] [ultrasonic_scan_to_range]: Configured: /ultrasonic_scan_front_up -> /us/front_up
[INFO] [ultrasonic_scan_to_range]: Configured: /ultrasonic_scan_front_down -> /us/front_down
...
[INFO] [ultrasonic_scan_to_range]: Front: front_up:1.23m, front_down:0.98m | Left: left_up:2.45m, left_down:2.10m | Right: right_up:3.50m, right_down:3.20m
```

### Check Range Topics

```bash
# List all Range topics
ros2 topic list | grep /us/

# Echo a specific Range topic
ros2 topic echo /us/front_up

# Check message rate
ros2 topic hz /us/front_up
```

### Check LaserScan Topics (Raw Ultrasonic Data)

```bash
# List ultrasonic LaserScan topics
ros2 topic list | grep ultrasonic_scan

# Echo raw ultrasonic data
ros2 topic echo /ultrasonic_scan_front_up
```

### Verify TF Frames

```bash
# Check that all ultrasonic frames exist
ros2 run tf2_ros tf2_echo base_link ultrasonic_frame_front_up
ros2 run tf2_ros tf2_echo base_link ultrasonic_frame_left_up
ros2 run tf2_ros tf2_echo base_link ultrasonic_frame_right_up

# View TF tree
ros2 run tf2_tools view_frames
```

### Inspect Local Costmap Configuration

```bash
# Get local_costmap parameters
ros2 param list /local_costmap/local_costmap

# Check RangeSensorLayer config
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /local_costmap/local_costmap range_layer.topics
ros2 param get /local_costmap/local_costmap range_layer.phi
```

## Configuration Parameters

### Converter Node Parameters

Located in `nav_with_ultrasonic_range_layer.launch.py`:

```python
parameters=[{
    'use_sim_time': True,
    'min_range': 0.02,      # Minimum valid range (meters)
    'max_range': 4.0,       # Maximum valid range (meters)
    'fov_rad': 0.24,        # Field of view (radians)
    'timeout_sec': 0.5,     # Timeout for stale sensor detection
}]
```

### RangeSensorLayer Parameters

Located in `config/nav2_params.yaml`:

```yaml
range_layer:
  plugin: "nav2_costmap_2d::RangeSensorLayer"
  enabled: true
  topics: ["/us/front_up", "/us/front_down", ...]  # Range topics to subscribe
  no_readings_timeout: 0.5      # Clear obstacles if no readings (seconds)
  clear_on_max_reading: true    # Clear when sensor reads max_range
  phi: 0.24                     # Field of view (radians)
  inflate_cone: 1.0             # Cone inflation factor
  mark_threshold: 0.7           # Probability threshold to mark obstacle
  clear_threshold: 0.3          # Probability threshold to clear obstacle
```

### Tuning Guide

**For more aggressive obstacle avoidance:**
- Increase `inflate_cone` (e.g., 1.2-1.5)
- Decrease `mark_threshold` (e.g., 0.5)
- Increase `BaseObstacle.scale` in DWB critic

**For less conservative behavior:**
- Decrease `inflate_cone` (e.g., 0.8)
- Increase `mark_threshold` (e.g., 0.8)
- Decrease inflation_radius in inflation_layer

**For faster response:**
- Decrease `no_readings_timeout` (e.g., 0.3)
- Increase local_costmap `update_frequency`

## Troubleshooting

### Ultrasonics not appearing in local costmap

1. **Check Range topics are publishing:**
   ```bash
   ros2 topic list | grep /us/
   ros2 topic echo /us/front_up
   ```

2. **Verify converter node is running:**
   ```bash
   ros2 node list | grep ultrasonic
   ```

3. **Check TF frames exist:**
   ```bash
   ros2 run tf2_ros tf2_echo base_link ultrasonic_frame_front_up
   ```

4. **Verify RangeSensorLayer is loaded:**
   ```bash
   ros2 param get /local_costmap/local_costmap plugins
   ```

### Robot ignores ultrasonic obstacles

1. **Check local costmap visualization in RViz** - obstacles should appear
2. **Verify inflation settings** - might be too small
3. **Check DWB BaseObstacle critic weight** - might need increase
4. **Verify use_sim_time is consistent** across all nodes

### Converter node warnings about stale sensors

- **Cause**: One or more ultrasonic LaserScan topics not publishing
- **Check**: `ros2 topic list | grep ultrasonic_scan`
- **Verify**: Gazebo simulation is running and sensors are enabled

### Range sensor detection too sensitive/not sensitive enough

Adjust in `nav2_params.yaml`:
- `mark_threshold`: Lower = more sensitive (marks obstacles easier)
- `clear_threshold`: Higher = clears obstacles faster
- `inflate_cone`: Higher = larger detection cone

## Key Implementation Details

### Why Convert LaserScan to Range?

Nav2's `RangeSensorLayer` is designed for sensor_msgs/Range (sonar/IR), not LaserScan. Range messages represent:
- **Single distance reading** per sensor (not a scan)
- **Cone-shaped detection** (field of view)
- **Proper uncertainty modeling** for sonar

### Why Only Local Costmap?

- Ultrasonic sensors detect **dynamic/temporary** obstacles (e.g., low tables, chairs)
- These should **not** be permanently added to the global map
- Local costmap updates in real-time as robot moves
- This matches Nav2's design for sensor-based obstacle avoidance

### TF Frame Consistency

The converter preserves the **exact frame_id** from the incoming LaserScan. This is critical because:
- Nav2 RangeSensorLayer uses TF to project range cones
- Frame must match the URDF link name (e.g., `ultrasonic_frame_front_up`)
- Any mismatch causes TF lookup failures

## File Structure

```
src/my_bot/
├── config/
│   └── nav2_params.yaml              # Modified with RangeSensorLayer
├── description/
│   └── ultrasonic.xacr               # Ultrasonic sensor URDF definitions
├── launch/
│   └── nav_with_ultrasonic_range_layer.launch.py  # Integrated navigation launch
├── scripts/
│   └── ultrasonic_scan_to_range.py   # LaserScan→Range converter node
├── CMakeLists.txt                     # Updated to install new script
└── package.xml                        # Updated with dependencies
```

## Testing Procedure

1. **Static Obstacle Test**:
   - Place low obstacle (z < 0.2m) in front of robot in Gazebo
   - Verify ultrasonic detects it in RViz (/us/front_down topic)
   - Verify obstacle appears in local costmap
   - Command navigation goal - robot should avoid obstacle

2. **Dynamic Obstacle Test**:
   - Insert obstacle during navigation
   - Robot should react and replan around it
   - After obstacle removed, costmap should clear

3. **Side Obstacle Test**:
   - Place obstacles at 45° (left/right sensors)
   - Verify left_up/left_down or right_up/right_down detect
   - Robot should avoid during navigation

## References

- [Nav2 RangeSensorLayer Documentation](https://navigation.ros.org/configuration/packages/costmap-plugins/range.html)
- [sensor_msgs/Range Message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html)
- ROS2 Humble Documentation
- Nav2 Costmap 2D Package

## License

See package LICENSE.md

---

**Created**: January 2026
**ROS2 Version**: Humble
**Simulation**: Gazebo Classic
**Navigation**: Nav2
