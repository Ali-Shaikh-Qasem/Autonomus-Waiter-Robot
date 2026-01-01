#!/bin/bash
# Quick start script for ultrasonic-integrated navigation
# ROS2 Humble + Gazebo + Nav2 + Ultrasonic RangeSensorLayer

echo "==================================================================="
echo "  Ultrasonic Range Integration - Quick Start"
echo "==================================================================="
echo ""

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced! Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

WORKSPACE_DIR="$HOME/Autonomous-Waiter-Robot-Simulations/dev_ws"

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "❌ Workspace not found at $WORKSPACE_DIR"
    exit 1
fi

cd "$WORKSPACE_DIR"

echo "📦 Building my_bot package..."
colcon build --packages-select my_bot --symlink-install

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✅ Build successful!"
echo ""
echo "🚀 Ready to launch! Run these in separate terminals:"
echo ""
echo "Terminal 1 - Gazebo Simulation:"
echo "  cd $WORKSPACE_DIR"
echo "  source install/setup.bash"
echo "  ros2 launch my_bot launch_sim.launch.py"
echo ""
echo "Terminal 2 - Navigation with Ultrasonic Integration:"
echo "  cd $WORKSPACE_DIR"
echo "  source install/setup.bash"
echo "  ros2 launch my_bot nav_with_ultrasonic_range_layer.launch.py map:=/path/to/your_map.yaml"
echo ""
echo "Terminal 3 - RViz:"
echo "  cd $WORKSPACE_DIR"
echo "  source install/setup.bash"
echo "  ros2 launch my_bot launch_rviz.launch.py"
echo ""
echo "📊 Monitor ultrasonic sensors:"
echo "  ros2 topic list | grep /us/"
echo "  ros2 topic echo /us/front_up"
echo ""
echo "📖 See ULTRASONIC_INTEGRATION_GUIDE.md for full documentation"
echo "==================================================================="
