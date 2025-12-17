#!/bin/bash
# KETI Robot Full System Start Script
# Starts: rosbridge, robot base, camera+AI, web_video_server

set -e

export HOME=/home/nvidia
export USER=nvidia
export ROS_DOMAIN_ID=30
export KETI_MODEL=burger
export LDS_MODEL=LDS-04
export LIDAR_MODEL=LD19

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash

# Wait for system
sleep 5

echo "=== Starting KETI Robot Full System ==="
echo "Time: $(date)"

# Kill any existing ROS processes
pkill -f "ros2" 2>/dev/null || true
sleep 2

# 1. Start rosbridge (for web interface)
echo "[1/4] Starting rosbridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
sleep 3

# 2. Start robot base (LiDAR, motors, odometry)
echo "[2/4] Starting robot base..."
ros2 launch robot_bringup robot.launch.py &
sleep 5

# 3. Start camera + AI pipeline
echo "[3/4] Starting camera + AI..."
ros2 run robot_ai camera_ai_pipeline.py &
sleep 2

# 4. Start web_video_server (if available)
echo "[4/4] Starting video server..."
ros2 run web_video_server web_video_server 2>/dev/null &

echo ""
echo "=== KETI Robot System Started ==="
echo "Web Interface: http://10.42.0.1/"
echo ""

# Keep running
wait
