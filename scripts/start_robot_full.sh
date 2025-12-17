#!/bin/bash
# KETI Robot Full System Start Script
# Starts: robot base, rosbridge, camera

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

# Wait for system to be ready
sleep 3

echo "Starting KETI Robot Full System..."

# Start rosbridge in background
echo "Starting rosbridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!
sleep 2

# Start robot base
echo "Starting robot base..."
ros2 launch robot_bringup robot.launch.py &
ROBOT_PID=$!
sleep 3

# Start camera (optional)
echo "Starting camera..."
ros2 run robot_ai jetson_camera_hw.py &
CAMERA_PID=$!

echo "All services started"
echo "  Rosbridge PID: $ROSBRIDGE_PID"
echo "  Robot PID: $ROBOT_PID"
echo "  Camera PID: $CAMERA_PID"

# Wait for any process to exit
wait -n $ROSBRIDGE_PID $ROBOT_PID

# If we get here, something crashed - exit to trigger restart
echo "Process exited, shutting down..."
kill $ROSBRIDGE_PID $ROBOT_PID $CAMERA_PID 2>/dev/null || true
exit 1
