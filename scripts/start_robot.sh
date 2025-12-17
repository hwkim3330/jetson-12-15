#!/bin/bash
# KETI Robot Start Script
# Called by systemd service on boot

export HOME=/home/nvidia
export USER=nvidia
export ROS_DOMAIN_ID=30
export LIDAR_MODEL=LD19

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash

# Wait for system to be ready
sleep 3

# Launch robot
exec ros2 launch robot_bringup robot.launch.py
