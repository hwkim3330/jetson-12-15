#!/bin/bash
# RSSAEM Robot Auto-start Script

# Wait for system to be ready
sleep 10

# Source ROS2
source /opt/ros/humble/setup.bash
source /home/nvidia/rsaembot_ws/install/setup.bash

# Export environment variables
export LIDAR_MODEL=LDS-04
export RSSAEM_MODEL=rssaem
export ROS_DOMAIN_ID=30

# Log file
LOG_FILE=/home/nvidia/rsaembot_ws/autostart.log
echo "$(date): Starting RSSAEM Robot..." >> $LOG_FILE

# Launch the web interface (includes bringup)
ros2 launch rssaem_web web_interface.launch.py >> $LOG_FILE 2>&1
