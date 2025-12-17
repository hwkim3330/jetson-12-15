#!/bin/bash
# Stop all robot ROS2 nodes
pkill -f "ros2" 2>/dev/null
pkill -f "robot_" 2>/dev/null
echo "Robot stopped"
