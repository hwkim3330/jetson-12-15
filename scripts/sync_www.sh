#!/bin/bash
# Sync web interface files to nginx directory
# Run after modifying www/ files

set -e

SRC="/home/nvidia/ros2_ws/install/robot_web/share/robot_web/www"
DST="/var/www/robot"

echo "Syncing web files..."
sudo rm -rf "$DST"
sudo mkdir -p "$DST"
sudo cp -rL "$SRC"/* "$DST"/
sudo chown -R www-data:www-data "$DST"
sudo chmod -R 644 "$DST"/*
sudo chmod 755 "$DST" "$DST"/js

echo "Done! Files synced to $DST"
echo "Access: http://$(hostname -I | awk '{print $1}')/"
