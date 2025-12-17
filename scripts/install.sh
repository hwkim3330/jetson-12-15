#!/bin/bash
# KETI Robot Installation Script
# Installs dependencies, builds workspace, and enables autostart service

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "==================================="
echo "  KETI Robot Installation"
echo "==================================="
echo "Workspace: $WS_DIR"
echo ""

# Check if running on Jetson
if ! uname -r | grep -q tegra; then
    echo "Warning: This script is designed for NVIDIA Jetson"
fi

# Step 1: Install ROS2 dependencies
echo "[1/5] Installing ROS2 dependencies..."
sudo apt update
sudo apt install -y \
    ros-humble-rosbridge-server \
    ros-humble-tf2-ros \
    ros-humble-nav2-bringup \
    ros-humble-cartographer-ros \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    python3-opencv \
    python3-pip

# Step 2: Build workspace
echo "[2/5] Building ROS2 workspace..."
cd "$WS_DIR"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Step 3: Setup environment
echo "[3/5] Setting up environment..."
if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# KETI Robot" >> ~/.bashrc
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    echo "export LIDAR_MODEL=LD19" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
fi

# Step 4: Install systemd service
echo "[4/5] Installing systemd service..."
chmod +x "$WS_DIR/scripts/start_robot.sh"
sudo cp "$WS_DIR/config/robot.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot.service
echo "  Service enabled: robot.service"

# Step 5: Sync web files
echo "[5/5] Syncing web files..."
if [ -f "$WS_DIR/scripts/sync_www.sh" ]; then
    chmod +x "$WS_DIR/scripts/sync_www.sh"
    "$WS_DIR/scripts/sync_www.sh" || true
fi

echo ""
echo "==================================="
echo "  Installation Complete!"
echo "==================================="
echo ""
echo "Commands:"
echo "  Start now:    sudo systemctl start robot"
echo "  Stop:         sudo systemctl stop robot"
echo "  Status:       sudo systemctl status robot"
echo "  Logs:         journalctl -u robot -f"
echo ""
echo "Access:"
echo "  Web UI:       http://<ROBOT_IP>:8888/"
echo ""
echo "Reboot to auto-start, or run:"
echo "  sudo systemctl start robot"
echo ""
