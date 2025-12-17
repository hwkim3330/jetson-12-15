#!/bin/bash
# KETI Robot Full Setup Script
# Run with sudo: sudo ./full_setup.sh

set -e

echo "=========================================="
echo "    KETI Robot Full System Setup"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo $0"
    exit 1
fi

SCRIPTS_DIR="/home/nvidia/ros2_ws/scripts"
CONFIG_DIR="/home/nvidia/ros2_ws/config"

# 1. Setup WiFi AP
echo "[1/5] Setting up WiFi Access Point..."
"$SCRIPTS_DIR/setup_wifi_ap.sh" || echo "WiFi setup skipped"

# 2. Setup Web Server
echo ""
echo "[2/5] Setting up Web Server..."
"$SCRIPTS_DIR/setup_webserver.sh" || echo "Web server setup failed"

# 3. Install systemd services
echo ""
echo "[3/5] Installing systemd services..."

# Copy service files
cp "$CONFIG_DIR/robot.service" /etc/systemd/system/
cp "$CONFIG_DIR/robot-full.service" /etc/systemd/system/

# Reload systemd
systemctl daemon-reload

# Disable old service, enable new one
systemctl disable robot.service 2>/dev/null || true
systemctl enable robot-full.service
systemctl start robot-full.service || echo "Service start failed (may need reboot)"

# 4. Setup udev rules
echo ""
echo "[4/5] Setting up udev rules..."
UDEV_RULES="/home/nvidia/ros2_ws/src/keti/robot_bringup/script/99-robot-cdc.rules"
if [ -f "$UDEV_RULES" ]; then
    cp "$UDEV_RULES" /etc/udev/rules.d/
    udevadm control --reload-rules
    udevadm trigger
    echo "Udev rules installed"
else
    echo "Udev rules file not found"
fi

# 5. Create AI TensorRT engine
echo ""
echo "[5/5] TensorRT setup info..."
echo ""
echo "To create TensorRT engine for YOLO, run:"
echo "  python3 /home/nvidia/ros2_ws/scripts/create_tensorrt_engine.py"
echo ""

echo "=========================================="
echo "    Setup Complete!"
echo "=========================================="
echo ""
echo "System Configuration:"
echo "  WiFi SSID: KETI_ROBOT"
echo "  WiFi Password: 12345678"
echo "  Web Interface: http://192.168.10.1:8888/"
echo ""
echo "Services:"
echo "  robot-full.service: $(systemctl is-enabled robot-full.service)"
echo "  nginx.service: $(systemctl is-enabled nginx.service)"
echo ""
echo "Please reboot to apply all changes:"
echo "  sudo reboot"
