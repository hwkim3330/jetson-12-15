#!/bin/bash
# Enable WiFi Client Mode for KETI Robot
# Connects to existing WiFi network

set -e

echo "Enabling Client Mode..."

# Stop AP services
sudo systemctl stop hostapd 2>/dev/null || true
sudo systemctl stop dnsmasq 2>/dev/null || true

# Start NetworkManager
sudo systemctl start NetworkManager

echo ""
echo "====================================="
echo "  Client Mode Enabled!"
echo "====================================="
echo "  Robot will connect to configured WiFi"
echo "  Check IP with: hostname -I"
echo "====================================="
