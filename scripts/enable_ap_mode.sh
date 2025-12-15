#!/bin/bash
# Enable WiFi AP Mode for RSAEM Robot
# Creates WiFi hotspot for offline operation

set -e

echo "Enabling AP Mode..."
echo "This will disconnect from current WiFi network!"
read -p "Continue? (y/n) " -n 1 -r
echo

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

# Stop NetworkManager for WiFi
sudo systemctl stop NetworkManager 2>/dev/null || true

# Start AP services
sudo systemctl start hostapd
sudo systemctl start dnsmasq

echo ""
echo "====================================="
echo "  AP Mode Enabled!"
echo "====================================="
echo "  SSID: RSAEM_Robot"
echo "  Password: rsaem1234"
echo "  IP: 192.168.4.1"
echo ""
echo "  Connect to: http://192.168.4.1/"
echo "====================================="
