#!/bin/bash
# KETI Robot WiFi AP Setup Script
# Run with sudo: sudo ./setup_wifi_ap.sh

set -e

SSID="KETI_ROBOT"
PASSWORD="12345678"
CONNECTION_NAME="KETI-AP"
OLD_CONNECTION="RSSAEM-AP"

echo "=== KETI Robot WiFi AP Setup ==="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run with sudo: sudo $0"
    exit 1
fi

# Check if old connection exists
if nmcli connection show "$OLD_CONNECTION" &>/dev/null; then
    echo "Found old connection: $OLD_CONNECTION"
    echo "Modifying SSID to: $SSID"

    # Modify existing connection
    nmcli connection modify "$OLD_CONNECTION" \
        802-11-wireless.ssid "$SSID" \
        connection.id "$CONNECTION_NAME"

    echo "Restarting connection..."
    nmcli connection down "$CONNECTION_NAME" 2>/dev/null || true
    sleep 1
    nmcli connection up "$CONNECTION_NAME"

    echo "WiFi AP updated successfully!"
else
    echo "Creating new WiFi AP: $SSID"

    # Delete old hotspot if exists
    nmcli connection delete "$CONNECTION_NAME" 2>/dev/null || true

    # Create new AP
    nmcli device wifi hotspot \
        ifname wlP1p1s0 \
        con-name "$CONNECTION_NAME" \
        ssid "$SSID" \
        password "$PASSWORD"

    # Set to auto-connect with high priority
    nmcli connection modify "$CONNECTION_NAME" \
        connection.autoconnect yes \
        connection.autoconnect-priority 100

    echo "WiFi AP created successfully!"
fi

echo ""
echo "=== WiFi AP Configuration ==="
echo "  SSID: $SSID"
echo "  Password: $PASSWORD"
echo "  IP: 10.42.0.1"
echo ""
