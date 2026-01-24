#!/bin/bash
#
# XPressCan Field Hotspot Setup
# Creates a WiFi Access Point for field operations
#

set -e

CONFIG_FILE="/etc/xpresscan/network.conf"
DEFAULT_SSID="Crawler"
DEFAULT_PASSWORD="Crawler1"
INTERFACE="wlan0"
CHANNEL="36"
IP_ADDRESS="10.42.0.1/24"
CONNECTION_NAME="XPressCan-Hotspot"

echo "========================================"
echo "  XPressCan Field Hotspot Setup"
echo "========================================"
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo)"
    exit 1
fi

# Check if NetworkManager is available
if ! command -v nmcli &> /dev/null; then
    echo "Error: NetworkManager (nmcli) not found"
    exit 1
fi

# Check if interface exists
if ! ip link show "$INTERFACE" &> /dev/null; then
    echo "Error: Interface $INTERFACE not found"
    exit 1
fi

# Load config if it exists
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    SSID="${HOTSPOT_SSID:-$DEFAULT_SSID}"
    PASSWORD="${HOTSPOT_PASSWORD:-$DEFAULT_PASSWORD}"
    echo "Loaded configuration from $CONFIG_FILE"
else
    SSID="$DEFAULT_SSID"
    PASSWORD="$DEFAULT_PASSWORD"
    echo "Using default configuration"
fi

# Remove existing connection if it exists
if nmcli connection show "$CONNECTION_NAME" &> /dev/null; then
    echo "Removing existing hotspot connection..."
    nmcli connection delete "$CONNECTION_NAME"
fi

echo "Creating hotspot: $SSID on $INTERFACE (5GHz channel $CHANNEL)"

# Create the hotspot connection
nmcli connection add \
    type wifi \
    ifname "$INTERFACE" \
    con-name "$CONNECTION_NAME" \
    autoconnect yes \
    ssid "$SSID" \
    mode ap \
    ipv4.method shared \
    ipv4.addresses "$IP_ADDRESS" \
    wifi.band a \
    wifi.channel "$CHANNEL" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD"

echo "Activating hotspot..."
nmcli connection up "$CONNECTION_NAME"

echo
echo "Hotspot created successfully!"
echo "  SSID: $SSID"
echo "  Password: $PASSWORD"
echo "  IP Address: $IP_ADDRESS"
echo "  Channel: $CHANNEL (5GHz)"
echo
echo "Clients can connect to this network."
echo "Router IP: 10.42.0.1"
