#!/bin/bash
#
# XPressCan Field Network - Client Connection Script
# Connects crawler to the configured hotspot with static IP
#

set -e

CONFIG_FILE="/etc/xpresscan/network.conf"
DEFAULT_SSID="Crawler"
DEFAULT_PASSWORD="Crawler1"
INTERFACE="wlan0"
CONNECTION_NAME="XPressCan-Client"

echo "========================================"
echo "  XPressCan Field Network Connection"
echo "========================================"
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo)"
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

# Determine IP address based on hostname or config
get_static_ip() {
    # Check config file first
    if [ -n "$STATIC_IP" ]; then
        echo "$STATIC_IP"
        return
    fi

    # Fall back to hostname-based assignment
    local hostname=$(hostname)
    case "$hostname" in
        autobug-1|autobug1)
            echo "10.42.0.11/24"
            ;;
        autobug-2|autobug2)
            echo "10.42.0.12/24"
            ;;
        *)
            echo "Error: Unknown hostname '$hostname'"
            echo "Expected: autobug-1 or autobug-2"
            echo "Or set STATIC_IP in $CONFIG_FILE"
            exit 1
            ;;
    esac
}

STATIC_IP=$(get_static_ip)
echo "Hostname: $(hostname)"
echo "Static IP: $STATIC_IP"
echo "Connecting to: $SSID"
echo

# Remove existing connection if it exists
if nmcli connection show "$CONNECTION_NAME" &> /dev/null; then
    echo "Removing existing connection..."
    nmcli connection delete "$CONNECTION_NAME"
fi

echo "Creating connection to $SSID..."

# Create the client connection with static IP
nmcli connection add \
    type wifi \
    ifname "$INTERFACE" \
    con-name "$CONNECTION_NAME" \
    autoconnect yes \
    ssid "$SSID" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD" \
    ipv4.method manual \
    ipv4.addresses "$STATIC_IP" \
    ipv4.gateway "10.42.0.1"

echo "Connecting to $SSID..."
nmcli connection up "$CONNECTION_NAME"

echo
echo "Connected successfully!"
echo "  Network: $SSID"
echo "  IP: $STATIC_IP"
echo "  Gateway: 10.42.0.1"
