#!/bin/bash
#
# XPressCan Hotspot System Installer
# Installs and configures the WiFi hotspot and captive portal system
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="$(dirname "$SCRIPT_DIR")/../systemd"
INSTALL_DIR="/opt/xpresscan"

echo "========================================"
echo "  XPressCan Hotspot System Installer"
echo "========================================"
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo)"
    exit 1
fi

# Check for required packages
echo "Checking dependencies..."

if ! command -v nmcli &> /dev/null; then
    echo "Installing NetworkManager..."
    apt-get update && apt-get install -y network-manager
fi

if ! python3 -c "import flask" &> /dev/null 2>&1; then
    echo "Installing Flask..."
    apt-get install -y python3-flask || pip3 install flask
fi

if ! command -v nginx &> /dev/null; then
    echo "Installing nginx..."
    apt-get install -y nginx
fi

# Create install directory
echo "Creating installation directory..."
mkdir -p "$INSTALL_DIR"
mkdir -p /etc/xpresscan

# Copy scripts
echo "Installing scripts..."
cp "$SCRIPT_DIR/setup_hotspot.sh" "$INSTALL_DIR/"
cp "$SCRIPT_DIR/connect_to_field.sh" "$INSTALL_DIR/"
cp "$SCRIPT_DIR/wifi_boot_manager.sh" "$INSTALL_DIR/"
cp "$SCRIPT_DIR/captive_portal.py" "$INSTALL_DIR/"
cp "$SCRIPT_DIR/crawler_ui.py" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR"/*.sh
chmod +x "$INSTALL_DIR"/*.py

# Copy config files
CONFIG_DIR="$(dirname "$SCRIPT_DIR")/config"
if [ -d "$CONFIG_DIR" ]; then
    mkdir -p "$INSTALL_DIR/config"
    cp "$CONFIG_DIR"/* "$INSTALL_DIR/config/" 2>/dev/null || true
fi

# Set up captive portal (dnsmasq and iptables)
echo "Setting up captive portal..."
"$INSTALL_DIR/setup_captive_portal.sh"

# Create symlinks for easy access
ln -sf "$INSTALL_DIR/setup_hotspot.sh" /usr/local/bin/xpresscan-hotspot
ln -sf "$INSTALL_DIR/connect_to_field.sh" /usr/local/bin/xpresscan-connect

# Install systemd services
echo "Installing systemd services..."
cp "$SYSTEMD_DIR/xpresscan-hotspot.service" /etc/systemd/system/
cp "$SYSTEMD_DIR/xpresscan-portal.service" /etc/systemd/system/
cp "$SYSTEMD_DIR/xpresscan-crawler-ui.service" /etc/systemd/system/

# Set up captive portal DNS redirect
echo "Configuring captive portal DNS..."
mkdir -p /etc/NetworkManager/dnsmasq-shared.d
cat > /etc/NetworkManager/dnsmasq-shared.d/captive-portal.conf << EOF
# XPressCan Captive Portal DNS Configuration
address=/#/10.42.0.1
EOF

# Reload systemd
systemctl daemon-reload

echo
echo "Installation complete!"
echo
echo "Usage:"
echo "  1. Enable services:"
echo "     sudo systemctl enable xpresscan-hotspot"
echo "     sudo systemctl enable xpresscan-portal"
echo "     sudo systemctl enable xpresscan-crawler-ui"
echo "     sudo systemctl start xpresscan-hotspot"
echo "     sudo systemctl start xpresscan-portal"
echo "     sudo systemctl start xpresscan-crawler-ui"
echo
echo "  2. Web Interfaces:"
echo "     Crawler UI:      http://autobug.local (port 80)"
echo "                      - System status, ROS2 nodes, controls"
echo "     Config Portal:   http://10.42.0.1:8080 (via hotspot)"
echo "                      - Network settings, crawler identity"
echo "                      - Opens automatically via captive portal"
echo
echo "  3. Manual client connection:"
echo "     sudo $INSTALL_DIR/connect_to_field.sh"
echo
echo "Network:"
echo "  Router/Master: 10.42.0.1"
echo "  autobug-1:     10.42.0.11"
echo "  autobug-2:     10.42.0.12"
echo "  Default SSID:  Crawler"
echo "  Default Pass:  Crawler1"
