#!/bin/bash
#
# XPressCan Captive Portal Setup
# Configures DNS redirection and nginx proxy for captive portal functionality
#

set -e

INTERFACE="wlan0"
GATEWAY_IP="10.42.0.1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(dirname "$SCRIPT_DIR")/config"

echo "========================================"
echo "  XPressCan Captive Portal Setup"
echo "========================================"
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo)"
    exit 1
fi

# Install nginx if not present
if ! command -v nginx &> /dev/null; then
    echo "Installing nginx..."
    apt-get update && apt-get install -y nginx
fi

# Create NetworkManager dnsmasq configuration for captive portal
# This redirects ALL DNS queries to the gateway
echo "Configuring DNS redirection for captive portal..."
mkdir -p /etc/NetworkManager/dnsmasq-shared.d
cat > /etc/NetworkManager/dnsmasq-shared.d/captive-portal.conf << EOF
# XPressCan Captive Portal DNS Configuration
# Redirect ALL DNS queries to gateway for captive portal detection

# Redirect all domains to the gateway IP
address=/#/$GATEWAY_IP
EOF

echo "DNS redirection configured"

# Install nginx configuration
echo "Configuring nginx as reverse proxy..."
if [ -f "$CONFIG_DIR/nginx-captive-portal.conf" ]; then
    cp "$CONFIG_DIR/nginx-captive-portal.conf" /etc/nginx/sites-available/captive-portal
else
    # Create inline if config file not found
    cat > /etc/nginx/sites-available/captive-portal << 'NGINX'
# XPressCan Captive Portal Nginx Configuration
server {
    listen 80 default_server;
    listen [::]:80 default_server;
    server_name _;

    location / {
        proxy_pass http://127.0.0.1:8080;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_buffering off;
    }
}
NGINX
fi

# Enable the nginx site
rm -f /etc/nginx/sites-enabled/default
ln -sf /etc/nginx/sites-available/captive-portal /etc/nginx/sites-enabled/captive-portal

# Test nginx configuration
nginx -t

# Reload nginx
systemctl reload nginx

# Disable standalone dnsmasq if enabled (NetworkManager handles it)
if systemctl is-enabled dnsmasq &>/dev/null; then
    echo "Disabling standalone dnsmasq (NetworkManager provides DNS)..."
    systemctl stop dnsmasq
    systemctl disable dnsmasq
fi

# Restart the hotspot to apply DNS configuration
echo "Restarting hotspot to apply DNS configuration..."
HOTSPOT_CONN=$(nmcli -t -f NAME,TYPE connection show --active | grep wifi | cut -d: -f1)
if [ -n "$HOTSPOT_CONN" ]; then
    nmcli connection down "$HOTSPOT_CONN" 2>/dev/null || true
    sleep 2
    nmcli connection up "$HOTSPOT_CONN"
fi

echo
echo "Captive portal setup complete!"
echo
echo "How it works:"
echo "  1. Device connects to hotspot"
echo "  2. NetworkManager's dnsmasq redirects ALL DNS to $GATEWAY_IP"
echo "  3. nginx proxies HTTP port 80 to Flask portal on 8080"
echo "  4. Captive portal detection URLs trigger popup on device"
echo
echo "Supported devices:"
echo "  - Android: /generate_204, /gen_204"
echo "  - iOS/macOS: /hotspot-detect.html"
echo "  - Windows: /connecttest.txt, /ncsi.txt"
echo "  - Firefox: /success.txt"
echo "  - Ubuntu: /canonical.html"
echo
