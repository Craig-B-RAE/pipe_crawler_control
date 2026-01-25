#!/bin/bash
#
# Remove Captive Portal DNS Hijacking
# Keeps the hotspot functionality but removes DNS redirection
#

set -e

echo "========================================"
echo "  Removing Captive Portal Hijacking"
echo "========================================"
echo

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo)"
    exit 1
fi

# Remove DNS hijacking configuration
if [ -f /etc/NetworkManager/dnsmasq-shared.d/captive-portal.conf ]; then
    echo "Removing DNS hijacking configuration..."
    rm -f /etc/NetworkManager/dnsmasq-shared.d/captive-portal.conf
    echo "DNS hijacking removed"
else
    echo "DNS hijacking config not found (already removed)"
fi

# Remove nginx captive portal config if present
if [ -f /etc/nginx/sites-enabled/captive-portal ]; then
    echo "Removing nginx captive portal config..."
    rm -f /etc/nginx/sites-enabled/captive-portal
    rm -f /etc/nginx/sites-available/captive-portal

    # Restore default nginx site if it exists
    if [ -f /etc/nginx/sites-available/default ]; then
        ln -sf /etc/nginx/sites-available/default /etc/nginx/sites-enabled/default
    fi

    # Reload nginx
    systemctl reload nginx 2>/dev/null || true
    echo "nginx config cleaned up"
fi

# Restart NetworkManager to apply DNS changes
echo "Restarting NetworkManager..."
systemctl restart NetworkManager

# Wait for network to stabilize
sleep 3

# Restart hotspot if active
HOTSPOT_CONN=$(nmcli -t -f NAME,TYPE connection show --active 2>/dev/null | grep wifi | cut -d: -f1)
if [ -n "$HOTSPOT_CONN" ]; then
    echo "Restarting hotspot: $HOTSPOT_CONN"
    nmcli connection down "$HOTSPOT_CONN" 2>/dev/null || true
    sleep 2
    nmcli connection up "$HOTSPOT_CONN" 2>/dev/null || true
fi

echo
echo "Captive portal hijacking removed!"
echo
echo "The hotspot will still work, but devices won't be auto-redirected."
echo "Users should navigate directly to http://autobug.local"
echo
