#!/bin/bash
#
# Fleet VPN Install Script
# Installs WireGuard, wstunnel, ttyd, and fleet services on a crawler.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WSTUNNEL_VERSION="10.5.1"
WSTUNNEL_URL="https://github.com/erebe/wstunnel/releases/download/v${WSTUNNEL_VERSION}/wstunnel_${WSTUNNEL_VERSION}_linux_arm64.tar.gz"
WSTUNNEL_BIN="/usr/local/bin/wstunnel"
TTYD_BIN="/usr/local/bin/ttyd"

# 1. Check root
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root."
    exit 1
fi

echo "=== Fleet VPN Installer ==="
echo ""

# 2. Install WireGuard
if command -v wg >/dev/null 2>&1; then
    echo "[OK] WireGuard already installed"
else
    echo "[INSTALL] Installing WireGuard..."
    apt-get update -qq
    apt-get install -y wireguard wireguard-tools
    echo "[OK] WireGuard installed"
fi

# 3. Install ttyd
if [ -f "$TTYD_BIN" ] || command -v ttyd >/dev/null 2>&1; then
    echo "[OK] ttyd already installed"
else
    echo "[INSTALL] Installing ttyd..."
    apt-get update -qq
    apt-get install -y ttyd
    # Symlink to /usr/local/bin if installed elsewhere
    if [ ! -f "$TTYD_BIN" ] && command -v ttyd >/dev/null 2>&1; then
        ln -sf "$(command -v ttyd)" "$TTYD_BIN"
    fi
    echo "[OK] ttyd installed"
fi

# 4. Install wstunnel
if [ -f "$WSTUNNEL_BIN" ]; then
    echo "[OK] wstunnel already installed at $WSTUNNEL_BIN"
else
    echo "[INSTALL] Downloading wstunnel v${WSTUNNEL_VERSION}..."
    TMPDIR=$(mktemp -d)
    curl -fsSL "$WSTUNNEL_URL" -o "$TMPDIR/wstunnel.tar.gz"
    tar -xzf "$TMPDIR/wstunnel.tar.gz" -C "$TMPDIR"
    cp "$TMPDIR/wstunnel" "$WSTUNNEL_BIN"
    chmod +x "$WSTUNNEL_BIN"
    rm -rf "$TMPDIR"
    echo "[OK] wstunnel v${WSTUNNEL_VERSION} installed to $WSTUNNEL_BIN"
fi

# 5. Install MQTT bridge dependencies
if pip3 show paho-mqtt >/dev/null 2>&1; then
    echo "[OK] paho-mqtt already installed"
else
    echo "[INSTALL] Installing paho-mqtt..."
    pip3 install paho-mqtt
    echo "[OK] paho-mqtt installed"
fi

if python3 -c "import yaml" 2>/dev/null; then
    echo "[OK] PyYAML already installed"
else
    echo "[INSTALL] Installing PyYAML..."
    apt-get update -qq
    apt-get install -y python3-yaml
    echo "[OK] PyYAML installed"
fi

# 6. Create directories
mkdir -p /opt/crawler
mkdir -p /etc/crawler
echo "[OK] Directories /opt/crawler and /etc/crawler ready"

# 7. Copy Python scripts
cp "$SCRIPT_DIR/vpn_register.py" /opt/crawler/vpn_register.py
cp "$SCRIPT_DIR/crawler_heartbeat.py" /opt/crawler/crawler_heartbeat.py
chmod +x /opt/crawler/vpn_register.py /opt/crawler/crawler_heartbeat.py
echo "[OK] Copied vpn_register.py and crawler_heartbeat.py to /opt/crawler/"

# 8. Copy service files
cp "$SCRIPT_DIR/services/"*.service /etc/systemd/system/
echo "[OK] Copied service files to /etc/systemd/system/"

# 9. Write default wstunnel URL if not present
if [ ! -f /etc/crawler/wstunnel_url ]; then
    echo "wss://edge-primeinspections.com/wstunnel" > /etc/crawler/wstunnel_url
    echo "[OK] Wrote default wstunnel URL to /etc/crawler/wstunnel_url"
else
    echo "[OK] /etc/crawler/wstunnel_url already exists"
fi

# 10. Create default MQTT config if not present
if [ ! -f /etc/crawler/mqtt.conf ]; then
    cat > /etc/crawler/mqtt.conf <<'MQTTCONF'
api_key: ''
broker: edge-primeinspections.com
enabled: false
hostname: ''
port: 443
publish_interval: 2.0
transport: websockets
use_tls: true
ws_path: /mqtt
topics:
  heartbeat:
    enabled: true
    interval: 10.0
  imu:
    enabled: true
    interval: 15.0
    fields:
      roll: true
      pitch: true
      cal_roll: true
      cal_pitch: true
  motor:
    enabled: true
    interval: 15.0
    fields:
      connected: true
      m1_speed: true
      m1_torque: true
      m2_speed: true
      m2_torque: true
  system:
    enabled: true
    interval: 60.0
    fields:
      cpu_temp: true
      ram: true
      storage: true
      revision: true
MQTTCONF
    echo "[OK] Created default MQTT config at /etc/crawler/mqtt.conf"
else
    echo "[OK] /etc/crawler/mqtt.conf already exists"
fi

# 11. Reload systemd
systemctl daemon-reload
echo "[OK] systemd daemon reloaded"

# 12. Enable services
systemctl enable wstunnel-client
systemctl enable crawler-vpn-register
systemctl enable crawler-heartbeat
systemctl enable ttyd
echo "[OK] Enabled services: wstunnel-client, crawler-vpn-register, crawler-heartbeat, ttyd"

# 13. Remove old WireGuard config for fresh registration
rm -f /etc/wireguard/wg0.conf /etc/wireguard/private.key
systemctl disable wg-quick@wg0 2>/dev/null || true
echo "[OK] Cleared old WireGuard config (fresh registration on next boot)"

# 14. Summary
echo ""
echo "=== Installation Summary ==="
echo "  WireGuard:          $(wg --version 2>/dev/null || echo 'installed')"
echo "  wstunnel:           $($WSTUNNEL_BIN --version 2>/dev/null || echo 'installed')"
echo "  ttyd:               $(ttyd --version 2>/dev/null || echo 'installed')"
echo "  paho-mqtt:          $(pip3 show paho-mqtt 2>/dev/null | grep Version || echo 'installed')"
echo "  VPN register:       /opt/crawler/vpn_register.py"
echo "  Heartbeat:          /opt/crawler/crawler_heartbeat.py"
echo "  MQTT config:        /etc/crawler/mqtt.conf"
echo "  Services enabled:   wstunnel-client, crawler-vpn-register, crawler-heartbeat, ttyd"
echo ""

# 15. Reminder
echo "=== Next Steps ==="
echo "  VPN token and MQTT API key are configured through the crawler"
echo "  web UI at config.html. Set hostname and reboot when ready."
echo ""
