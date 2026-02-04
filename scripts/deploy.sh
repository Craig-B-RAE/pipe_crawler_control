#!/bin/bash
#
# Deploy Script — pipe_crawler_control
# Copies all deployable files from the repo to their system targets.
# Must be run as root.
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RESTART=false

if [ "$1" = "--restart" ]; then
    RESTART=true
fi

# Check root
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root."
    exit 1
fi

echo "=== Deploying pipe_crawler_control ==="
echo "  Repo: $REPO_DIR"
echo ""

# Clean __pycache__ from repo to prevent git conflicts
find "$REPO_DIR" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
echo "[OK] Cleaned __pycache__"

# Ensure target directories exist
mkdir -p /opt/xpresscan/html
mkdir -p /opt/crawler

# 1. Web UI
rsync -a --delete --exclude='__pycache__' "$REPO_DIR/web/" /opt/xpresscan/html/
echo "[OK] Web UI -> /opt/xpresscan/html/"

# 2. Flask backend
cp "$REPO_DIR/scripts/network/crawler_ui.py" /opt/xpresscan/crawler_ui.py
echo "[OK] crawler_ui.py -> /opt/xpresscan/"

# 3. Network scripts -> /opt/xpresscan/
for f in captive_portal.py report_graphs.py setup_hotspot.sh connect_to_field.sh install_hotspot.sh wifi_boot_manager.sh setup_captive_portal.sh; do
    if [ -f "$REPO_DIR/scripts/network/$f" ]; then
        cp "$REPO_DIR/scripts/network/$f" "/opt/xpresscan/$f"
    fi
done
echo "[OK] Network scripts -> /opt/xpresscan/"

# 4. security_utils.py -> /opt/crawler/
if [ -f "$REPO_DIR/scripts/network/security_utils.py" ]; then
    cp "$REPO_DIR/scripts/network/security_utils.py" /opt/crawler/security_utils.py
    echo "[OK] security_utils.py -> /opt/crawler/"
fi

# 5. Fleet scripts -> /opt/crawler/
for f in vpn_register.py crawler_heartbeat.py; do
    if [ -f "$REPO_DIR/fleet/$f" ]; then
        cp "$REPO_DIR/fleet/$f" "/opt/crawler/$f"
        chmod +x "/opt/crawler/$f"
    fi
done
echo "[OK] Fleet scripts -> /opt/crawler/"

# 6. Systemd service files
SERVICES_COPIED=0
if [ -d "$REPO_DIR/systemd" ]; then
    cp "$REPO_DIR/systemd/"*.service /etc/systemd/system/ 2>/dev/null && SERVICES_COPIED=1
fi
if [ -d "$REPO_DIR/fleet/services" ]; then
    cp "$REPO_DIR/fleet/services/"*.service /etc/systemd/system/ 2>/dev/null && SERVICES_COPIED=1
fi
if [ "$SERVICES_COPIED" -eq 1 ]; then
    systemctl daemon-reload
    echo "[OK] Systemd services updated and daemon reloaded"
else
    echo "[OK] No systemd service files found (skipped)"
fi

# 7. Restart services if requested
if [ "$RESTART" = true ]; then
    echo ""
    echo "=== Restarting services ==="
    systemctl restart xpresscan-crawler-ui 2>/dev/null && echo "[OK] Restarted xpresscan-crawler-ui" || echo "[SKIP] xpresscan-crawler-ui not active"
    systemctl restart pipe_crawler 2>/dev/null && echo "[OK] Restarted pipe_crawler" || echo "[SKIP] pipe_crawler not active"
    systemctl restart crawler-heartbeat 2>/dev/null && echo "[OK] Restarted crawler-heartbeat" || echo "[SKIP] crawler-heartbeat not active"
fi

echo ""
echo "=== Deploy complete ==="
