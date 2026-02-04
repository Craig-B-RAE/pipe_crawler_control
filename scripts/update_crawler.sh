#!/bin/bash
#
# Crawler Update Script
# Called by update_manager.py when user clicks "Update" in the web UI.
# Pulls latest from GitHub main branch for all repos and redeploys.
#
set -e

WS=~/ros2_ws
SRC=$WS/src
WEB_SRC=$SRC/pipe_crawler_control/web
WEB_DEPLOY=/opt/xpresscan/html
WEB_DEPLOY2=/var/www/html

REPOS=(
  pipe_crawler_control
  clearlink_driver
  clearlink_interfaces
  roboclaw_driver2
  roboclaw_interfaces
)

echo "=== Crawler Update Started ==="

# Step 1: Pull latest main for all repos
for repo in "${REPOS[@]}"; do
  echo "Updating $repo..."
  cd "$SRC/$repo"
  git fetch origin main
  git checkout main
  git reset --hard origin/main
done

echo "All repos updated to latest main."

# Step 2: Deploy all files (web UI, backend, scripts) WITHOUT restart
echo "Running deploy script..."
sudo "$SRC/pipe_crawler_control/scripts/deploy.sh"

# Step 2b: Schedule delayed service restart (5 seconds)
# This allows the update_manager to send 'update_complete' status before services restart
echo "Scheduling service restart in 5 seconds..."
nohup bash -c 'sleep 5 && sudo systemctl restart xpresscan-crawler-ui pipe_crawler crawler-heartbeat 2>/dev/null' >/dev/null 2>&1 &

# Step 4: Set active system HTML if configured
ACTIVE_SYSTEM_FILE=~/active_system
if [ -f "$ACTIVE_SYSTEM_FILE" ]; then
  ACTIVE=$(cat "$ACTIVE_SYSTEM_FILE")
  if [ -f "$WEB_SRC/${ACTIVE}.html" ] && [ -d "$WEB_DEPLOY2" ]; then
    sudo cp "$WEB_SRC/${ACTIVE}.html" "$WEB_DEPLOY2/index.html"
    echo "Active system HTML deployed: ${ACTIVE}.html"
  fi
fi

# Step 5: Switch repos back to development for continued work
for repo in "${REPOS[@]}"; do
  cd "$SRC/$repo"
  git checkout development 2>/dev/null || true
done

# Step 7: Delete all WiFi connections except hotspot
echo "Resetting WiFi connections..."
HOTSPOT_CON="XPressCan-Hotspot"
nmcli -t -f NAME,TYPE connection show | while IFS=: read name type; do
    if [ "$type" = "802-11-wireless" ] && [ "$name" != "$HOTSPOT_CON" ]; then
        sudo nmcli connection delete "$name" 2>/dev/null || true
        echo "Deleted WiFi: $name"
    fi
done

# Step 8: Restart hotspot
sudo nmcli connection up "$HOTSPOT_CON" 2>/dev/null || true
echo "Hotspot restarted"

echo "=== Update complete! ==="
