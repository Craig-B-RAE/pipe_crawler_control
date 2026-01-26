#!/bin/bash
#
# WiFi Boot Manager
# Tries to connect to known WiFi networks first, falls back to hotspot if none available
#

CONFIG_FILE="/etc/xpresscan/network.conf"
HOTSPOT_CONNECTION="XPressCan-Hotspot"
INTERFACE="wlan0"
WIFI_SCAN_TIMEOUT=20
LOG_TAG="wifi-boot-manager"

log() {
    echo "$1"
    logger -t "$LOG_TAG" "$1"
}

# Load hotspot config
load_config() {
    if [ -f "$CONFIG_FILE" ]; then
        source "$CONFIG_FILE"
    fi
}

# Get list of saved WiFi connections (excluding hotspot)
get_saved_wifi_connections() {
    nmcli -t -f NAME,TYPE connection show | grep ":802-11-wireless$" | cut -d: -f1 | grep -v "^${HOTSPOT_CONNECTION}$"
}

# Check if a network is currently visible
is_network_visible() {
    local ssid="$1"
    nmcli -t -f SSID device wifi list 2>/dev/null | grep -qx "$ssid"
}

# Try to connect to a specific saved connection
try_connect() {
    local connection="$1"
    log "Attempting to connect to: $connection"

    if nmcli connection up "$connection" 2>/dev/null; then
        log "Successfully connected to: $connection"
        return 0
    else
        log "Failed to connect to: $connection"
        return 1
    fi
}

# Wait for WiFi hardware and scan for networks
wait_and_scan() {
    local max_wait=10
    local waited=0

    # Wait for wlan0 to be ready
    while [ $waited -lt $max_wait ]; do
        if ip link show "$INTERFACE" 2>/dev/null | grep -q "state UP\|state DOWN"; then
            break
        fi
        sleep 1
        waited=$((waited + 1))
    done

    # Trigger a WiFi scan
    nmcli device wifi rescan 2>/dev/null || true
    sleep 2

    # Get list of visible networks
    nmcli -t -f SSID device wifi list 2>/dev/null | sort -u | grep -v "^$"
}

# Update hotspot connection if settings changed
update_hotspot_if_needed() {
    load_config

    local current_ssid=$(nmcli -t -f 802-11-wireless.ssid connection show "$HOTSPOT_CONNECTION" 2>/dev/null | cut -d: -f2)
    local config_ssid="${HOTSPOT_SSID:-Crawler}"
    local config_password="${HOTSPOT_PASSWORD:-Crawler1}"

    if [ "$current_ssid" != "$config_ssid" ]; then
        log "Hotspot SSID changed from '$current_ssid' to '$config_ssid', updating connection..."
        nmcli connection modify "$HOTSPOT_CONNECTION" \
            802-11-wireless.ssid "$config_ssid" \
            wifi-sec.psk "$config_password" 2>/dev/null
    fi
}

# Start the hotspot
start_hotspot() {
    log "Starting hotspot mode..."
    update_hotspot_if_needed

    if nmcli connection up "$HOTSPOT_CONNECTION" 2>/dev/null; then
        log "Hotspot started successfully"
        return 0
    else
        log "Failed to start hotspot"
        return 1
    fi
}

# Main logic
main() {
    log "=== WiFi Boot Manager starting ==="

    # First, ensure we're not already connected
    local current_conn=$(nmcli -t -f GENERAL.CONNECTION device show "$INTERFACE" 2>/dev/null | grep "GENERAL.CONNECTION:" | cut -d: -f2)
    if [ -n "$current_conn" ] && [ "$current_conn" != "--" ] && [ "$current_conn" != "$HOTSPOT_CONNECTION" ]; then
        log "Already connected to: $current_conn"
        exit 0
    fi

    # Get saved WiFi connections
    local saved_connections=$(get_saved_wifi_connections)

    if [ -z "$saved_connections" ]; then
        log "No saved WiFi connections found, starting hotspot"
        start_hotspot
        exit $?
    fi

    log "Found saved connections: $(echo $saved_connections | tr '\n' ' ')"

    # Scan for available networks
    log "Scanning for available WiFi networks (timeout: ${WIFI_SCAN_TIMEOUT}s)..."
    local visible_networks=$(wait_and_scan)

    if [ -n "$visible_networks" ]; then
        log "Visible networks: $(echo $visible_networks | tr '\n' ' ')"
    else
        log "No visible networks found"
    fi

    # Try to connect to each saved connection that's visible
    local connected=0
    local start_time=$(date +%s)

    while IFS= read -r connection; do
        [ -z "$connection" ] && continue

        # Check if we've exceeded timeout
        local elapsed=$(($(date +%s) - start_time))
        if [ $elapsed -gt $WIFI_SCAN_TIMEOUT ]; then
            log "WiFi connection timeout reached"
            break
        fi

        # Check if this network is visible
        if echo "$visible_networks" | grep -qx "$connection"; then
            log "Network '$connection' is visible, trying to connect..."
            if try_connect "$connection"; then
                connected=1
                break
            fi
        else
            log "Network '$connection' not visible, skipping"
        fi
    done <<< "$saved_connections"

    # If no WiFi connected, start hotspot
    if [ $connected -eq 0 ]; then
        log "No WiFi connection established, starting hotspot"
        start_hotspot
    fi

    log "=== WiFi Boot Manager complete ==="
}

# Handle stop action
stop_hotspot() {
    log "Stopping hotspot..."
    nmcli connection down "$HOTSPOT_CONNECTION" 2>/dev/null || true
}

# Parse command line
case "${1:-start}" in
    start)
        main
        ;;
    stop)
        stop_hotspot
        ;;
    status)
        nmcli device show "$INTERFACE"
        ;;
    *)
        echo "Usage: $0 {start|stop|status}"
        exit 1
        ;;
esac
