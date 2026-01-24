#!/bin/bash
#
# Pipe Crawler System Setup
# Selects the active crawler configuration and sets up web interface
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="$PKG_DIR/config"
WEB_DIR="$PKG_DIR/web"
ACTIVE_SYSTEM_FILE="$HOME/active_system"
WEB_ROOT="/var/www/html"

# Define available systems
declare -A SYSTEMS
SYSTEMS["1"]="magnetic_phased_array"
SYSTEMS["2"]="edgeflex"
SYSTEMS["3"]="new_xpress_scan"

declare -A SYSTEM_NAMES
SYSTEM_NAMES["magnetic_phased_array"]="Magnetic Phased Array Crawler"
SYSTEM_NAMES["edgeflex"]="EdgeFlex"
SYSTEM_NAMES["new_xpress_scan"]="New Xpress Scan"

show_current() {
    if [ -f "$ACTIVE_SYSTEM_FILE" ]; then
        current=$(cat "$ACTIVE_SYSTEM_FILE")
        name="${SYSTEM_NAMES[$current]:-Unknown}"
        echo "Current system: $name ($current)"
    else
        echo "No system currently selected."
    fi
    echo
}

show_menu() {
    echo "========================================"
    echo "  Pipe Crawler System Setup"
    echo "========================================"
    echo
    show_current
    echo "Select your system:"
    echo "  1) Magnetic Phased Array Crawler"
    echo "  2) EdgeFlex"
    echo "  3) New Xpress Scan"
    echo
    echo "  q) Quit without changes"
    echo
}

get_web_interface() {
    local config_file="$CONFIG_DIR/$1.yaml"
    if [ -f "$config_file" ]; then
        grep "web_interface:" "$config_file" | sed 's/.*web_interface: *"//' | sed 's/".*//'
    fi
}

setup_system() {
    local system_id="$1"
    local system_name="${SYSTEM_NAMES[$system_id]}"

    echo
    echo "Setting up $system_name..."

    # Write active system
    echo "$system_id" > "$ACTIVE_SYSTEM_FILE"
    echo "- Written $system_id to $ACTIVE_SYSTEM_FILE"

    # Copy web interface (uses sudo if needed)
    local web_file=$(get_web_interface "$system_id")
    if [ -n "$web_file" ] && [ -f "$WEB_DIR/$web_file" ]; then
        if [ -w "$WEB_ROOT" ] || [ -w "$WEB_ROOT/index.html" ]; then
            cp "$WEB_DIR/$web_file" "$WEB_ROOT/index.html"
            echo "- Copied $web_file to $WEB_ROOT/index.html"
        else
            sudo cp "$WEB_DIR/$web_file" "$WEB_ROOT/index.html"
            echo "- Copied $web_file to $WEB_ROOT/index.html (sudo)"
        fi
    else
        echo "- Web interface file not found: $WEB_DIR/$web_file"
        echo "  (Web interfaces will be created separately)"
    fi

    echo
    echo "Restarting pipe_crawler service..."
    sudo systemctl restart pipe_crawler
    sleep 2
    if systemctl is-active --quiet pipe_crawler; then
        echo "Done. Service restarted successfully."
    else
        echo "Warning: Service may not have started correctly. Check with:"
        echo "  sudo systemctl status pipe_crawler"
    fi
    echo
}

# Main
show_menu

read -p "Enter choice [1-3, q]: " choice

case "$choice" in
    1|2|3)
        system_id="${SYSTEMS[$choice]}"
        setup_system "$system_id"
        ;;
    q|Q)
        echo "No changes made."
        exit 0
        ;;
    *)
        echo "Invalid choice: $choice"
        exit 1
        ;;
esac
