#!/usr/bin/env python3
"""
Crawler Configuration Portal

Always-running web server for crawler and hotspot configuration.
Accessible at http://[IP]:8080 - allows changing crawler identity,
hotspot settings, and WiFi client connections.
"""

import os
import re
import subprocess
import json
from flask import Flask, render_template_string, request, jsonify

app = Flask(__name__)

CONFIG_DIR = "/etc/xpresscan"
CONFIG_FILE = f"{CONFIG_DIR}/network.conf"

DEFAULT_SSID = "Crawler"
DEFAULT_PASSWORD = "Crawler1"

# Default crawler configurations
DEFAULT_CRAWLERS = {
    "magnetic": {
        "hostname": "magnetic",
        "ip": "10.42.0.11/24",
        "name": "Magnetic Pipe Crawler",
        "yaml": "magnetic_phased_array.yaml",
        "html": "magnetic_phased_array.html",
        "motor_driver": "roboclaw"
    },
    "xpressscan_singlewall": {
        "hostname": "xpressscan-sw",
        "ip": "10.42.0.15/24",
        "name": "Express Scan - Single Wall",
        "yaml": "xpressscan_singlewall.yaml",
        "html": "xpressscan_singlewall.html",
        "motor_driver": "roboclaw"
    },
    "xpressscan_doublewall": {
        "hostname": "xpressscan-dw",
        "ip": "10.42.0.16/24",
        "name": "Express Scan - Double Wall",
        "yaml": "xpressscan_doublewall.yaml",
        "html": "xpressscan_doublewall.html",
        "motor_driver": "roboclaw"
    },
    "edgeflex": {
        "hostname": "edgeflex",
        "ip": "10.42.0.13/24",
        "name": "EdgeFlex",
        "yaml": "edgeflex.yaml",
        "html": "edgeflex.html",
        "motor_driver": "clearlink"
    }
}


def get_network_info():
    """Get current network interface information."""
    info = {
        "wifi": {"ip": None, "ssid": None, "interface": None},
        "ethernet": {"ip": None, "network": None, "interface": None},
        "hostname": None
    }

    try:
        # Get hostname - fast, local operation
        result = subprocess.run(["hostname"], capture_output=True, text=True, timeout=2)
        info["hostname"] = result.stdout.strip()

        # Get interface info using ip command - fast
        result = subprocess.run(
            ["ip", "-j", "addr", "show"],
            capture_output=True, text=True, timeout=2
        )
        interfaces = json.loads(result.stdout) if result.stdout else []

        wifi_iface = None
        eth_iface = None

        for iface in interfaces:
            name = iface.get("ifname", "")
            addr_info = iface.get("addr_info", [])

            # Get IPv4 address
            ipv4 = None
            for addr in addr_info:
                if addr.get("family") == "inet":
                    ipv4 = addr.get("local")
                    break

            if not ipv4 or ipv4.startswith("127."):
                continue

            # WiFi interface (wlan*)
            if name.startswith("wlan"):
                info["wifi"]["ip"] = ipv4
                info["wifi"]["interface"] = name
                wifi_iface = name

            # Ethernet interface (eth* or enp*)
            elif name.startswith("eth") or name.startswith("enp"):
                info["ethernet"]["ip"] = ipv4
                info["ethernet"]["interface"] = name
                eth_iface = name

        # Get SSID using iwgetid (faster than nmcli for just SSID)
        if wifi_iface:
            try:
                result = subprocess.run(
                    ["iwgetid", "-r"],
                    capture_output=True, text=True, timeout=2
                )
                ssid = result.stdout.strip()
                if ssid:
                    info["wifi"]["ssid"] = ssid
            except Exception:
                pass

    except subprocess.TimeoutExpired:
        print("Timeout getting network info")
    except Exception as e:
        print(f"Error getting network info: {e}")

    return info


def scan_wifi_networks():
    """Scan for available WiFi networks."""
    networks = []
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE", "device", "wifi", "list", "--rescan", "yes"],
            capture_output=True, text=True, timeout=15
        )

        seen = set()
        for line in result.stdout.strip().split("\n"):
            if not line:
                continue
            parts = line.split(":")
            if len(parts) >= 3:
                ssid = parts[0].strip()
                if not ssid or ssid in seen:
                    continue
                seen.add(ssid)

                signal = parts[1].strip() if len(parts) > 1 else "0"
                security = parts[2].strip() if len(parts) > 2 else ""
                in_use = parts[3].strip() == "*" if len(parts) > 3 else False

                networks.append({
                    "ssid": ssid,
                    "signal": int(signal) if signal.isdigit() else 0,
                    "security": security,
                    "connected": in_use
                })

        # Sort by signal strength
        networks.sort(key=lambda x: x["signal"], reverse=True)
    except Exception as e:
        print(f"Error scanning WiFi: {e}")

    return networks


def connect_to_wifi(ssid, password=None):
    """Connect to a WiFi network.

    Creates a PERSISTENT connection using 'nmcli connection add' so credentials
    survive reboots (including gocryptfs unlock/reboot cycles).
    """
    try:
        # Check if connection already exists
        result = subprocess.run(
            ["nmcli", "connection", "show", ssid],
            capture_output=True, text=True
        )

        if result.returncode == 0:
            # Connection exists - update password if provided, then activate
            if password:
                subprocess.run(
                    ["nmcli", "connection", "modify", ssid,
                     "wifi-sec.key-mgmt", "wpa-psk",
                     "wifi-sec.psk", password],
                    capture_output=True, text=True
                )
            result = subprocess.run(
                ["nmcli", "connection", "up", ssid],
                capture_output=True, text=True, timeout=30
            )
        else:
            # Create new PERSISTENT connection (not volatile 'device wifi connect')
            # This stores credentials in /etc/NetworkManager/system-connections/
            # which gets converted to netplan files on Ubuntu
            if password:
                cmd = [
                    "nmcli", "connection", "add",
                    "type", "wifi",
                    "con-name", ssid,
                    "ifname", "wlan0",
                    "ssid", ssid,
                    "wifi-sec.key-mgmt", "wpa-psk",
                    "wifi-sec.psk", password,
                    "connection.autoconnect", "yes",
                    "connection.autoconnect-priority", "100"
                ]
            else:
                # Open network (no password)
                cmd = [
                    "nmcli", "connection", "add",
                    "type", "wifi",
                    "con-name", ssid,
                    "ifname", "wlan0",
                    "ssid", ssid,
                    "connection.autoconnect", "yes",
                    "connection.autoconnect-priority", "100"
                ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            if result.returncode == 0:
                # Now activate the connection
                result = subprocess.run(
                    ["nmcli", "connection", "up", ssid],
                    capture_output=True, text=True, timeout=30
                )

        if result.returncode == 0:
            return True, "Connected successfully"
        else:
            return False, result.stderr.strip() or "Connection failed"
    except subprocess.TimeoutExpired:
        return False, "Connection timed out"
    except Exception as e:
        return False, str(e)


def load_current_config():
    """Load current configuration from file."""
    config = {
        "crawler_id": None,
        "ssid": DEFAULT_SSID,
        "password": DEFAULT_PASSWORD,
        "crawlers": DEFAULT_CRAWLERS
    }

    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("CRAWLER_ID="):
                        config["crawler_id"] = line.split("=", 1)[1]
                    elif line.startswith("HOTSPOT_SSID="):
                        config["ssid"] = line.split("=", 1)[1]
                    elif line.startswith("HOTSPOT_PASSWORD="):
                        config["password"] = line.split("=", 1)[1]
        except Exception:
            pass

    return config


HOTSPOT_CONNECTION = "XPressCan-Hotspot"


def update_hotspot_connection(ssid, password):
    """Update the NetworkManager hotspot connection with new SSID/password."""
    try:
        # Check if hotspot connection exists
        result = subprocess.run(
            ["nmcli", "connection", "show", HOTSPOT_CONNECTION],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"Hotspot connection '{HOTSPOT_CONNECTION}' not found")
            return False

        # Update the connection
        result = subprocess.run(
            ["sudo", "nmcli", "connection", "modify", HOTSPOT_CONNECTION,
             "802-11-wireless.ssid", ssid,
             "wifi-sec.psk", password],
            capture_output=True, text=True, timeout=10
        )

        if result.returncode == 0:
            print(f"Updated hotspot connection: SSID={ssid}")
            return True
        else:
            print(f"Failed to update hotspot: {result.stderr}")
            return False
    except Exception as e:
        print(f"Error updating hotspot connection: {e}")
        return False


def save_config(crawler_id, ssid, password):
    """Save configuration to file and update NetworkManager hotspot."""
    try:
        os.makedirs(CONFIG_DIR, exist_ok=True)

        crawler = DEFAULT_CRAWLERS.get(crawler_id, {})
        hostname = crawler.get("hostname", "crawler")
        ip = crawler.get("ip", "10.42.0.11/24")
        yaml_file = crawler.get("yaml", "")
        html_file = crawler.get("html", "")
        motor_driver = crawler.get("motor_driver", "roboclaw")

        with open(CONFIG_FILE, "w") as f:
            f.write("# Crawler Configuration\n")
            f.write(f"CRAWLER_ID={crawler_id}\n")
            f.write(f"CRAWLER_NAME={crawler.get('name', crawler_id)}\n")
            f.write(f"HOSTNAME={hostname}\n")
            f.write(f"STATIC_IP={ip}\n")
            f.write(f"HOTSPOT_SSID={ssid}\n")
            f.write(f"HOTSPOT_PASSWORD={password}\n")
            f.write(f"CONFIG_YAML={yaml_file}\n")
            f.write(f"UI_HTML={html_file}\n")
            f.write(f"MOTOR_DRIVER={motor_driver}\n")

        # Also update the actual NetworkManager hotspot connection
        update_hotspot_connection(ssid, password)

        return True
    except Exception as e:
        print(f"Error saving config: {e}")
        return False


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Crawler Configuration</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #f5f5f7;
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            background: #ffffff;
            border-radius: 16px;
            padding: 30px;
            max-width: 500px;
            margin: 0 auto;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.08);
        }
        h1 {
            color: #1d1d1f;
            margin-bottom: 8px;
            font-size: 22px;
            text-align: center;
        }
        .subtitle {
            color: #86868b;
            margin-bottom: 25px;
            font-size: 13px;
            text-align: center;
        }
        .section {
            margin-bottom: 25px;
            padding-bottom: 20px;
            border-bottom: 1px solid #f0f0f0;
        }
        .section:last-child {
            border-bottom: none;
            margin-bottom: 0;
            padding-bottom: 0;
        }
        .section-title {
            font-size: 12px;
            font-weight: 600;
            color: #86868b;
            margin-bottom: 12px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .network-info {
            display: grid;
            gap: 8px;
        }
        .network-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 12px;
            background: #f5f5f7;
            border-radius: 8px;
            font-size: 13px;
        }
        .network-item .label {
            color: #86868b;
        }
        .network-item .value {
            color: #1d1d1f;
            font-weight: 500;
            font-family: monospace;
        }
        .network-item .value.none {
            color: #c0c0c0;
            font-style: italic;
        }
        .crawler-options {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }
        .crawler-option {
            padding: 12px 15px;
            border: 2px solid #e0e0e0;
            border-radius: 10px;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: space-between;
            transition: all 0.2s;
        }
        .crawler-option:hover {
            border-color: #5856d6;
        }
        .crawler-option.selected {
            border-color: #5856d6;
            background: #f0f0ff;
        }
        .crawler-option.current {
            border-color: #34c759;
            background: #e8f5e9;
        }
        .crawler-option input[type="radio"] {
            display: none;
        }
        .crawler-name {
            font-size: 14px;
            font-weight: 600;
            color: #1d1d1f;
        }
        .crawler-ip {
            font-size: 12px;
            color: #86868b;
            font-family: monospace;
        }
        .form-group {
            margin-bottom: 12px;
        }
        .form-label {
            display: block;
            font-size: 12px;
            color: #86868b;
            margin-bottom: 5px;
        }
        .form-input {
            width: 100%;
            padding: 10px 12px;
            font-size: 14px;
            border: 1px solid #e0e0e0;
            border-radius: 8px;
            background: #f5f5f7;
            color: #1d1d1f;
        }
        .form-input:focus {
            outline: none;
            border-color: #5856d6;
            box-shadow: 0 0 0 3px rgba(88, 86, 214, 0.15);
        }
        .form-hint {
            font-size: 10px;
            color: #86868b;
            margin-top: 3px;
        }
        .btn {
            padding: 12px 20px;
            font-size: 14px;
            font-weight: 600;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn-primary {
            width: 100%;
            color: #ffffff;
            background: #5856d6;
        }
        .btn-primary:hover {
            background: #4240a8;
        }
        .btn-secondary {
            color: #5856d6;
            background: #f0f0ff;
        }
        .btn-secondary:hover {
            background: #e0e0ff;
        }
        .btn-small {
            padding: 8px 12px;
            font-size: 12px;
        }
        .btn-danger {
            width: 100%;
            color: #ffffff;
            background: #ff3b30;
        }
        .btn-danger:hover {
            background: #d63028;
        }
        .reboot-notice {
            background: #fff3e0;
            color: #e65100;
            padding: 12px;
            border-radius: 8px;
            margin-bottom: 12px;
            font-size: 13px;
            line-height: 1.4;
        }
        .hotspot-info {
            background: #f5f5f7;
            padding: 12px;
            border-radius: 8px;
            font-size: 13px;
        }
        .hotspot-info .label {
            color: #86868b;
        }
        .hotspot-info .value {
            color: #1d1d1f;
            font-weight: 500;
            margin-left: 8px;
        }
        .access-denied {
            background: #ffebee;
            color: #c62828;
            padding: 10px;
            border-radius: 6px;
            margin-top: 10px;
            font-size: 13px;
            display: none;
        }
        .error {
            background: #ffebee;
            color: #c62828;
            padding: 10px 12px;
            border-radius: 8px;
            margin-bottom: 15px;
            font-size: 13px;
        }
        .success {
            background: #e8f5e9;
            color: #2e7d32;
            padding: 10px 12px;
            border-radius: 8px;
            margin-bottom: 15px;
            font-size: 13px;
        }
        .wifi-list {
            max-height: 250px;
            overflow-y: auto;
        }
        .wifi-network {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 12px;
            border-radius: 8px;
            margin-bottom: 6px;
            background: #f5f5f7;
            cursor: pointer;
            transition: background 0.2s;
        }
        .wifi-network:hover {
            background: #e8e8ed;
        }
        .wifi-network.connected {
            background: #e8f5e9;
        }
        .wifi-info {
            flex: 1;
        }
        .wifi-ssid {
            font-size: 14px;
            font-weight: 500;
            color: #1d1d1f;
        }
        .wifi-details {
            font-size: 11px;
            color: #86868b;
            margin-top: 2px;
        }
        .wifi-signal {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .signal-bars {
            display: flex;
            align-items: flex-end;
            gap: 2px;
            height: 16px;
        }
        .signal-bar {
            width: 4px;
            background: #c0c0c0;
            border-radius: 1px;
        }
        .signal-bar.active {
            background: #34c759;
        }
        .signal-bar:nth-child(1) { height: 4px; }
        .signal-bar:nth-child(2) { height: 8px; }
        .signal-bar:nth-child(3) { height: 12px; }
        .signal-bar:nth-child(4) { height: 16px; }
        .scan-btn {
            margin-bottom: 12px;
        }
        .modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0,0,0,0.5);
            justify-content: center;
            align-items: center;
            z-index: 100;
        }
        .modal.show {
            display: flex;
        }
        .modal-content {
            background: white;
            padding: 25px;
            border-radius: 16px;
            width: 90%;
            max-width: 350px;
        }
        .modal-title {
            font-size: 18px;
            font-weight: 600;
            margin-bottom: 15px;
        }
        .modal-buttons {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }
        .modal-buttons .btn {
            flex: 1;
        }
        .loading {
            text-align: center;
            padding: 20px;
            color: #86868b;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Crawler Configuration</h1>

        {% if error %}
        <div class="error">{{ error }}</div>
        {% endif %}

        {% if success %}
        <div class="success">{{ success }}</div>
        {% endif %}

        <!-- Network Status -->
        <div class="section">
            <div class="section-title">Network Status</div>
            <div class="network-info">
                <div class="network-item">
                    <span class="label">WiFi</span>
                    <span class="value {% if not network_info.wifi.ip %}none{% endif %}">
                        {% if network_info.wifi.ip %}
                            {{ network_info.wifi.ip }} ({{ network_info.wifi.ssid or 'Unknown' }})
                        {% else %}
                            Not connected
                        {% endif %}
                    </span>
                </div>
                <div class="network-item">
                    <span class="label">Ethernet</span>
                    <span class="value {% if not network_info.ethernet.ip %}none{% endif %}">
                        {% if network_info.ethernet.ip %}
                            {{ network_info.ethernet.ip }}{% if network_info.ethernet.network %} ({{ network_info.ethernet.network }}){% endif %}
                        {% else %}
                            Not connected
                        {% endif %}
                    </span>
                </div>
            </div>
        </div>

        <form action="/configure" method="POST" id="configForm">
            <!-- Crawler Selection -->
            <div class="section">
                <div class="section-title">Crawler Type</div>
                <div class="crawler-options">
                    {% for id, crawler in crawlers.items() %}
                    <label class="crawler-option {% if id == current_crawler %}current selected{% endif %}" onclick="selectCrawler('{{ id }}', this)">
                        <input type="radio" name="crawler_id" value="{{ id }}" {% if id == current_crawler %}checked{% endif %}>
                        <span class="crawler-name">{{ crawler.name }}</span>
                        <span class="crawler-ip">{{ crawler.hostname }}</span>
                    </label>
                    {% endfor %}
                </div>
            </div>

            <!-- Hidden hotspot fields (populated by modal) -->
            <input type="hidden" name="ssid" id="hiddenSsid" value="{{ current_ssid }}">
            <input type="hidden" name="password" id="hiddenPassword" value="{{ current_password }}">

            <button type="submit" class="btn btn-primary">Apply Changes</button>
        </form>

        <!-- Hotspot Settings Section -->
        <div class="section" style="margin-top: 20px; text-align: center;">
            <div class="section-title">WiFi Hotspot</div>
            <div class="hotspot-info" style="display: inline-block;">
                <span class="label">Current SSID:</span>
                <span class="value" id="currentSsidDisplay">{{ current_ssid }}</span>
            </div>
            <div>
                <button type="button" class="btn btn-secondary" onclick="showHotspotAuth()" style="margin-top: 12px;">
                    Change Hotspot Settings
                </button>
            </div>
        </div>

        <!-- Reboot Section -->
        <div class="section" style="margin-top: 20px; text-align: center;">
            <div class="section-title">System Control</div>
            <div class="reboot-notice" style="text-align: left;">
                Reboot required after changing crawler type or WiFi settings for changes to take effect.
            </div>
            <button type="button" class="btn btn-danger" onclick="confirmReboot()">Reboot Crawler</button>
        </div>

        <!-- WiFi Scanner -->
        <div class="section" style="margin-top: 25px;">
            <div class="section-title">Available WiFi Networks</div>
            <button type="button" class="btn btn-secondary btn-small scan-btn" onclick="scanWifi()">
                Scan Networks
            </button>
            <div id="wifiList" class="wifi-list">
                <div class="loading">Click "Scan Networks" to find available networks</div>
            </div>
        </div>
    </div>

    <!-- WiFi Password Modal -->
    <div id="wifiModal" class="modal">
        <div class="modal-content">
            <div class="modal-title">Connect to <span id="modalSsid"></span></div>
            <div class="form-group">
                <label class="form-label">Password</label>
                <input type="password" id="wifiPassword" class="form-input" placeholder="Enter WiFi password">
            </div>
            <div class="modal-buttons">
                <button type="button" class="btn btn-secondary" onclick="closeModal()">Cancel</button>
                <button type="button" class="btn btn-primary" onclick="connectWifi()">Connect</button>
            </div>
        </div>
    </div>

    <!-- Hotspot Auth Modal -->
    <div id="hotspotAuthModal" class="modal">
        <div class="modal-content">
            <div class="modal-title">Authentication Required</div>
            <div class="form-group">
                <label class="form-label">Enter admin password to change hotspot settings</label>
                <input type="password" id="hotspotAuthPassword" class="form-input" placeholder="Password">
            </div>
            <div id="authDenied" class="access-denied">Access denied. Incorrect password.</div>
            <div class="modal-buttons">
                <button type="button" class="btn btn-secondary" onclick="closeHotspotAuth()">Cancel</button>
                <button type="button" class="btn btn-primary" onclick="checkHotspotAuth()">Continue</button>
            </div>
        </div>
    </div>

    <!-- Hotspot Settings Modal -->
    <div id="hotspotSettingsModal" class="modal">
        <div class="modal-content">
            <div class="modal-title">Hotspot Settings</div>
            <div class="form-group">
                <label class="form-label">Network Name (SSID)</label>
                <input type="text" id="hotspotSsid" class="form-input" value="{{ current_ssid }}" maxlength="32">
            </div>
            <div class="form-group">
                <label class="form-label">Password</label>
                <input type="text" id="hotspotPassword" class="form-input" value="{{ current_password }}" maxlength="63">
                <div class="form-hint">Minimum 8 characters</div>
            </div>
            <div class="modal-buttons">
                <button type="button" class="btn btn-secondary" onclick="closeHotspotSettings()">Cancel</button>
                <button type="button" class="btn btn-primary" onclick="saveHotspotSettings()">Save</button>
            </div>
        </div>
    </div>

    <script>
        let selectedSsid = '';

        function selectCrawler(id, element) {
            document.querySelectorAll('.crawler-option').forEach(el => {
                el.classList.remove('selected');
            });
            element.classList.add('selected');
            element.querySelector('input[type="radio"]').checked = true;
        }

        function scanWifi() {
            const list = document.getElementById('wifiList');
            list.innerHTML = '<div class="loading">Scanning...</div>';

            fetch('/api/wifi/scan')
                .then(r => r.json())
                .then(data => {
                    if (data.networks.length === 0) {
                        list.innerHTML = '<div class="loading">No networks found</div>';
                        return;
                    }

                    list.innerHTML = data.networks.map(net => `
                        <div class="wifi-network ${net.connected ? 'connected' : ''}"
                             onclick="showConnectModal('${net.ssid.replace(/'/g, "\\'")}', ${net.connected}, '${net.security}')">
                            <div class="wifi-info">
                                <div class="wifi-ssid">${net.ssid}${net.connected ? ' (Connected)' : ''}</div>
                                <div class="wifi-details">${net.security || 'Open'}</div>
                            </div>
                            <div class="wifi-signal">
                                <div class="signal-bars">
                                    <div class="signal-bar ${net.signal > 0 ? 'active' : ''}"></div>
                                    <div class="signal-bar ${net.signal > 25 ? 'active' : ''}"></div>
                                    <div class="signal-bar ${net.signal > 50 ? 'active' : ''}"></div>
                                    <div class="signal-bar ${net.signal > 75 ? 'active' : ''}"></div>
                                </div>
                            </div>
                        </div>
                    `).join('');
                })
                .catch(err => {
                    list.innerHTML = '<div class="loading">Error scanning networks</div>';
                });
        }

        function showConnectModal(ssid, connected, security) {
            if (connected) return;
            selectedSsid = ssid;
            document.getElementById('modalSsid').textContent = ssid;
            document.getElementById('wifiPassword').value = '';

            // Show/hide password field based on security
            const pwField = document.getElementById('wifiPassword').parentElement;
            pwField.style.display = security ? 'block' : 'none';

            document.getElementById('wifiModal').classList.add('show');
        }

        function closeModal() {
            document.getElementById('wifiModal').classList.remove('show');
        }

        function connectWifi() {
            const password = document.getElementById('wifiPassword').value;
            closeModal();

            const list = document.getElementById('wifiList');
            list.innerHTML = '<div class="loading">Connecting to ' + selectedSsid + '...</div>';

            fetch('/api/wifi/connect', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ssid: selectedSsid, password: password})
            })
            .then(r => r.json())
            .then(data => {
                if (data.success) {
                    location.reload();
                } else {
                    list.innerHTML = '<div class="error">' + data.message + '</div>';
                    setTimeout(scanWifi, 2000);
                }
            })
            .catch(err => {
                list.innerHTML = '<div class="error">Connection failed</div>';
                setTimeout(scanWifi, 2000);
            });
        }

        // Initialize
        document.addEventListener('DOMContentLoaded', function() {
            const current = document.querySelector('.crawler-option.current');
            if (current) current.classList.add('selected');
        });

        function confirmReboot() {
            if (confirm('Reboot the crawler now?\\n\\nMake sure you have saved your changes first.')) {
                fetch('/api/reboot', {method: 'POST'})
                    .then(r => r.json())
                    .then(data => {
                        if (data.success) {
                            document.body.innerHTML = '<div style="display:flex;justify-content:center;align-items:center;height:100vh;font-family:sans-serif;"><div style="text-align:center;"><h2>Rebooting...</h2><p>The crawler is restarting. Please wait and reconnect.</p></div></div>';
                        } else {
                            alert('Reboot failed: ' + data.error);
                        }
                    })
                    .catch(err => alert('Reboot failed'));
            }
        }

        // Hotspot settings functions
        function showHotspotAuth() {
            document.getElementById('hotspotAuthPassword').value = '';
            document.getElementById('authDenied').style.display = 'none';
            document.getElementById('hotspotAuthModal').classList.add('show');
            document.getElementById('hotspotAuthPassword').focus();
        }

        function closeHotspotAuth() {
            document.getElementById('hotspotAuthModal').classList.remove('show');
        }

        function checkHotspotAuth() {
            const password = document.getElementById('hotspotAuthPassword').value;
            if (password === 'Prime') {
                closeHotspotAuth();
                showHotspotSettings();
            } else {
                document.getElementById('authDenied').style.display = 'block';
                document.getElementById('hotspotAuthPassword').value = '';
                document.getElementById('hotspotAuthPassword').focus();
            }
        }

        function showHotspotSettings() {
            document.getElementById('hotspotSettingsModal').classList.add('show');
            document.getElementById('hotspotSsid').focus();
        }

        function closeHotspotSettings() {
            document.getElementById('hotspotSettingsModal').classList.remove('show');
        }

        function saveHotspotSettings() {
            const ssid = document.getElementById('hotspotSsid').value.trim();
            const password = document.getElementById('hotspotPassword').value.trim();

            if (ssid.length < 1 || ssid.length > 32) {
                alert('SSID must be 1-32 characters');
                return;
            }
            if (password.length < 8) {
                alert('Password must be at least 8 characters');
                return;
            }

            // Update hidden form fields
            document.getElementById('hiddenSsid').value = ssid;
            document.getElementById('hiddenPassword').value = password;

            // Update display
            document.getElementById('currentSsidDisplay').textContent = ssid;

            closeHotspotSettings();
            alert('Hotspot settings updated. Click "Apply Changes" to save.');
        }

        // Allow Enter key in auth modal
        document.getElementById('hotspotAuthPassword').addEventListener('keypress', function(e) {
            if (e.key === 'Enter') checkHotspotAuth();
        });
    </script>
</body>
</html>
"""


SUCCESS_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="refresh" content="3;url=/">
    <title>Configuration Saved</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #f5f5f7;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }
        .container {
            background: #ffffff;
            border-radius: 16px;
            padding: 40px;
            max-width: 400px;
            width: 100%;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.08);
            text-align: center;
        }
        .checkmark {
            width: 60px;
            height: 60px;
            background: #34c759;
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
            margin: 0 auto 20px;
        }
        .checkmark svg {
            width: 30px;
            height: 30px;
            stroke: white;
            stroke-width: 3;
            fill: none;
        }
        h1 { color: #1d1d1f; margin-bottom: 15px; font-size: 20px; }
        .info { color: #86868b; font-size: 14px; }
        .config-summary {
            background: #f5f5f7;
            border-radius: 10px;
            padding: 15px;
            margin: 20px 0;
            text-align: left;
        }
        .config-item {
            display: flex;
            justify-content: space-between;
            padding: 6px 0;
            font-size: 13px;
        }
        .config-label { color: #86868b; }
        .config-value { color: #1d1d1f; font-weight: 500; }
        .warning {
            background: #fff3e0;
            color: #e65100;
            padding: 12px;
            border-radius: 8px;
            margin-top: 15px;
            font-size: 13px;
            line-height: 1.4;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="checkmark">
            <svg viewBox="0 0 24 24"><polyline points="20 6 9 17 4 12"/></svg>
        </div>
        <h1>Configuration Saved</h1>

        <div class="config-summary">
            <div class="config-item">
                <span class="config-label">Crawler</span>
                <span class="config-value">{{ crawler_name }}</span>
            </div>
            <div class="config-item">
                <span class="config-label">Hostname</span>
                <span class="config-value">{{ hostname }}.local</span>
            </div>
            <div class="config-item">
                <span class="config-label">Hotspot</span>
                <span class="config-value">{{ ssid }}</span>
            </div>
        </div>

        <div class="warning">
            Reboot required for changes to take effect. Use the Reboot button on the main page.
        </div>

        <p class="info">Redirecting back...</p>
    </div>
</body>
</html>
"""


def configure_system(crawler_id, ssid, password):
    """Configure the system with the selected settings (does NOT reboot)."""
    if crawler_id not in DEFAULT_CRAWLERS:
        return False, "Invalid crawler ID"

    if len(password) < 8:
        return False, "Password must be at least 8 characters"

    if len(ssid) < 1 or len(ssid) > 32:
        return False, "SSID must be 1-32 characters"

    try:
        # Save config file
        save_config(crawler_id, ssid, password)

        # Note: Hostname stays as "autobug" for all crawler types
        # NO automatic reboot - user must click Reboot button
        return True, None
    except Exception as e:
        return False, str(e)


@app.route("/")
def index():
    """Serve the configuration page."""
    config = load_current_config()
    network_info = get_network_info()

    return render_template_string(
        HTML_TEMPLATE,
        crawlers=config["crawlers"],
        current_crawler=config["crawler_id"],
        current_ssid=config["ssid"],
        current_password=config["password"],
        network_info=network_info,
        error=request.args.get("error"),
        success=request.args.get("success")
    )


@app.route("/configure", methods=["POST"])
def configure():
    """Handle configuration submission."""
    crawler_id = request.form.get("crawler_id")
    ssid = request.form.get("ssid", DEFAULT_SSID).strip()
    password = request.form.get("password", DEFAULT_PASSWORD).strip()

    if not crawler_id:
        config = load_current_config()
        return render_template_string(
            HTML_TEMPLATE,
            crawlers=config["crawlers"],
            current_crawler=config["crawler_id"],
            current_ssid=ssid,
            current_password=password,
            network_info=get_network_info(),
            error="Please select a crawler type",
            success=None
        )

    success, error = configure_system(crawler_id, ssid, password)

    if not success:
        config = load_current_config()
        return render_template_string(
            HTML_TEMPLATE,
            crawlers=config["crawlers"],
            current_crawler=config["crawler_id"],
            current_ssid=ssid,
            current_password=password,
            network_info=get_network_info(),
            error=error,
            success=None
        )

    crawler = DEFAULT_CRAWLERS[crawler_id]
    return render_template_string(
        SUCCESS_TEMPLATE,
        crawler_name=crawler["name"],
        hostname=crawler["hostname"],
        ssid=ssid
    )


@app.route("/api/wifi/scan")
def api_wifi_scan():
    """API endpoint to scan for WiFi networks."""
    networks = scan_wifi_networks()
    return jsonify({"networks": networks})


@app.route("/api/wifi/connect", methods=["POST"])
def api_wifi_connect():
    """API endpoint to connect to a WiFi network."""
    data = request.get_json()
    ssid = data.get("ssid", "")
    password = data.get("password", "")

    if not ssid:
        return jsonify({"success": False, "message": "SSID required"})

    success, message = connect_to_wifi(ssid, password if password else None)
    return jsonify({"success": success, "message": message})


@app.route("/api/reboot", methods=["POST"])
def api_reboot():
    """API endpoint to reboot the system."""
    try:
        subprocess.Popen(["sudo", "reboot"], start_new_session=True)
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/status")
def status():
    """Check configuration status (JSON API)."""
    config = load_current_config()
    network_info = get_network_info()

    return jsonify({
        "configured": config["crawler_id"] is not None,
        "crawler_id": config["crawler_id"],
        "ssid": config["ssid"],
        "network": network_info
    })


def main():
    """Run the configuration portal server."""
    print("Starting Crawler Configuration Portal on port 8080")
    print("Access at http://10.42.0.1:8080 or http://autobug.local:8080")
    app.run(host="0.0.0.0", port=8080, debug=False, threaded=True)


if __name__ == "__main__":
    main()
