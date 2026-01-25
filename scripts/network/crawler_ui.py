#!/usr/bin/env python3
"""
Crawler UI

Main operational status and control interface.
Accessible at http://[hostname].local (port 80)

Serves different HTML interfaces based on crawler type:
- Magnetic Pipe Crawler: magnetic_phased_array.html
- XPressCan: new_xpress_scan.html
- EdgeFlex: edgeflex.html
"""

import os
import subprocess
import json
from flask import Flask, render_template_string, jsonify, send_from_directory

app = Flask(__name__)

CONFIG_FILE = "/etc/xpresscan/network.conf"
HTML_DIR = "/opt/xpresscan/html"

# Crawler type to HTML file mapping
CRAWLER_HTML = {
    "magnetic": "magnetic_phased_array.html",
    "xpresscan": "new_xpress_scan.html",
    "edgeflex": "edgeflex.html"
}


def load_crawler_config():
    """Load crawler configuration from file."""
    config = {
        "crawler_id": None,
        "crawler_name": None,
        "hostname": None,
        "ui_html": None,
        "motor_driver": "roboclaw"
    }

    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("CRAWLER_ID="):
                        config["crawler_id"] = line.split("=", 1)[1]
                    elif line.startswith("CRAWLER_NAME="):
                        config["crawler_name"] = line.split("=", 1)[1]
                    elif line.startswith("HOSTNAME="):
                        config["hostname"] = line.split("=", 1)[1]
                    elif line.startswith("UI_HTML="):
                        config["ui_html"] = line.split("=", 1)[1]
                    elif line.startswith("MOTOR_DRIVER="):
                        config["motor_driver"] = line.split("=", 1)[1]
        except Exception:
            pass

    return config


def get_system_status():
    """Get current system status."""
    status = {
        "hostname": None,
        "crawler_name": None,
        "uptime": None,
        "cpu_temp": None,
        "memory": {"used": 0, "total": 0, "percent": 0},
        "disk": {"used": 0, "total": 0, "percent": 0},
        "ros2": {"running": False, "nodes": []},
        "network": {
            "wifi_ip": None,
            "wifi_ssid": None,
            "eth_ip": None
        }
    }

    try:
        # Hostname
        result = subprocess.run(["hostname"], capture_output=True, text=True)
        status["hostname"] = result.stdout.strip()

        # Crawler name from config
        crawler_config = load_crawler_config()
        if crawler_config["crawler_name"]:
            status["crawler_name"] = crawler_config["crawler_name"]
        else:
            # Fallback to defaults based on hostname
            default_names = {
                "magnetic": "Magnetic Pipe Crawler",
                "xpresscan": "XPressCan",
                "edgeflex": "EdgeFlex"
            }
            status["crawler_name"] = default_names.get(status["hostname"], status["hostname"])

        status["ui_html"] = crawler_config.get("ui_html")
        status["motor_driver"] = crawler_config.get("motor_driver", "roboclaw")

        # Uptime
        result = subprocess.run(["uptime", "-p"], capture_output=True, text=True)
        status["uptime"] = result.stdout.strip().replace("up ", "")

        # CPU Temperature
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = int(f.read().strip()) / 1000
                status["cpu_temp"] = round(temp, 1)
        except:
            pass

        # Memory
        result = subprocess.run(["free", "-b"], capture_output=True, text=True)
        for line in result.stdout.split("\n"):
            if line.startswith("Mem:"):
                parts = line.split()
                total = int(parts[1])
                used = int(parts[2])
                status["memory"] = {
                    "used": round(used / (1024**3), 1),
                    "total": round(total / (1024**3), 1),
                    "percent": round(used / total * 100, 1)
                }
                break

        # Disk
        result = subprocess.run(["df", "-B1", "/"], capture_output=True, text=True)
        lines = result.stdout.strip().split("\n")
        if len(lines) > 1:
            parts = lines[1].split()
            total = int(parts[1])
            used = int(parts[2])
            status["disk"] = {
                "used": round(used / (1024**3), 1),
                "total": round(total / (1024**3), 1),
                "percent": round(used / total * 100, 1)
            }

        # Network
        result = subprocess.run(["ip", "-j", "addr", "show"], capture_output=True, text=True)
        interfaces = json.loads(result.stdout) if result.stdout else []
        for iface in interfaces:
            name = iface.get("ifname", "")
            for addr in iface.get("addr_info", []):
                if addr.get("family") == "inet":
                    ip = addr.get("local")
                    if ip and not ip.startswith("127."):
                        if name.startswith("wlan"):
                            status["network"]["wifi_ip"] = ip
                            # Get SSID
                            try:
                                r = subprocess.run(
                                    ["nmcli", "-t", "-f", "GENERAL.CONNECTION", "device", "show", name],
                                    capture_output=True, text=True
                                )
                                for l in r.stdout.split("\n"):
                                    if "GENERAL.CONNECTION:" in l:
                                        ssid = l.split(":", 1)[1].strip()
                                        if ssid and ssid != "--":
                                            status["network"]["wifi_ssid"] = ssid
                            except:
                                pass
                        elif name.startswith("eth") or name.startswith("enp"):
                            status["network"]["eth_ip"] = ip

        # ROS2 status
        try:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True, text=True, timeout=5,
                env={**os.environ, "ROS_DOMAIN_ID": "0"}
            )
            if result.returncode == 0:
                nodes = [n.strip() for n in result.stdout.strip().split("\n") if n.strip()]
                status["ros2"]["running"] = len(nodes) > 0
                status["ros2"]["nodes"] = nodes
        except:
            pass

    except Exception as e:
        print(f"Error getting status: {e}")

    return status


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta http-equiv="refresh" content="10">
    <title>{{ status.crawler_name or status.hostname }} - Crawler</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #1a1a2e;
            color: #eee;
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
        }
        header {
            text-align: center;
            margin-bottom: 30px;
        }
        h1 {
            font-size: 28px;
            font-weight: 600;
            margin-bottom: 5px;
        }
        .subtitle {
            color: #888;
            font-size: 14px;
        }
        .status-indicator {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 8px;
            animation: pulse 2s infinite;
        }
        .status-indicator.online { background: #34c759; }
        .status-indicator.offline { background: #ff3b30; }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        .card {
            background: #16213e;
            border-radius: 12px;
            padding: 20px;
        }
        .card-title {
            font-size: 12px;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: #888;
            margin-bottom: 15px;
        }
        .stat-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 8px 0;
            border-bottom: 1px solid #1a1a2e;
        }
        .stat-row:last-child { border-bottom: none; }
        .stat-label { color: #aaa; font-size: 13px; }
        .stat-value { font-weight: 500; font-family: monospace; }
        .stat-value.good { color: #34c759; }
        .stat-value.warn { color: #ff9500; }
        .stat-value.bad { color: #ff3b30; }
        .progress-bar {
            width: 100%;
            height: 6px;
            background: #1a1a2e;
            border-radius: 3px;
            overflow: hidden;
            margin-top: 8px;
        }
        .progress-fill {
            height: 100%;
            border-radius: 3px;
            transition: width 0.3s;
        }
        .progress-fill.low { background: #34c759; }
        .progress-fill.medium { background: #ff9500; }
        .progress-fill.high { background: #ff3b30; }
        .nodes-list {
            max-height: 200px;
            overflow-y: auto;
        }
        .node-item {
            padding: 6px 10px;
            background: #1a1a2e;
            border-radius: 6px;
            margin-bottom: 6px;
            font-size: 12px;
            font-family: monospace;
            color: #34c759;
        }
        .node-item:last-child { margin-bottom: 0; }
        .no-nodes {
            color: #666;
            font-style: italic;
            text-align: center;
            padding: 20px;
        }
        .actions {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        .btn {
            padding: 12px 24px;
            font-size: 14px;
            font-weight: 500;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.2s;
            text-decoration: none;
            display: inline-block;
        }
        .btn-primary {
            background: #5856d6;
            color: white;
        }
        .btn-primary:hover { background: #4240a8; }
        .btn-secondary {
            background: #2a2a4a;
            color: #aaa;
        }
        .btn-secondary:hover { background: #3a3a5a; }
        .btn-danger {
            background: #ff3b30;
            color: white;
        }
        .btn-danger:hover { background: #d63028; }
        footer {
            text-align: center;
            margin-top: 30px;
            color: #666;
            font-size: 12px;
        }
        footer a { color: #5856d6; text-decoration: none; }
        footer a:hover { text-decoration: underline; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>
                <span class="status-indicator {{ 'online' if status.ros2.running else 'offline' }}"></span>
                {{ status.crawler_name or status.hostname }}
            </h1>
            <p class="subtitle">{{ status.hostname }}.local | Uptime: {{ status.uptime or 'Unknown' }}</p>
        </header>

        <div class="grid">
            <!-- System Stats -->
            <div class="card">
                <div class="card-title">System</div>
                <div class="stat-row">
                    <span class="stat-label">CPU Temperature</span>
                    <span class="stat-value {{ 'good' if status.cpu_temp and status.cpu_temp < 60 else 'warn' if status.cpu_temp and status.cpu_temp < 75 else 'bad' }}">
                        {{ status.cpu_temp }}°C if status.cpu_temp else 'N/A'
                    </span>
                </div>
                <div class="stat-row">
                    <span class="stat-label">Memory</span>
                    <span class="stat-value">{{ status.memory.used }}GB / {{ status.memory.total }}GB</span>
                </div>
                <div class="progress-bar">
                    <div class="progress-fill {{ 'low' if status.memory.percent < 60 else 'medium' if status.memory.percent < 85 else 'high' }}"
                         style="width: {{ status.memory.percent }}%"></div>
                </div>
                <div class="stat-row" style="margin-top: 10px;">
                    <span class="stat-label">Disk</span>
                    <span class="stat-value">{{ status.disk.used }}GB / {{ status.disk.total }}GB</span>
                </div>
                <div class="progress-bar">
                    <div class="progress-fill {{ 'low' if status.disk.percent < 60 else 'medium' if status.disk.percent < 85 else 'high' }}"
                         style="width: {{ status.disk.percent }}%"></div>
                </div>
            </div>

            <!-- Network -->
            <div class="card">
                <div class="card-title">Network</div>
                <div class="stat-row">
                    <span class="stat-label">WiFi</span>
                    <span class="stat-value">{{ status.network.wifi_ip or 'Not connected' }}</span>
                </div>
                {% if status.network.wifi_ssid %}
                <div class="stat-row">
                    <span class="stat-label">SSID</span>
                    <span class="stat-value">{{ status.network.wifi_ssid }}</span>
                </div>
                {% endif %}
                <div class="stat-row">
                    <span class="stat-label">Ethernet</span>
                    <span class="stat-value">{{ status.network.eth_ip or 'Not connected' }}</span>
                </div>
            </div>
        </div>

        <!-- ROS2 Nodes -->
        <div class="card">
            <div class="card-title">ROS2 Nodes</div>
            {% if status.ros2.nodes %}
            <div class="nodes-list">
                {% for node in status.ros2.nodes %}
                <div class="node-item">{{ node }}</div>
                {% endfor %}
            </div>
            {% else %}
            <div class="no-nodes">No ROS2 nodes running</div>
            {% endif %}
        </div>

        <!-- Actions -->
        <div class="card" style="margin-top: 20px;">
            <div class="card-title">Actions</div>
            <div class="actions">
                <a href="http://{{ status.network.wifi_ip }}:8080" class="btn btn-primary">Network Config</a>
                <button class="btn btn-secondary" onclick="location.reload()">Refresh</button>
                <button class="btn btn-danger" onclick="if(confirm('Reboot crawler?')) fetch('/api/reboot', {method:'POST'})">Reboot</button>
            </div>
        </div>

        <footer>
            <p>Crawler | <a href="http://{{ status.network.wifi_ip }}:8080">Configuration Portal</a></p>
        </footer>
    </div>
</body>
</html>
"""


@app.route("/")
def index():
    """Serve the main status page or custom crawler UI."""
    status = get_system_status()

    # Check if a custom HTML file is configured and exists
    if status.get("ui_html"):
        html_path = os.path.join(HTML_DIR, status["ui_html"])
        if os.path.exists(html_path):
            try:
                with open(html_path, "r") as f:
                    return f.read()
            except Exception:
                pass

    # Fall back to default status template
    return render_template_string(HTML_TEMPLATE, status=status)


@app.route("/api/status")
def api_status():
    """API endpoint for status."""
    return jsonify(get_system_status())


@app.route("/api/reboot", methods=["POST"])
def api_reboot():
    """API endpoint to reboot the system."""
    try:
        subprocess.Popen(["sudo", "reboot"], start_new_session=True)
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# Captive portal detection routes - redirect to config portal
@app.route("/generate_204")
@app.route("/gen_204")
@app.route("/hotspot-detect.html")
@app.route("/library/test/success.html")
@app.route("/connecttest.txt")
@app.route("/ncsi.txt")
@app.route("/success.txt")
@app.route("/canonical.html")
def captive_portal_redirect():
    """Redirect captive portal detection to config page."""
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <meta http-equiv="refresh" content="0;url=http://10.42.0.1:8080/">
        <title>Redirecting...</title>
    </head>
    <body>
        <p>Redirecting to <a href="http://10.42.0.1:8080/">configuration portal</a>...</p>
    </body>
    </html>
    """, 302, {"Location": "http://10.42.0.1:8080/"}


def main():
    """Run the crawler UI server."""
    print("Starting Crawler UI on port 80")
    print("Access at http://autobug.local")
    app.run(host="0.0.0.0", port=80, debug=False)


if __name__ == "__main__":
    main()
