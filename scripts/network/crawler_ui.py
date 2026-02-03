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
import time
import logging
import threading
import secrets
import io
import cv2
import numpy as np
from datetime import datetime
from flask import Flask, render_template_string, jsonify, send_from_directory, request, make_response, redirect, Response

app = Flask(__name__)

# --- Camera streaming via rpicam-vid (Arducam 64MP CSI camera) ---
_camera_frame = None
_camera_frame_lock = threading.Lock()

def _camera_capture_thread():
    global _camera_frame
    cmd = [
        "rpicam-vid", "-t", "0",
        "--width", "1920", "--height", "1080",
        "--framerate", "30",
        "--codec", "mjpeg",
        "--inline",
        "-o", "-",
    ]
    while True:
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            buf = b""
            while True:
                chunk = proc.stdout.read(4096)
                if not chunk:
                    break
                buf += chunk
                while True:
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                    if start == -1 or end == -1:
                        break
                    frame = buf[start:end + 2]
                    buf = buf[end + 2:]
                    with _camera_frame_lock:
                        _camera_frame = frame
            proc.terminate()
        except Exception:
            pass
        time.sleep(1)

threading.Thread(target=_camera_capture_thread, daemon=True).start()

@app.route('/api/camera/snapshot')
def camera_snapshot():
    with _camera_frame_lock:
        frame = _camera_frame
    if frame is None:
        return Response(status=503)
    return Response(frame, mimetype='image/jpeg',
                    headers={'Cache-Control': 'no-store'})

# Configure logging for security events
logging.basicConfig(level=logging.INFO)
security_logger = logging.getLogger("security")

# Bypass session settings
BYPASS_COOKIE_NAME = "mismatch_bypass"
BYPASS_COOKIE_MAX_AGE = 30 * 60  # 30 minutes in seconds

# Boot state tracking
boot_state = {
    "in_progress": True,
    "current_check": "Initializing",
    "checks_complete": 0,
    "checks_total": 3,
    "start_time": time.time(),
    "result": None,  # "ok", "mismatch", "error"
    "redirect": None
}
boot_state_lock = threading.Lock()

# Operator lock — only one browser client can control motors at a time
_operator_lock = {
    "session_token": None,
    "client_ip": None,
    "acquired_at": None,
    "last_heartbeat": None,
}
_operator_lock_mutex = threading.Lock()
OPERATOR_LOCK_TIMEOUT = 10  # seconds without heartbeat before auto-release


def _is_operator_lock_expired():
    if _operator_lock["session_token"] is None:
        return True
    if _operator_lock["last_heartbeat"] is None:
        return True
    return (time.time() - _operator_lock["last_heartbeat"]) > OPERATOR_LOCK_TIMEOUT

CONFIG_FILE = "/etc/xpresscan/network.conf"
HTML_DIR = "/opt/xpresscan/html"
UI_SETTINGS_DIR = "/etc/xpresscan"


def get_settings_file():
    """Get the settings file path for the current crawler config."""
    config = load_crawler_config()
    crawler_id = config.get("crawler_id", "default")
    return os.path.join(UI_SETTINGS_DIR, f"ui_settings_{crawler_id}.json")

# Default UI settings
# IMPORTANT: Update this dict when adding new settings to the HTML pages!
# These defaults are used if the JSON config file is missing or corrupted.
DEFAULT_UI_SETTINGS = {
    "motorParams": {
        "encoderRes": 1425.1,
        "wheelDia": 1.77,
        "gearboxRatio": 1,
        "gearRatio": 2,
        "maxRpm": 179,
        "numMotors": 1,
        "motorTorque": 0.5
    },
    "pipeDiameter": 12,
    "speed": 4.0,
    "ramp": 50,
    "motorDriver": "clearlink"  # Default for EdgeFlex; magnetic/xpresscan use "roboclaw"
}

# Crawler type to HTML file mapping
CRAWLER_HTML = {
    "magnetic": "magnetic_phased_array.html",
    "xpresscan": "new_xpress_scan.html",
    "edgeflex": "edgeflex.html",
    "edgeflex_clearlink": "edgeflex_clearlink.html"
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
                    response = make_response(f.read())
                    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
                    return response
            except Exception:
                pass

    # Fall back to default status template
    return render_template_string(HTML_TEMPLATE, status=status)


@app.route("/api/status")
def api_status():
    """API endpoint for status."""
    return jsonify(get_system_status())


@app.route("/api/last_updated")
def api_last_updated():
    """Return the date of the last git commit (i.e. last system update)."""
    repo = "/home/craig/ros2_ws/src/pipe_crawler_control"
    try:
        env = os.environ.copy()
        env["GIT_DIR"] = os.path.join(repo, ".git")
        result = subprocess.run(
            ["git", "log", "-1", "--format=%ci"],
            capture_output=True, text=True,
            env=env
        )
        if result.returncode == 0 and result.stdout.strip():
            return jsonify({"last_updated": result.stdout.strip()})
    except Exception:
        pass
    return jsonify({"last_updated": None})


@app.route("/api/reboot", methods=["POST"])
def api_reboot():
    """API endpoint to reboot the system."""
    try:
        subprocess.Popen(["sudo", "reboot"], start_new_session=True)
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/usb")
def api_usb():
    """API endpoint to get USB device info including storage devices."""
    usb_devices = {"usb1": None, "usb2": None}

    try:
        found_devices = []
        usb_base = "/sys/bus/usb/devices"

        # First, find USB storage devices from /sys/block
        storage_devices = {}  # Maps USB device path to (name, size)
        for block_dev in os.listdir("/sys/block"):
            if not block_dev.startswith("sd"):
                continue
            # Check if this is a USB device by following the device symlink
            device_link = f"/sys/block/{block_dev}/device"
            if not os.path.exists(device_link):
                continue
            real_path = os.path.realpath(device_link)
            # Walk up to find the USB device (look for product file)
            path_parts = real_path.split("/")
            for i in range(len(path_parts), 0, -1):
                test_path = "/".join(path_parts[:i])
                product_file = os.path.join(test_path, "product")
                if os.path.exists(product_file):
                    with open(product_file, "r") as f:
                        product_name = f.read().strip()
                    # Get size
                    size_file = f"/sys/block/{block_dev}/size"
                    if os.path.exists(size_file):
                        with open(size_file, "r") as f:
                            sectors = int(f.read().strip())
                            size_gb = (sectors * 512) / (1024**3)
                            storage_devices[test_path] = (product_name, f"{size_gb:.1f}GB")
                    break

        # Add storage devices first (prioritized)
        for path, (name, size) in storage_devices.items():
            found_devices.append(f"{name} ({size})")

        # Then scan for other USB devices
        for entry in sorted(os.listdir(usb_base)):
            if entry.startswith("usb") or ":" in entry:
                continue

            device_path = os.path.realpath(os.path.join(usb_base, entry))
            # Skip if already found as storage
            if device_path in storage_devices:
                continue

            product_file = os.path.join(usb_base, entry, "product")
            if os.path.exists(product_file):
                with open(product_file, "r") as f:
                    product_name = f.read().strip()
                # Skip internal hubs
                if "hub" in product_name.lower():
                    continue
                found_devices.append(product_name)

        # Assign to usb1 and usb2 slots
        if len(found_devices) >= 1:
            usb_devices["usb1"] = found_devices[0]
        if len(found_devices) >= 2:
            usb_devices["usb2"] = found_devices[1]

    except Exception as e:
        print(f"Error getting USB info: {e}")

    return jsonify(usb_devices)


@app.route("/api/fan")
def api_fan():
    """API endpoint to get fan speed as percentage."""
    fan_info = {"speed_percent": None, "running": False}

    try:
        # Check if fan is actually spinning via RPM reading
        rpm_file = "/sys/class/hwmon/hwmon2/fan1_input"
        if os.path.exists(rpm_file):
            with open(rpm_file, "r") as f:
                rpm = int(f.read().strip())
                fan_info["running"] = rpm > 0

        # Get PWM percentage
        pwm_file = "/sys/class/hwmon/hwmon2/pwm1"
        if os.path.exists(pwm_file):
            with open(pwm_file, "r") as f:
                pwm_value = int(f.read().strip())
                fan_info["speed_percent"] = round((pwm_value / 255) * 100)
        else:
            # Fallback to thermal cooling device (state 0-4)
            state_file = "/sys/class/thermal/cooling_device0/cur_state"
            max_file = "/sys/class/thermal/cooling_device0/max_state"
            if os.path.exists(state_file) and os.path.exists(max_file):
                with open(state_file, "r") as f:
                    cur_state = int(f.read().strip())
                with open(max_file, "r") as f:
                    max_state = int(f.read().strip())
                if max_state > 0:
                    fan_info["speed_percent"] = round((cur_state / max_state) * 100)
                fan_info["running"] = cur_state > 0
    except Exception as e:
        print(f"Error getting fan info: {e}")

    return jsonify(fan_info)


@app.route("/api/system_stats")
def api_system_stats():
    """API endpoint to get CPU, RAM, disk usage, and Pi voltage."""
    stats = {"cpu_percent": None, "ram_percent": None, "ram_used_gb": None, "ram_total_gb": None,
             "disk_percent": None, "disk_used_gb": None, "disk_total_gb": None,
             "pi_voltage": None}

    try:
        # CPU usage - read /proc/stat twice with small delay
        def read_cpu():
            with open("/proc/stat", "r") as f:
                line = f.readline()
                parts = line.split()
                idle = int(parts[4])
                total = sum(int(p) for p in parts[1:])
                return idle, total

        idle1, total1 = read_cpu()
        import time
        time.sleep(0.1)
        idle2, total2 = read_cpu()

        idle_delta = idle2 - idle1
        total_delta = total2 - total1
        if total_delta > 0:
            stats["cpu_percent"] = round(100 * (1 - idle_delta / total_delta))

        # RAM usage from /proc/meminfo
        meminfo = {}
        with open("/proc/meminfo", "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    meminfo[parts[0].rstrip(":")] = int(parts[1])

        total_kb = meminfo.get("MemTotal", 0)
        avail_kb = meminfo.get("MemAvailable", 0)
        used_kb = total_kb - avail_kb
        if total_kb > 0:
            stats["ram_percent"] = round(100 * used_kb / total_kb)
            stats["ram_used_gb"] = round(used_kb / 1024 / 1024, 1)
            stats["ram_total_gb"] = round(total_kb / 1024 / 1024, 1)

        # Disk usage for root filesystem
        statvfs = os.statvfs("/")
        total_bytes = statvfs.f_frsize * statvfs.f_blocks
        free_bytes = statvfs.f_frsize * statvfs.f_bavail
        used_bytes = total_bytes - free_bytes
        if total_bytes > 0:
            stats["disk_percent"] = round(100 * used_bytes / total_bytes)
            stats["disk_used_gb"] = round(used_bytes / 1024 / 1024 / 1024, 1)
            stats["disk_total_gb"] = round(total_bytes / 1024 / 1024 / 1024, 1)

        # Pi voltage using vcgencmd
        try:
            result = subprocess.run(["vcgencmd", "measure_volts", "core"],
                                    capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                # Output format: "volt=0.8563V"
                voltage_str = result.stdout.strip()
                if "volt=" in voltage_str:
                    voltage = float(voltage_str.split("=")[1].rstrip("V"))
                    stats["pi_voltage"] = round(voltage, 3)
        except Exception:
            pass

    except Exception as e:
        print(f"Error getting system stats: {e}")

    return jsonify(stats)


@app.route("/api/settings", methods=["GET"])
def api_get_settings():
    """API endpoint to get UI settings for the current crawler config."""
    settings = DEFAULT_UI_SETTINGS.copy()
    settings_file = get_settings_file()

    if os.path.exists(settings_file):
        try:
            with open(settings_file, "r") as f:
                saved = json.load(f)
                # Deep merge saved settings with defaults
                for key, value in saved.items():
                    if key in settings and isinstance(settings[key], dict) and isinstance(value, dict):
                        settings[key].update(value)
                    else:
                        settings[key] = value
        except (json.JSONDecodeError, IOError) as e:
            print(f"Error loading settings from {settings_file}, using defaults: {e}")

    return jsonify(settings)


@app.route("/api/settings", methods=["POST"])
def api_save_settings():
    """API endpoint to save UI settings for the current crawler config."""
    try:
        new_settings = request.get_json()
        if not new_settings:
            return jsonify({"success": False, "error": "No data provided"}), 400

        settings_file = get_settings_file()

        # Load existing settings and merge
        settings = DEFAULT_UI_SETTINGS.copy()
        if os.path.exists(settings_file):
            try:
                with open(settings_file, "r") as f:
                    saved = json.load(f)
                    for key, value in saved.items():
                        if key in settings and isinstance(settings[key], dict) and isinstance(value, dict):
                            settings[key].update(value)
                        else:
                            settings[key] = value
            except:
                pass

        # Update with new values
        for key, value in new_settings.items():
            if key in settings and isinstance(settings[key], dict) and isinstance(value, dict):
                settings[key].update(value)
            else:
                settings[key] = value

        # Ensure directory exists
        os.makedirs(os.path.dirname(settings_file), exist_ok=True)

        # Save to file
        with open(settings_file, "w") as f:
            json.dump(settings, f, indent=2)

        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/api/driver", methods=["GET"])
def api_get_driver():
    """API endpoint to get current motor driver setting from YAML config."""
    try:
        config = load_crawler_config()
        system_id = config.get("crawler_id", "edgeflex")

        # Try installed config first, then source
        yaml_paths = [
            f"/home/craig/ros2_ws/install/pipe_crawler_control/share/pipe_crawler_control/config/{system_id}.yaml",
            f"/home/craig/ros2_ws/src/pipe_crawler_control/config/{system_id}.yaml"
        ]

        for yaml_path in yaml_paths:
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    import yaml
                    yaml_config = yaml.safe_load(f)
                driver = yaml_config.get('driver', {}).get('type', 'clearlink')
                return jsonify({"driver": driver})

        return jsonify({"driver": "clearlink"})  # Default
    except Exception as e:
        return jsonify({"driver": "clearlink", "error": str(e)})


@app.route("/api/driver", methods=["POST"])
def api_set_driver():
    """API endpoint to change motor driver in YAML config."""
    try:
        data = request.get_json()
        driver = data.get("driver")

        if driver not in ["clearlink", "roboclaw"]:
            return jsonify({"success": False, "error": "Invalid driver type"})

        # Get current crawler config
        config = load_crawler_config()
        system_id = config.get("crawler_id", "edgeflex")

        # Update source YAML config file (will be used after rebuild/restart)
        yaml_path = f"/home/craig/ros2_ws/src/pipe_crawler_control/config/{system_id}.yaml"
        if os.path.exists(yaml_path):
            import yaml
            with open(yaml_path, 'r') as f:
                yaml_config = yaml.safe_load(f)

            if 'driver' not in yaml_config:
                yaml_config['driver'] = {}
            yaml_config['driver']['type'] = driver

            with open(yaml_path, 'w') as f:
                yaml.dump(yaml_config, f, default_flow_style=False, sort_keys=False)

            return jsonify({"success": True, "restart_required": True})

        return jsonify({"success": False, "error": "Config file not found"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


# Configuration page route
@app.route("/config")
@app.route("/config.html")
def config_page():
    """Serve the configuration page."""
    config_file = os.path.join(HTML_DIR, "config.html")
    if os.path.exists(config_file):
        return send_from_directory(HTML_DIR, "config.html")
    return "Configuration page not found", 404


# Configuration API
DEFAULT_CRAWLERS = {
    "magnetic": {
        "hostname": "magnetic",
        "ip": "10.42.0.11/24",
        "name": "Magnetic Pipe Crawler",
        "yaml": "magnetic_phased_array.yaml",
        "html": "magnetic_phased_array.html",
        "motor_driver": "roboclaw"
    },
    "xpresscan": {
        "hostname": "xpresscan",
        "ip": "10.42.0.12/24",
        "name": "XPressCan",
        "yaml": "new_xpress_scan.yaml",
        "html": "new_xpress_scan.html",
        "motor_driver": "roboclaw"
    },
    "edgeflex": {
        "hostname": "edgeflex",
        "ip": "10.42.0.13/24",
        "name": "EdgeFlex RoboClaw",
        "yaml": "edgeflex.yaml",
        "html": "edgeflex.html",
        "motor_driver": "roboclaw"
    },
    "edgeflex_clearlink": {
        "hostname": "edgeflex",
        "ip": "10.42.0.14/24",
        "name": "EdgeFlex ClearLink",
        "yaml": "edgeflex_clearlink.yaml",
        "html": "edgeflex_clearlink.html",
        "motor_driver": "clearlink"
    }
}

DEFAULT_SSID = "Crawler"
DEFAULT_PASSWORD = "Crawler1"


def get_network_info():
    """Get current network interface information."""
    info = {
        "wifi": {"ip": None, "ssid": None},
        "ethernet": {"ip": None},
        "hostname": None
    }

    try:
        # Get hostname
        result = subprocess.run(["hostname"], capture_output=True, text=True, timeout=2)
        info["hostname"] = result.stdout.strip()

        # Get interface info
        result = subprocess.run(["ip", "-j", "addr", "show"], capture_output=True, text=True, timeout=2)
        interfaces = json.loads(result.stdout) if result.stdout else []

        wifi_iface = None
        for iface in interfaces:
            name = iface.get("ifname", "")
            addr_info = iface.get("addr_info", [])

            ipv4 = None
            for addr in addr_info:
                if addr.get("family") == "inet":
                    ipv4 = addr.get("local")
                    break

            if not ipv4 or ipv4.startswith("127."):
                continue

            if name.startswith("wlan"):
                info["wifi"]["ip"] = ipv4
                wifi_iface = name
            elif name.startswith("eth") or name.startswith("enp"):
                info["ethernet"]["ip"] = ipv4

        # Get SSID
        if wifi_iface:
            try:
                result = subprocess.run(["iwgetid", "-r"], capture_output=True, text=True, timeout=2)
                ssid = result.stdout.strip()
                if ssid:
                    info["wifi"]["ssid"] = ssid
            except Exception:
                pass

    except Exception as e:
        print(f"Error getting network info: {e}")

    return info


def load_full_config():
    """Load full configuration from file."""
    config = {
        "crawler_id": None,
        "ssid": DEFAULT_SSID,
        "password": DEFAULT_PASSWORD
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


def save_full_config(crawler_id, ssid, password):
    """Save configuration to file and update NetworkManager hotspot."""
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)

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


@app.route("/api/config", methods=["GET"])
def api_config_get():
    """Get current configuration."""
    config = load_full_config()
    network_info = get_network_info()

    return jsonify({
        "crawler_id": config["crawler_id"],
        "ssid": config["ssid"],
        "password": config["password"],
        "crawlers": DEFAULT_CRAWLERS,
        "network": network_info
    })


@app.route("/api/config", methods=["POST"])
def api_config_set():
    """Save configuration."""
    data = request.get_json()
    crawler_id = data.get("crawler_id")
    ssid = data.get("ssid", DEFAULT_SSID)
    password = data.get("password", DEFAULT_PASSWORD)

    if not crawler_id or crawler_id not in DEFAULT_CRAWLERS:
        return jsonify({"success": False, "error": "Invalid crawler ID"})

    if len(password) < 8:
        return jsonify({"success": False, "error": "Password must be at least 8 characters"})

    if len(ssid) < 1 or len(ssid) > 32:
        return jsonify({"success": False, "error": "SSID must be 1-32 characters"})

    if save_full_config(crawler_id, ssid, password):
        return jsonify({"success": True})
    else:
        return jsonify({"success": False, "error": "Failed to save configuration"})


@app.route("/api/config/hostname", methods=["POST"])
def api_config_hostname():
    """Change system hostname. Requires master password."""
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import verify_master_password

        data = request.get_json() or {}
        master_password = data.get("master_password", "")
        new_hostname = data.get("hostname", "").strip()

        if not master_password:
            return jsonify({"success": False, "error": "Master password is required."})

        if not verify_master_password(master_password):
            return jsonify({"success": False, "error": "Invalid master password."})

        if not new_hostname:
            return jsonify({"success": False, "error": "Hostname cannot be empty."})

        # Validate hostname: lowercase letters, digits, hyphens, max 63 chars
        import re
        if not re.match(r"^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$", new_hostname):
            return jsonify({"success": False, "error": "Invalid hostname. Use lowercase letters, digits, and hyphens (max 63 chars). Must start and end with a letter or digit."})

        # Set hostname via hostnamectl
        result = subprocess.run(["sudo", "hostnamectl", "set-hostname", new_hostname],
                                capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            return jsonify({"success": False, "error": f"Failed to set hostname: {result.stderr.strip()}"})

        # Update /etc/hosts
        try:
            with open("/etc/hosts", "r") as f:
                lines = f.readlines()
            with open("/etc/hosts", "w") as f:
                for line in lines:
                    if "127.0.1.1" in line:
                        f.write(f"127.0.1.1\t{new_hostname}\n")
                    else:
                        f.write(line)
        except Exception:
            pass

        # Update config file if it exists
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, "r") as f:
                    config_lines = f.readlines()
                with open(CONFIG_FILE, "w") as f:
                    found = False
                    for line in config_lines:
                        if line.startswith("HOSTNAME="):
                            f.write(f"HOSTNAME={new_hostname}\n")
                            found = True
                        else:
                            f.write(line)
                    if not found:
                        f.write(f"HOSTNAME={new_hostname}\n")
        except Exception:
            pass

        # Restart avahi so .local mDNS name updates immediately
        subprocess.run(["sudo", "systemctl", "restart", "avahi-daemon"],
                        capture_output=True, timeout=10)

        return jsonify({"success": True, "hostname": new_hostname})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


def scan_wifi_networks():
    """Scan for available WiFi networks."""
    networks = []
    try:
        subprocess.run(["nmcli", "device", "wifi", "rescan"], capture_output=True, timeout=5)
        result = subprocess.run(
            ["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE", "device", "wifi", "list"],
            capture_output=True, text=True, timeout=5
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

        networks.sort(key=lambda x: x["signal"], reverse=True)
    except Exception as e:
        print(f"Error scanning WiFi: {e}")

    return networks


@app.route("/api/vpn/status")
def api_vpn_status():
    """Full VPN status: enabled state, token, IP, service states."""
    VPN_TOKEN_PATH = "/etc/crawler/vpn_token"
    result = {
        "enabled": False,
        "token_exists": False,
        "connected": False,
        "vpn_ip": None,
        "tunnel_status": "inactive",
        "wg_status": "inactive",
    }

    # Check if token exists and has content
    try:
        if os.path.exists(VPN_TOKEN_PATH):
            with open(VPN_TOKEN_PATH, "r") as f:
                token = f.read().strip()
            result["token_exists"] = len(token) > 0
    except Exception:
        pass

    # Check service states
    for svc_name, key in [("wstunnel-client", "tunnel_status"), ("wg-quick@wg0", "wg_status")]:
        try:
            r = subprocess.run(
                ["systemctl", "is-active", svc_name],
                capture_output=True, text=True, timeout=5
            )
            result[key] = r.stdout.strip()  # "active", "inactive", "failed", etc.
        except Exception:
            pass

    # Enabled = token exists and at least wstunnel service is not masked/disabled
    result["enabled"] = result["token_exists"] and result["tunnel_status"] != "inactive"

    # Get VPN IP from wg0 interface
    try:
        r = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "wg0"],
            capture_output=True, text=True, timeout=5
        )
        if r.returncode == 0 and r.stdout.strip():
            # Parse: "N: wg0    inet 10.0.0.10/24 ..."
            parts = r.stdout.strip().split()
            for i, p in enumerate(parts):
                if p == "inet" and i + 1 < len(parts):
                    result["vpn_ip"] = parts[i + 1].split("/")[0]
                    break
    except Exception:
        pass

    # Check WireGuard handshake for "connected" status
    try:
        r = subprocess.run(
            ["wg", "show", "wg0", "latest-handshakes"],
            capture_output=True, text=True, timeout=5
        )
        if r.returncode == 0 and r.stdout.strip():
            parts = r.stdout.strip().split()
            if len(parts) >= 2:
                last_handshake = int(parts[1])
                result["connected"] = last_handshake > 0 and (time.time() - last_handshake) < 180
    except Exception:
        pass

    return jsonify(result)


@app.route("/api/vpn/config", methods=["POST"])
def api_vpn_config():
    """Enable/disable VPN and save registration token."""
    VPN_TOKEN_PATH = "/etc/crawler/vpn_token"
    VPN_SERVICES = ["wstunnel-client", "wg-quick@wg0", "crawler-heartbeat", "ttyd"]

    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import verify_master_password
    except ImportError:
        return jsonify({"success": False, "error": "Security module not available."})

    data = request.get_json()
    master_password = data.get("master_password", "")

    if not master_password:
        return jsonify({"success": False, "error": "Master password is required."})

    if not verify_master_password(master_password):
        return jsonify({"success": False, "error": "Invalid master password."})

    enabled = data.get("enabled", False)
    token = data.get("token", "").strip()

    try:
        if enabled:
            # Save token if provided
            if token:
                os.makedirs(os.path.dirname(VPN_TOKEN_PATH), exist_ok=True)
                with open(VPN_TOKEN_PATH, "w") as f:
                    f.write(token + "\n")
                os.chmod(VPN_TOKEN_PATH, 0o600)

            # Verify token exists before enabling
            if not os.path.exists(VPN_TOKEN_PATH):
                return jsonify({"success": False, "error": "No VPN token saved. Provide a registration token."})
            with open(VPN_TOKEN_PATH, "r") as f:
                if not f.read().strip():
                    return jsonify({"success": False, "error": "VPN token file is empty. Provide a registration token."})

            # Enable and start services in order
            for svc in VPN_SERVICES:
                subprocess.run(["systemctl", "enable", svc], capture_output=True, timeout=10)
            for svc in VPN_SERVICES:
                subprocess.run(["systemctl", "restart", svc], capture_output=True, timeout=15)

            return jsonify({"success": True, "message": "VPN enabled. Services starting..."})

        else:
            # Disable VPN: stop services and remove token
            for svc in reversed(VPN_SERVICES):
                subprocess.run(["systemctl", "stop", svc], capture_output=True, timeout=15)
                subprocess.run(["systemctl", "disable", svc], capture_output=True, timeout=10)

            # Remove token
            if os.path.exists(VPN_TOKEN_PATH):
                os.remove(VPN_TOKEN_PATH)

            return jsonify({"success": True, "message": "VPN disabled. Services stopped and token removed."})

    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/wifi/scan")
def api_wifi_scan():
    """Scan for WiFi networks."""
    networks = scan_wifi_networks()
    return jsonify({"networks": networks})


@app.route("/api/wifi/connect", methods=["POST"])
def api_wifi_connect():
    """Connect to a WiFi network.

    Creates a PERSISTENT connection using 'nmcli connection add' so credentials
    survive reboots (including gocryptfs unlock/reboot cycles).
    """
    data = request.get_json()
    ssid = data.get("ssid", "")
    password = data.get("password", "")

    if not ssid:
        return jsonify({"success": False, "message": "SSID required"})

    try:
        # Check if connection exists
        result = subprocess.run(["nmcli", "connection", "show", ssid], capture_output=True, text=True)

        if result.returncode == 0:
            # Connection exists - update password if provided, then activate
            if password:
                subprocess.run(
                    ["nmcli", "connection", "modify", ssid,
                     "wifi-sec.key-mgmt", "wpa-psk",
                     "wifi-sec.psk", password],
                    capture_output=True, text=True
                )
            result = subprocess.run(["nmcli", "connection", "up", ssid], capture_output=True, text=True, timeout=30)
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
            return jsonify({"success": True, "message": "Connected successfully"})
        else:
            return jsonify({"success": False, "message": result.stderr.strip() or "Connection failed"})
    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "message": "Connection timed out"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


# =============================================================================
# Device Security Lock API (v2 - Multi-Device Support)
# =============================================================================

DEVICE_MISMATCH_FILE = "/tmp/device_mismatch.json"


def check_device_mismatch():
    """Check if device mismatch file exists and has mismatch."""
    try:
        if os.path.exists(DEVICE_MISMATCH_FILE):
            with open(DEVICE_MISMATCH_FILE, 'r') as f:
                data = json.load(f)
                return data
    except Exception:
        pass
    return None


def verify_bypass_cookie():
    """Check if valid bypass cookie exists."""
    bypass_token = request.cookies.get(BYPASS_COOKIE_NAME)
    if not bypass_token:
        return False
    try:
        # Token format: timestamp:hash
        parts = bypass_token.split(":")
        if len(parts) != 2:
            return False
        timestamp = int(parts[0])
        # Check if expired (30 minutes)
        if time.time() - timestamp > BYPASS_COOKIE_MAX_AGE:
            return False
        return True
    except Exception:
        return False


def run_boot_security_checks():
    """Run security checks in background thread during boot.

    Ensures minimum 30 second boot splash is shown to allow all
    services (ROS2, rosbridge, WiFi) to fully initialize.
    """
    global boot_state
    import sys
    sys.path.insert(0, "/opt/crawler")

    # Minimum boot time in seconds - matches boot_splash.html timer
    MINIMUM_BOOT_TIME = 30

    try:
        from security_utils import (
            check_all_device_locks,
            get_cpu_serial,
            get_roboclaw_serial,
            get_clearlink_mac,
            write_device_mismatch
        )

        devices_to_check = ["CM5 CPU", "RoboClaw", "ClearLink"]

        # Check CM5
        with boot_state_lock:
            boot_state["current_check"] = "CM5 CPU"
            boot_state["checks_complete"] = 0
        get_cpu_serial()
        time.sleep(0.3)  # Brief pause for UI update

        with boot_state_lock:
            boot_state["checks_complete"] = 1

        # Check RoboClaw
        with boot_state_lock:
            boot_state["current_check"] = "RoboClaw"
        get_roboclaw_serial()
        time.sleep(0.3)

        with boot_state_lock:
            boot_state["checks_complete"] = 2

        # Check ClearLink
        with boot_state_lock:
            boot_state["current_check"] = "ClearLink"
        get_clearlink_mac()
        time.sleep(0.3)

        with boot_state_lock:
            boot_state["checks_complete"] = 3

        # Now do the actual lock check
        with boot_state_lock:
            boot_state["current_check"] = "Verifying locks"

        result = check_all_device_locks()

        # Determine result
        has_mismatch = False
        mismatch_data = {"devices": {}}

        for device in ["cm5", "roboclaw", "clearlink"]:
            dev_result = result.get(device, {})
            device_name = {"cm5": "CM5 CPU", "roboclaw": "RoboClaw", "clearlink": "ClearLink"}[device]
            mismatch_data["devices"][device] = {
                "name": device_name,
                "locked": dev_result.get("locked", False),
                "expected": dev_result.get("expected", ""),
                "found": dev_result.get("found", ""),
                "detected": dev_result.get("detected", False),
                "match": dev_result.get("match", True)
            }
            if dev_result.get("locked") and not dev_result.get("match"):
                has_mismatch = True

        # Set result but keep in_progress=True until minimum boot time
        with boot_state_lock:
            if has_mismatch:
                boot_state["result"] = "mismatch"
                boot_state["redirect"] = "/device_mismatch.html"
                # Write mismatch file for device_mismatch page
                write_device_mismatch(mismatch_data)
            else:
                boot_state["result"] = "ok"
                boot_state["redirect"] = "/"

            boot_state["current_check"] = "Ready"

        # Wait until minimum boot time has elapsed
        # This ensures the splash screen shows for full 30 seconds
        with boot_state_lock:
            start_time = boot_state["start_time"]

        elapsed = time.time() - start_time
        remaining = MINIMUM_BOOT_TIME - elapsed
        if remaining > 0:
            security_logger.info(f"Security checks done, waiting {remaining:.1f}s for minimum boot time")
            time.sleep(remaining)

        # Now mark boot as complete
        with boot_state_lock:
            boot_state["in_progress"] = False

        security_logger.info(f"Boot security checks complete: {boot_state['result']}")

    except Exception as e:
        security_logger.error(f"Boot security check error: {e}")
        # On error, still wait for minimum boot time before allowing access
        with boot_state_lock:
            boot_state["result"] = "error"
            boot_state["redirect"] = "/"
            start_time = boot_state["start_time"]

        elapsed = time.time() - start_time
        remaining = MINIMUM_BOOT_TIME - elapsed
        if remaining > 0:
            time.sleep(remaining)

        with boot_state_lock:
            boot_state["in_progress"] = False


@app.before_request
def check_boot_and_mismatch():
    """Handle boot splash and device mismatch redirects."""
    global boot_state

    # Always allow these paths (including static assets needed by boot splash)
    always_skip = [
        '/boot_splash.html',
        '/api/',
        '/images/',
        '/static/',
        '/css/',
        '/js/',
        '/favicon.ico',
        '/manifest.json',
        '/sw.js'
    ]
    for path in always_skip:
        if request.path.startswith(path) or request.path == path:
            return None

    # During boot, redirect everything to splash page
    with boot_state_lock:
        if boot_state["in_progress"]:
            # Allow debug mode to bypass boot check
            if request.args.get('debug') == 'true':
                return None
            return redirect('/boot_splash.html')

    # After boot, check for device mismatch
    skip_mismatch_paths = [
        '/device_mismatch.html',
        '/api/device_mismatch',
        '/api/security/bypass_mismatch',
        '/api/security/status',
        '/api/security/lock',
        '/api/security/unlock',
        '/api/security/boot_status',
        '/api/support/'
    ]
    for path in skip_mismatch_paths:
        if request.path.startswith(path) or request.path == path:
            return None

    mismatch = check_device_mismatch()
    if mismatch and mismatch.get("devices"):
        # Check if any LOCKED device has a mismatch (detected or not)
        has_mismatch = False
        for device_key, device_data in mismatch.get("devices", {}).items():
            if device_data.get("locked") and not device_data.get("match"):
                has_mismatch = True
                break

        if has_mismatch:
            # Check for valid bypass cookie - allows config.html access
            if verify_bypass_cookie():
                # Only allow config page and its API endpoints when bypassed
                allowed_bypass_paths = [
                    '/config.html',
                    '/config',
                    '/api/config',
                    '/api/security/',
                    '/api/wifi/'
                ]
                for path in allowed_bypass_paths:
                    if request.path.startswith(path) or request.path == path:
                        return None
                # Still block main control page even with bypass
                return redirect('/device_mismatch.html')
            else:
                # No bypass - redirect to error page
                return redirect('/device_mismatch.html')
    return None


@app.route("/boot_splash.html")
def boot_splash_page():
    """Serve the boot splash page."""
    return send_from_directory(HTML_DIR, "boot_splash.html")


@app.route("/images/<path:filename>")
def serve_images(filename):
    """Serve images from the images directory."""
    images_dir = os.path.join(HTML_DIR, "images")
    return send_from_directory(images_dir, filename)


@app.route("/css/<path:filename>")
def serve_css(filename):
    """Serve CSS files from the css directory."""
    css_dir = os.path.join(HTML_DIR, "css")
    return send_from_directory(css_dir, filename)


@app.route("/js/<path:filename>")
def serve_js(filename):
    """Serve JavaScript files from the js directory."""
    js_dir = os.path.join(HTML_DIR, "js")
    return send_from_directory(js_dir, filename)


@app.route("/api/security/boot_status")
def api_boot_status():
    """Get current boot/security check status."""
    global boot_state
    with boot_state_lock:
        result = boot_state["result"]
        return jsonify({
            "in_progress": boot_state["in_progress"],
            "current_check": boot_state["current_check"],
            "checks_complete": boot_state["checks_complete"],
            "checks_total": boot_state["checks_total"],
            "result": result,
            "redirect": boot_state["redirect"],
            # New fields for manual continue button flow
            "all_passed": result == "ok",
            "mismatch_detected": result == "mismatch"
        })


@app.route("/device_mismatch.html")
def device_mismatch_page():
    """Serve the device mismatch error page."""
    return send_from_directory(HTML_DIR, "device_mismatch.html")


@app.route("/api/device_mismatch")
def api_device_mismatch():
    """Get current device mismatch data."""
    mismatch = check_device_mismatch()
    if mismatch:
        return jsonify(mismatch)
    return jsonify(None)


@app.route("/api/security/status")
def api_security_status():
    """Get current security/lock status for all devices."""
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import (
            load_security_config_v2,
            check_all_device_locks,
            get_cpu_serial,
            get_device_id,
            get_roboclaw_serial,
            get_clearlink_mac,
            get_crawler_type,
            get_relevant_devices
        )

        config = load_security_config_v2()
        device_status = check_all_device_locks()
        relevant = get_relevant_devices()

        result = {
            "cm5": {
                "locked": config["cm5"]["lock_state"] == "locked",
                "locked_serial": config["cm5"]["serial"],
                "current_serial": get_cpu_serial(),
                "match": device_status["cm5"]["match"],
                "detected": device_status["cm5"]["detected"]
            },
            "roboclaw": {
                "locked": config["roboclaw"]["lock_state"] == "locked",
                "locked_serial": config["roboclaw"]["serial"],
                "current_serial": device_status["roboclaw"]["found"],
                "match": device_status["roboclaw"]["match"],
                "detected": device_status["roboclaw"]["detected"]
            },
            "clearlink": {
                "locked": config["clearlink"]["lock_state"] == "locked",
                "locked_mac": config["clearlink"]["mac"],
                "current_mac": device_status["clearlink"]["found"],
                "match": device_status["clearlink"]["match"],
                "detected": device_status["clearlink"]["detected"]
            },
            "any_mismatch": device_status["any_mismatch"],
            "master_password_set": bool(config.get("master_password_hash")),
            "crawler_type": get_crawler_type(),
            "relevant_devices": relevant,
            # Legacy fields for backwards compatibility
            "lock_state": config["cm5"]["lock_state"],
            "locked_to_device": get_device_id() if config["cm5"]["lock_state"] == "locked" else "",
            "current_device": get_device_id(),
            "is_same_device": device_status["cm5"]["match"]
        }

        return jsonify(result)

    except ImportError:
        return jsonify({
            "cm5": {"locked": False, "locked_serial": "", "current_serial": "", "match": True, "detected": False},
            "roboclaw": {"locked": False, "locked_serial": "", "current_serial": "", "match": True, "detected": False},
            "clearlink": {"locked": False, "locked_mac": "", "current_mac": "", "match": True, "detected": False},
            "any_mismatch": False,
            "master_password_set": False,
            "crawler_type": "unknown",
            "relevant_devices": ["cm5"],
            "lock_state": "unlocked",
            "locked_to_device": "",
            "current_device": "",
            "is_same_device": True,
            "error": "Security module not installed"
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/security/lock", methods=["POST"])
def api_security_lock():
    """Lock a device to current hardware."""
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import lock_single_device, lock_device, get_device_id, verify_master_password

        data = request.get_json() or {}
        device = data.get("device", "cm5")
        confirmation = data.get("confirmation", "")
        master_password = data.get("master_password", "")

        if not master_password:
            return jsonify({
                "success": False,
                "error": "Master password is required."
            })

        if not verify_master_password(master_password):
            return jsonify({
                "success": False,
                "error": "Invalid master password."
            })

        if confirmation != "LOCK":
            return jsonify({
                "success": False,
                "error": "Type 'LOCK' to confirm device locking."
            })

        # Use new lock_single_device for specific device, or legacy for cm5+encryption
        if device == "cm5":
            # Legacy behavior - locks CM5 AND encrypts source code
            success, message = lock_device()
            if success:
                subprocess.Popen(["sudo", "reboot"], start_new_session=True)
                return jsonify({
                    "success": True,
                    "message": message + " System will reboot.",
                    "locked_to": f"CM5-{get_device_id()}"
                })
        else:
            # New behavior - just lock the specific device binding
            success, message = lock_single_device(device)
            if success:
                return jsonify({
                    "success": True,
                    "message": message
                })

        return jsonify({"success": False, "error": message})

    except ImportError:
        return jsonify({"success": False, "error": "Security module not installed"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/security/unlock", methods=["POST"])
def api_security_unlock():
    """Unlock a device binding."""
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import unlock_single_device, unlock_device, clear_device_mismatch

        data = request.get_json() or {}
        device = data.get("device", "cm5")
        master_password = data.get("master_password", "")

        if not master_password:
            return jsonify({
                "success": False,
                "error": "Master password required."
            })

        if device == "cm5":
            # Legacy behavior - unlocks CM5 AND decrypts source code
            success, message = unlock_device(master_password)
            if success:
                clear_device_mismatch()
                subprocess.Popen(["sudo", "reboot"], start_new_session=True)
                return jsonify({
                    "success": True,
                    "message": message + " System will reboot."
                })
        else:
            # New behavior - just unlock the specific device binding
            success, message = unlock_single_device(device, master_password)
            if success:
                clear_device_mismatch()
                return jsonify({
                    "success": True,
                    "message": message
                })

        return jsonify({"success": False, "error": message})

    except ImportError:
        return jsonify({"success": False, "error": "Security module not installed"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/security/program_roboclaw", methods=["POST"])
def api_program_roboclaw():
    """Program a unique ID to RoboClaw EEPROM."""
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import write_roboclaw_id, generate_roboclaw_id, read_roboclaw_id

        data = request.get_json() or {}
        custom_id = data.get("custom_id", "").strip().upper()

        # Generate ID if not provided
        if not custom_id:
            custom_id = generate_roboclaw_id()
        else:
            # Validate custom ID (must be 12 hex chars)
            if len(custom_id) < 12:
                custom_id = custom_id.zfill(12)  # Pad with zeros
            if len(custom_id) != 12:
                return jsonify({
                    "success": False,
                    "error": "ID must be 12 hex characters or less"
                })
            try:
                bytes.fromhex(custom_id)
            except ValueError:
                return jsonify({
                    "success": False,
                    "error": "ID must contain only hex characters (0-9, A-F)"
                })

        # Write to RoboClaw EEPROM
        success, message = write_roboclaw_id(custom_id)

        if success:
            # Verify write by reading back
            verify_id = read_roboclaw_id()
            if verify_id == custom_id:
                return jsonify({
                    "success": True,
                    "new_id": custom_id,
                    "message": f"RoboClaw programmed with ID: {custom_id}"
                })
            else:
                return jsonify({
                    "success": False,
                    "error": f"Verification failed. Written: {custom_id}, Read: {verify_id}"
                })
        else:
            return jsonify({"success": False, "error": message})

    except ImportError:
        return jsonify({"success": False, "error": "Security module not installed"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/security/bypass_mismatch", methods=["POST"])
def api_bypass_mismatch():
    """
    Authenticate with master password to bypass device mismatch and access config page.
    Sets a cookie that allows config.html access for 30 minutes.
    """
    try:
        import sys
        import hashlib
        sys.path.insert(0, "/opt/crawler")
        from security_utils import verify_master_password

        data = request.get_json() or {}
        master_password = data.get("master_password", "")

        # Get client IP for logging
        client_ip = request.remote_addr

        if not master_password:
            security_logger.warning(f"Bypass attempt with empty password from {client_ip}")
            return jsonify({
                "success": False,
                "error": "Master password required"
            })

        if verify_master_password(master_password):
            # Success - create bypass cookie
            timestamp = int(time.time())
            # Simple token: timestamp:hash (hash prevents tampering with timestamp)
            token_data = f"{timestamp}:{master_password}"
            token_hash = hashlib.sha256(token_data.encode()).hexdigest()[:16]
            cookie_value = f"{timestamp}:{token_hash}"

            security_logger.info(f"Bypass authentication SUCCESS from {client_ip}")

            response = make_response(jsonify({
                "success": True,
                "message": "Authentication successful"
            }))
            response.set_cookie(
                BYPASS_COOKIE_NAME,
                cookie_value,
                max_age=BYPASS_COOKIE_MAX_AGE,
                httponly=True,
                samesite='Strict'
            )
            return response
        else:
            security_logger.warning(f"Bypass authentication FAILED from {client_ip}")
            return jsonify({
                "success": False,
                "error": "Invalid password"
            })

    except ImportError:
        return jsonify({"success": False, "error": "Security module not installed"})
    except Exception as e:
        security_logger.error(f"Bypass error: {e}")
        return jsonify({"success": False, "error": str(e)})


# =============================================================================
# Support / Diagnostics API
# =============================================================================

def get_support_system_info():
    """Gather comprehensive system information for support report."""
    info = {
        "report_timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "crawler": {},
        "system": {},
        "network": {},
        "ros2": {},
        "devices": {},
        "services": {},
        "logs": {},
        "hardware": {},
        "processes": {},
        "boot": {},
        "filesystem": {},
        "versions": {},
        "motor_status": {},
        "sensors": {}
    }

    try:
        # Crawler config
        crawler_config = load_crawler_config()
        info["crawler"] = {
            "id": crawler_config.get("crawler_id", "unknown"),
            "name": crawler_config.get("crawler_name", "unknown"),
            "hostname": crawler_config.get("hostname", "unknown"),
            "ui_html": crawler_config.get("ui_html", "unknown"),
            "motor_driver": crawler_config.get("motor_driver", "unknown")
        }

        # Read VERSION file
        try:
            with open("/home/craig/ros2_ws/src/pipe_crawler_control/VERSION", "r") as f:
                info["crawler"]["version"] = f.read().strip()
        except:
            info["crawler"]["version"] = "unknown"

        # System info
        try:
            result = subprocess.run(["uname", "-a"], capture_output=True, text=True, timeout=5)
            info["system"]["uname"] = result.stdout.strip()
        except:
            info["system"]["uname"] = "unknown"

        try:
            result = subprocess.run(["uptime", "-p"], capture_output=True, text=True, timeout=5)
            info["system"]["uptime"] = result.stdout.strip().replace("up ", "")
        except:
            info["system"]["uptime"] = "unknown"

        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp = int(f.read().strip()) / 1000
                info["system"]["cpu_temp"] = f"{temp:.1f}C"
        except:
            info["system"]["cpu_temp"] = "unknown"

        # Memory
        try:
            meminfo = {}
            with open("/proc/meminfo", "r") as f:
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        meminfo[parts[0].rstrip(":")] = int(parts[1])
            total_kb = meminfo.get("MemTotal", 0)
            avail_kb = meminfo.get("MemAvailable", 0)
            used_kb = total_kb - avail_kb
            info["system"]["memory"] = {
                "total_gb": round(total_kb / 1024 / 1024, 1),
                "used_gb": round(used_kb / 1024 / 1024, 1),
                "percent": round(100 * used_kb / total_kb) if total_kb > 0 else 0
            }
        except:
            info["system"]["memory"] = {"total_gb": 0, "used_gb": 0, "percent": 0}

        # Disk
        try:
            statvfs = os.statvfs("/")
            total_bytes = statvfs.f_frsize * statvfs.f_blocks
            free_bytes = statvfs.f_frsize * statvfs.f_bavail
            used_bytes = total_bytes - free_bytes
            info["system"]["disk"] = {
                "total_gb": round(total_bytes / 1024 / 1024 / 1024, 1),
                "used_gb": round(used_bytes / 1024 / 1024 / 1024, 1),
                "percent": round(100 * used_bytes / total_bytes) if total_bytes > 0 else 0
            }
        except:
            info["system"]["disk"] = {"total_gb": 0, "used_gb": 0, "percent": 0}

        # Pi voltage
        try:
            result = subprocess.run(["vcgencmd", "measure_volts", "core"],
                                    capture_output=True, text=True, timeout=2)
            if result.returncode == 0 and "volt=" in result.stdout:
                voltage = float(result.stdout.strip().split("=")[1].rstrip("V"))
                info["system"]["pi_voltage"] = f"{voltage:.3f}V"
        except:
            info["system"]["pi_voltage"] = "unknown"

        # Network
        try:
            result = subprocess.run(["ip", "-j", "addr", "show"], capture_output=True, text=True, timeout=5)
            interfaces = json.loads(result.stdout) if result.stdout else []
            for iface in interfaces:
                name = iface.get("ifname", "")
                if name.startswith("lo"):
                    continue
                for addr in iface.get("addr_info", []):
                    if addr.get("family") == "inet":
                        ip = addr.get("local")
                        if ip:
                            if name.startswith("wlan"):
                                info["network"]["wifi_ip"] = ip
                            elif name.startswith("eth") or name.startswith("enp"):
                                info["network"]["eth_ip"] = ip
        except:
            pass

        try:
            result = subprocess.run(["iwgetid", "-r"], capture_output=True, text=True, timeout=2)
            info["network"]["wifi_ssid"] = result.stdout.strip() or "Not connected"
        except:
            info["network"]["wifi_ssid"] = "unknown"

        # ROS2 nodes - must source environment first and set HOME for logging
        ROS2_SOURCE = "export HOME=/home/craig && source /opt/ros/jazzy/setup.bash && source /home/craig/ros2_ws/install/setup.bash"
        try:
            result = subprocess.run(
                ["bash", "-c", f"{ROS2_SOURCE} && ros2 node list"],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                nodes = [n.strip() for n in result.stdout.strip().split("\n") if n.strip()]
                info["ros2"]["nodes"] = nodes
                info["ros2"]["running"] = len(nodes) > 0
            else:
                info["ros2"]["nodes"] = []
                info["ros2"]["running"] = False
                info["ros2"]["error"] = result.stderr.strip() if result.stderr else "Unknown error"
        except Exception as e:
            info["ros2"]["nodes"] = []
            info["ros2"]["running"] = False
            info["ros2"]["error"] = str(e)

        # ROS2 topics
        try:
            result = subprocess.run(
                ["bash", "-c", f"{ROS2_SOURCE} && ros2 topic list"],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                topics = [t.strip() for t in result.stdout.strip().split("\n") if t.strip()]
                info["ros2"]["topics"] = topics
        except:
            info["ros2"]["topics"] = []

        # ROS2 services
        try:
            result = subprocess.run(
                ["bash", "-c", f"{ROS2_SOURCE} && ros2 service list"],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                services_list = [s.strip() for s in result.stdout.strip().split("\n") if s.strip()]
                info["ros2"]["services"] = services_list
        except:
            info["ros2"]["services"] = []

        # Device detection (security status)
        try:
            import sys
            sys.path.insert(0, "/opt/crawler")
            from security_utils import (
                get_cpu_serial, get_roboclaw_serial, get_clearlink_mac,
                load_security_config_v2, check_all_device_locks
            )
            config = load_security_config_v2()
            device_status = check_all_device_locks()

            info["devices"]["cm5"] = {
                "serial": get_cpu_serial(),
                "locked": config["cm5"]["lock_state"] == "locked",
                "locked_to": config["cm5"]["serial"],
                "match": device_status["cm5"]["match"]
            }
            info["devices"]["roboclaw"] = {
                "serial": device_status["roboclaw"]["found"],
                "detected": device_status["roboclaw"]["detected"],
                "locked": config["roboclaw"]["lock_state"] == "locked",
                "locked_to": config["roboclaw"]["serial"],
                "match": device_status["roboclaw"]["match"]
            }
            info["devices"]["clearlink"] = {
                "mac": device_status["clearlink"]["found"],
                "detected": device_status["clearlink"]["detected"],
                "locked": config["clearlink"]["lock_state"] == "locked",
                "locked_to": config["clearlink"]["mac"],
                "match": device_status["clearlink"]["match"]
            }
        except Exception as e:
            info["devices"]["error"] = str(e)

        # Service status
        services = ["pipe_crawler.service", "xpresscan-crawler-ui.service", "rosbridge.service", "xpresscan-portal.service"]
        for service in services:
            try:
                result = subprocess.run(
                    ["systemctl", "is-active", service],
                    capture_output=True, text=True, timeout=5
                )
                info["services"][service] = result.stdout.strip()
            except:
                info["services"][service] = "unknown"

        # Recent logs (last 20 lines from each service)
        for service in ["pipe_crawler.service", "xpresscan-crawler-ui.service"]:
            try:
                result = subprocess.run(
                    ["journalctl", "-u", service, "-n", "20", "--no-pager"],
                    capture_output=True, text=True, timeout=10
                )
                info["logs"][service] = result.stdout.strip()
            except:
                info["logs"][service] = "Unable to retrieve logs"

        # ==================== EXTENDED DIAGNOSTICS ====================

        # Error logs (journalctl errors only)
        try:
            result = subprocess.run(
                ["journalctl", "-p", "err", "-n", "50", "--no-pager"],
                capture_output=True, text=True, timeout=15
            )
            info["logs"]["system_errors"] = result.stdout.strip()
        except:
            info["logs"]["system_errors"] = "Unable to retrieve error logs"

        # Kernel messages (dmesg)
        try:
            result = subprocess.run(
                ["dmesg", "--time-format=short", "-l", "err,warn"],
                capture_output=True, text=True, timeout=10
            )
            # Take last 30 lines
            lines = result.stdout.strip().split("\n")[-30:]
            info["logs"]["dmesg"] = "\n".join(lines)
        except:
            info["logs"]["dmesg"] = "Unable to retrieve kernel messages"

        # Failed systemd units
        try:
            result = subprocess.run(
                ["systemctl", "--failed", "--no-pager", "--plain"],
                capture_output=True, text=True, timeout=10
            )
            info["logs"]["failed_units"] = result.stdout.strip()
        except:
            info["logs"]["failed_units"] = "Unable to retrieve"

        # Hardware - USB devices
        try:
            result = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5)
            info["hardware"]["usb_devices"] = result.stdout.strip()
        except:
            info["hardware"]["usb_devices"] = "Unable to retrieve"

        # Hardware - Serial ports
        try:
            result = subprocess.run(
                ["bash", "-c", "ls -la /dev/ttyACM* /dev/ttyUSB* /dev/ttyAMA* 2>/dev/null || echo 'No serial devices'"],
                capture_output=True, text=True, timeout=5
            )
            info["hardware"]["serial_ports"] = result.stdout.strip()
        except:
            info["hardware"]["serial_ports"] = "Unable to retrieve"

        # Hardware - I2C devices
        try:
            result = subprocess.run(
                ["bash", "-c", "i2cdetect -y 1 2>/dev/null || echo 'I2C not available'"],
                capture_output=True, text=True, timeout=5
            )
            info["hardware"]["i2c_devices"] = result.stdout.strip()
        except:
            info["hardware"]["i2c_devices"] = "Unable to retrieve"

        # Hardware - Video devices (cameras)
        try:
            result = subprocess.run(
                ["bash", "-c", "ls -la /dev/video* 2>/dev/null || echo 'No video devices'"],
                capture_output=True, text=True, timeout=5
            )
            info["hardware"]["video_devices"] = result.stdout.strip()
        except:
            info["hardware"]["video_devices"] = "Unable to retrieve"

        # Network - Full IP configuration
        try:
            result = subprocess.run(["ip", "addr", "show"], capture_output=True, text=True, timeout=5)
            info["network"]["ip_config"] = result.stdout.strip()
        except:
            info["network"]["ip_config"] = "Unable to retrieve"

        # Network - Routing table
        try:
            result = subprocess.run(["ip", "route"], capture_output=True, text=True, timeout=5)
            info["network"]["routing"] = result.stdout.strip()
        except:
            info["network"]["routing"] = "Unable to retrieve"

        # Network - Listening ports
        try:
            result = subprocess.run(["ss", "-tuln"], capture_output=True, text=True, timeout=5)
            info["network"]["listening_ports"] = result.stdout.strip()
        except:
            info["network"]["listening_ports"] = "Unable to retrieve"

        # Network - Gateway ping test
        try:
            # Get default gateway
            gw_result = subprocess.run(
                ["bash", "-c", "ip route | grep default | awk '{print $3}' | head -1"],
                capture_output=True, text=True, timeout=5
            )
            gateway = gw_result.stdout.strip()
            if gateway:
                ping_result = subprocess.run(
                    ["ping", "-c", "3", "-W", "2", gateway],
                    capture_output=True, text=True, timeout=10
                )
                info["network"]["gateway_ping"] = {
                    "gateway": gateway,
                    "success": ping_result.returncode == 0,
                    "output": ping_result.stdout.strip()[-200:] if ping_result.stdout else ""
                }
            else:
                info["network"]["gateway_ping"] = {"gateway": "None", "success": False}
        except:
            info["network"]["gateway_ping"] = {"error": "Unable to test"}

        # Network - Internet connectivity test
        try:
            result = subprocess.run(
                ["ping", "-c", "2", "-W", "2", "8.8.8.8"],
                capture_output=True, text=True, timeout=10
            )
            info["network"]["internet_ping"] = {
                "success": result.returncode == 0,
                "output": result.stdout.strip()[-200:] if result.stdout else ""
            }
        except:
            info["network"]["internet_ping"] = {"success": False, "error": "Unable to test"}

        # Network - DNS test
        try:
            result = subprocess.run(
                ["bash", "-c", "host google.com 2>&1 | head -3"],
                capture_output=True, text=True, timeout=10
            )
            info["network"]["dns_test"] = {
                "success": result.returncode == 0,
                "output": result.stdout.strip()
            }
        except:
            info["network"]["dns_test"] = {"success": False, "error": "Unable to test"}

        # Filesystem - df output
        try:
            result = subprocess.run(["df", "-h"], capture_output=True, text=True, timeout=5)
            info["filesystem"]["disk_usage"] = result.stdout.strip()
        except:
            info["filesystem"]["disk_usage"] = "Unable to retrieve"

        # Filesystem - key config file times
        try:
            config_files = [
                "/etc/xpresscan/network.conf",
                "/etc/xpresscan/security.conf",
                "/home/craig/ros2_ws/src/pipe_crawler_control/VERSION"
            ]
            file_times = {}
            for cf in config_files:
                try:
                    stat = os.stat(cf)
                    file_times[cf] = datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
                except:
                    file_times[cf] = "Not found"
            info["filesystem"]["config_file_times"] = file_times
        except:
            info["filesystem"]["config_file_times"] = {}

        # Processes - Top CPU
        try:
            result = subprocess.run(
                ["bash", "-c", "ps aux --sort=-%cpu | head -11"],
                capture_output=True, text=True, timeout=5
            )
            info["processes"]["top_cpu"] = result.stdout.strip()
        except:
            info["processes"]["top_cpu"] = "Unable to retrieve"

        # Processes - Top memory
        try:
            result = subprocess.run(
                ["bash", "-c", "ps aux --sort=-%mem | head -11"],
                capture_output=True, text=True, timeout=5
            )
            info["processes"]["top_memory"] = result.stdout.strip()
        except:
            info["processes"]["top_memory"] = "Unable to retrieve"

        # Processes - Total count
        try:
            result = subprocess.run(
                ["bash", "-c", "ps aux | wc -l"],
                capture_output=True, text=True, timeout=5
            )
            info["processes"]["total_count"] = int(result.stdout.strip()) - 1  # Subtract header
        except:
            info["processes"]["total_count"] = 0

        # Boot info - Last boot time
        try:
            result = subprocess.run(["uptime", "-s"], capture_output=True, text=True, timeout=5)
            info["boot"]["last_boot"] = result.stdout.strip()
        except:
            info["boot"]["last_boot"] = "Unknown"

        # Boot info - Boot count (from journalctl)
        try:
            result = subprocess.run(
                ["bash", "-c", "journalctl --list-boots 2>/dev/null | wc -l"],
                capture_output=True, text=True, timeout=5
            )
            info["boot"]["boot_count"] = int(result.stdout.strip())
        except:
            info["boot"]["boot_count"] = 0

        # Versions - Git info (need HOME=/root to find safe.directory config)
        git_env = os.environ.copy()
        git_env["HOME"] = "/root"
        git_repo = "/home/craig/ros2_ws/src/pipe_crawler_control"

        # Git commit hash
        try:
            result = subprocess.run(
                ["git", "-C", git_repo, "rev-parse", "--short", "HEAD"],
                capture_output=True, text=True, timeout=5, env=git_env
            )
            commit = result.stdout.strip() if result.returncode == 0 else ""
            info["versions"]["git_commit"] = commit if commit else "Unknown"
            if result.returncode != 0:
                info["versions"]["git_commit_error"] = result.stderr.strip()
        except Exception as e:
            info["versions"]["git_commit"] = f"Error: {str(e)}"

        # Git branch
        try:
            result = subprocess.run(
                ["git", "-C", git_repo, "branch", "--show-current"],
                capture_output=True, text=True, timeout=5, env=git_env
            )
            branch = result.stdout.strip() if result.returncode == 0 else ""
            info["versions"]["git_branch"] = branch if branch else "Unknown"
            if result.returncode != 0:
                info["versions"]["git_branch_error"] = result.stderr.strip()
        except Exception as e:
            info["versions"]["git_branch"] = f"Error: {str(e)}"

        # Git last commit message
        try:
            result = subprocess.run(
                ["git", "-C", git_repo, "log", "-1", "--pretty=%s"],
                capture_output=True, text=True, timeout=5, env=git_env
            )
            if result.returncode == 0 and result.stdout.strip():
                info["versions"]["git_commit_msg"] = result.stdout.strip()[:100]
        except:
            pass

        # Versions - Python version
        try:
            import sys
            info["versions"]["python"] = sys.version.split()[0]
        except:
            info["versions"]["python"] = "Unknown"

        # Versions - Key packages
        try:
            result = subprocess.run(
                ["bash", "-c", "pip3 show pycomm3 2>/dev/null | grep Version || echo 'Not installed'"],
                capture_output=True, text=True, timeout=5
            )
            info["versions"]["pycomm3"] = result.stdout.strip().replace("Version: ", "")
        except:
            info["versions"]["pycomm3"] = "Unknown"

        try:
            result = subprocess.run(
                ["bash", "-c", "pip3 show fpdf2 2>/dev/null | grep Version || echo 'Not installed'"],
                capture_output=True, text=True, timeout=5
            )
            info["versions"]["fpdf2"] = result.stdout.strip().replace("Version: ", "")
        except:
            info["versions"]["fpdf2"] = "Unknown"

        # Motor status - depends on motor driver type
        motor_driver = info.get("crawler", {}).get("motor_driver", "unknown")
        MOTOR_PEAK_TORQUE_NM = 2.0  # CPM-SDHP-2311S-ELN @ 60V

        # Get steps_per_inch from UI settings for conversions
        steps_per_inch = 53950  # Default
        try:
            crawler_id = info.get("crawler", {}).get("id", "edgeflex_clearlink")
            settings_path = f"/etc/xpresscan/ui_settings_{crawler_id}.json"
            if os.path.exists(settings_path):
                with open(settings_path, "r") as f:
                    ui_settings_data = json.load(f)
                    steps_per_inch = ui_settings_data.get("motorParams", {}).get("stepsPerInch", 53950)
        except:
            pass

        if motor_driver == "clearlink":
            # ClearLink motor status via ROS2 - get structured data
            try:
                ROS2_SOURCE = "export HOME=/home/craig && source /opt/ros/jazzy/setup.bash && source /home/craig/ros2_ws/install/setup.bash"
                result = subprocess.run(
                    ["bash", "-c", f"{ROS2_SOURCE} && ros2 topic echo /clearlink/status --once --no-arr 2>/dev/null"],
                    capture_output=True, text=True, timeout=10
                )
                if result.returncode == 0 and result.stdout.strip():
                    raw_status = result.stdout.strip()
                    info["motor_status"]["clearlink_raw"] = raw_status

                    # Parse YAML-like output into structured data
                    motor_data = {}
                    for line in raw_status.split("\n"):
                        line = line.strip()
                        if ": " in line:
                            key, val = line.split(": ", 1)
                            motor_data[key] = val

                    # Extract and convert values
                    def parse_bool(v):
                        return str(v).lower() == "true"

                    def parse_int(v, default=0):
                        try:
                            return int(v)
                        except:
                            return default

                    def parse_float(v, default=0.0):
                        try:
                            return float(v)
                        except:
                            return default

                    # Motor 0 (Axis 1)
                    m0_vel_steps = parse_int(motor_data.get("axis1_velocity", 0))
                    m0_vel_in_s = abs(m0_vel_steps) / steps_per_inch if steps_per_inch else 0
                    m0_torque_pct = parse_float(motor_data.get("axis1_torque", 0))
                    m0_torque_nm = abs(m0_torque_pct) / 100.0 * MOTOR_PEAK_TORQUE_NM if m0_torque_pct != -9999 else 0

                    info["motor_status"]["motor0"] = {
                        "name": "Motor 0 (Left/Axis 1)",
                        "velocity_steps": m0_vel_steps,
                        "velocity_in_s": round(m0_vel_in_s, 3),
                        "torque_percent": m0_torque_pct if m0_torque_pct != -9999 else "N/A",
                        "torque_nm": round(m0_torque_nm, 3),
                        "position": parse_int(motor_data.get("axis1_position", 0)),
                        "enabled": parse_bool(motor_data.get("axis1_enabled", "false")),
                        "moving": parse_bool(motor_data.get("axis1_moving", "false")),
                        "fault": parse_bool(motor_data.get("axis1_fault", "false")),
                        "homed": parse_bool(motor_data.get("axis1_homed", "false")),
                        "direction": "Forward" if m0_vel_steps > 0 else "Reverse" if m0_vel_steps < 0 else "Stopped"
                    }

                    # Motor 1 (Axis 2)
                    m1_vel_steps = parse_int(motor_data.get("axis2_velocity", 0))
                    m1_vel_in_s = abs(m1_vel_steps) / steps_per_inch if steps_per_inch else 0
                    m1_torque_pct = parse_float(motor_data.get("axis2_torque", 0))
                    m1_torque_nm = abs(m1_torque_pct) / 100.0 * MOTOR_PEAK_TORQUE_NM if m1_torque_pct != -9999 else 0

                    info["motor_status"]["motor1"] = {
                        "name": "Motor 1 (Right/Axis 2)",
                        "velocity_steps": m1_vel_steps,
                        "velocity_in_s": round(m1_vel_in_s, 3),
                        "torque_percent": m1_torque_pct if m1_torque_pct != -9999 else "N/A",
                        "torque_nm": round(m1_torque_nm, 3),
                        "position": parse_int(motor_data.get("axis2_position", 0)),
                        "enabled": parse_bool(motor_data.get("axis2_enabled", "false")),
                        "moving": parse_bool(motor_data.get("axis2_moving", "false")),
                        "fault": parse_bool(motor_data.get("axis2_fault", "false")),
                        "homed": parse_bool(motor_data.get("axis2_homed", "false")),
                        "direction": "Forward" if m1_vel_steps > 0 else "Reverse" if m1_vel_steps < 0 else "Stopped"
                    }

                    # ClearLink controller info
                    info["motor_status"]["controller"] = {
                        "type": "ClearLink",
                        "ip": "192.168.20.240",
                        "port": 44818,
                        "connected": parse_bool(motor_data.get("connected", "false")),
                        "firmware": motor_data.get("firmware_version", "Unknown")
                    }
                else:
                    info["motor_status"]["clearlink_raw"] = "No data available"
            except Exception as e:
                info["motor_status"]["clearlink_error"] = str(e)

        elif motor_driver == "roboclaw":
            # RoboClaw status via ROS2
            try:
                ROS2_SOURCE = "export HOME=/home/craig && source /opt/ros/jazzy/setup.bash && source /home/craig/ros2_ws/install/setup.bash"
                result = subprocess.run(
                    ["bash", "-c", f"{ROS2_SOURCE} && ros2 topic echo /roboclaw/status --once --no-arr 2>/dev/null"],
                    capture_output=True, text=True, timeout=10
                )
                if result.returncode == 0 and result.stdout.strip():
                    info["motor_status"]["roboclaw"] = result.stdout.strip()
                else:
                    info["motor_status"]["roboclaw"] = "No data available"
            except:
                info["motor_status"]["roboclaw"] = "Unable to retrieve"

        # Historical motor data (from data logger if available)
        try:
            import sqlite3
            DB_PATH = "/var/lib/crawler/motor_data.db"
            if os.path.exists(DB_PATH):
                conn = sqlite3.connect(DB_PATH)
                conn.row_factory = sqlite3.Row
                cursor = conn.cursor()

                # Get last hour of data for graphs
                from datetime import timedelta
                cutoff = datetime.now() - timedelta(hours=1)
                cursor.execute('''
                    SELECT * FROM motor_data
                    WHERE timestamp > ?
                    ORDER BY timestamp ASC
                ''', (cutoff,))
                rows = cursor.fetchall()
                info["motor_status"]["historical_data"] = [dict(row) for row in rows]
                info["motor_status"]["historical_count"] = len(rows)

                # Get command history
                cursor.execute('''
                    SELECT * FROM command_history
                    ORDER BY timestamp DESC
                    LIMIT 10
                ''')
                cmd_rows = cursor.fetchall()
                info["motor_status"]["command_history"] = [dict(row) for row in cmd_rows]

                conn.close()
        except Exception as e:
            info["motor_status"]["historical_error"] = str(e)

        # Motor-related log entries
        try:
            result = subprocess.run(
                ["bash", "-c", "journalctl -u pipe_crawler.service -n 100 --no-pager 2>/dev/null | grep -iE '(clearlink|motor|fault|error|warn)' | tail -30"],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0 and result.stdout.strip():
                info["motor_status"]["motor_logs"] = result.stdout.strip()
        except:
            pass

        # IMU/Sensor status - improved detection
        # Check multiple indicators: serial port, ROS node, topic publisher
        imu_connected = False
        imu_details = {}

        # Check 1: Serial port exists
        try:
            imu_details["serial_port"] = os.path.exists("/dev/ttyAMA0")
        except:
            imu_details["serial_port"] = False

        # Check 2: wt901_imu node is running
        try:
            nodes = info.get("ros2", {}).get("nodes", [])
            imu_details["node_running"] = any("wt901" in n.lower() for n in nodes)
        except:
            imu_details["node_running"] = False

        # Check 3: IMU topic has a publisher
        try:
            ROS2_SOURCE = "export HOME=/home/craig && source /opt/ros/jazzy/setup.bash && source /home/craig/ros2_ws/install/setup.bash"
            result = subprocess.run(
                ["bash", "-c", f"{ROS2_SOURCE} && ros2 topic info /imu/pitch 2>/dev/null"],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0 and "Publisher count:" in result.stdout:
                pub_count = int(result.stdout.split("Publisher count:")[1].split()[0])
                imu_details["topic_publisher"] = pub_count > 0
            else:
                imu_details["topic_publisher"] = False
        except:
            imu_details["topic_publisher"] = False

        # IMU is connected if node is running OR (serial port exists AND topic has publisher)
        imu_connected = imu_details.get("node_running", False) or \
                       (imu_details.get("serial_port", False) and imu_details.get("topic_publisher", False))

        info["sensors"]["imu_connected"] = imu_connected
        info["sensors"]["imu_details"] = imu_details

        # Get saved WiFi networks from NetworkManager
        try:
            result = subprocess.run(
                ["nmcli", "-t", "-f", "NAME,TYPE,AUTOCONNECT", "connection", "show"],
                capture_output=True, text=True, timeout=5
            )
            wifi_connections = []
            if result.returncode == 0:
                for line in result.stdout.strip().split("\n"):
                    if line:
                        parts = line.split(":")
                        if len(parts) >= 3 and "wireless" in parts[1].lower():
                            wifi_connections.append({
                                "name": parts[0],
                                "type": parts[1],
                                "autoconnect": parts[2] == "yes"
                            })
            info["network"]["saved_wifi"] = wifi_connections
        except:
            info["network"]["saved_wifi"] = []

        # Get UI settings for all crawlers
        try:
            all_ui_settings = {}
            for crawler_id in ["magnetic", "xpresscan", "edgeflex", "edgeflex_clearlink"]:
                settings_path = f"/etc/xpresscan/ui_settings_{crawler_id}.json"
                if os.path.exists(settings_path):
                    with open(settings_path, "r") as f:
                        all_ui_settings[crawler_id] = {
                            "settings": json.load(f),
                            "path": settings_path,
                            "mtime": datetime.fromtimestamp(os.path.getmtime(settings_path)).strftime("%Y-%m-%d %H:%M:%S")
                        }
            info["ui_settings"] = all_ui_settings
            info["active_crawler_id"] = info.get("crawler", {}).get("id", "unknown")
        except Exception as e:
            info["ui_settings"] = {}
            info["ui_settings_error"] = str(e)

    except Exception as e:
        info["error"] = str(e)

    return info


@app.route("/api/support/system_info")
def api_support_system_info():
    """Get comprehensive system information for support."""
    info = get_support_system_info()
    return jsonify(info)


@app.route("/api/support/download_report")
def api_support_download_report():
    """Generate and download a PDF support report."""
    try:
        from fpdf import FPDF

        info = get_support_system_info()
        issue_description = request.args.get("issue", "No description provided")

        # Helper to sanitize text for PDF (remove/replace unsupported Unicode)
        def sanitize_text(text):
            if not text:
                return ""
            # Replace common problematic Unicode characters
            replacements = {
                '\u2192': '->',   # →
                '\u2190': '<-',   # ←
                '\u2022': '*',    # •
                '\u2013': '-',    # –
                '\u2014': '--',   # —
                '\u201c': '"',    # "
                '\u201d': '"',    # "
                '\u2018': "'",    # '
                '\u2019': "'",    # '
                '\u00a0': ' ',    # non-breaking space
                '\u2026': '...',  # …
                '\u00b0': ' deg', # °
            }
            for char, replacement in replacements.items():
                text = text.replace(char, replacement)
            # Remove any remaining non-ASCII characters
            return text.encode('ascii', 'replace').decode('ascii')

        # Prime Inspections brand colors
        PRIME_CYAN = (0, 200, 212)       # #00C8D4 - Primary brand color
        PRIME_DARK = (51, 51, 51)        # #333333 - Dark text
        PRIME_GRAY = (102, 102, 102)     # #666666 - Secondary text
        PRIME_LIGHT_GRAY = (153, 153, 153)  # #999999 - Muted text

        HEADER_BG = (0, 200, 212)        # Cyan header background
        HEADER_TEXT = (255, 255, 255)    # White header text
        ROW_ALT = (245, 245, 247)        # Light gray alternating row
        ROW_WHITE = (255, 255, 255)      # White row
        BORDER_COLOR = (224, 224, 224)   # Light border
        TEXT_DARK = (51, 51, 51)         # Dark text
        TEXT_GRAY = (102, 102, 102)      # Gray text
        GREEN = (0, 200, 81)             # #00C851 - Success green
        YELLOW = (255, 215, 0)           # #FFD700 - Warning yellow
        RED = (255, 68, 68)              # #FF4444 - Error red
        BLUE = (0, 102, 204)             # Link blue

        def draw_section_header(pdf, title):
            """Draw a Prime Inspections-style section header with cyan accent."""
            pdf.set_font("Helvetica", "B", 13)
            pdf.set_text_color(*PRIME_CYAN)
            pdf.cell(0, 10, title, ln=True)
            pdf.set_draw_color(*PRIME_CYAN)
            pdf.set_line_width(0.8)
            pdf.line(10, pdf.get_y(), 200, pdf.get_y())
            pdf.set_line_width(0.2)
            pdf.ln(3)

        def draw_table(pdf, headers, rows, col_widths=None):
            """Draw a Prime Inspections-style table with cyan header and alternating rows."""
            if not col_widths:
                col_widths = [190 // len(headers)] * len(headers)

            # Header row with cyan background
            pdf.set_fill_color(*PRIME_CYAN)
            pdf.set_text_color(*HEADER_TEXT)
            pdf.set_font("Helvetica", "B", 9)
            for i, header in enumerate(headers):
                pdf.cell(col_widths[i], 7, header, border=0, fill=True)
            pdf.ln()

            # Data rows
            pdf.set_font("Helvetica", "", 9)
            pdf.set_text_color(*TEXT_DARK)
            for row_idx, row in enumerate(rows):
                fill_color = ROW_ALT if row_idx % 2 == 0 else ROW_WHITE
                pdf.set_fill_color(*fill_color)
                for i, cell in enumerate(row):
                    # Handle status coloring with symbols
                    if isinstance(cell, tuple):
                        text, color = cell
                        # Add status symbols
                        if color == GREEN:
                            text = f"[OK] {text}" if "OK" not in str(text).upper() and "PASS" not in str(text).upper() else text
                        elif color == RED:
                            text = f"[X] {text}" if "FAIL" not in str(text).upper() and "ERROR" not in str(text).upper() else text
                        elif color == YELLOW:
                            text = f"[!] {text}"
                        pdf.set_text_color(*color)
                        pdf.cell(col_widths[i], 6, str(text), border=0, fill=True)
                        pdf.set_text_color(*TEXT_DARK)
                    else:
                        pdf.cell(col_widths[i], 6, str(cell)[:50], border=0, fill=True)
                pdf.ln()
            pdf.ln(3)

        def draw_kv_table(pdf, data, col_widths=None):
            """Draw a key-value table."""
            if not col_widths:
                col_widths = [60, 130]
            rows = [[k, v] for k, v in data.items()]
            draw_table(pdf, ["Setting", "Value"], rows, col_widths)

        # Create PDF
        pdf = FPDF()
        pdf.set_auto_page_break(auto=True, margin=15)
        pdf.add_page()

        # Professional header with Prime Inspections branding
        # Try to add logo
        logo_path = "/opt/xpresscan/html/images/prime_inspections_logo.png"
        if os.path.exists(logo_path):
            try:
                pdf.image(logo_path, x=10, y=10, w=60)
                pdf.set_xy(75, 10)
            except:
                pdf.set_xy(10, 10)
        else:
            pdf.set_xy(10, 10)

        # Title
        pdf.set_font("Helvetica", "B", 18)
        pdf.set_text_color(*PRIME_CYAN)
        pdf.cell(0, 10, "CRAWLER SUPPORT REPORT", ln=True, align="R")

        # Subtitle with timestamp
        pdf.set_font("Helvetica", "", 10)
        pdf.set_text_color(*TEXT_GRAY)
        pdf.cell(0, 5, f"Generated: {info.get('report_timestamp', 'Unknown')}", ln=True, align="R")
        crawler_name = info.get("crawler", {}).get("name", "Unknown Crawler")
        pdf.cell(0, 5, f"Crawler: {crawler_name}", ln=True, align="R")

        # Horizontal line separator
        pdf.ln(5)
        pdf.set_draw_color(*PRIME_CYAN)
        pdf.set_line_width(1)
        pdf.line(10, pdf.get_y(), 200, pdf.get_y())
        pdf.set_line_width(0.2)
        pdf.ln(8)

        # Diagnostic Summary Box
        pdf.set_fill_color(245, 245, 247)
        pdf.set_draw_color(*PRIME_CYAN)
        summary_y = pdf.get_y()
        pdf.rect(10, summary_y, 190, 40, style='D')

        pdf.set_xy(15, summary_y + 3)
        pdf.set_font("Helvetica", "B", 12)
        pdf.set_text_color(*PRIME_CYAN)
        pdf.cell(0, 6, "DIAGNOSTIC SUMMARY", ln=True)

        # Quick status checks
        services = info.get("services", {})
        ros2 = info.get("ros2", {})
        network = info.get("network", {})
        devices = info.get("devices", {})
        system = info.get("system", {})
        motor_status = info.get("motor_status", {})

        checks = []
        warnings = 0
        criticals = 0

        # Service check - distinguish critical vs optional services
        critical_services = ["pipe_crawler.service", "xpresscan-crawler-ui.service", "rosbridge.service"]
        optional_services = ["xpresscan-portal.service"]

        critical_ok = all(services.get(s) == "active" for s in critical_services)
        optional_ok = all(services.get(s) == "active" for s in optional_services)
        active_count = sum(1 for s in services.values() if s == "active")
        total_count = len(services)

        if critical_ok and optional_ok:
            checks.append(("[OK]", "System Services", GREEN))
        elif critical_ok and not optional_ok:
            # Optional services inactive - just a warning
            checks.append(("[!]", f"Services ({active_count}/{total_count} active)", YELLOW))
            warnings += 1
        else:
            # Critical service failed
            failed_critical = [s for s in critical_services if services.get(s) != "active"]
            checks.append(("[X]", f"Services FAILED ({', '.join(s.replace('.service','') for s in failed_critical)})", RED))
            criticals += 1

        # ROS2 check
        ros2_ok = ros2.get("running", False)
        checks.append(("[OK]", "ROS2 Running", GREEN) if ros2_ok else ("[X]", "ROS2 Stopped", RED))
        if not ros2_ok:
            criticals += 1

        # Network check
        inet_ok = network.get("internet_ping", {}).get("success", False)
        checks.append(("[OK]", "Network Connectivity", GREEN) if inet_ok else ("[!]", "No Internet", YELLOW))
        if not inet_ok:
            warnings += 1

        # Device security check
        cm5_match = devices.get("cm5", {}).get("match", True)
        checks.append(("[OK]", "Device Security", GREEN) if cm5_match else ("[X]", "Device Mismatch", RED))
        if not cm5_match:
            criticals += 1

        # Motor check
        m0_fault = motor_status.get("motor0", {}).get("fault", False)
        m1_fault = motor_status.get("motor1", {}).get("fault", False)
        motors_ok = not (m0_fault or m1_fault)
        checks.append(("[OK]", "Motors OK", GREEN) if motors_ok else ("[X]", "Motor Fault", RED))
        if not motors_ok:
            criticals += 1

        # Disk check
        disk_pct = system.get("disk", {}).get("percent", 0)
        if disk_pct > 90:
            checks.append(("[X]", f"Disk {disk_pct}% Full", RED))
            criticals += 1
        elif disk_pct > 75:
            checks.append(("[!]", f"Disk {disk_pct}% Used", YELLOW))
            warnings += 1
        else:
            checks.append(("[OK]", f"Disk {disk_pct}% Used", GREEN))

        # Display checks in two columns
        pdf.set_font("Helvetica", "", 9)
        col1_x = 15
        col2_x = 105
        check_y = summary_y + 12

        for i, (symbol, text, color) in enumerate(checks):
            x = col1_x if i % 2 == 0 else col2_x
            if i % 2 == 0 and i > 0:
                check_y += 5
            pdf.set_xy(x, check_y)
            pdf.set_text_color(*color)
            pdf.cell(10, 5, symbol)
            pdf.set_text_color(*TEXT_DARK)
            pdf.cell(0, 5, text)

        # Overall status
        pdf.set_xy(15, summary_y + 32)
        pdf.set_font("Helvetica", "B", 10)
        if criticals > 0:
            pdf.set_text_color(*RED)
            pdf.cell(0, 5, f"Overall: CRITICAL ({criticals} issues, {warnings} warnings)")
        elif warnings > 0:
            pdf.set_text_color(*YELLOW)
            pdf.cell(0, 5, f"Overall: WARNING ({warnings} warnings)")
        else:
            pdf.set_text_color(*GREEN)
            pdf.cell(0, 5, "Overall: HEALTHY")

        pdf.set_text_color(*TEXT_DARK)
        pdf.set_y(summary_y + 45)
        pdf.ln(5)

        # Issue Description Box
        pdf.set_fill_color(255, 243, 224)  # Light orange/warning background
        pdf.set_draw_color(255, 149, 0)    # Orange border
        pdf.rect(10, pdf.get_y(), 190, 25, style='DF')
        pdf.set_xy(15, pdf.get_y() + 3)
        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "Issue Description", ln=True)
        pdf.set_x(15)
        pdf.set_font("Helvetica", "", 9)
        # Truncate issue description if too long
        issue_text = issue_description[:200] + "..." if len(issue_description) > 200 else issue_description
        pdf.multi_cell(180, 5, sanitize_text(issue_text))
        pdf.ln(8)

        # ==================== CRAWLER CONFIGURATION ====================
        draw_section_header(pdf, "Crawler Configuration")
        crawler = info.get("crawler", {})
        crawler_data = {
            "Crawler ID": crawler.get('id', 'unknown'),
            "Name": crawler.get('name', 'unknown'),
            "Hostname": crawler.get('hostname', 'unknown'),
            "Software Version": crawler.get('version', 'unknown'),
            "Motor Driver": crawler.get('motor_driver', 'unknown'),
            "UI HTML": crawler.get('ui_html', 'unknown')
        }
        draw_kv_table(pdf, crawler_data)

        # ==================== SYSTEM INFORMATION ====================
        draw_section_header(pdf, "System Information")
        system = info.get("system", {})
        mem = system.get("memory", {})
        disk = system.get("disk", {})

        system_rows = [
            ["Uptime", system.get('uptime', 'unknown')],
            ["CPU Temperature", system.get('cpu_temp', 'unknown')],
            ["Pi Core Voltage", system.get('pi_voltage', 'unknown')],
            ["Memory Usage", f"{mem.get('used_gb', 0)}GB / {mem.get('total_gb', 0)}GB ({mem.get('percent', 0)}%)"],
            ["Disk Usage", f"{disk.get('used_gb', 0)}GB / {disk.get('total_gb', 0)}GB ({disk.get('percent', 0)}%)"],
        ]
        draw_table(pdf, ["Metric", "Value"], system_rows, [60, 130])

        # Kernel info (separate, smaller)
        pdf.set_font("Helvetica", "", 8)
        pdf.set_text_color(*TEXT_GRAY)
        kernel = system.get('uname', 'unknown')[:90]
        pdf.cell(0, 5, f"Kernel: {sanitize_text(kernel)}", ln=True)
        pdf.ln(5)

        # ==================== NETWORK ====================
        draw_section_header(pdf, "Network Status")
        network = info.get("network", {})

        # Current connection table
        net_rows = [
            ["WiFi SSID", network.get('wifi_ssid', 'Not connected')],
            ["WiFi IP Address", network.get('wifi_ip', 'None')],
            ["Ethernet IP", network.get('eth_ip', 'None')],
        ]
        draw_table(pdf, ["Connection", "Value"], net_rows, [60, 130])

        # Saved WiFi Networks
        saved_wifi = network.get("saved_wifi", [])
        if saved_wifi:
            pdf.set_font("Helvetica", "B", 10)
            pdf.set_text_color(*TEXT_DARK)
            pdf.cell(0, 6, "Saved WiFi Networks:", ln=True)
            wifi_rows = []
            for wifi in saved_wifi:
                auto = ("Yes", GREEN) if wifi.get("autoconnect") else ("No", TEXT_GRAY)
                wifi_rows.append([wifi.get("name", "Unknown"), auto])
            draw_table(pdf, ["Network Name", "Auto-Connect"], wifi_rows, [120, 70])
        pdf.ln(3)

        # ==================== DEVICE SECURITY ====================
        draw_section_header(pdf, "Device Security Status")
        devices = info.get("devices", {})
        if "error" in devices:
            pdf.set_text_color(*RED)
            pdf.cell(0, 6, f"Error: {devices['error']}", ln=True)
            pdf.set_text_color(*TEXT_DARK)
        else:
            device_rows = []
            for dev_name in ["cm5", "roboclaw", "clearlink"]:
                dev = devices.get(dev_name, {})
                serial = dev.get("serial") or dev.get("mac") or "N/A"
                lock_status = ("LOCKED", TEXT_DARK) if dev.get("locked") else ("Unlocked", TEXT_GRAY)
                match_status = ("OK", GREEN) if dev.get("match") else ("MISMATCH", RED)
                detected = "Yes" if dev.get("detected", True) else "No"
                device_rows.append([dev_name.upper(), serial[:20], lock_status, match_status])
            draw_table(pdf, ["Device", "Serial/MAC", "Lock", "Status"], device_rows, [35, 75, 40, 40])

        # ==================== SERVICES ====================
        draw_section_header(pdf, "System Services")
        services = info.get("services", {})
        optional_services = ["xpresscan-portal.service"]
        service_rows = []
        for svc, status in services.items():
            # Color: active=green, inactive optional=yellow, inactive critical=red
            is_optional = svc in optional_services
            if status == "active":
                status_colored = (status, GREEN)
            elif is_optional:
                status_colored = (f"{status} (optional)", YELLOW)
            else:
                status_colored = (status, RED)
            svc_name = svc.replace(".service", "")
            service_rows.append([svc_name, status_colored])
        draw_table(pdf, ["Service", "Status"], service_rows, [140, 50])

        # Add note about optional services
        pdf.set_font("Helvetica", "I", 8)
        pdf.set_text_color(*TEXT_GRAY)
        pdf.cell(0, 4, "Note: xpresscan-portal is optional (only needed for WiFi captive portal setup).", ln=True)
        pdf.set_text_color(*TEXT_DARK)

        # ==================== ROS2 STATUS ====================
        draw_section_header(pdf, "ROS2 Status")
        ros2 = info.get("ros2", {})
        running_status = ("Running", GREEN) if ros2.get('running') else ("Stopped", RED)

        ros2_summary = [
            ["Status", running_status],
            ["Active Nodes", str(len(ros2.get("nodes", [])))],
            ["Active Topics", str(len(ros2.get("topics", [])))],
            ["Active Services", str(len(ros2.get("services", [])))],
        ]
        draw_table(pdf, ["Metric", "Value"], ros2_summary, [80, 110])

        # List nodes in a compact format
        nodes = ros2.get("nodes", [])
        if nodes:
            pdf.set_font("Helvetica", "B", 9)
            pdf.cell(0, 5, "Nodes:", ln=True)
            pdf.set_font("Courier", "", 8)
            pdf.set_text_color(*TEXT_GRAY)
            node_text = ", ".join(sanitize_text(n) for n in nodes[:10])
            if len(nodes) > 10:
                node_text += f" (+{len(nodes)-10} more)"
            pdf.multi_cell(0, 4, node_text)
            pdf.set_text_color(*TEXT_DARK)
        pdf.ln(5)

        # ==================== VERSION INFO ====================
        draw_section_header(pdf, "Version Information")
        versions = info.get("versions", {})
        git_commit = versions.get('git_commit', 'Unknown')
        git_commit_display = git_commit[:12] if git_commit and not git_commit.startswith("Error") else git_commit
        version_data = {
            "Git Branch": versions.get('git_branch', 'Unknown'),
            "Git Commit": git_commit_display,
            "Python": versions.get('python', 'Unknown'),
            "pycomm3": versions.get('pycomm3', 'Unknown'),
            "fpdf2": versions.get('fpdf2', 'Unknown'),
        }
        draw_kv_table(pdf, version_data)

        # Show commit message if available
        commit_msg = versions.get('git_commit_msg', '')
        if commit_msg:
            pdf.set_font("Helvetica", "I", 8)
            pdf.set_text_color(*TEXT_GRAY)
            pdf.cell(0, 4, f"Last commit: {sanitize_text(commit_msg[:80])}", ln=True)
            pdf.set_text_color(*TEXT_DARK)

        # ==================== BOOT INFO ====================
        boot = info.get("boot", {})
        boot_data = {
            "Last Boot": boot.get('last_boot', 'Unknown'),
            "Boot Count": str(boot.get('boot_count', 0)),
        }
        draw_kv_table(pdf, boot_data)

        # ==================== HARDWARE ====================
        pdf.add_page()
        draw_section_header(pdf, "Hardware Details")

        hardware = info.get("hardware", {})

        # USB Devices
        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "USB Devices:", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        usb_lines = str(hardware.get("usb_devices", "None")).split("\n")[:12]
        for line in usb_lines:
            if len(line) > 100:
                line = line[:100] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(4)

        # Serial Ports
        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "Serial Ports:", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        for line in str(hardware.get("serial_ports", "None")).split("\n")[:8]:
            if len(line) > 100:
                line = line[:100] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(4)

        # Video Devices
        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "Video Devices (Cameras):", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        for line in str(hardware.get("video_devices", "None")).split("\n")[:5]:
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(6)
        pdf.set_text_color(*TEXT_DARK)

        # ==================== NETWORK DIAGNOSTICS ====================
        draw_section_header(pdf, "Network Diagnostics")

        network = info.get("network", {})

        # Connectivity tests as a table
        gw_ping = network.get("gateway_ping", {})
        inet_ping = network.get("internet_ping", {})
        dns_test = network.get("dns_test", {})

        conn_rows = [
            [f"Gateway ({gw_ping.get('gateway', 'N/A')})",
             ("PASS", GREEN) if gw_ping.get("success") else ("FAIL", RED)],
            ["Internet (8.8.8.8)",
             ("PASS", GREEN) if inet_ping.get("success") else ("FAIL", RED)],
            ["DNS Resolution",
             ("PASS", GREEN) if dns_test.get("success") else ("FAIL", RED)],
        ]
        draw_table(pdf, ["Test", "Result"], conn_rows, [130, 60])

        # Routing table
        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "Routing Table:", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        for line in str(network.get("routing", "None")).split("\n")[:6]:
            if len(line) > 100:
                line = line[:100] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(4)

        # Listening ports
        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "Key Listening Ports:", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        for line in str(network.get("listening_ports", "None")).split("\n")[:12]:
            if len(line) > 105:
                line = line[:105] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(6)
        pdf.set_text_color(*TEXT_DARK)

        # ==================== MOTOR STATUS ====================
        pdf.add_page()
        draw_section_header(pdf, "Motor Driver Status")

        motor_status = info.get("motor_status", {})
        motor_driver = info.get("crawler", {}).get("motor_driver", "unknown")

        # Always show this section - display errors if data unavailable
        if not motor_status or (not motor_status.get("motor0") and not motor_status.get("controller")):
            # No live motor data available - try to use historical data
            pdf.set_font("Helvetica", "", 10)
            pdf.set_text_color(*TEXT_GRAY)
            pdf.cell(0, 6, f"Motor Driver Type: {motor_driver.upper()}", ln=True)

            historical_data = motor_status.get("historical_data", [])
            if historical_data:
                # Use most recent historical data point
                latest = historical_data[-1]
                pdf.ln(2)
                pdf.set_font("Helvetica", "B", 10)
                pdf.set_text_color(*PRIME_CYAN)
                pdf.cell(0, 6, "Latest Recorded Motor Status", ln=True)
                pdf.set_font("Helvetica", "I", 8)
                pdf.set_text_color(*TEXT_GRAY)
                pdf.cell(0, 4, f"(from database: {latest.get('timestamp', 'Unknown')})", ln=True)
                pdf.set_text_color(*TEXT_DARK)

                # Motor 0 from historical
                pdf.set_font("Helvetica", "B", 10)
                pdf.cell(0, 6, "Motor 0 (Left/Axis 1)", ln=True)
                m0_rows = [
                    ["Velocity", f"{latest.get('m0_velocity_steps', 0):,} steps/s ({latest.get('m0_velocity_in_s', 0):.3f} in/s)"],
                    ["Torque", f"{latest.get('m0_torque_percent', 0):.1f}% ({latest.get('m0_torque_nm', 0):.3f} Nm)"],
                    ["Position", f"{latest.get('m0_position', 0):,} steps"],
                    ["Enabled", ("Yes", GREEN) if latest.get("m0_enabled") else ("No", TEXT_GRAY)],
                    ["Moving", ("Yes", PRIME_CYAN) if latest.get("m0_moving") else ("No", TEXT_GRAY)],
                    ["Fault", ("FAULT", RED) if latest.get("m0_fault") else ("OK", GREEN)],
                ]
                draw_table(pdf, ["Parameter", "Value"], m0_rows, [60, 130])

                # Motor 1 from historical
                pdf.set_font("Helvetica", "B", 10)
                pdf.cell(0, 6, "Motor 1 (Right/Axis 2)", ln=True)
                m1_rows = [
                    ["Velocity", f"{latest.get('m1_velocity_steps', 0):,} steps/s ({latest.get('m1_velocity_in_s', 0):.3f} in/s)"],
                    ["Torque", f"{latest.get('m1_torque_percent', 0):.1f}% ({latest.get('m1_torque_nm', 0):.3f} Nm)"],
                    ["Position", f"{latest.get('m1_position', 0):,} steps"],
                    ["Enabled", ("Yes", GREEN) if latest.get("m1_enabled") else ("No", TEXT_GRAY)],
                    ["Moving", ("Yes", PRIME_CYAN) if latest.get("m1_moving") else ("No", TEXT_GRAY)],
                    ["Fault", ("FAULT", RED) if latest.get("m1_fault") else ("OK", GREEN)],
                ]
                draw_table(pdf, ["Parameter", "Value"], m1_rows, [60, 130])

                pdf.ln(3)
                pdf.set_font("Helvetica", "I", 8)
                pdf.set_text_color(*YELLOW)
                pdf.cell(0, 4, "Note: Live ROS data unavailable. Showing last recorded values from database.", ln=True)

            elif motor_status.get("clearlink_error"):
                pdf.set_text_color(*RED)
                error_text = str(motor_status.get('clearlink_error'))[:100]
                pdf.cell(0, 6, f"Error: {error_text}", ln=True)
            elif motor_status.get("clearlink_raw"):
                pdf.cell(0, 6, f"Status: {motor_status.get('clearlink_raw')}", ln=True)
            elif motor_status.get("roboclaw"):
                pdf.cell(0, 6, f"Status: {motor_status.get('roboclaw')}", ln=True)
            else:
                pdf.cell(0, 6, "Motor status data unavailable - ROS node may not be responding.", ln=True)

            pdf.ln(3)
            pdf.set_font("Helvetica", "I", 8)
            pdf.set_text_color(*TEXT_GRAY)
            if motor_driver == "clearlink":
                pdf.cell(0, 4, "Expected ROS topic: /clearlink/status", ln=True)
            elif motor_driver == "roboclaw":
                pdf.cell(0, 4, "Expected ROS topic: /roboclaw/status", ln=True)
            pdf.set_text_color(*TEXT_DARK)
        else:
            # Controller info
            controller = motor_status.get("controller", {})
            if controller:
                pdf.set_font("Helvetica", "B", 10)
                pdf.set_text_color(*TEXT_DARK)
                pdf.cell(0, 6, f"ClearLink Controller ({controller.get('ip', 'Unknown')}:{controller.get('port', 44818)})", ln=True)
                conn_status = ("Connected", GREEN) if controller.get("connected") else ("Disconnected", RED)
                ctrl_rows = [
                    ["Connection Status", conn_status],
                    ["Firmware Version", controller.get("firmware", "Unknown")],
                ]
                draw_table(pdf, ["Setting", "Value"], ctrl_rows, [80, 110])

            # Motor 0 details
            motor0 = motor_status.get("motor0", {})
            if motor0:
                pdf.set_font("Helvetica", "B", 11)
                pdf.set_text_color(*PRIME_CYAN)
                pdf.cell(0, 8, motor0.get("name", "Motor 0"), ln=True)
                pdf.set_text_color(*TEXT_DARK)

                fault_status = ("FAULT", RED) if motor0.get("fault") else ("OK", GREEN)
                enabled_status = ("Yes", GREEN) if motor0.get("enabled") else ("No", TEXT_GRAY)
                moving_status = ("Yes", PRIME_CYAN) if motor0.get("moving") else ("No", TEXT_GRAY)

                m0_rows = [
                    ["Velocity", f"{motor0.get('velocity_steps', 0):,} steps/s ({motor0.get('velocity_in_s', 0):.3f} in/s)"],
                    ["Torque", f"{motor0.get('torque_percent', 'N/A')}% ({motor0.get('torque_nm', 0):.3f} Nm)"],
                    ["Position", f"{motor0.get('position', 0):,} steps"],
                    ["Enabled", enabled_status],
                    ["Moving", moving_status],
                    ["Direction", motor0.get("direction", "Unknown")],
                    ["Fault Status", fault_status],
                ]
                draw_table(pdf, ["Parameter", "Value"], m0_rows, [60, 130])

            # Motor 1 details
            motor1 = motor_status.get("motor1", {})
            if motor1:
                pdf.set_font("Helvetica", "B", 11)
                pdf.set_text_color(*PRIME_CYAN)
                pdf.cell(0, 8, motor1.get("name", "Motor 1"), ln=True)
                pdf.set_text_color(*TEXT_DARK)

                fault_status = ("FAULT", RED) if motor1.get("fault") else ("OK", GREEN)
                enabled_status = ("Yes", GREEN) if motor1.get("enabled") else ("No", TEXT_GRAY)
                moving_status = ("Yes", PRIME_CYAN) if motor1.get("moving") else ("No", TEXT_GRAY)

                m1_rows = [
                    ["Velocity", f"{motor1.get('velocity_steps', 0):,} steps/s ({motor1.get('velocity_in_s', 0):.3f} in/s)"],
                    ["Torque", f"{motor1.get('torque_percent', 'N/A')}% ({motor1.get('torque_nm', 0):.3f} Nm)"],
                    ["Position", f"{motor1.get('position', 0):,} steps"],
                    ["Enabled", enabled_status],
                    ["Moving", moving_status],
                    ["Direction", motor1.get("direction", "Unknown")],
                    ["Fault Status", fault_status],
                ]
                draw_table(pdf, ["Parameter", "Value"], m1_rows, [60, 130])

            # Command history
            cmd_history = motor_status.get("command_history", [])
            if cmd_history:
                pdf.set_font("Helvetica", "B", 10)
                pdf.set_text_color(*PRIME_CYAN)
                pdf.cell(0, 8, "Recent Command History", ln=True)
                pdf.set_text_color(*TEXT_DARK)

                cmd_rows = []
                for cmd in cmd_history[:8]:
                    ts = cmd.get("timestamp", "")[:19]
                    command = cmd.get("command", "")
                    speed = f"{cmd.get('speed_in_s', 0):.2f} in/s"
                    result_text = cmd.get("result", "unknown")
                    result = (result_text, GREEN) if "success" in result_text.lower() else (result_text, RED)
                    cmd_rows.append([ts, command, speed, result])
                draw_table(pdf, ["Timestamp", "Command", "Speed", "Result"], cmd_rows, [50, 50, 40, 50])

            # Historical graphs
            historical_data = motor_status.get("historical_data", [])
            if historical_data and len(historical_data) > 10:
                pdf.add_page()
                draw_section_header(pdf, "Motor Performance Graphs (Past Hour)")

                # Generate graphs
                try:
                    # Import graph generator
                    import sys
                    sys.path.insert(0, "/home/craig/ros2_ws/src/pipe_crawler_control/scripts/network")
                    from report_graphs import generate_velocity_graph, generate_torque_graph, generate_temperature_graph, generate_position_graph

                    graph_files = []

                    # Velocity graph
                    vel_graph = generate_velocity_graph(historical_data)
                    if vel_graph and os.path.exists(vel_graph):
                        graph_files.append(vel_graph)
                        pdf.image(vel_graph, x=10, w=190)
                        pdf.ln(5)

                    # Torque graph
                    torque_graph = generate_torque_graph(historical_data)
                    if torque_graph and os.path.exists(torque_graph):
                        graph_files.append(torque_graph)
                        pdf.image(torque_graph, x=10, w=190)
                        pdf.ln(5)

                    # Temperature graph
                    temp_graph = generate_temperature_graph(historical_data)
                    if temp_graph and os.path.exists(temp_graph):
                        graph_files.append(temp_graph)
                        pdf.add_page()
                        draw_section_header(pdf, "System Temperature")
                        pdf.image(temp_graph, x=10, w=190)
                        pdf.ln(5)

                    # Position graph
                    pos_graph = generate_position_graph(historical_data)
                    if pos_graph and os.path.exists(pos_graph):
                        graph_files.append(pos_graph)
                        pdf.image(pos_graph, x=10, w=190)
                        pdf.ln(5)

                    # Cleanup temp files after PDF is generated
                    for gf in graph_files:
                        try:
                            os.remove(gf)
                        except:
                            pass

                except Exception as e:
                    pdf.set_font("Helvetica", "", 9)
                    pdf.set_text_color(*TEXT_GRAY)
                    pdf.cell(0, 6, f"Graph generation error: {str(e)[:80]}", ln=True)
            else:
                pdf.set_font("Helvetica", "", 9)
                pdf.set_text_color(*TEXT_GRAY)
                pdf.cell(0, 6, f"Historical data: {motor_status.get('historical_count', 0)} samples available", ln=True)
                if "historical_error" in motor_status:
                    pdf.cell(0, 5, f"Note: {motor_status['historical_error'][:60]}", ln=True)

            # Motor-related logs
            motor_logs = motor_status.get("motor_logs", "")
            if motor_logs:
                pdf.add_page()
                draw_section_header(pdf, "Motor-Related Log Entries")
                pdf.set_font("Courier", "", 7)
                pdf.set_text_color(*TEXT_GRAY)
                for line in motor_logs.split("\n")[:35]:
                    if len(line) > 105:
                        line = line[:105] + "..."
                    pdf.cell(0, 4, sanitize_text(line), ln=True)
                pdf.ln(4)

            # Raw status (if no structured data)
            if "clearlink_raw" in motor_status and not motor_status.get("motor0"):
                pdf.set_font("Helvetica", "B", 10)
                pdf.set_text_color(*TEXT_DARK)
                pdf.cell(0, 6, "Raw ClearLink Status:", ln=True)
                pdf.set_font("Courier", "", 7)
                pdf.set_text_color(*TEXT_GRAY)
                for line in str(motor_status["clearlink_raw"]).split("\n")[:25]:
                    if len(line) > 100:
                        line = line[:100] + "..."
                    pdf.cell(0, 4, sanitize_text(line), ln=True)
                pdf.ln(4)

            pdf.set_text_color(*TEXT_DARK)

        # ==================== SENSOR STATUS ====================
        sensors = info.get("sensors", {})
        if sensors:
            draw_section_header(pdf, "Sensor Status")

            imu_connected = sensors.get("imu_connected", False)
            imu_details = sensors.get("imu_details", {})

            sensor_rows = []
            # IMU main status
            imu_status = ("Connected", GREEN) if imu_connected else ("Not Connected", RED)
            sensor_rows.append(["IMU (WT901)", imu_status])

            # IMU details
            serial_ok = imu_details.get("serial_port", False)
            node_ok = imu_details.get("node_running", False)
            topic_ok = imu_details.get("topic_publisher", False)

            sensor_rows.append(["  Serial Port (/dev/ttyAMA0)", ("Present", GREEN) if serial_ok else ("Missing", RED)])
            sensor_rows.append(["  ROS2 Node (wt901_imu)", ("Running", GREEN) if node_ok else ("Stopped", RED)])
            sensor_rows.append(["  Topic Publisher", ("Active", GREEN) if topic_ok else ("Inactive", TEXT_GRAY)])

            draw_table(pdf, ["Sensor", "Status"], sensor_rows, [110, 80])

        # ==================== UI SETTINGS ====================
        ui_settings = info.get("ui_settings", {})
        active_crawler = info.get("active_crawler_id", "unknown")

        if ui_settings:
            pdf.add_page()
            draw_section_header(pdf, "Crawler UI Settings")

            pdf.set_font("Helvetica", "", 9)
            pdf.set_text_color(*TEXT_GRAY)
            pdf.cell(0, 5, f"Active crawler: {active_crawler}", ln=True)
            pdf.ln(3)

            # Show settings for each crawler
            for crawler_id, data in ui_settings.items():
                is_active = crawler_id == active_crawler
                settings = data.get("settings", {})
                mtime = data.get("mtime", "Unknown")
                path = data.get("path", "Unknown")

                # Section header with active indicator
                pdf.set_font("Helvetica", "B", 10)
                if is_active:
                    pdf.set_fill_color(232, 245, 233)  # Light green for active
                    pdf.set_text_color(*GREEN)
                    prefix = "[ACTIVE] "
                else:
                    pdf.set_fill_color(*ROW_WHITE)
                    pdf.set_text_color(*TEXT_DARK)
                    prefix = ""
                pdf.cell(0, 7, f"{prefix}{crawler_id}", ln=True, fill=is_active)

                pdf.set_font("Helvetica", "", 8)
                pdf.set_text_color(*TEXT_GRAY)
                pdf.cell(0, 4, f"File: {path}  |  Modified: {mtime}", ln=True)
                pdf.ln(2)

                # Settings table
                pdf.set_text_color(*TEXT_DARK)
                motor_params = settings.get("motorParams", {})

                settings_rows = [
                    ["Pipe Diameter", str(DEFAULT_UI_SETTINGS.get("pipeDiameter", "-")), str(settings.get("pipeDiameter", "-"))],
                    ["Speed", str(DEFAULT_UI_SETTINGS.get("speed", "-")), str(settings.get("speed", "-"))],
                    ["Ramp", str(DEFAULT_UI_SETTINGS.get("ramp", "-")), str(settings.get("ramp", "-"))],
                    ["Motor Driver", str(DEFAULT_UI_SETTINGS.get("motorDriver", "-")), str(settings.get("motorDriver", "-"))],
                    ["Encoder Resolution", str(DEFAULT_UI_SETTINGS.get("motorParams", {}).get("encoderRes", "-")), str(motor_params.get("encoderRes", "-"))],
                    ["Wheel Diameter", str(DEFAULT_UI_SETTINGS.get("motorParams", {}).get("wheelDia", "-")), str(motor_params.get("wheelDia", "-"))],
                    ["Gear Ratio", str(DEFAULT_UI_SETTINGS.get("motorParams", {}).get("gearRatio", "-")), str(motor_params.get("gearRatio", "-"))],
                    ["Max RPM", str(DEFAULT_UI_SETTINGS.get("motorParams", {}).get("maxRpm", "-")), str(motor_params.get("maxRpm", "-"))],
                ]

                draw_table(pdf, ["Setting", "Default", "Current"], settings_rows, [70, 50, 50])
                pdf.ln(3)

        # ==================== PROCESSES ====================
        pdf.add_page()
        draw_section_header(pdf, "Process Information")

        processes = info.get("processes", {})
        pdf.set_font("Helvetica", "", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, f"Total Process Count: {processes.get('total_count', 0)}", ln=True)
        pdf.ln(4)

        pdf.set_font("Helvetica", "B", 10)
        pdf.cell(0, 6, "Top CPU Consumers:", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        for line in str(processes.get("top_cpu", "None")).split("\n")[:10]:
            if len(line) > 105:
                line = line[:105] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(4)

        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "Top Memory Consumers:", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        for line in str(processes.get("top_memory", "None")).split("\n")[:10]:
            if len(line) > 105:
                line = line[:105] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(6)
        pdf.set_text_color(*TEXT_DARK)

        # ==================== FILESYSTEM ====================
        draw_section_header(pdf, "Filesystem")

        filesystem = info.get("filesystem", {})

        pdf.set_font("Helvetica", "B", 10)
        pdf.cell(0, 6, "Disk Usage:", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        for line in str(filesystem.get("disk_usage", "None")).split("\n")[:8]:
            if len(line) > 100:
                line = line[:100] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(4)

        # Config file modification times as a table
        config_times = filesystem.get("config_file_times", {})
        if config_times:
            pdf.set_font("Helvetica", "B", 10)
            pdf.set_text_color(*TEXT_DARK)
            pdf.cell(0, 6, "Config File Modification Times:", ln=True)
            config_rows = []
            for filepath, mtime in config_times.items():
                short_path = filepath.split("/")[-1]
                config_rows.append([short_path, mtime])
            draw_table(pdf, ["File", "Last Modified"], config_rows, [100, 90])
        pdf.ln(4)

        # ==================== LOGS ====================
        pdf.add_page()
        draw_section_header(pdf, "Service Logs")

        logs = info.get("logs", {})

        # Service logs
        for svc in ["pipe_crawler.service", "xpresscan-crawler-ui.service"]:
            log_text = logs.get(svc, "")
            if log_text:
                pdf.set_font("Helvetica", "B", 10)
                pdf.set_text_color(*TEXT_DARK)
                pdf.cell(0, 6, svc, ln=True)
                pdf.set_font("Courier", "", 7)
                pdf.set_text_color(*TEXT_GRAY)
                for line in log_text.split("\n")[:22]:
                    if len(line) > 105:
                        line = line[:105] + "..."
                    pdf.cell(0, 4, sanitize_text(line), ln=True)
                pdf.ln(4)
        pdf.set_text_color(*TEXT_DARK)

        # System errors
        pdf.add_page()
        draw_section_header(pdf, "Error Logs")

        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "System Errors (journalctl -p err):", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        error_log = logs.get("system_errors", "")
        for line in error_log.split("\n")[:35]:
            if len(line) > 105:
                line = line[:105] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(4)

        pdf.set_font("Helvetica", "B", 10)
        pdf.set_text_color(*TEXT_DARK)
        pdf.cell(0, 6, "Kernel Messages (dmesg errors/warnings):", ln=True)
        pdf.set_font("Courier", "", 7)
        pdf.set_text_color(*TEXT_GRAY)
        dmesg_log = logs.get("dmesg", "")
        for line in dmesg_log.split("\n")[:22]:
            if len(line) > 105:
                line = line[:105] + "..."
            pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.ln(4)

        # Failed units
        failed_units = logs.get("failed_units", "")
        if failed_units and "0 loaded" not in failed_units:
            pdf.set_font("Helvetica", "B", 10)
            pdf.set_text_color(*RED)
            pdf.cell(0, 6, "Failed Systemd Units:", ln=True)
            pdf.set_text_color(*TEXT_GRAY)
            pdf.set_font("Courier", "", 7)
            for line in failed_units.split("\n")[:10]:
                if len(line) > 100:
                    line = line[:100] + "..."
                pdf.cell(0, 4, sanitize_text(line), ln=True)
        pdf.set_text_color(*TEXT_DARK)

        # Generate PDF bytes
        pdf_bytes = bytes(pdf.output())

        # Create response
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        crawler_id = info.get("crawler", {}).get("id", "crawler")
        filename = f"crawler_support_report_{crawler_id}_{timestamp}.pdf"

        response = Response(
            pdf_bytes,
            mimetype="application/pdf",
            headers={"Content-Disposition": f"attachment; filename={filename}"}
        )
        return response

    except ImportError:
        return jsonify({"error": "fpdf2 library not installed"}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/api/operator/acquire', methods=['POST'])
def operator_acquire():
    data = request.get_json(silent=True) or {}
    token = data.get("token")
    client_ip = request.remote_addr
    with _operator_lock_mutex:
        # If lock is free or expired, grant it
        if _is_operator_lock_expired():
            new_token = token or secrets.token_hex(16)
            _operator_lock["session_token"] = new_token
            _operator_lock["client_ip"] = client_ip
            _operator_lock["acquired_at"] = time.time()
            _operator_lock["last_heartbeat"] = time.time()
            return jsonify({"success": True, "token": new_token})
        # If same token is re-acquiring (page refresh), allow it
        if token and token == _operator_lock["session_token"]:
            _operator_lock["client_ip"] = client_ip
            _operator_lock["last_heartbeat"] = time.time()
            return jsonify({"success": True, "token": token})
        return jsonify({"success": False, "message": "Another operator holds the lock"}), 409


@app.route('/api/operator/heartbeat', methods=['POST'])
def operator_heartbeat():
    data = request.get_json(silent=True) or {}
    token = data.get("token")
    if not token:
        return jsonify({"success": False, "message": "Token required"}), 400
    with _operator_lock_mutex:
        if _operator_lock["session_token"] == token:
            _operator_lock["last_heartbeat"] = time.time()
            return jsonify({"success": True})
        return jsonify({"success": False, "message": "Token does not match"}), 403


@app.route('/api/operator/release', methods=['POST'])
def operator_release():
    data = request.get_json(silent=True) or {}
    token = data.get("token")
    if not token:
        return jsonify({"success": False, "message": "Token required"}), 400
    with _operator_lock_mutex:
        if _operator_lock["session_token"] == token:
            _operator_lock["session_token"] = None
            _operator_lock["client_ip"] = None
            _operator_lock["acquired_at"] = None
            _operator_lock["last_heartbeat"] = None
            return jsonify({"success": True})
        return jsonify({"success": False, "message": "Token does not match"}), 403


@app.route('/api/operator/status', methods=['GET'])
def operator_status():
    with _operator_lock_mutex:
        if _is_operator_lock_expired():
            # Auto-clear expired lock
            _operator_lock["session_token"] = None
            _operator_lock["client_ip"] = None
            _operator_lock["acquired_at"] = None
            _operator_lock["last_heartbeat"] = None
            return jsonify({"locked": False, "holder_ip": None})
        return jsonify({"locked": True, "holder_ip": _operator_lock["client_ip"]})


# =====================================================================
# MQTT Cloud Dashboard Endpoints
# =====================================================================

MQTT_CONF_PATH = '/etc/crawler/mqtt.conf'

@app.route('/api/mqtt/status', methods=['GET'])
def mqtt_status():
    """Return MQTT config and service status."""
    import yaml
    config = {
        'enabled': False,
        'broker': '',
        'port': 443,
        'api_key_set': False,
        'hostname': '',
        'service_running': False,
    }
    try:
        with open(MQTT_CONF_PATH, 'r') as f:
            conf = yaml.safe_load(f) or {}
        config['enabled'] = bool(conf.get('enabled', False))
        config['broker'] = conf.get('broker', '')
        config['port'] = conf.get('port', 8883)
        config['api_key_set'] = bool(conf.get('api_key', ''))
        config['hostname'] = conf.get('hostname', '') or subprocess.check_output(['hostname'], text=True).strip()
    except FileNotFoundError:
        pass
    except Exception as e:
        config['error'] = str(e)

    # Check if mqtt-bridge service is running
    try:
        result = subprocess.run(
            ['systemctl', 'is-active', 'mqtt-bridge'],
            capture_output=True, text=True, timeout=5
        )
        config['service_running'] = result.stdout.strip() == 'active'
    except Exception:
        pass

    return jsonify(config)


@app.route('/api/mqtt/config', methods=['POST'])
def mqtt_config_save():
    """Save MQTT configuration. Requires master password."""
    import yaml
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import verify_master_password
    except ImportError:
        return jsonify({"success": False, "error": "Security module not available"}), 500

    data = request.get_json() or {}
    master_password = data.get('master_password', '')

    if not master_password:
        return jsonify({"success": False, "error": "Master password is required."})

    if not verify_master_password(master_password):
        return jsonify({"success": False, "error": "Invalid master password."})

    # Load existing config
    try:
        with open(MQTT_CONF_PATH, 'r') as f:
            conf = yaml.safe_load(f) or {}
    except FileNotFoundError:
        conf = {}

    # Update fields if provided
    if 'enabled' in data:
        conf['enabled'] = bool(data['enabled'])
    if 'broker' in data:
        conf['broker'] = str(data['broker']).strip()
    if 'port' in data:
        conf['port'] = int(data['port'])
    if 'api_key' in data:
        conf['api_key'] = str(data['api_key']).strip()

    # Write config
    try:
        os.makedirs(os.path.dirname(MQTT_CONF_PATH), exist_ok=True)
        with open(MQTT_CONF_PATH, 'w') as f:
            yaml.dump(conf, f, default_flow_style=False)
    except Exception as e:
        return jsonify({"success": False, "error": f"Failed to write config: {e}"})

    # Restart service if it exists
    try:
        subprocess.run(['systemctl', 'restart', 'mqtt-bridge'], timeout=10)
    except Exception:
        pass

    return jsonify({"success": True})


@app.route('/api/mqtt/test', methods=['POST'])
def mqtt_test():
    """Test MQTT connection. Requires master password."""
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import verify_master_password
    except ImportError:
        return jsonify({"success": False, "error": "Security module not available"}), 500

    data = request.get_json() or {}
    master_password = data.get('master_password', '')

    if not master_password:
        return jsonify({"success": False, "error": "Master password is required."})

    if not verify_master_password(master_password):
        return jsonify({"success": False, "error": "Invalid master password."})

    broker = data.get('broker', '').strip()
    port = int(data.get('port', 8883))
    api_key = data.get('api_key', '').strip()

    if not broker:
        return jsonify({"success": False, "error": "Broker URL is required."})

    import ssl
    import socket as sock

    try:
        hostname = sock.gethostname()
        import paho.mqtt.client as paho_mqtt

        result_data = {"connected": False, "rc": None}

        def on_connect(client, userdata, flags, rc, properties=None):
            # paho-mqtt v2 returns ReasonCode object, use .value for int
            rc_val = rc.value if hasattr(rc, 'value') else rc
            result_data["connected"] = (rc_val == 0)
            result_data["rc"] = rc_val
            client.disconnect()

        # Read transport settings from config
        import yaml as _yaml
        _transport = 'websockets'
        _ws_path = '/mqtt'
        try:
            with open(MQTT_CONF_PATH, 'r') as _f:
                _conf = _yaml.safe_load(_f) or {}
            _transport = _conf.get('transport', 'websockets')
            _ws_path = _conf.get('ws_path', '/mqtt')
        except Exception:
            pass

        client = paho_mqtt.Client(
            callback_api_version=paho_mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'crawler-{hostname}-test',
            protocol=paho_mqtt.MQTTv311,
            transport=_transport,
        )
        if _transport == 'websockets':
            client.ws_set_options(path=_ws_path)
        if api_key:
            client.username_pw_set(f'crawler_{hostname}', api_key)
        client.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        client.on_connect = on_connect

        client.connect(broker, port, keepalive=10)
        client.loop_start()

        # Wait up to 10 seconds for connection
        import time as _time
        deadline = _time.time() + 10
        while _time.time() < deadline and result_data["rc"] is None:
            _time.sleep(0.1)
        _time.sleep(0.5)

        client.loop_stop()
        client.disconnect()

        if result_data["connected"]:
            return jsonify({"success": True, "message": "Connection successful"})
        else:
            rc = result_data.get("rc", -1)
            reasons = {
                1: "Incorrect protocol version",
                2: "Invalid client identifier",
                3: "Server unavailable",
                4: "Bad username or password",
                5: "Not authorized",
            }
            reason = reasons.get(rc, f"Connection refused (code {rc})")
            return jsonify({"success": False, "error": reason})

    except ImportError:
        return jsonify({"success": False, "error": "paho-mqtt not installed"})
    except sock.timeout:
        return jsonify({"success": False, "error": f"Connection timed out to {broker}:{port}"})
    except sock.gaierror:
        return jsonify({"success": False, "error": f"Could not resolve hostname: {broker}"})
    except ssl.SSLError as e:
        return jsonify({"success": False, "error": f"TLS error: {e}"})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/mqtt/topics', methods=['GET'])
def mqtt_topics_get():
    """Return MQTT topics config from mqtt.conf."""
    import yaml
    defaults = {
        'heartbeat': {'enabled': True, 'interval': 5.0},
        'system': {'enabled': True, 'interval': 5.0, 'fields': {
            'cpu_temp': True, 'ram': True, 'storage': True, 'revision': True,
        }},
        'imu': {'enabled': True, 'interval': 2.0, 'fields': {
            'roll': True, 'pitch': True, 'cal_roll': True, 'cal_pitch': True,
        }},
        'motor': {'enabled': True, 'interval': 2.0, 'fields': {
            'connected': True, 'm1_speed': True, 'm2_speed': True,
            'm1_torque': True, 'm2_torque': True,
        }},
    }
    try:
        with open(MQTT_CONF_PATH, 'r') as f:
            conf = yaml.safe_load(f) or {}
        topics = conf.get('topics', {})
        # Merge defaults for any missing topics
        for name, defs in defaults.items():
            if name not in topics:
                topics[name] = defs
            else:
                for k, v in defs.items():
                    if k not in topics[name]:
                        topics[name][k] = v
                # Merge nested fields defaults
                if 'fields' in defs and 'fields' in topics[name]:
                    for fk, fv in defs['fields'].items():
                        if fk not in topics[name]['fields']:
                            topics[name]['fields'][fk] = fv
        # Settings fields defaults
        settings_defaults = {
            'pipeDiameter': True, 'jogSpeedInSec': True, 'rampPercent': True,
            'distanceMode': True, 'motorDriver': True,
            'imuInvertAngle': True, 'motorParams': True,
        }
        settings_fields = conf.get('settings_fields', {})
        for k, v in settings_defaults.items():
            if k not in settings_fields:
                settings_fields[k] = v
        return jsonify({"topics": topics, "settings_fields": settings_fields})
    except FileNotFoundError:
        settings_defaults = {
            'pipeDiameter': True, 'jogSpeedInSec': True, 'rampPercent': True,
            'distanceMode': True, 'motorDriver': True,
            'imuInvertAngle': True, 'motorParams': True,
        }
        return jsonify({"topics": defaults, "settings_fields": settings_defaults})
    except Exception as e:
        settings_defaults = {
            'pipeDiameter': True, 'jogSpeedInSec': True, 'rampPercent': True,
            'distanceMode': True, 'motorDriver': True,
            'imuInvertAngle': True, 'motorParams': True,
        }
        return jsonify({"topics": defaults, "settings_fields": settings_defaults, "error": str(e)})


@app.route('/api/mqtt/topics', methods=['POST'])
def mqtt_topics_save():
    """Save MQTT topics configuration. Requires master password."""
    import yaml
    try:
        import sys
        sys.path.insert(0, "/opt/crawler")
        from security_utils import verify_master_password
    except ImportError:
        return jsonify({"success": False, "error": "Security module not available"}), 500

    data = request.get_json() or {}
    master_password = data.get('master_password', '')

    if not master_password:
        return jsonify({"success": False, "error": "Master password is required."})

    if not verify_master_password(master_password):
        return jsonify({"success": False, "error": "Invalid master password."})

    # Password-only verification (no save)
    if data.get('verify_only'):
        return jsonify({"verified": True})

    topics = data.get('topics', {})
    if not topics:
        return jsonify({"success": False, "error": "No topics provided."})

    # Validate topic data
    valid_topics = {
        'heartbeat': [],
        'system': ['cpu_temp', 'ram', 'storage', 'revision'],
        'imu': ['roll', 'pitch', 'cal_roll', 'cal_pitch'],
        'motor': ['connected', 'm1_speed', 'm2_speed',
                  'm1_torque', 'm2_torque'],
    }
    sanitized = {}
    for name, valid_fields in valid_topics.items():
        if name in topics:
            t = topics[name]
            enabled = bool(t.get('enabled', True))
            interval = float(t.get('interval', 2.0))
            if interval < 0.5 or interval > 300:
                return jsonify({"success": False, "error": f"Interval for {name} must be between 0.5 and 300 seconds."})
            entry = {'enabled': enabled, 'interval': interval}
            # Handle per-field toggles
            if valid_fields and 'fields' in t:
                fields = {}
                for f in valid_fields:
                    fields[f] = bool(t['fields'].get(f, True))
                entry['fields'] = fields
            # Handle ICCID for heartbeat
            if name == 'heartbeat':
                entry['iccid_enabled'] = bool(t.get('iccid_enabled', False))
                entry['iccid'] = str(t.get('iccid', '')).strip()
            sanitized[name] = entry

    # Load existing config
    try:
        with open(MQTT_CONF_PATH, 'r') as f:
            conf = yaml.safe_load(f) or {}
    except FileNotFoundError:
        conf = {}

    conf['topics'] = sanitized

    # Validate and save settings_fields
    valid_settings_keys = [
        'pipeDiameter', 'jogSpeedInSec', 'rampPercent', 'distanceMode',
        'motorDriver', 'imuInvertAngle',
        'motorParams',
    ]
    sf_input = data.get('settings_fields', {})
    if sf_input:
        settings_fields = {}
        for k in valid_settings_keys:
            settings_fields[k] = bool(sf_input.get(k, True))
        conf['settings_fields'] = settings_fields

    # Write config
    try:
        os.makedirs(os.path.dirname(MQTT_CONF_PATH), exist_ok=True)
        with open(MQTT_CONF_PATH, 'w') as f:
            yaml.dump(conf, f, default_flow_style=False)
    except Exception as e:
        return jsonify({"success": False, "error": f"Failed to write config: {e}"})

    # Restart mqtt-bridge service
    try:
        subprocess.run(['systemctl', 'restart', 'mqtt-bridge'], timeout=10)
    except Exception:
        pass

    return jsonify({"success": True})


def main():
    """Run the crawler UI server."""
    global boot_state

    print("Starting Crawler UI on port 80")
    print("Access at http://autobug.local")

    # Reset boot state
    with boot_state_lock:
        boot_state["in_progress"] = True
        boot_state["current_check"] = "Initializing"
        boot_state["checks_complete"] = 0
        boot_state["start_time"] = time.time()
        boot_state["result"] = None
        boot_state["redirect"] = None

    # Start security checks in background thread
    security_thread = threading.Thread(target=run_boot_security_checks, daemon=True)
    security_thread.start()
    print("Boot security checks started in background")

    app.run(host="0.0.0.0", port=80, debug=False)


if __name__ == "__main__":
    main()
