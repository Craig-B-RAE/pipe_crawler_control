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
from flask import Flask, render_template_string, jsonify, send_from_directory, request

app = Flask(__name__)

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
        "name": "EdgeFlex",
        "yaml": "edgeflex.yaml",
        "html": "edgeflex.html",
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


@app.route("/api/wifi/scan")
def api_wifi_scan():
    """Scan for WiFi networks."""
    networks = scan_wifi_networks()
    return jsonify({"networks": networks})


@app.route("/api/wifi/connect", methods=["POST"])
def api_wifi_connect():
    """Connect to a WiFi network."""
    data = request.get_json()
    ssid = data.get("ssid", "")
    password = data.get("password", "")

    if not ssid:
        return jsonify({"success": False, "message": "SSID required"})

    try:
        # Check if connection exists
        result = subprocess.run(["nmcli", "connection", "show", ssid], capture_output=True, text=True)

        if result.returncode == 0:
            result = subprocess.run(["nmcli", "connection", "up", ssid], capture_output=True, text=True, timeout=30)
        else:
            cmd = ["nmcli", "device", "wifi", "connect", ssid]
            if password:
                cmd.extend(["password", password])
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

        if result.returncode == 0:
            return jsonify({"success": True, "message": "Connected successfully"})
        else:
            return jsonify({"success": False, "message": result.stderr.strip() or "Connection failed"})
    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "message": "Connection timed out"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


def main():
    """Run the crawler UI server."""
    print("Starting Crawler UI on port 80")
    print("Access at http://autobug.local")
    app.run(host="0.0.0.0", port=80, debug=False)


if __name__ == "__main__":
    main()
