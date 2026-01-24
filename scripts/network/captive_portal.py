#!/usr/bin/env python3
"""
XPressCan Configuration Portal

Always-running web server for crawler and hotspot configuration.
Accessible at 10.42.0.1 - allows changing crawler identity and hotspot settings.
"""

import os
import subprocess
from flask import Flask, render_template_string, request

app = Flask(__name__)

CONFIG_DIR = "/etc/xpresscan"
CONFIG_FILE = f"{CONFIG_DIR}/network.conf"

DEFAULT_SSID = "Crawler"
DEFAULT_PASSWORD = "Crawler1"

CRAWLERS = {
    "autobug-1": {
        "hostname": "autobug-1",
        "ip": "10.42.0.11/24",
        "description": "Crawler 1"
    },
    "autobug-2": {
        "hostname": "autobug-2",
        "ip": "10.42.0.12/24",
        "description": "Crawler 2"
    }
}


def load_current_config():
    """Load current configuration from file."""
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

HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>XPressCan Configuration</title>
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
            border-radius: 20px;
            padding: 40px;
            max-width: 420px;
            width: 100%;
            box-shadow: 0 4px 30px rgba(0, 0, 0, 0.1);
        }
        h1 {
            color: #1d1d1f;
            margin-bottom: 10px;
            font-size: 24px;
            text-align: center;
        }
        .subtitle {
            color: #86868b;
            margin-bottom: 30px;
            font-size: 14px;
            text-align: center;
        }
        .current-config {
            background: #e8f5e9;
            border-radius: 10px;
            padding: 12px 15px;
            margin-bottom: 25px;
            font-size: 13px;
        }
        .current-config .label {
            color: #2e7d32;
            font-weight: 600;
        }
        .current-config .value {
            color: #1b5e20;
        }
        .section {
            margin-bottom: 25px;
        }
        .section-title {
            font-size: 14px;
            font-weight: 600;
            color: #1d1d1f;
            margin-bottom: 12px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .crawler-options {
            display: flex;
            gap: 10px;
        }
        .crawler-option {
            flex: 1;
            padding: 15px;
            border: 2px solid #e0e0e0;
            border-radius: 12px;
            cursor: pointer;
            text-align: center;
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
            border-color: #4caf50;
            background: #e8f5e9;
        }
        .crawler-option input {
            display: none;
        }
        .crawler-name {
            font-weight: 600;
            color: #1d1d1f;
            margin-bottom: 4px;
        }
        .crawler-ip {
            font-size: 12px;
            color: #86868b;
        }
        .form-group {
            margin-bottom: 15px;
        }
        .form-label {
            display: block;
            font-size: 13px;
            color: #86868b;
            margin-bottom: 6px;
        }
        .form-input {
            width: 100%;
            padding: 12px 15px;
            font-size: 16px;
            border: 1px solid #e0e0e0;
            border-radius: 10px;
            background: #f5f5f7;
            color: #1d1d1f;
            transition: all 0.2s;
        }
        .form-input:focus {
            outline: none;
            border-color: #5856d6;
            box-shadow: 0 0 0 3px rgba(88, 86, 214, 0.2);
        }
        .form-hint {
            font-size: 11px;
            color: #86868b;
            margin-top: 4px;
        }
        .submit-btn {
            width: 100%;
            padding: 16px;
            font-size: 16px;
            font-weight: 600;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            color: #ffffff;
            background: #5856d6;
            transition: all 0.2s;
            margin-top: 10px;
        }
        .submit-btn:hover {
            background: #4240a8;
        }
        .submit-btn:disabled {
            background: #c0c0c0;
            cursor: not-allowed;
        }
        .error {
            background: #ffebee;
            color: #c62828;
            padding: 12px;
            border-radius: 8px;
            margin-bottom: 20px;
            font-size: 14px;
        }
        .success {
            background: #e8f5e9;
            color: #2e7d32;
            padding: 12px;
            border-radius: 8px;
            margin-bottom: 20px;
            font-size: 14px;
        }
        .info {
            margin-top: 25px;
            padding: 15px;
            background: #f5f5f7;
            border-radius: 10px;
            font-size: 12px;
            color: #86868b;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>XPressCan Configuration</h1>
        <p class="subtitle">Network and Hotspot Settings</p>

        {% if current_crawler %}
        <div class="current-config">
            <span class="label">Current:</span>
            <span class="value">{{ current_crawler }} ({{ current_ssid }})</span>
        </div>
        {% endif %}

        {% if error %}
        <div class="error">{{ error }}</div>
        {% endif %}

        {% if success %}
        <div class="success">{{ success }}</div>
        {% endif %}

        <form action="/configure" method="POST" id="configForm">
            <div class="section">
                <div class="section-title">Select Crawler</div>
                <div class="crawler-options">
                    {% for id, crawler in crawlers.items() %}
                    <label class="crawler-option {% if id == current_crawler %}current{% endif %}" onclick="selectCrawler('{{ id }}')">
                        <input type="radio" name="crawler_id" value="{{ id }}" {% if id == current_crawler or (not current_crawler and loop.first) %}checked{% endif %}>
                        <div class="crawler-name">{{ crawler.description }}</div>
                        <div class="crawler-ip">{{ crawler.ip.split('/')[0] }}</div>
                    </label>
                    {% endfor %}
                </div>
            </div>

            <div class="section">
                <div class="section-title">WiFi Hotspot Settings</div>
                <div class="form-group">
                    <label class="form-label">Hotspot Name (SSID)</label>
                    <input type="text" name="ssid" class="form-input" value="{{ current_ssid }}" required minlength="1" maxlength="32">
                </div>
                <div class="form-group">
                    <label class="form-label">Password</label>
                    <input type="text" name="password" class="form-input" value="{{ current_password }}" required minlength="8" maxlength="63">
                    <div class="form-hint">Minimum 8 characters (WPA2 requirement)</div>
                </div>
            </div>

            <button type="submit" class="submit-btn">Apply Changes</button>
        </form>

        <div class="info">
            Changes will be applied immediately. The hotspot will restart with new settings.
        </div>
    </div>

    <script>
        function selectCrawler(id) {
            document.querySelectorAll('.crawler-option').forEach(el => {
                el.classList.remove('selected');
            });
            event.currentTarget.classList.add('selected');
        }
        // Select current or first option on load
        var current = document.querySelector('.crawler-option.current');
        if (current) {
            current.classList.add('selected');
        } else {
            document.querySelector('.crawler-option').classList.add('selected');
        }
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
    <title>Configuration Applied</title>
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
            border-radius: 20px;
            padding: 40px;
            max-width: 400px;
            width: 100%;
            box-shadow: 0 4px 30px rgba(0, 0, 0, 0.1);
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
        h1 { color: #1d1d1f; margin-bottom: 20px; font-size: 22px; }
        .config-item {
            display: flex;
            justify-content: space-between;
            padding: 10px 0;
            border-bottom: 1px solid #f0f0f0;
            font-size: 14px;
        }
        .config-item:last-child { border-bottom: none; }
        .config-label { color: #86868b; }
        .config-value { color: #1d1d1f; font-weight: 600; }
        .info { color: #86868b; font-size: 14px; margin-top: 25px; }
        .back-link {
            display: inline-block;
            margin-top: 20px;
            color: #5856d6;
            text-decoration: none;
            font-weight: 600;
        }
        .back-link:hover { text-decoration: underline; }
        .warning {
            background: #fff3e0;
            color: #e65100;
            padding: 12px;
            border-radius: 8px;
            margin-top: 20px;
            font-size: 13px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="checkmark">
            <svg viewBox="0 0 24 24"><polyline points="20 6 9 17 4 12"/></svg>
        </div>
        <h1>Configuration Applied</h1>

        <div class="config-item">
            <span class="config-label">Hostname</span>
            <span class="config-value">{{ hostname }}</span>
        </div>
        <div class="config-item">
            <span class="config-label">IP Address</span>
            <span class="config-value">{{ ip }}</span>
        </div>
        <div class="config-item">
            <span class="config-label">Hotspot Name</span>
            <span class="config-value">{{ ssid }}</span>
        </div>
        <div class="config-item">
            <span class="config-label">Password</span>
            <span class="config-value">{{ password }}</span>
        </div>

        <p class="info">Hotspot has been restarted with new settings.</p>

        {% if ssid_changed %}
        <div class="warning">
            WiFi name changed! You may need to reconnect to "{{ ssid }}"
        </div>
        {% endif %}

        <a href="/" class="back-link">Back to Configuration</a>
    </div>
</body>
</html>
"""


def configure_system(crawler_id, ssid, password):
    """Configure the system with the selected settings."""
    if crawler_id not in CRAWLERS:
        return False, "Invalid crawler ID", False

    if len(password) < 8:
        return False, "Password must be at least 8 characters (WPA2 requirement)", False

    if len(ssid) < 1 or len(ssid) > 32:
        return False, "SSID must be 1-32 characters", False

    # Check if SSID is changing
    old_config = load_current_config()
    ssid_changed = old_config["ssid"] != ssid

    crawler = CRAWLERS[crawler_id]
    hostname = crawler["hostname"]
    ip = crawler["ip"]

    try:
        # Create config directory
        os.makedirs(CONFIG_DIR, exist_ok=True)

        # Write network config
        with open(CONFIG_FILE, "w") as f:
            f.write(f"# XPressCan Network Configuration\n")
            f.write(f"CRAWLER_ID={crawler_id}\n")
            f.write(f"HOSTNAME={hostname}\n")
            f.write(f"STATIC_IP={ip}\n")
            f.write(f"HOTSPOT_SSID={ssid}\n")
            f.write(f"HOTSPOT_PASSWORD={password}\n")

        # Set hostname
        subprocess.run(["hostnamectl", "set-hostname", hostname], check=True)

        # Update /etc/hosts
        with open("/etc/hosts", "r") as f:
            hosts = f.read()

        # Replace or add hostname entry
        new_hosts = []
        found_127_0_1_1 = False
        for line in hosts.split("\n"):
            if "127.0.1.1" in line:
                new_hosts.append(f"127.0.1.1\t{hostname}")
                found_127_0_1_1 = True
            else:
                new_hosts.append(line)

        if not found_127_0_1_1:
            new_hosts.insert(1, f"127.0.1.1\t{hostname}")

        with open("/etc/hosts", "w") as f:
            f.write("\n".join(new_hosts))

        # Restart hotspot with new settings
        restart_hotspot()

        return True, None, ssid_changed

    except Exception as e:
        return False, str(e), False


def restart_hotspot():
    """Restart the hotspot service with new configuration."""
    try:
        # Restart the hotspot service
        subprocess.run(
            ["systemctl", "restart", "xpresscan-hotspot"],
            check=False,
            timeout=30
        )
    except Exception:
        # If systemd service fails, try direct script
        try:
            subprocess.run(
                ["/opt/xpresscan/setup_hotspot.sh"],
                check=False,
                timeout=30
            )
        except Exception:
            pass


@app.route("/")
def index():
    """Serve the configuration page."""
    config = load_current_config()
    return render_template_string(
        HTML_TEMPLATE,
        crawlers=CRAWLERS,
        current_crawler=config["crawler_id"],
        current_ssid=config["ssid"],
        current_password=config["password"],
        error=None,
        success=None
    )


@app.route("/configure", methods=["POST"])
def configure():
    """Handle configuration submission."""
    crawler_id = request.form.get("crawler_id")
    ssid = request.form.get("ssid", DEFAULT_SSID).strip()
    password = request.form.get("password", DEFAULT_PASSWORD).strip()

    config = load_current_config()

    if not crawler_id:
        return render_template_string(
            HTML_TEMPLATE,
            crawlers=CRAWLERS,
            current_crawler=config["crawler_id"],
            current_ssid=ssid,
            current_password=password,
            error="Please select a crawler",
            success=None
        )

    success, error, ssid_changed = configure_system(crawler_id, ssid, password)

    if not success:
        return render_template_string(
            HTML_TEMPLATE,
            crawlers=CRAWLERS,
            current_crawler=config["crawler_id"],
            current_ssid=ssid,
            current_password=password,
            error=error,
            success=None
        )

    crawler = CRAWLERS[crawler_id]

    return render_template_string(
        SUCCESS_TEMPLATE,
        hostname=crawler["hostname"],
        ip=crawler["ip"].split("/")[0],
        ssid=ssid,
        password=password,
        ssid_changed=ssid_changed
    )


@app.route("/status")
def status():
    """Check configuration status (JSON API)."""
    config = load_current_config()
    if config["crawler_id"]:
        return {
            "configured": True,
            "crawler_id": config["crawler_id"],
            "ssid": config["ssid"]
        }, 200
    return {"configured": False}, 200


def main():
    """Run the configuration portal server."""
    print("Starting XPressCan Configuration Portal on http://10.42.0.1:8080")
    app.run(host="0.0.0.0", port=8080, debug=False)


if __name__ == "__main__":
    main()
