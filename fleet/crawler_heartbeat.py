#!/usr/bin/env python3
"""
Crawler Fleet Heartbeat

Sends periodic status updates to the fleet API over the VPN.
"""

import json
import os
import socket
import subprocess
import sys
import time
import traceback
import urllib.request
import urllib.error

FLEET_API_URL = os.environ.get("FLEET_API_URL", "http://10.0.0.1:9100")
HEARTBEAT_INTERVAL = int(os.environ.get("HEARTBEAT_INTERVAL", "30"))
HEARTBEAT_ENDPOINT = f"{FLEET_API_URL}/api/heartbeat"

CRAWLER_TYPE_PATHS = [
    "/etc/xpresscan/crawler_type.yaml",
    "/etc/crawler/crawler_type.yaml",
    "/opt/crawler/crawler_type.yaml",
]

GIT_REPO = os.path.expanduser("~/ros2_ws/src/pipe_crawler_control")


def log(msg):
    print(f"[heartbeat] {msg}", flush=True)


def get_vpn_ip():
    try:
        out = subprocess.run(
            ["ip", "-4", "addr", "show", "wg0"],
            capture_output=True, text=True, timeout=5
        ).stdout
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("inet "):
                return line.split()[1].split("/")[0]
    except Exception as e:
        log(f"  get_vpn_ip error: {e}")
    return ""


def get_crawler_type():
    for path in CRAWLER_TYPE_PATHS:
        try:
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("crawler_type:"):
                        return line.split(":", 1)[1].strip()
        except Exception:
            continue
    return "unknown"


def get_wifi_ssid():
    try:
        return subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True, text=True, timeout=5
        ).stdout.strip()
    except Exception as e:
        log(f"  get_wifi_ssid error: {e}")
        return ""


def get_cpu_temp():
    # Try thermal_zone first (more reliable on CM5/Ubuntu)
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            raw = int(f.read().strip())
            return f"{raw / 1000:.1f}\u00b0C"
    except Exception as e:
        log(f"  get_cpu_temp thermal_zone error: {e}")
    # Fallback to vcgencmd
    try:
        result = subprocess.run(
            ["vcgencmd", "measure_temp"],
            capture_output=True, text=True, timeout=5
        )
        out = result.stdout.strip()
        if "=" in out:
            temp = out.split("=")[1].rstrip(" \t\n'C°c")
            return f"{temp}\u00b0C"
        else:
            log(f"  vcgencmd unexpected output: {out!r}")
    except Exception as e:
        log(f"  get_cpu_temp vcgencmd error: {e}")
    return ""


def get_uptime():
    try:
        with open("/proc/uptime", "r") as f:
            secs = float(f.read().split()[0])
        days = int(secs // 86400)
        hours = int((secs % 86400) // 3600)
        mins = int((secs % 3600) // 60)
        return f"{days}d {hours}h {mins}m"
    except Exception as e:
        log(f"  get_uptime error: {e}")
        return ""


def get_ros_status():
    try:
        out = subprocess.run(
            ["pgrep", "-f", "ros2"],
            capture_output=True, text=True, timeout=5
        ).stdout.strip()
        if out:
            pids = [p for p in out.splitlines() if p.strip()]
            return f"running ({len(pids)} nodes)"
        return "stopped"
    except Exception as e:
        log(f"  get_ros_status error: {e}")
        return ""


def get_version():
    try:
        branch = subprocess.run(
            ["git", "-C", GIT_REPO, "rev-parse", "--abbrev-ref", "HEAD"],
            capture_output=True, text=True, timeout=5
        ).stdout.strip()
        sha = subprocess.run(
            ["git", "-C", GIT_REPO, "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=5
        ).stdout.strip()
        if branch and sha:
            return f"{branch}@{sha}"
    except Exception as e:
        log(f"  get_version error: {e}")
    return ""


def send_heartbeat():
    log("Collecting data...")
    collectors = [
        ("hostname", lambda: socket.gethostname()),
        ("vpn_ip", get_vpn_ip),
        ("crawler_type", get_crawler_type),
        ("wifi_ssid", get_wifi_ssid),
        ("cpu_temp", get_cpu_temp),
        ("uptime", get_uptime),
        ("ros_status", get_ros_status),
        ("version", get_version),
    ]
    data = {}
    for key, func in collectors:
        try:
            val = func()
            data[key] = val
            log(f"  {key} = {val!r}")
        except Exception as e:
            log(f"  {key} FAILED: {e}")
            data[key] = ""

    payload = json.dumps(data).encode("utf-8")
    log(f"Sending heartbeat to {HEARTBEAT_ENDPOINT} ({len(payload)} bytes)...")
    req = urllib.request.Request(
        HEARTBEAT_ENDPOINT,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    resp = urllib.request.urlopen(req, timeout=10)
    status = resp.getcode()
    log(f"Heartbeat sent OK (HTTP {status})")
    return data


def wg0_exists():
    """Check if the wg0 network interface exists."""
    return os.path.isdir("/sys/class/net/wg0")


def main():
    log(f"Starting heartbeat to {HEARTBEAT_ENDPOINT} every {HEARTBEAT_INTERVAL}s")
    log(f"Python {sys.version}")
    log(f"PID {os.getpid()}, UID {os.getuid()}")
    consecutive_failures = 0
    was_failing = False

    while True:
        if not wg0_exists():
            log("No wg0 interface — VPN not active, waiting...")
            time.sleep(HEARTBEAT_INTERVAL)
            continue

        try:
            data = send_heartbeat()
            if was_failing:
                log(f"Connection restored after {consecutive_failures} failures")
                was_failing = False
            consecutive_failures = 0
        except Exception as e:
            consecutive_failures += 1
            was_failing = True
            if consecutive_failures <= 3 or consecutive_failures % 10 == 0:
                log(f"Heartbeat failed ({consecutive_failures}): {e}")
                log(traceback.format_exc())

        time.sleep(HEARTBEAT_INTERVAL)


if __name__ == "__main__":
    main()
