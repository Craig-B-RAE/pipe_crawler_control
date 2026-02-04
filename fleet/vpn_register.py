#!/usr/bin/env python3
"""
Crawler VPN Auto-Registration

Generates WireGuard keys, registers with the fleet API,
writes wg0.conf, and starts the VPN tunnel.
Runs once on boot via systemd; exits immediately if already registered.
"""

import json
import os
import socket
import ssl
import subprocess
import sys
import time
import urllib.request
import urllib.error

REGISTRATION_URL = "https://edge-primeinspections.com/fleet-api/register"
TOKEN_PATH = "/etc/crawler/vpn_token"
WG_CONF_PATH = "/etc/wireguard/wg0.conf"
PRIVATE_KEY_PATH = "/etc/wireguard/private.key"
MAX_RETRIES = 20
RETRY_INTERVAL = 30


def log(msg):
    print(f"[vpn-register] {msg}", flush=True)


def wg_is_up():
    """Check if wg0 interface is active."""
    try:
        result = subprocess.run(
            ["wg", "show", "wg0"],
            capture_output=True, timeout=5
        )
        return result.returncode == 0
    except Exception:
        return False


def generate_keys():
    """Generate WireGuard private and public key pair."""
    private = subprocess.run(
        ["wg", "genkey"], capture_output=True, text=True, check=True
    ).stdout.strip()
    public = subprocess.run(
        ["wg", "pubkey"], input=private, capture_output=True, text=True, check=True
    ).stdout.strip()
    return private, public


def read_token():
    """Read registration token from file."""
    with open(TOKEN_PATH, "r") as f:
        return f.read().strip()


def register(hostname, public_key, token):
    """POST registration to fleet API. Returns response dict."""
    payload = json.dumps({
        "hostname": hostname,
        "public_key": public_key,
        "registration_token": token,
    }).encode("utf-8")

    ctx = ssl.create_default_context()
    req = urllib.request.Request(
        REGISTRATION_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    resp = urllib.request.urlopen(req, context=ctx, timeout=30)
    return json.loads(resp.read().decode("utf-8"))


def write_wg_conf(private_key, data):
    """Write /etc/wireguard/wg0.conf from API response."""
    os.makedirs("/etc/wireguard", exist_ok=True)
    conf = (
        f"[Interface]\n"
        f"PrivateKey = {private_key}\n"
        f"Address = {data['vpn_ip']}/24\n"
        f"\n"
        f"[Peer]\n"
        f"PublicKey = {data['server_public_key']}\n"
        f"PresharedKey = {data['preshared_key']}\n"
        f"Endpoint = {data['server_endpoint']}\n"
        f"AllowedIPs = {data['subnet']}\n"
        f"PersistentKeepalive = 25\n"
    )
    with open(WG_CONF_PATH, "w") as f:
        f.write(conf)
    os.chmod(WG_CONF_PATH, 0o600)

    with open(PRIVATE_KEY_PATH, "w") as f:
        f.write(private_key + "\n")
    os.chmod(PRIVATE_KEY_PATH, 0o600)

    log(f"Wrote {WG_CONF_PATH} with VPN IP {data['vpn_ip']}")


def start_wireguard():
    """Enable and start wg-quick@wg0."""
    subprocess.run(["systemctl", "enable", "wg-quick@wg0"], check=True)
    subprocess.run(["systemctl", "start", "wg-quick@wg0"], check=True)
    log("WireGuard started")


def verify_vpn():
    """Ping 10.0.0.1 to verify VPN connectivity."""
    try:
        result = subprocess.run(
            ["ping", "-c", "3", "-W", "3", "10.0.0.1"],
            capture_output=True, timeout=15
        )
        if result.returncode == 0:
            log("VPN verified: 10.0.0.1 reachable")
            return True
        else:
            log("VPN ping failed but tunnel may still be establishing")
            return False
    except Exception as e:
        log(f"VPN verify error: {e}")
        return False


def main():
    hostname = socket.gethostname()
    log(f"Hostname: {hostname}")

    # Already registered?
    if os.path.exists(WG_CONF_PATH):
        if wg_is_up():
            log("WireGuard already configured and running. Nothing to do.")
            return 0
        else:
            # wg0.conf exists but WireGuard not running.
            # Don't call 'systemctl start wg-quick@wg0' here — this service has
            # Before=wg-quick@wg0.service, so systemd will start wg0 automatically
            # after we exit. Calling systemctl start from here causes a deadlock.
            log("wg0.conf exists. Ensuring wg0 is enabled; systemd will start it after this service exits.")
            subprocess.run(["systemctl", "enable", "wg-quick@wg0"], capture_output=True)
            return 0

    # Read token
    try:
        token = read_token()
    except FileNotFoundError:
        log(f"ERROR: Token file not found at {TOKEN_PATH}")
        return 1
    except Exception as e:
        log(f"ERROR: Could not read token: {e}")
        return 1

    # Generate keys
    log("Generating WireGuard keys...")
    private_key, public_key = generate_keys()
    log(f"Public key: {public_key}")

    # Register with retries
    for attempt in range(1, MAX_RETRIES + 1):
        log(f"Registration attempt {attempt}/{MAX_RETRIES}...")
        try:
            data = register(hostname, public_key, token)
            log(f"Registered successfully: VPN IP {data['vpn_ip']}")
            break
        except urllib.error.HTTPError as e:
            body = ""
            try:
                body = e.read().decode("utf-8", errors="replace")
            except Exception:
                pass
            if e.code == 409:
                log(f"ERROR: Hostname conflict (409): {body}")
                return 1
            elif e.code in (401, 403):
                log(f"ERROR: Invalid token ({e.code}): {body}")
                return 1
            else:
                log(f"HTTP error {e.code}: {body}")
        except Exception as e:
            log(f"Connection error: {e}")

        if attempt < MAX_RETRIES:
            log(f"Retrying in {RETRY_INTERVAL}s...")
            time.sleep(RETRY_INTERVAL)
    else:
        log(f"ERROR: Failed to register after {MAX_RETRIES} attempts")
        return 1

    # Write config and start
    write_wg_conf(private_key, data)
    start_wireguard()
    verify_vpn()

    return 0


if __name__ == "__main__":
    sys.exit(main())
