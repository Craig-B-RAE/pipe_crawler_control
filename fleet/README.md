# Fleet VPN Management

Connects crawlers to the fleet management server via a WireGuard VPN tunneled over WebSockets (wstunnel). Once connected, crawlers send periodic heartbeats with system status and are accessible via a remote web terminal (ttyd).

## Installation

```bash
sudo bash fleet/install.sh
```

This installs WireGuard, wstunnel, ttyd, fleet scripts, and systemd services. It does **not** start services or reboot — that happens after configuration.

## Configuration

VPN and MQTT are **optional**. Both are configured through the crawler's web UI at `config.html`:

- **VPN Token** — paste the registration token provided by the fleet server
- **MQTT API Key** — paste the MQTT broker credentials
- **Hostname** — set via the crawler config page before first VPN registration

Crawlers without a VPN token configured will boot normally with no errors — the VPN registration script exits cleanly if no token is present, and the heartbeat service waits quietly for the wg0 interface to appear.

## Boot Sequence (when VPN is configured)

1. **wstunnel-client** — establishes a WebSocket tunnel to `edge-primeinspections.com:443`, forwarding UDP port 51820 locally
2. **crawler-vpn-register** — generates WireGuard keys, registers with the fleet API using the saved token, writes `wg0.conf` (runs once, skips if already registered)
3. **wg-quick@wg0** — brings up the WireGuard VPN interface using the generated config
4. **ttyd** — starts a web terminal on port 7681 bound to the wg0 interface (VPN-accessible only)
5. **crawler-heartbeat** — sends system status (hostname, CPU temp, uptime, ROS status, version) to the fleet API every 60 seconds

## Checking Status After Reboot

```bash
# VPN tunnel
sudo wg show wg0

# Service status
systemctl status wstunnel-client
systemctl status crawler-vpn-register
systemctl status crawler-heartbeat
systemctl status ttyd

# Heartbeat logs
journalctl -u crawler-heartbeat -f

# VPN registration logs
journalctl -u crawler-vpn-register

# Verify fleet connectivity
ping 10.0.0.1
```

## Files

| File | Description |
|------|-------------|
| `vpn_register.py` | Auto-registration script (runs on boot) |
| `crawler_heartbeat.py` | Periodic fleet heartbeat sender |
| `install.sh` | One-time setup script |
| `services/*.service` | systemd unit files |
