# Project Memory — pipe_crawler_control

## Repository
- **Local path:** ~/ros2_ws/src/pipe_crawler_control/
- **Remote:** https://github.com/Craig-B-RAE/pipe_crawler_control.git
- **Primary branch:** development

## Deployment
- **Web source:** ~/ros2_ws/src/pipe_crawler_control/web/
- **Web deploy target:** /opt/xpresscan/html/
- **Backend:** /opt/xpresscan/crawler_ui.py (Flask, port 80, runs as root via systemd)
- **Backend source:** scripts/network/crawler_ui.py
- **Services:** `xpresscan-crawler-ui.service` (Flask UI), `pipe_crawler.service` (ROS2 launch)
- **Active system config:** ~/active_system (currently `edgeflex_clearlink`)
- After modifying HTML files, copy to /opt/xpresscan/html/
- After modifying crawler_ui.py, copy to /opt/xpresscan/ and restart xpresscan-crawler-ui service
- After modifying ROS2 nodes, rebuild with `colcon build --packages-select pipe_crawler_control --symlink-install` and restart pipe_crawler service
- Git runs as root in service context — use `GIT_DIR` env var to bypass safe.directory issues

## Web HTML Files

### Crawler Control Pages (motor control + pipe visualization)

| File | Motor Type | Speed/Ramp Parameters |
|------|------------|-----------------------|
| `web/edgeflex_clearlink.html` | ClearLink motor | `jogSpeedInSec`, `rampPercent` |
| `web/edgeflex.html` | Roboclaw motor | `speed`, `ramp` from DOM |
| `web/new_xpress_scan.html` | Roboclaw motor | `speed`, `ramp` from DOM |
| `web/magnetic_phased_array.html` | Roboclaw motor | `speed`, `ramp` from DOM |

### Other Pages

| File | Purpose |
|------|---------|
| `web/boot_splash.html` | Boot splash screen with progress bar, redirects to main page |
| `web/index.html` | Main page (Roboclaw-based crawlers) |
| `web/config.html` | Crawler configuration page |
| `web/device_mismatch.html` | Security mismatch warning page (auto-refreshes every 30s) |

## edgeflex_clearlink.html Screen Layout (top to bottom)
1. **Header bar** — version number, refresh button, system gear icon (opens System popup)
2. **Update banner** — hidden by default, shows when git update available
3. **Info toolbar** — M1 status/velocity/torque | Pipe Diameter | M2 status/velocity/torque
4. **Button row** — Stats popup, Settings popup, Motors popup (all draggable)
5. **Pipe section** — circular pipe visualization with bug/radiation markers, degree display, distance traveled, reset position button
6. **Control buttons** — Forward, Backward, Stop (grid layout)
7. **Camera section** — video feed (Wall-E placeholder when offline)
8. **Footer** — version + last updated date (from git commit date via /api/last_updated)

### Popups (edgeflex_clearlink.html)
- **Stats** — ClearLink connection status, motor fault status (--/OK/FAULT), encoder positions (--/value), IMU status/roll/pitch, version
- **Settings** — pipe diameter, circumference, distance mode (encoder/degrees), jog speed, ramp, save defaults
- **Motors** — protected motor parameters (requires password unlock)
- **System** — system settings, WiFi config, support report

### ClearLink Disconnected Behavior
When ClearLink `connected: false`: motor faults show "--", encoder positions show "--", velocity/torque/position updates skip, motor status dots show stopped (yellow).

## PWA Support
- `web/manifest.json` — app name "Pipe Crawler Control", standalone display
- `web/sw.js` — service worker, caches shell assets, skips websocket/API requests
- `web/images/icon-192.png`, `icon-512.png`, `apple-touch-icon.png` — EdgeFlex logo (white on black, 3D shine effect)
- All 8 HTML files have PWA meta tags (theme-color, apple-mobile-web-app-capable, manifest link, apple-touch-icon) and SW registration script
- `manifest.json` and `sw.js` are in the always_skip list for boot redirect

## ROS2 Topics (key ones)
- `/clearlink/status` (clearlink_interfaces/MotorStatus) — connection, faults, velocity, torque, position
- `/clearlink/command` (clearlink_interfaces/MotorCommand) — motor commands
- `/imu/roll`, `/imu/pitch` (std_msgs/Float32) — IMU sensor data
- `/cpu_temp` (std_msgs/Float32) — CPU temperature
- `/robot_command` (std_msgs/String) — forward/backward/stop
- `/robot_speed`, `/robot_ramp` (std_msgs/String) — speed/ramp settings
- `/update_status` (std_msgs/String) — OTA update status

## API Endpoints (crawler_ui.py)
- `GET /api/status` — system status (hostname, crawler name, uptime, CPU temp, memory, disk, network)
- `GET /api/last_updated` — last git commit date
- `GET /api/system_stats` — CPU/RAM/disk stats
- `GET /api/fan` — fan status
- `GET /api/usb` — USB device info
- `POST /api/reboot` — reboot system
- `GET /api/security/boot_status` — device mismatch check
- `GET /api/support/system_info` — comprehensive system info for support
- `GET /api/mqtt/status` — MQTT config and service status
- `POST /api/mqtt/config` — save MQTT broker/connection settings
- `POST /api/mqtt/test` — test MQTT connection
- `GET /api/mqtt/topics` — get MQTT topic configuration
- `POST /api/mqtt/topics` — save MQTT topic configuration

## MQTT Bridge

- **Code:** `pipe_crawler_control/mqtt_bridge.py` (ROS2 node, launched via pipe_crawler.launch.py)
- **Config:** `/etc/crawler/mqtt.conf` (YAML)
- **Broker:** `edge-primeinspections.com:443` via WebSocket with TLS
- **Client ID:** `crawler-{hostname}` (currently `crawler-testcrawler`)
- **Auth:** username `crawler_{hostname}`, password from `api_key` in mqtt.conf
- **Publishes:** heartbeat (5s), system (60s), imu (10s), motor (10s) to `crawler/{hostname}/...`
- **Subscribes:** `crawler/{hostname}/cmd/#`, `crawler/{hostname}/settings/request`
- **Reconnection:** Has a 30s watchdog timer that forces full client teardown/reconnect if disconnected >90s (added to fix stalls after WiFi outages)
- **paho-mqtt:** v2.1.0 installed via pip

## Other Crawler Services

- `crawler-decrypt.service` — decryption on boot (oneshot, prerequisite)
- `cputemp.service` — ROS2 CPU temp publisher
- `rosbridge.service` — ROS2 rosbridge WebSocket server

## Web UI Design Standards

All web pages must follow these rules consistently:

### CSS Architecture
- **Shared styles:** `web/css/base.css` — all shared component styles live here. Never duplicate these in page `<style>` blocks.
- **Theme variables:** `web/css/theme.css` — all colors via CSS custom properties (`var(--name)`). Never use hardcoded hex colors.
- **Page-specific CSS:** Only put overrides or truly unique styles in page `<style>` blocks.

### Visual Theme
- **Card borders:** Every card/section element gets `border: 1px solid var(--border-color)` — info-toolbar, pipe-section, camera-section, graph-container, stats-button, popups, `.section` cards.
- **Border radius:** Cards use `border-radius: 12px–16px`, buttons use `border-radius: 20px` (pill) or `6px` (icon buttons).
- **Backgrounds:** Cards use `var(--bg-secondary)`, page background is `var(--bg-primary)`.
- **Collapsible sections:** Use chevron toggles with accordion behavior (only one section open at a time). CSS classes: `.collapsible`, `.expanded`, `.section-body`, `.section-body.open`.
- **Dark/light mode:** All colors must come from theme.css CSS variables. Both modes must work.

### Interaction Patterns
- **Password-protected actions:** VPN config, MQTT config, hostname changes, and motor settings require master password. Use the cached password pattern (`cacheMasterPassword`/`getCachedMasterPassword`, 5-minute TTL).
- **Modals:** Use `position: fixed` overlay with centered content. Config Tools modal is NOT background-dismissable; other modals are.
- **Popups:** Stats/Settings/Motors popups are draggable, centered with `position: fixed`.
- **Transitions:** Use `transition: 0.2s–0.35s ease` for all interactive state changes. Collapsible content uses `max-height` + `opacity` transitions.

### JavaScript Constraints
- **Safari compatibility:** No optional chaining (`?.`), no nullish coalescing (`??`), no arrow functions (`=>`). Use `function(){}` and explicit null checks.
- **No ES6 modules:** All JS is inline `<script>` blocks.

## Fleet VPN System

The fleet/ directory contains a WireGuard VPN system tunneled over WebSocket (wstunnel) for remote crawler management.

### Architecture
- WireGuard UDP is wrapped in WebSocket (TCP 443) using wstunnel
- This allows VPN to work through any network including corporate firewalls, hotel WiFi, cellular
- VM server at edge-primeinspections.com handles:
  - wstunnel server (WebSocket→UDP on localhost)
  - WireGuard server (localhost:51820)
  - Fleet API with auto-registration (port 9100)
  - Fleet dashboard at /fleet
  - nginx proxies everything on TCP 443

### Traffic Flow
Crawler WireGuard → localhost:51820 → wstunnel client → wss://edge-primeinspections.com/wstunnel (TCP 443) → nginx → wstunnel server → WireGuard server

### Boot Sequence (when VPN is configured)
1. wstunnel-client.service - WebSocket tunnel
2. crawler-vpn-register.service - auto-registers if no wg0.conf
3. wg-quick@wg0 - WireGuard VPN
4. ttyd.service - web terminal (VPN interface only)
5. crawler-heartbeat.service - status to fleet API every 60s

### VPN is Optional
- Configured via config.html VPN section
- If /etc/crawler/vpn_token doesn't exist, registration skips silently, no errors
- Crawlers work standalone on local network without VPN

### Fleet Files
- fleet/vpn_register.py → /opt/crawler/ (auto-registration)
- fleet/crawler_heartbeat.py → /opt/crawler/ (heartbeat)
- fleet/services/*.service → /etc/systemd/system/
- fleet/install.sh - installs everything, run as root
- VPN token set via config.html, stored at /etc/crawler/vpn_token

### Network
- VPN subnet: 10.0.0.0/24, VM is 10.0.0.1
- Crawler IPs: 10.0.0.10-254 (auto-assigned on registration)
- Heartbeat: 60 second interval, 180 second offline threshold
- Fleet API accessible at http://10.0.0.1:9100 over VPN
- Dashboard: https://edge-primeinspections.com/fleet

### Key Rules
- Never expose WireGuard port externally - localhost only
- Never store VPN token or MQTT keys in git
- All external traffic goes through TCP 443 via wstunnel
- ttyd binds to wg0 interface only (not local network)

## Key Config Files

- `/etc/crawler/mqtt.conf` — MQTT broker/topic settings
- `/etc/crawler/vpn_token` — VPN registration token (chmod 600)
- `/etc/crawler/wstunnel_url` — wstunnel WebSocket URL
- `/etc/xpresscan/network.conf` — crawler name, hostname, hotspot SSID/password, system config
- `/etc/xpresscan/ui_settings_{system_id}.json` — motor params, IMU offsets, jog speed, etc.
- `/home/craig/active_system` — currently `edgeflex_clearlink`
- `/var/lib/crawler/motor_data.db` — persistent motor telemetry (SQLite)
