# XPressCan Pipe Crawler Control

ROS2 control system for the XPressCan pipe crawler robot.

## Network Configuration

### WiFi Hotspot Setup

The crawler creates a WiFi hotspot for field operations:
- **Default SSID:** Crawler
- **Default Password:** Crawler1
- **Gateway IP:** 10.42.0.1
- **Frequency:** 5GHz (Channel 36)

### Web Interfaces

The system provides two separate web interfaces:

**1. Crawler UI (Port 80) - autobug.local**
- `http://autobug.local` - Main operational status and control
- Shows system stats (CPU temp, memory, disk)
- Shows network status
- Lists active ROS2 nodes
- Quick actions (reboot, refresh)

**2. Configuration Portal (Port 8080) - Hotspot**
- `http://10.42.0.1:8080` - Network and crawler configuration
- Captive portal auto-opens this when connecting to hotspot
- Configure crawler identity (custom names like "Magnetic", "EdgeFlex")
- Change hotspot SSID and password
- Scan and connect to WiFi networks

### Installation

```bash
cd ~/ros2_ws/src/pipe_crawler_control/scripts/network
sudo ./install_hotspot.sh
```

This installs:
- Hotspot configuration service
- Configuration portal web server (port 8080)

### Manual Commands

```bash
# Start hotspot manually
sudo /opt/xpresscan/setup_hotspot.sh

# Connect as client to another crawler's hotspot
sudo /opt/xpresscan/connect_to_field.sh
```

## System Backup

### Using rpi-clone

rpi-clone creates a bootable backup of the eMMC to a USB drive.

**Installation:**
```bash
git clone https://github.com/billw2/rpi-clone.git
cd rpi-clone
sudo cp rpi-clone /usr/local/sbin/
```

**Create Backup:**
```bash
# Insert USB drive (appears as /dev/sda)
sudo rpi-clone sda

# Unattended mode (no prompts)
sudo rpi-clone sda -U
```

**Important Notes:**
- USB drive must be at least the same size as the eMMC (16GB minimum)
- The resulting USB is bootable - can be used to restore or clone new units
- All data on the USB drive will be overwritten
- Backup includes the complete system: OS, ROS2 workspace, and all configurations

**Restore from Backup:**
1. Boot from the USB backup drive
2. Run `sudo rpi-clone mmcblk0` to restore to eMMC

**Clone to New Unit:**
1. Insert the backup USB into the new Raspberry Pi
2. Boot from USB
3. Run `sudo rpi-clone mmcblk0` to clone to the new eMMC

## Network Architecture

```
Hotspot Mode (Field):
  Crawler (10.42.0.1) <-- WiFi --> Tablet/Laptop

Multi-Crawler Mode:
  Master Crawler (10.42.0.1)
    |
    +-- autobug-1 (10.42.0.11)
    +-- autobug-2 (10.42.0.12)
```

## Services

```bash
# Enable services to start on boot
sudo systemctl enable xpresscan-hotspot
sudo systemctl enable xpresscan-portal

# Start services manually
sudo systemctl start xpresscan-hotspot
sudo systemctl start xpresscan-portal

# Check status
sudo systemctl status xpresscan-hotspot
sudo systemctl status xpresscan-portal
```
