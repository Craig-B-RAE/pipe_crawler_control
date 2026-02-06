#!/usr/bin/env python3
"""
Security utilities for crawler device locking.

This module provides functions for:
- Reading CPU serial number
- Reading RoboClaw serial number via USB
- Reading ClearLink MAC address via EtherNet/IP
- Deriving encryption keys from hardware identifiers
- Locking/unlocking devices (CM5, RoboClaw, ClearLink)
- Password verification
- Device mismatch checking on boot
"""

import hashlib
import subprocess
import configparser
import os
import shutil
import json
import logging
import time
import serial
from datetime import datetime
from typing import Dict, Optional, Tuple

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("security_utils")

SECURITY_CONF = "/etc/crawler/security.conf"
NETWORK_CONF = "/etc/xpresscan/network.conf"
ROS2_SRC = "/home/craig/ros2_ws/src"
ENCRYPTED_DIR = "/home/craig/ros2_ws/.src_encrypted"
DEVICE_MISMATCH_FILE = "/tmp/device_mismatch.json"

# SECRET: This salt is compiled into the code and not stored on the device.
# Change this for different product lines.
SECRET_SALT = "XPressCan_Crawler_2024_RAE_Engineering"

# Crawler type to device mapping
CRAWLER_DEVICES = {
    "magnetic": ["cm5", "roboclaw"],
    "xpresscan": ["cm5", "roboclaw"],
    "edgeflex": ["cm5", "roboclaw"],
    "edgeflex_clearlink": ["cm5", "clearlink"],
}

# Default ClearLink IP (can be overridden by config)
DEFAULT_CLEARLINK_IP = "192.168.1.70"

# RoboClaw EEPROM ID storage constants
ROBOCLAW_ID_START_ADDR = 128  # First EEPROM address for ID storage
ROBOCLAW_ID_WORDS = 4         # Number of 16-bit words (8 bytes total)
ROBOCLAW_ID_MAGIC = 0x5241    # "RA" - indicates valid ID present


def get_crawler_type() -> str:
    """Get the current crawler type from network.conf."""
    try:
        if os.path.exists(NETWORK_CONF):
            with open(NETWORK_CONF, 'r') as f:
                for line in f:
                    if line.startswith("CRAWLER_ID="):
                        return line.split("=")[1].strip()
    except Exception:
        pass
    return "magnetic"  # Default


def get_relevant_devices() -> list:
    """Get list of devices relevant to the current crawler type."""
    crawler_type = get_crawler_type()
    return CRAWLER_DEVICES.get(crawler_type, ["cm5", "roboclaw"])


# =============================================================================
# Device Serial/ID Reading Functions
# =============================================================================

def get_cpu_serial() -> str:
    """Read CPU serial from /proc/cpuinfo (Raspberry Pi/ARM)."""
    try:
        with open("/proc/cpuinfo", "r") as f:
            for line in f:
                if line.startswith("Serial"):
                    serial = line.split(":")[1].strip()
                    return serial
    except Exception:
        pass
    return ""


def get_device_id() -> str:
    """Get short device identifier (last 4 chars of serial)."""
    serial = get_cpu_serial()
    return serial[-4:] if len(serial) >= 4 else serial


# =============================================================================
# RoboClaw EEPROM ID Functions
# =============================================================================

class RoboclawEEPROM:
    """Low-level RoboClaw EEPROM access for unique ID storage."""

    def __init__(self, port: str, rate: int = 115200, timeout: float = 0.1):
        self.port = port
        self.rate = rate
        self.timeout = timeout
        self._port = None
        self._crc = 0
        self._trystimeout = 3

    def open(self) -> bool:
        try:
            self._port = serial.Serial(port=self.port, baudrate=self.rate, timeout=self.timeout)
            return True
        except Exception:
            return False

    def close(self):
        if self._port:
            self._port.close()
            self._port = None

    def _crc_clear(self):
        self._crc = 0

    def _crc_update(self, data: int):
        self._crc = self._crc ^ (data << 8)
        for _ in range(8):
            if (self._crc & 0x8000) == 0x8000:
                self._crc = ((self._crc << 1) ^ 0x1021)
            else:
                self._crc = self._crc << 1

    def _sendcommand(self, address: int, command: int):
        self._crc_clear()
        self._crc_update(address)
        self._port.write(address.to_bytes(1, 'big'))
        self._crc_update(command)
        self._port.write(command.to_bytes(1, 'big'))

    def _writebyte(self, val: int):
        self._crc_update(val & 0xFF)
        self._port.write((val & 0xFF).to_bytes(1, 'big'))

    def _writeword(self, val: int):
        self._writebyte((val >> 8) & 0xFF)
        self._writebyte(val & 0xFF)

    def _readbyte(self) -> Tuple[int, int]:
        data = self._port.read(1)
        if len(data):
            val = data[0]
            self._crc_update(val)
            return (1, val)
        return (0, 0)

    def _readword(self) -> Tuple[int, int]:
        val1 = self._readbyte()
        if val1[0]:
            val2 = self._readbyte()
            if val2[0]:
                return (1, val1[1] << 8 | val2[1])
        return (0, 0)

    def _readchecksumword(self) -> Tuple[int, int]:
        data = self._port.read(2)
        if len(data) == 2:
            crc = (data[0] << 8) | data[1]
            return (1, crc)
        return (0, 0)

    def _writechecksum(self) -> bool:
        self._writeword(self._crc & 0xFFFF)
        val = self._readbyte()
        return val[0] == 1

    def read_version(self, address: int = 0x80) -> Tuple[bool, str]:
        """Read firmware version to verify connection."""
        trys = self._trystimeout
        while trys > 0:
            self._port.flushInput()
            self._sendcommand(address, 21)  # GETVERSION
            result = ""
            passed = True
            for _ in range(48):
                data = self._port.read(1)
                if len(data):
                    val = data[0]
                    self._crc_update(val)
                    if val == 0:
                        break
                    result += chr(val)
                else:
                    passed = False
                    break
            if passed:
                crc = self._readchecksumword()
                if crc[0] and (self._crc & 0xFFFF) == (crc[1] & 0xFFFF):
                    return (True, result)
            trys -= 1
            time.sleep(0.01)
        return (False, "")

    def read_eeprom(self, address: int, ee_address: int) -> Tuple[bool, int]:
        """Read a 16-bit word from EEPROM."""
        trys = self._trystimeout
        while trys > 0:
            self._port.flushInput()
            self._sendcommand(address, 252)  # READEEPROM
            self._crc_update(ee_address)
            self._port.write(ee_address.to_bytes(1, 'big'))
            val1 = self._readword()
            if val1[0]:
                crc = self._readchecksumword()
                if crc[0] and (self._crc & 0xFFFF) == (crc[1] & 0xFFFF):
                    return (True, val1[1])
            trys -= 1
            time.sleep(0.01)
        return (False, 0)

    def write_eeprom(self, address: int, ee_address: int, ee_word: int) -> bool:
        """Write a 16-bit word to EEPROM."""
        trys = self._trystimeout
        while trys > 0:
            self._sendcommand(address, 253)  # WRITEEEPROM
            self._writebyte(ee_address)
            self._writebyte((ee_word >> 8) & 0xFF)
            self._writebyte(ee_word & 0xFF)
            if self._writechecksum():
                time.sleep(0.05)
                for _ in range(20):
                    data = self._port.read(1)
                    if len(data) and data[0] == 0xAA:
                        return True
                    time.sleep(0.01)
            trys -= 1
            time.sleep(0.05)
        return False


def read_roboclaw_id(port: str = None) -> str:
    """
    Read unique ID from RoboClaw EEPROM.

    Args:
        port: Serial port (auto-detected if None)

    Returns:
        12-character hex string ID, or empty string if not set/error
    """
    ports = [port] if port else ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]

    for p in ports:
        try:
            rc = RoboclawEEPROM(p)
            if not rc.open():
                continue

            # Verify connection
            success, version = rc.read_version(0x80)
            if not success:
                rc.close()
                continue

            # Read ID words from EEPROM
            words = []
            for i in range(ROBOCLAW_ID_WORDS):
                success, value = rc.read_eeprom(0x80, ROBOCLAW_ID_START_ADDR + i)
                if not success:
                    rc.close()
                    return ""
                words.append(value)

            rc.close()

            # Check magic header
            if words[0] != ROBOCLAW_ID_MAGIC:
                return ""  # No valid ID stored

            # Convert remaining words to hex string (6 bytes = 12 hex chars)
            id_bytes = b''
            for w in words[1:]:
                id_bytes += w.to_bytes(2, 'big')

            return id_bytes.hex().upper()

        except Exception as e:
            logger.debug(f"RoboClaw ID read error on {p}: {e}")
            continue

    return ""


def write_roboclaw_id(unique_id: str, port: str = None) -> Tuple[bool, str]:
    """
    Write unique ID to RoboClaw EEPROM.

    Args:
        unique_id: 12-character hex string (6 bytes)
        port: Serial port (auto-detected if None)

    Returns:
        Tuple of (success, message)
    """
    # Validate ID format
    if len(unique_id) != 12:
        return False, f"ID must be 12 hex characters, got {len(unique_id)}"

    try:
        id_bytes = bytes.fromhex(unique_id)
    except ValueError:
        return False, "ID must be valid hex string"

    ports = [port] if port else ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]

    for p in ports:
        try:
            rc = RoboclawEEPROM(p)
            if not rc.open():
                continue

            # Verify connection
            success, version = rc.read_version(0x80)
            if not success:
                rc.close()
                continue

            # Build words: [MAGIC, byte0-1, byte2-3, byte4-5]
            words = [
                ROBOCLAW_ID_MAGIC,
                (id_bytes[0] << 8) | id_bytes[1],
                (id_bytes[2] << 8) | id_bytes[3],
                (id_bytes[4] << 8) | id_bytes[5],
            ]

            # Write each word
            for i, word in enumerate(words):
                if not rc.write_eeprom(0x80, ROBOCLAW_ID_START_ADDR + i, word):
                    rc.close()
                    return False, f"Failed to write word {i}"
                time.sleep(0.05)

            rc.close()
            return True, f"ID {unique_id} written successfully"

        except Exception as e:
            logger.debug(f"RoboClaw ID write error on {p}: {e}")
            continue

    return False, "Could not connect to RoboClaw"


def generate_roboclaw_id() -> str:
    """Generate a unique 6-byte ID (12 hex chars) for RoboClaw."""
    import random
    data = f"{time.time()}{random.random()}{os.urandom(8).hex()}"
    hash_bytes = hashlib.sha256(data.encode()).digest()[:6]
    return hash_bytes.hex().upper()


def get_roboclaw_serial() -> str:
    """
    Read unique identifier from RoboClaw motor controller.

    First tries to read the custom EEPROM ID (preferred).
    Falls back to firmware version string if no ID is set.

    Returns empty string if not connected or error.
    """
    # First try to read EEPROM ID
    eeprom_id = read_roboclaw_id()
    if eeprom_id:
        logger.info(f"RoboClaw EEPROM ID: {eeprom_id}")
        return f"EEPROM:{eeprom_id}"

    # Fall back to firmware version
    port_exists = False
    try:
        import sys
        sys.path.insert(0, "/home/craig/ros2_ws/src/roboclaw_driver2/roboclaw_driver2")
        from roboclaw_3 import Roboclaw

        for port in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]:
            if os.path.exists(port):
                port_exists = True
            try:
                rc = Roboclaw(port, 115200)
                if rc.Open():
                    version = rc.ReadVersion(0x80)
                    rc._port.close()
                    if version and version[0]:
                        ver_str = version[1].strip() if len(version) > 1 else str(version[0])
                        logger.info(f"RoboClaw firmware: {ver_str} (no EEPROM ID set)")
                        return ver_str
            except Exception as e:
                logger.debug(f"RoboClaw port {port}: {e}")
                continue
    except ImportError as e:
        logger.warning(f"RoboClaw library not available: {e}")
    except Exception as e:
        logger.warning(f"RoboClaw detection error: {e}")

    # Device file exists but port is in use by another process (e.g. roboclaw_node)
    if port_exists:
        logger.info("RoboClaw device detected (port in use by another process)")
        return "detected-in-use"

    return ""


def get_clearlink_mac() -> str:
    """
    Read MAC address from ClearLink controller via EtherNet/IP.

    Reads the Identity object which contains device identification.
    Returns empty string if not connected or error.
    """
    clearlink_ip = DEFAULT_CLEARLINK_IP

    # Try to read from config file
    try:
        import yaml
        config_paths = [
            "/home/craig/ros2_ws/src/pipe_crawler_control/config/edgeflex_clearlink.yaml",
        ]
        for path in config_paths:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    config = yaml.safe_load(f)
                    if config and 'driver' in config:
                        clearlink_ip = config['driver'].get('ip_address', clearlink_ip)
                logger.info(f"ClearLink IP from config: {clearlink_ip}")
                break
    except Exception as e:
        logger.debug(f"Could not read ClearLink config: {e}")

    logger.info(f"Attempting ClearLink connection to {clearlink_ip}")

    try:
        from pycomm3 import CIPDriver

        with CIPDriver(clearlink_ip) as plc:
            logger.info(f"ClearLink connected to {clearlink_ip}")

            # Read Identity object - Class 0x01, Instance 1
            # Get Attributes All (service 0x01)
            result = plc.generic_message(
                service=0x01,  # Get Attributes All
                class_code=0x01,  # Identity
                instance=1
            )

            logger.info(f"ClearLink Identity result: {result}")

            if result and result.value:
                # Parse identity response - extract serial number
                data = result.value
                logger.info(f"ClearLink Identity data type: {type(data)}, length: {len(data) if hasattr(data, '__len__') else 'N/A'}")

                if isinstance(data, bytes) and len(data) >= 16:
                    # Serial number is at offset 10-13 (4 bytes, little-endian)
                    serial = int.from_bytes(data[10:14], 'little')
                    mac_result = f"{serial:08X}"
                    logger.info(f"ClearLink serial extracted: {mac_result}")
                    return mac_result
                elif hasattr(result, 'tag_list') or isinstance(data, dict):
                    # Try to extract serial from structured response
                    result_str = str(data)
                    logger.info(f"ClearLink structured response: {result_str}")
                    return result_str

            # Fallback: try to read TCP/IP Interface object for MAC
            # Class 0xF5, Instance 1, Attribute 3 (MAC Address)
            logger.info("Trying TCP/IP Interface object for MAC...")
            result = plc.generic_message(
                service=0x0E,  # Get Attribute Single
                class_code=0xF5,  # TCP/IP Interface
                instance=1,
                attribute=3  # MAC Address
            )

            logger.info(f"ClearLink TCP/IP result: {result}")

            if result and result.value:
                data = result.value
                if isinstance(data, bytes) and len(data) >= 6:
                    mac = ":".join(f"{b:02X}" for b in data[:6])
                    logger.info(f"ClearLink MAC extracted: {mac}")
                    return mac

    except ImportError as e:
        logger.warning(f"pycomm3 not installed: {e}")
    except Exception as e:
        logger.warning(f"ClearLink detection error: {e}")

    logger.info("ClearLink not detected")
    return ""


# =============================================================================
# Security Configuration - Multi-Device Support
# =============================================================================

def load_security_config_v2() -> dict:
    """
    Load security configuration (v2 format with multi-device support).

    Returns dict with structure:
    {
        "config_version": "2",
        "master_password_hash": "...",
        "last_operation": "...",
        "cm5": {"lock_state": "locked", "serial": "..."},
        "roboclaw": {"lock_state": "unlocked", "serial": ""},
        "clearlink": {"lock_state": "unlocked", "mac": ""}
    }
    """
    config = {
        "config_version": "2",
        "master_password_hash": "",
        "last_operation": "",
        "cm5": {"lock_state": "unlocked", "serial": ""},
        "roboclaw": {"lock_state": "unlocked", "serial": ""},
        "clearlink": {"lock_state": "unlocked", "mac": ""}
    }

    if not os.path.exists(SECURITY_CONF):
        return config

    parser = configparser.ConfigParser()
    try:
        parser.read(SECURITY_CONF)

        # Check config version
        version = parser.get("security", "config_version", fallback="1")

        if version == "1":
            # Migrate from v1 format
            if "security" in parser:
                config["master_password_hash"] = parser.get("security", "master_password_hash", fallback="")
                config["last_operation"] = parser.get("security", "last_operation", fallback="")
                # Migrate old lock state to cm5 section
                old_state = parser.get("security", "lock_state", fallback="unlocked")
                old_device = parser.get("security", "locked_to_device", fallback="")
                config["cm5"]["lock_state"] = old_state
                config["cm5"]["serial"] = get_cpu_serial() if old_state == "locked" else ""
        else:
            # v2 format
            if "security" in parser:
                config["config_version"] = parser.get("security", "config_version", fallback="2")
                config["master_password_hash"] = parser.get("security", "master_password_hash", fallback="")
                config["last_operation"] = parser.get("security", "last_operation", fallback="")

            if "cm5" in parser:
                config["cm5"]["lock_state"] = parser.get("cm5", "lock_state", fallback="unlocked")
                config["cm5"]["serial"] = parser.get("cm5", "serial", fallback="")

            if "roboclaw" in parser:
                config["roboclaw"]["lock_state"] = parser.get("roboclaw", "lock_state", fallback="unlocked")
                config["roboclaw"]["serial"] = parser.get("roboclaw", "serial", fallback="")

            if "clearlink" in parser:
                config["clearlink"]["lock_state"] = parser.get("clearlink", "lock_state", fallback="unlocked")
                config["clearlink"]["mac"] = parser.get("clearlink", "mac", fallback="")

    except Exception:
        pass

    return config


def save_security_config_v2(config: dict) -> bool:
    """Save security configuration (v2 format)."""
    try:
        os.makedirs(os.path.dirname(SECURITY_CONF), exist_ok=True)

        parser = configparser.ConfigParser()

        # Main security section
        parser["security"] = {
            "config_version": "2",
            "master_password_hash": config.get("master_password_hash", ""),
            "last_operation": config.get("last_operation", "")
        }

        # Device sections
        parser["cm5"] = {
            "lock_state": config.get("cm5", {}).get("lock_state", "unlocked"),
            "serial": config.get("cm5", {}).get("serial", "")
        }

        parser["roboclaw"] = {
            "lock_state": config.get("roboclaw", {}).get("lock_state", "unlocked"),
            "serial": config.get("roboclaw", {}).get("serial", "")
        }

        parser["clearlink"] = {
            "lock_state": config.get("clearlink", {}).get("lock_state", "unlocked"),
            "mac": config.get("clearlink", {}).get("mac", "")
        }

        with open(SECURITY_CONF, "w") as f:
            f.write("# Crawler Security Configuration (v2)\n")
            f.write("# DO NOT EDIT MANUALLY\n\n")
            parser.write(f)

        os.chmod(SECURITY_CONF, 0o600)
        return True
    except Exception as e:
        print(f"Error saving config: {e}")
        return False


# Backwards compatibility - load old format
def load_security_config() -> dict:
    """Load security configuration (legacy v1 compatibility)."""
    config_v2 = load_security_config_v2()
    # Return legacy format for backwards compatibility
    return {
        "lock_state": config_v2["cm5"]["lock_state"],
        "locked_to_device": get_device_id() if config_v2["cm5"]["lock_state"] == "locked" else "",
        "master_password_hash": config_v2["master_password_hash"],
        "last_operation": config_v2["last_operation"],
        "config_version": config_v2["config_version"]
    }


def save_security_config(config: dict) -> bool:
    """Save security configuration (legacy v1 compatibility)."""
    # Convert to v2 format
    config_v2 = load_security_config_v2()
    config_v2["master_password_hash"] = config.get("master_password_hash", config_v2["master_password_hash"])
    config_v2["last_operation"] = config.get("last_operation", "")
    config_v2["cm5"]["lock_state"] = config.get("lock_state", "unlocked")
    if config.get("lock_state") == "locked":
        config_v2["cm5"]["serial"] = get_cpu_serial()
    else:
        config_v2["cm5"]["serial"] = ""
    return save_security_config_v2(config_v2)


# =============================================================================
# Device Lock/Unlock Functions
# =============================================================================

def lock_single_device(device: str) -> Tuple[bool, str]:
    """
    Lock a single device (cm5, roboclaw, or clearlink).

    Args:
        device: Device name to lock

    Returns:
        tuple: (success: bool, message: str)
    """
    config = load_security_config_v2()

    if not config.get("master_password_hash"):
        return False, "Master password not set. Run /opt/crawler/set_master_password.sh first."

    # Get current device serial/ID
    if device == "cm5":
        current_id = get_cpu_serial()
        if not current_id:
            return False, "Could not read CM5 CPU serial"
        config["cm5"]["lock_state"] = "locked"
        config["cm5"]["serial"] = current_id
        display_id = f"CM5-{get_device_id()}"

    elif device == "roboclaw":
        current_id = get_roboclaw_serial()
        if not current_id:
            return False, "RoboClaw not detected. Connect it and try again."
        config["roboclaw"]["lock_state"] = "locked"
        config["roboclaw"]["serial"] = current_id
        display_id = f"RoboClaw-{current_id[:16]}"

    elif device == "clearlink":
        current_id = get_clearlink_mac()
        if not current_id:
            return False, "ClearLink not detected. Check network connection and try again."
        config["clearlink"]["lock_state"] = "locked"
        config["clearlink"]["mac"] = current_id
        display_id = f"ClearLink-{current_id}"

    else:
        return False, f"Unknown device: {device}"

    config["last_operation"] = datetime.now().isoformat()

    if save_security_config_v2(config):
        return True, f"Locked to {display_id}"
    else:
        return False, "Failed to save configuration"


def unlock_single_device(device: str, master_password: str) -> Tuple[bool, str]:
    """
    Unlock a single device.

    Args:
        device: Device name to unlock
        master_password: Master password for verification

    Returns:
        tuple: (success: bool, message: str)
    """
    if not verify_master_password(master_password):
        return False, "Invalid master password"

    config = load_security_config_v2()

    if device == "cm5":
        config["cm5"]["lock_state"] = "unlocked"
        config["cm5"]["serial"] = ""
    elif device == "roboclaw":
        config["roboclaw"]["lock_state"] = "unlocked"
        config["roboclaw"]["serial"] = ""
    elif device == "clearlink":
        config["clearlink"]["lock_state"] = "unlocked"
        config["clearlink"]["mac"] = ""
    else:
        return False, f"Unknown device: {device}"

    config["last_operation"] = datetime.now().isoformat()

    if save_security_config_v2(config):
        return True, f"{device.upper()} unlocked"
    else:
        return False, "Failed to save configuration"


def check_all_device_locks() -> Dict:
    """
    Check ALL devices against current hardware.

    IMPORTANT: Only locked devices are checked for mismatch.
    - Unlocked devices: just report detection status for UI display
    - Locked devices not detected: MISMATCH (error)
    - Locked devices detected but wrong serial: MISMATCH (error)
    - Locked devices detected and correct serial: OK

    Returns dict with structure:
    {
        "cm5": {"locked": bool, "expected": str, "found": str, "match": bool, "detected": bool},
        "roboclaw": {"locked": bool, "expected": str, "found": str, "match": bool, "detected": bool},
        "clearlink": {"locked": bool, "expected": str, "found": str, "match": bool, "detected": bool},
        "any_mismatch": bool,
        "relevant_devices": ["cm5", "roboclaw"] or ["cm5", "clearlink"]
    }
    """
    config = load_security_config_v2()
    relevant = get_relevant_devices()

    result = {
        "cm5": {"locked": False, "expected": "", "found": "", "match": True, "detected": False},
        "roboclaw": {"locked": False, "expected": "", "found": "", "match": True, "detected": False},
        "clearlink": {"locked": False, "expected": "", "found": "", "match": True, "detected": False},
        "any_mismatch": False,
        "relevant_devices": relevant,
        "crawler_type": get_crawler_type()
    }

    # Check CM5 (always)
    cm5_found = get_cpu_serial()
    result["cm5"]["found"] = cm5_found
    result["cm5"]["detected"] = bool(cm5_found)

    if config["cm5"]["lock_state"] == "locked":
        result["cm5"]["locked"] = True
        result["cm5"]["expected"] = config["cm5"]["serial"]
        # For LOCKED devices: mismatch if not detected OR wrong serial
        if not cm5_found:
            result["cm5"]["match"] = False  # Not detected = mismatch
        else:
            result["cm5"]["match"] = (result["cm5"]["expected"] == cm5_found)

    # Check RoboClaw (always - for UI display)
    roboclaw_found = get_roboclaw_serial()
    result["roboclaw"]["found"] = roboclaw_found
    result["roboclaw"]["detected"] = bool(roboclaw_found)

    if config["roboclaw"]["lock_state"] == "locked":
        result["roboclaw"]["locked"] = True
        result["roboclaw"]["expected"] = config["roboclaw"]["serial"]
        # For LOCKED devices: mismatch if not detected OR wrong serial
        if not roboclaw_found:
            result["roboclaw"]["match"] = False  # Not detected = mismatch
        else:
            result["roboclaw"]["match"] = (result["roboclaw"]["expected"] == roboclaw_found)

    # Check ClearLink (always - for UI display)
    clearlink_found = get_clearlink_mac()
    result["clearlink"]["found"] = clearlink_found
    result["clearlink"]["detected"] = bool(clearlink_found)

    if config["clearlink"]["lock_state"] == "locked":
        result["clearlink"]["locked"] = True
        result["clearlink"]["expected"] = config["clearlink"]["mac"]
        # For LOCKED devices: mismatch if not detected OR wrong serial
        if not clearlink_found:
            result["clearlink"]["match"] = False  # Not detected = mismatch
        else:
            result["clearlink"]["match"] = (result["clearlink"]["expected"] == clearlink_found)

    # Check for any mismatches - ONLY check LOCKED devices that are RELEVANT
    for device in relevant:
        if result[device]["locked"] and not result[device]["match"]:
            result["any_mismatch"] = True
            break

    return result


def write_device_mismatch(mismatch_data: Dict) -> None:
    """Write device mismatch information to temp file for UI."""
    try:
        with open(DEVICE_MISMATCH_FILE, 'w') as f:
            json.dump(mismatch_data, f, indent=2)
    except Exception:
        pass


def clear_device_mismatch() -> None:
    """Clear the device mismatch file."""
    try:
        if os.path.exists(DEVICE_MISMATCH_FILE):
            os.remove(DEVICE_MISMATCH_FILE)
    except Exception:
        pass


def read_device_mismatch() -> Optional[Dict]:
    """Read device mismatch file if it exists."""
    try:
        if os.path.exists(DEVICE_MISMATCH_FILE):
            with open(DEVICE_MISMATCH_FILE, 'r') as f:
                return json.load(f)
    except Exception:
        pass
    return None


# =============================================================================
# Legacy Functions (for backwards compatibility)
# =============================================================================

def derive_encryption_key(cpu_serial: str) -> str:
    """
    Derive gocryptfs password from CPU serial and secret salt.

    Uses PBKDF2 with SHA256 and 100,000 iterations for key derivation.
    The resulting key is deterministic for a given CPU serial.
    """
    combined = f"{cpu_serial}:{SECRET_SALT}"
    key = hashlib.pbkdf2_hmac(
        'sha256',
        combined.encode(),
        SECRET_SALT.encode(),
        100000
    )
    return key.hex()


def hash_password(password: str) -> str:
    """Hash a password with SHA256."""
    return hashlib.sha256(password.encode()).hexdigest()


def verify_master_password(password: str) -> bool:
    """Verify the master password against stored hash."""
    config = load_security_config_v2()
    stored_hash = config.get("master_password_hash", "")
    if not stored_hash:
        return False
    return hash_password(password) == stored_hash


def is_gocryptfs_mounted() -> bool:
    """Check if src directory is a FUSE/gocryptfs mount."""
    try:
        result = subprocess.run(
            ["mountpoint", "-q", ROS2_SRC],
            capture_output=True
        )
        return result.returncode == 0
    except Exception:
        return False


def mount_encrypted(password: str) -> tuple:
    """
    Mount the encrypted directory using gocryptfs.

    Returns:
        tuple: (success: bool, message: str)
    """
    if not os.path.exists(ENCRYPTED_DIR):
        return False, "Encrypted directory does not exist"

    if is_gocryptfs_mounted():
        return True, "Already mounted"

    try:
        # Ensure mount point exists and is empty
        if os.path.exists(ROS2_SRC):
            # Check if directory is empty or remove it
            if os.path.isdir(ROS2_SRC) and not os.listdir(ROS2_SRC):
                pass  # Empty dir is fine
            elif os.path.isdir(ROS2_SRC):
                # Has contents but not a mount - this shouldn't happen in locked state
                pass
        else:
            os.makedirs(ROS2_SRC, exist_ok=True)

        # Mount with gocryptfs, password via stdin
        result = subprocess.run(
            ["gocryptfs", "-passfile", "/dev/stdin", ENCRYPTED_DIR, ROS2_SRC],
            input=password.encode(),
            capture_output=True,
            timeout=30
        )

        if result.returncode == 0:
            # Fix ownership after mount
            subprocess.run(["chown", "craig:craig", ROS2_SRC], capture_output=True)
            return True, "Mounted successfully"
        else:
            stderr = result.stderr.decode() if result.stderr else "Unknown error"
            return False, f"Mount failed: {stderr}"
    except subprocess.TimeoutExpired:
        return False, "Mount timed out"
    except Exception as e:
        return False, str(e)


def unmount_encrypted() -> tuple:
    """
    Unmount the encrypted directory.

    Returns:
        tuple: (success: bool, message: str)
    """
    if not is_gocryptfs_mounted():
        return True, "Not mounted"

    try:
        # Try fusermount first
        result = subprocess.run(
            ["fusermount", "-u", ROS2_SRC],
            capture_output=True,
            timeout=10
        )

        if result.returncode == 0:
            return True, "Unmounted successfully"
        else:
            # Try lazy unmount if busy
            result = subprocess.run(
                ["fusermount", "-uz", ROS2_SRC],
                capture_output=True,
                timeout=10
            )
            if result.returncode == 0:
                return True, "Unmounted (lazy)"
            stderr = result.stderr.decode() if result.stderr else "Unknown error"
            return False, f"Unmount failed: {stderr}"
    except Exception as e:
        return False, str(e)


def lock_device() -> tuple:
    """
    Lock the device by encrypting the source directory.
    (Legacy function - locks CM5 only)
    """
    config = load_security_config()

    if config["lock_state"] == "locked":
        return False, "Device is already locked"

    if not config.get("master_password_hash"):
        return False, "Master password not set. Run /opt/crawler/set_master_password.sh first."

    cpu_serial = get_cpu_serial()
    if not cpu_serial:
        return False, "Could not read CPU serial"

    device_id = get_device_id()
    encryption_key = derive_encryption_key(cpu_serial)

    try:
        # Stop services that use the source directory
        subprocess.run(
            ["sudo", "systemctl", "stop", "pipe_crawler.service"],
            capture_output=True,
            timeout=30
        )

        # Check if encrypted directory already exists
        if os.path.exists(ENCRYPTED_DIR):
            return False, "Encrypted directory already exists. Remove it first or device may already be locked."

        os.makedirs(ENCRYPTED_DIR, exist_ok=True)

        # Initialize gocryptfs with the derived key
        result = subprocess.run(
            ["gocryptfs", "-init", "-passfile", "/dev/stdin", ENCRYPTED_DIR],
            input=encryption_key.encode(),
            capture_output=True,
            timeout=60
        )

        if result.returncode != 0:
            shutil.rmtree(ENCRYPTED_DIR, ignore_errors=True)
            stderr = result.stderr.decode() if result.stderr else "Unknown error"
            return False, f"gocryptfs init failed: {stderr}"

        # Mount the encrypted directory temporarily for copying
        temp_mount = "/tmp/crawler_encrypt_mount"
        os.makedirs(temp_mount, exist_ok=True)

        result = subprocess.run(
            ["gocryptfs", "-passfile", "/dev/stdin", ENCRYPTED_DIR, temp_mount],
            input=encryption_key.encode(),
            capture_output=True,
            timeout=30
        )

        if result.returncode != 0:
            shutil.rmtree(ENCRYPTED_DIR, ignore_errors=True)
            stderr = result.stderr.decode() if result.stderr else "Unknown error"
            return False, f"Failed to mount for encryption: {stderr}"

        # Copy files to encrypted storage
        result = subprocess.run(
            ["rsync", "-a", "--delete", f"{ROS2_SRC}/", f"{temp_mount}/"],
            capture_output=True,
            timeout=600  # Allow up to 10 minutes for large codebases
        )

        if result.returncode != 0:
            subprocess.run(["fusermount", "-u", temp_mount], capture_output=True)
            shutil.rmtree(ENCRYPTED_DIR, ignore_errors=True)
            stderr = result.stderr.decode() if result.stderr else "Unknown error"
            return False, f"Failed to copy files: {stderr}"

        # Unmount temp
        subprocess.run(["fusermount", "-u", temp_mount], timeout=10, capture_output=True)
        shutil.rmtree(temp_mount, ignore_errors=True)

        # Remove original source directory
        shutil.rmtree(ROS2_SRC)
        os.makedirs(ROS2_SRC, exist_ok=True)

        # Update config (v2 format)
        config_v2 = load_security_config_v2()
        config_v2["cm5"]["lock_state"] = "locked"
        config_v2["cm5"]["serial"] = cpu_serial
        config_v2["last_operation"] = datetime.now().isoformat()
        save_security_config_v2(config_v2)

        return True, f"Device locked to CM5-{device_id}"

    except subprocess.TimeoutExpired:
        return False, "Operation timed out"
    except Exception as e:
        return False, str(e)


def unlock_device(master_password: str) -> tuple:
    """
    Unlock the device by decrypting and removing encryption.
    (Legacy function - unlocks CM5 source code encryption only)
    """
    config = load_security_config()

    if config["lock_state"] != "locked":
        return False, "Device is not locked"

    if not verify_master_password(master_password):
        return False, "Invalid master password"

    cpu_serial = get_cpu_serial()
    encryption_key = derive_encryption_key(cpu_serial)

    try:
        # Stop services
        subprocess.run(
            ["sudo", "systemctl", "stop", "pipe_crawler.service"],
            capture_output=True,
            timeout=30
        )

        # Ensure mounted
        if not is_gocryptfs_mounted():
            success, msg = mount_encrypted(encryption_key)
            if not success:
                return False, f"Cannot mount encrypted filesystem: {msg}"

        # Create temp directory for decrypted files
        temp_copy = "/tmp/crawler_decrypt_temp"
        if os.path.exists(temp_copy):
            shutil.rmtree(temp_copy)

        # Copy files out of encrypted storage
        result = subprocess.run(
            ["rsync", "-a", f"{ROS2_SRC}/", f"{temp_copy}/"],
            capture_output=True,
            timeout=600
        )

        if result.returncode != 0:
            stderr = result.stderr.decode() if result.stderr else "Unknown error"
            return False, f"Failed to copy files: {stderr}"

        # Unmount encrypted filesystem
        unmount_encrypted()

        # Remove encrypted storage
        shutil.rmtree(ENCRYPTED_DIR, ignore_errors=True)

        # Remove mount point and replace with decrypted files
        shutil.rmtree(ROS2_SRC, ignore_errors=True)
        shutil.move(temp_copy, ROS2_SRC)

        # Fix ownership
        subprocess.run(
            ["chown", "-R", "craig:craig", ROS2_SRC],
            capture_output=True,
            timeout=60
        )

        # Update config (v2 format)
        config_v2 = load_security_config_v2()
        config_v2["cm5"]["lock_state"] = "unlocked"
        config_v2["cm5"]["serial"] = ""
        config_v2["last_operation"] = datetime.now().isoformat()
        save_security_config_v2(config_v2)

        return True, "Device unlocked successfully"

    except subprocess.TimeoutExpired:
        return False, "Operation timed out"
    except Exception as e:
        return False, str(e)


if __name__ == "__main__":
    # Test functions
    print(f"CPU Serial: {get_cpu_serial()}")
    print(f"Device ID: {get_device_id()}")
    print(f"Crawler Type: {get_crawler_type()}")
    print(f"Relevant Devices: {get_relevant_devices()}")
    print(f"RoboClaw Serial: {get_roboclaw_serial()}")
    print(f"RoboClaw EEPROM ID: {read_roboclaw_id()}")
    print(f"ClearLink MAC: {get_clearlink_mac()}")
    print(f"Security Config (v2): {load_security_config_v2()}")
    print(f"Device Lock Status: {check_all_device_locks()}")
