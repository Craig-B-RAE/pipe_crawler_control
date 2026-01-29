#!/usr/bin/env python3
"""
Motor Data Logger Service

Background service that logs motor data every second for historical graphs.
Maintains a rolling 1-hour buffer of data points.
Data is stored in a SQLite database for persistence and efficient querying.
"""

import os
import sys
import time
import json
import sqlite3
import threading
from datetime import datetime, timedelta
from collections import deque

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

# Try to import ClearLink message type
try:
    from clearlink_interfaces.msg import MotorStatus as ClearlinkMotorStatus
    CLEARLINK_AVAILABLE = True
except ImportError:
    CLEARLINK_AVAILABLE = False

# Try to import RoboClaw message type
try:
    from roboclaw_interfaces.msg import RoboClawStatus
    ROBOCLAW_AVAILABLE = True
except ImportError:
    ROBOCLAW_AVAILABLE = False


# Constants
DB_PATH = "/var/lib/crawler/motor_data.db"
MAX_AGE_SECONDS = 3600  # 1 hour of data
SAMPLE_INTERVAL = 1.0   # Sample every 1 second
CLEANUP_INTERVAL = 300  # Cleanup old data every 5 minutes

# Motor constants for conversions
MOTOR_PEAK_TORQUE_NM = 2.0  # CPM-SDHP-2311S-ELN @ 60V


def ensure_db_directory():
    """Ensure the database directory exists."""
    db_dir = os.path.dirname(DB_PATH)
    if not os.path.exists(db_dir):
        os.makedirs(db_dir, mode=0o755)


def init_database():
    """Initialize the SQLite database with required tables."""
    ensure_db_directory()
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # Create motor data table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS motor_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            motor_driver TEXT,
            m0_velocity_steps INTEGER DEFAULT 0,
            m0_velocity_in_s REAL DEFAULT 0.0,
            m1_velocity_steps INTEGER DEFAULT 0,
            m1_velocity_in_s REAL DEFAULT 0.0,
            m0_torque_percent REAL DEFAULT 0.0,
            m0_torque_nm REAL DEFAULT 0.0,
            m1_torque_percent REAL DEFAULT 0.0,
            m1_torque_nm REAL DEFAULT 0.0,
            m0_position INTEGER DEFAULT 0,
            m1_position INTEGER DEFAULT 0,
            m0_enabled BOOLEAN DEFAULT 0,
            m1_enabled BOOLEAN DEFAULT 0,
            m0_moving BOOLEAN DEFAULT 0,
            m1_moving BOOLEAN DEFAULT 0,
            m0_fault BOOLEAN DEFAULT 0,
            m1_fault BOOLEAN DEFAULT 0,
            cpu_temp REAL DEFAULT 0.0,
            command_active TEXT DEFAULT 'none'
        )
    ''')

    # Create index for timestamp queries
    cursor.execute('''
        CREATE INDEX IF NOT EXISTS idx_timestamp ON motor_data(timestamp)
    ''')

    # Create command history table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS command_history (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
            command TEXT,
            speed_in_s REAL,
            ramp_percent REAL,
            result TEXT DEFAULT 'pending'
        )
    ''')

    conn.commit()
    conn.close()


def cleanup_old_data():
    """Remove data older than MAX_AGE_SECONDS."""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cutoff = datetime.now() - timedelta(seconds=MAX_AGE_SECONDS)
        cursor.execute('DELETE FROM motor_data WHERE timestamp < ?', (cutoff,))
        cursor.execute('DELETE FROM command_history WHERE timestamp < ?', (cutoff,))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[motor_data_logger] Cleanup error: {e}")


def get_motor_data(time_range_seconds=3600):
    """
    Query motor data for the specified time range.
    Returns list of dictionaries with motor data.
    """
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cutoff = datetime.now() - timedelta(seconds=time_range_seconds)
        cursor.execute('''
            SELECT * FROM motor_data
            WHERE timestamp > ?
            ORDER BY timestamp ASC
        ''', (cutoff,))

        rows = cursor.fetchall()
        conn.close()

        return [dict(row) for row in rows]
    except Exception as e:
        print(f"[motor_data_logger] Query error: {e}")
        return []


def get_command_history(limit=10):
    """Get recent command history."""
    try:
        conn = sqlite3.connect(DB_PATH)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute('''
            SELECT * FROM command_history
            ORDER BY timestamp DESC
            LIMIT ?
        ''', (limit,))

        rows = cursor.fetchall()
        conn.close()

        return [dict(row) for row in rows]
    except Exception as e:
        print(f"[motor_data_logger] Command history error: {e}")
        return []


def log_command(command, speed_in_s=0.0, ramp_percent=0.0, result='success'):
    """Log a motor command to history."""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute('''
            INSERT INTO command_history (command, speed_in_s, ramp_percent, result)
            VALUES (?, ?, ?, ?)
        ''', (command, speed_in_s, ramp_percent, result))
        conn.commit()
        conn.close()
    except Exception as e:
        print(f"[motor_data_logger] Log command error: {e}")


class MotorDataLoggerNode(Node):
    """ROS2 node that subscribes to motor status and logs data."""

    def __init__(self):
        super().__init__('motor_data_logger')

        # Determine motor driver from config
        self.motor_driver = self._detect_motor_driver()
        self.get_logger().info(f"Motor data logger starting for driver: {self.motor_driver}")

        # Current data state
        self.current_data = {
            'motor_driver': self.motor_driver,
            'm0_velocity_steps': 0,
            'm0_velocity_in_s': 0.0,
            'm1_velocity_steps': 0,
            'm1_velocity_in_s': 0.0,
            'm0_torque_percent': 0.0,
            'm0_torque_nm': 0.0,
            'm1_torque_percent': 0.0,
            'm1_torque_nm': 0.0,
            'm0_position': 0,
            'm1_position': 0,
            'm0_enabled': False,
            'm1_enabled': False,
            'm0_moving': False,
            'm1_moving': False,
            'm0_fault': False,
            'm1_fault': False,
            'cpu_temp': 0.0,
            'command_active': 'none'
        }

        # Steps per inch conversion (will be updated from UI settings if available)
        self.steps_per_inch = 53950  # Default for EdgeFlex

        # Load steps_per_inch from UI settings if available
        self._load_motor_params()

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribe to motor status based on driver type
        if self.motor_driver == 'clearlink' and CLEARLINK_AVAILABLE:
            self.clearlink_sub = self.create_subscription(
                ClearlinkMotorStatus,
                '/clearlink/status',
                self.clearlink_callback,
                qos
            )
            self.get_logger().info("Subscribed to /clearlink/status")
        elif self.motor_driver == 'roboclaw' and ROBOCLAW_AVAILABLE:
            self.roboclaw_sub = self.create_subscription(
                RoboClawStatus,
                '/roboclaw/status',
                self.roboclaw_callback,
                qos
            )
            self.get_logger().info("Subscribed to /roboclaw/status")

        # Subscribe to CPU temperature
        self.cpu_temp_sub = self.create_subscription(
            Float32,
            '/cpu_temp',
            self.cpu_temp_callback,
            qos
        )

        # Subscribe to robot commands for command history
        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            qos
        )

        # Timer to log data every second
        self.log_timer = self.create_timer(SAMPLE_INTERVAL, self.log_data)

        # Timer to cleanup old data every 5 minutes
        self.cleanup_timer = self.create_timer(CLEANUP_INTERVAL, cleanup_old_data)

        self.get_logger().info("Motor data logger ready")

    def _detect_motor_driver(self):
        """Detect motor driver from network.conf."""
        try:
            with open('/etc/xpresscan/network.conf', 'r') as f:
                for line in f:
                    if line.startswith('MOTOR_DRIVER='):
                        return line.strip().split('=')[1].lower()
        except:
            pass
        return 'unknown'

    def _load_motor_params(self):
        """Load motor parameters from UI settings."""
        try:
            crawler_id = 'edgeflex_clearlink' if self.motor_driver == 'clearlink' else 'magnetic'
            settings_path = f'/etc/xpresscan/ui_settings_{crawler_id}.json'

            if os.path.exists(settings_path):
                with open(settings_path, 'r') as f:
                    settings = json.load(f)
                    motor_params = settings.get('motorParams', {})
                    self.steps_per_inch = motor_params.get('stepsPerInch', 53950)
                    self.get_logger().info(f"Loaded steps_per_inch: {self.steps_per_inch}")
        except Exception as e:
            self.get_logger().warn(f"Could not load motor params: {e}")

    def _steps_to_in_per_sec(self, steps_per_sec):
        """Convert steps/sec to inches/sec."""
        if self.steps_per_inch == 0:
            return 0.0
        return abs(steps_per_sec) / self.steps_per_inch

    def _torque_percent_to_nm(self, percent):
        """Convert torque percentage to Newton-meters."""
        if percent == -9999:  # N/A value from ClearLink
            return 0.0
        return abs(percent) / 100.0 * MOTOR_PEAK_TORQUE_NM

    def clearlink_callback(self, msg):
        """Handle ClearLink motor status messages."""
        self.current_data['m0_velocity_steps'] = msg.axis1_velocity
        self.current_data['m0_velocity_in_s'] = self._steps_to_in_per_sec(msg.axis1_velocity)
        self.current_data['m1_velocity_steps'] = msg.axis2_velocity
        self.current_data['m1_velocity_in_s'] = self._steps_to_in_per_sec(msg.axis2_velocity)

        self.current_data['m0_torque_percent'] = msg.axis1_torque if msg.axis1_torque != -9999 else 0.0
        self.current_data['m0_torque_nm'] = self._torque_percent_to_nm(msg.axis1_torque)
        self.current_data['m1_torque_percent'] = msg.axis2_torque if msg.axis2_torque != -9999 else 0.0
        self.current_data['m1_torque_nm'] = self._torque_percent_to_nm(msg.axis2_torque)

        self.current_data['m0_position'] = msg.axis1_position
        self.current_data['m1_position'] = msg.axis2_position

        self.current_data['m0_enabled'] = msg.axis1_enabled
        self.current_data['m1_enabled'] = msg.axis2_enabled

        self.current_data['m0_moving'] = msg.axis1_moving
        self.current_data['m1_moving'] = msg.axis1_moving

        self.current_data['m0_fault'] = msg.axis1_fault
        self.current_data['m1_fault'] = msg.axis2_fault

        # Determine command active state
        if msg.axis1_moving or msg.axis2_moving:
            if msg.axis1_velocity > 0:
                self.current_data['command_active'] = 'forward'
            elif msg.axis1_velocity < 0:
                self.current_data['command_active'] = 'reverse'
            else:
                self.current_data['command_active'] = 'moving'
        else:
            self.current_data['command_active'] = 'stopped'

    def roboclaw_callback(self, msg):
        """Handle RoboClaw motor status messages."""
        # RoboClaw status handling - adapt to your message type
        # This is a placeholder - adjust based on actual RoboClaw message fields
        pass

    def cpu_temp_callback(self, msg):
        """Handle CPU temperature messages."""
        self.current_data['cpu_temp'] = msg.data

    def command_callback(self, msg):
        """Handle robot command messages and log to history."""
        command = msg.data
        speed = self.current_data.get('m0_velocity_in_s', 0.0)
        log_command(command, speed_in_s=speed, result='sent')

    def log_data(self):
        """Log current motor data to database."""
        try:
            conn = sqlite3.connect(DB_PATH)
            cursor = conn.cursor()

            cursor.execute('''
                INSERT INTO motor_data (
                    motor_driver,
                    m0_velocity_steps, m0_velocity_in_s,
                    m1_velocity_steps, m1_velocity_in_s,
                    m0_torque_percent, m0_torque_nm,
                    m1_torque_percent, m1_torque_nm,
                    m0_position, m1_position,
                    m0_enabled, m1_enabled,
                    m0_moving, m1_moving,
                    m0_fault, m1_fault,
                    cpu_temp, command_active
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                self.current_data['motor_driver'],
                self.current_data['m0_velocity_steps'],
                self.current_data['m0_velocity_in_s'],
                self.current_data['m1_velocity_steps'],
                self.current_data['m1_velocity_in_s'],
                self.current_data['m0_torque_percent'],
                self.current_data['m0_torque_nm'],
                self.current_data['m1_torque_percent'],
                self.current_data['m1_torque_nm'],
                self.current_data['m0_position'],
                self.current_data['m1_position'],
                self.current_data['m0_enabled'],
                self.current_data['m1_enabled'],
                self.current_data['m0_moving'],
                self.current_data['m1_moving'],
                self.current_data['m0_fault'],
                self.current_data['m1_fault'],
                self.current_data['cpu_temp'],
                self.current_data['command_active']
            ))

            conn.commit()
            conn.close()
        except Exception as e:
            self.get_logger().error(f"Failed to log data: {e}")


def main(args=None):
    """Main entry point."""
    # Initialize database
    init_database()

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        node = MotorDataLoggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
