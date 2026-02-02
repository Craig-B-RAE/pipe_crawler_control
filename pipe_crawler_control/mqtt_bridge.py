#!/usr/bin/env python3
"""
MQTT Bridge Node

Bridges ROS2 topics to an MQTT cloud dashboard.
Publishes heartbeat, status, and settings; subscribes to remote commands.
Config loaded from /etc/crawler/mqtt.conf.
"""

import os
import ssl
import json
import time
import socket
import subprocess
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

MQTT_CONF_PATH = '/etc/crawler/mqtt.conf'


def load_mqtt_config():
    """Load MQTT config from /etc/crawler/mqtt.conf."""
    defaults = {
        'broker': 'edge-primeinspections.com',
        'port': 443,
        'use_tls': True,
        'transport': 'websockets',
        'ws_path': '/mqtt',
        'hostname': '',
        'api_key': '',
        'publish_interval': 2.0,
        'enabled': False,
    }
    try:
        with open(MQTT_CONF_PATH, 'r') as f:
            conf = yaml.safe_load(f) or {}
        defaults.update(conf)
    except FileNotFoundError:
        pass
    # If hostname not set, use system hostname
    if not defaults['hostname']:
        defaults['hostname'] = socket.gethostname()
    return defaults


class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge')
        self.config = load_mqtt_config()
        self.mqtt_client = None
        self.connected = False
        self._last_status = {}
        self._last_cpu_temp = 0.0
        self._last_imu_roll = 0.0
        self._last_imu_pitch = 0.0
        self._last_robot_command = ''

        if not MQTT_AVAILABLE:
            self.get_logger().error('paho-mqtt not installed. MQTT bridge disabled.')
            return

        if not self.config['enabled']:
            self.get_logger().info('MQTT bridge disabled in config. Waiting for enable.')
            # Still create a timer to re-check config periodically
            self.create_timer(10.0, self._check_config_reload)
            return

        self._setup_subscriptions()
        self._setup_mqtt()

    def _setup_subscriptions(self):
        """Subscribe to ROS2 topics for status data."""
        self.create_subscription(Float32, '/cpu_temp', self._on_cpu_temp, 10)
        self.create_subscription(Float32, '/imu/roll', self._on_imu_roll, 10)
        self.create_subscription(Float32, '/imu/pitch', self._on_imu_pitch, 10)
        self.create_subscription(String, '/robot_command', self._on_robot_command, 10)

        # Try clearlink status if available
        try:
            from clearlink_interfaces.msg import MotorStatus
            self.create_subscription(MotorStatus, '/clearlink/status', self._on_clearlink_status, 10)
            self._has_clearlink = True
        except ImportError:
            self._has_clearlink = False

    def _setup_mqtt(self):
        """Initialize and connect MQTT client."""
        hostname = self.config['hostname']
        broker = self.config['broker']
        port = self.config['port']
        api_key = self.config['api_key']

        if not api_key:
            self.get_logger().warn('No API key configured. MQTT bridge will not connect.')
            return

        transport = self.config.get('transport', 'tcp')
        client_kwargs = dict(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'crawler-{hostname}',
            protocol=mqtt.MQTTv311,
            transport=transport,
        )
        self.mqtt_client = mqtt.Client(**client_kwargs)

        if transport == 'websockets':
            ws_path = self.config.get('ws_path', '/mqtt')
            self.mqtt_client.ws_set_options(path=ws_path)

        self.mqtt_client.username_pw_set(f'crawler_{hostname}', api_key)

        if self.config['use_tls']:
            self.mqtt_client.tls_set(
                cert_reqs=ssl.CERT_REQUIRED,
                tls_version=ssl.PROTOCOL_TLS_CLIENT,
            )

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=60)

        self.get_logger().info(f'Connecting to MQTT broker {broker}:{port} as {hostname}')
        try:
            self.mqtt_client.connect_async(broker, port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'MQTT connect failed: {e}')

        # Timers for publishing
        interval = self.config['publish_interval']
        self.create_timer(5.0, self._publish_heartbeat)
        self.create_timer(interval, self._publish_status)

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        rc_val = rc.value if hasattr(rc, 'value') else rc
        if rc_val == 0:
            self.connected = True
            hostname = self.config['hostname']
            self.get_logger().info('MQTT connected')
            # Subscribe to command topics
            client.subscribe(f'crawler/{hostname}/cmd/#', qos=1)
        else:
            self.get_logger().error(f'MQTT connect failed with code {rc_val} ({rc})')

    def _on_mqtt_disconnect(self, client, userdata, flags, rc, properties=None):
        self.connected = False
        if rc != 0:
            self.get_logger().warn(f'MQTT disconnected unexpectedly (rc={rc}), will reconnect')

    def _on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT commands."""
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode())
        except (json.JSONDecodeError, UnicodeDecodeError):
            payload = msg.payload.decode()

        hostname = self.config['hostname']
        cmd_prefix = f'crawler/{hostname}/cmd/'

        if not topic.startswith(cmd_prefix):
            return

        command = topic[len(cmd_prefix):]
        self.get_logger().info(f'MQTT command received: {command}')

        if command == 'get_settings':
            self._handle_get_settings()
        elif command == 'jog':
            self._handle_jog(payload)
        elif command == 'stop':
            self._handle_stop()
        elif command == 'get_support_report':
            self._handle_get_support_report()

    # --- ROS2 topic callbacks ---

    def _on_cpu_temp(self, msg):
        self._last_cpu_temp = msg.data

    def _on_imu_roll(self, msg):
        self._last_imu_roll = msg.data

    def _on_imu_pitch(self, msg):
        self._last_imu_pitch = msg.data

    def _on_robot_command(self, msg):
        self._last_robot_command = msg.data

    def _on_clearlink_status(self, msg):
        self._last_status = {
            'connected': msg.connected,
            'axis1_velocity': msg.axis1_velocity,
            'axis2_velocity': msg.axis2_velocity,
            'axis1_torque': msg.axis1_torque,
            'axis2_torque': msg.axis2_torque,
        }

    # --- MQTT publishing ---

    def _publish_heartbeat(self):
        if not self.connected or not self.mqtt_client:
            return
        hostname = self.config['hostname']
        payload = json.dumps({
            'hostname': hostname,
            'timestamp': time.time(),
            'uptime': self._get_uptime(),
        })
        self.mqtt_client.publish(
            f'crawler/{hostname}/heartbeat',
            payload, qos=0
        )

    def _publish_status(self):
        if not self.connected or not self.mqtt_client:
            return
        hostname = self.config['hostname']
        status = {
            'hostname': hostname,
            'timestamp': time.time(),
            'cpu_temp': round(self._last_cpu_temp, 1),
            'imu_roll': round(self._last_imu_roll, 2),
            'imu_pitch': round(self._last_imu_pitch, 2),
            'last_command': self._last_robot_command,
        }
        if self._last_status:
            status['motor'] = self._last_status
        self.mqtt_client.publish(
            f'crawler/{hostname}/status',
            json.dumps(status), qos=0
        )

    # --- Command handlers ---

    def _handle_get_settings(self):
        """Publish current settings to MQTT."""
        hostname = self.config['hostname']
        try:
            # Read crawler settings from the settings file
            settings_file = os.path.expanduser('~/ros2_ws/src/pipe_crawler_control/config/settings.yaml')
            if os.path.exists(settings_file):
                with open(settings_file, 'r') as f:
                    settings = yaml.safe_load(f) or {}
            else:
                settings = {}
        except Exception:
            settings = {}

        self.mqtt_client.publish(
            f'crawler/{hostname}/settings',
            json.dumps(settings), qos=1
        )

    def _handle_jog(self, payload):
        """Forward jog command to ROS2."""
        if isinstance(payload, dict):
            direction = payload.get('direction', 'forward')
        else:
            direction = str(payload)
        # Publish to ROS2 robot_command topic
        pub = self.create_publisher(String, '/robot_command', 10)
        msg = String()
        msg.data = direction
        pub.publish(msg)
        self.get_logger().info(f'MQTT jog command: {direction}')

    def _handle_stop(self):
        """Forward stop command to ROS2."""
        pub = self.create_publisher(String, '/robot_command', 10)
        msg = String()
        msg.data = 'stop'
        pub.publish(msg)
        self.get_logger().info('MQTT stop command')

    def _handle_get_support_report(self):
        """Generate and publish a support report."""
        hostname = self.config['hostname']
        try:
            result = subprocess.run(
                ['curl', '-s', 'http://localhost/api/support/system_info'],
                capture_output=True, text=True, timeout=30
            )
            report = result.stdout
        except Exception as e:
            report = json.dumps({'error': str(e)})

        self.mqtt_client.publish(
            f'crawler/{hostname}/support_report',
            report, qos=1
        )

    # --- Utilities ---

    def _get_uptime(self):
        try:
            with open('/proc/uptime', 'r') as f:
                return float(f.read().split()[0])
        except Exception:
            return 0.0

    def _check_config_reload(self):
        """Periodically check if config has been enabled."""
        new_config = load_mqtt_config()
        if new_config['enabled'] and not self.config['enabled']:
            self.get_logger().info('MQTT enabled in config, starting bridge...')
            self.config = new_config
            self._setup_subscriptions()
            self._setup_mqtt()
        self.config = new_config

    def destroy_node(self):
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
