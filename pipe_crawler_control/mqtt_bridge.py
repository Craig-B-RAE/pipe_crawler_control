#!/usr/bin/env python3
"""
MQTT Bridge Node

Bridges ROS2 topics to an MQTT cloud dashboard.
Publishes heartbeat, status, and settings; subscribes to remote commands.
Config loaded from /etc/crawler/mqtt.conf.
"""

import math
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


NETWORK_CONF_PATH = '/etc/xpresscan/network.conf'

DEFAULT_TOPICS = {
    'heartbeat': {'enabled': True, 'interval': 5.0},
    'system': {'enabled': True, 'interval': 5.0, 'fields': {
        'cpu_temp': True, 'ram': True, 'storage': True, 'revision': True,
    }},
    'imu': {'enabled': True, 'interval': 2.0, 'fields': {
        'roll': True, 'pitch': True, 'cal_roll': True, 'cal_pitch': True,
    }},
    'motor': {'enabled': True, 'interval': 2.0, 'fields': {
        'connected': True, 'm1_speed': True, 'm2_speed': True,
        'm1_torque': True, 'm2_torque': True, 'last_command': True,
    }},
}


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
    # Merge topic defaults
    topics = defaults.get('topics', {})
    for name, defs in DEFAULT_TOPICS.items():
        if name not in topics:
            topics[name] = dict(defs)
        else:
            for k, v in defs.items():
                if k not in topics[name]:
                    topics[name][k] = v
    defaults['topics'] = topics
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
        self._crawler_name = self._load_crawler_name()
        self._revision = self._load_revision()
        self._imu_offsets = self._load_imu_offsets()
        self._motor_params = self._load_motor_params()
        self._steps_per_metre = self._calc_steps_per_metre()
        self._peak_torque_nm = self._motor_params.get('peakTorqueNm', 2.0)

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

        # Timers for publishing (per-topic config)
        topics = self.config.get('topics', DEFAULT_TOPICS)
        hb = topics.get('heartbeat', DEFAULT_TOPICS['heartbeat'])
        if hb.get('enabled', True):
            self.create_timer(hb.get('interval', 5.0), self._publish_heartbeat)
        sys_t = topics.get('system', DEFAULT_TOPICS['system'])
        if sys_t.get('enabled', True):
            self.create_timer(sys_t.get('interval', 5.0), self._publish_system)
        imu_t = topics.get('imu', DEFAULT_TOPICS['imu'])
        if imu_t.get('enabled', True):
            self.create_timer(imu_t.get('interval', 2.0), self._publish_imu)
        motor_t = topics.get('motor', DEFAULT_TOPICS['motor'])
        if motor_t.get('enabled', True):
            self.create_timer(motor_t.get('interval', 2.0), self._publish_motor)

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
        topics = self.config.get('topics', DEFAULT_TOPICS)
        hb_conf = topics.get('heartbeat', {})
        data = {
            'hostname': hostname,
            'crawler_name': self._crawler_name,
            'timestamp': time.time(),
            'uptime': self._get_uptime(),
        }
        if hb_conf.get('iccid_enabled') and hb_conf.get('iccid'):
            data['iccid'] = hb_conf['iccid']
        self.mqtt_client.publish(
            f'crawler/{hostname}/heartbeat',
            json.dumps(data), qos=0
        )

    def _publish_system(self):
        if not self.connected or not self.mqtt_client:
            return
        hostname = self.config['hostname']
        topics = self.config.get('topics', DEFAULT_TOPICS)
        fields = topics.get('system', {}).get('fields', {})
        data = {
            'hostname': hostname,
            'timestamp': time.time(),
        }
        if fields.get('cpu_temp', True):
            data['cpu_temp'] = round(self._last_cpu_temp, 1)
        if fields.get('ram', True):
            data['ram'] = self._get_ram_usage()
        if fields.get('storage', True):
            data['storage'] = self._get_disk_usage()
        if fields.get('revision', True):
            data['revision'] = self._revision
        self.mqtt_client.publish(
            f'crawler/{hostname}/system',
            json.dumps(data), qos=0
        )

    def _publish_imu(self):
        if not self.connected or not self.mqtt_client:
            return
        hostname = self.config['hostname']
        topics = self.config.get('topics', DEFAULT_TOPICS)
        fields = topics.get('imu', {}).get('fields', {})
        data = {
            'hostname': hostname,
            'timestamp': time.time(),
        }
        if fields.get('roll', True):
            data['roll'] = round(self._last_imu_roll, 2)
        if fields.get('pitch', True):
            data['pitch'] = round(self._last_imu_pitch, 2)
        if fields.get('cal_roll', True):
            data['cal_roll'] = round(
                self._last_imu_roll - self._imu_offsets['roll'], 2)
        if fields.get('cal_pitch', True):
            data['cal_pitch'] = round(
                self._last_imu_pitch - self._imu_offsets['pitch'], 2)
        self.mqtt_client.publish(
            f'crawler/{hostname}/imu',
            json.dumps(data), qos=0
        )

    def _publish_motor(self):
        if not self.connected or not self.mqtt_client:
            return
        hostname = self.config['hostname']
        topics = self.config.get('topics', DEFAULT_TOPICS)
        fields = topics.get('motor', {}).get('fields', {})
        data = {
            'hostname': hostname,
            'timestamp': time.time(),
        }
        if self._last_status:
            if fields.get('connected', True):
                data['connected'] = self._last_status.get('connected')
            if fields.get('m1_speed', True):
                data['m1_speed'] = self._steps_to_ms(
                    self._last_status.get('axis1_velocity', 0))
            if fields.get('m2_speed', True):
                data['m2_speed'] = self._steps_to_ms(
                    self._last_status.get('axis2_velocity', 0))
            if fields.get('m1_torque', True):
                data['m1_torque'] = self._torque_to_nm(
                    self._last_status.get('axis1_torque', 0))
            if fields.get('m2_torque', True):
                data['m2_torque'] = self._torque_to_nm(
                    self._last_status.get('axis2_torque', 0))
        if fields.get('last_command', True):
            data['last_command'] = self._last_robot_command
        self.mqtt_client.publish(
            f'crawler/{hostname}/motor',
            json.dumps(data), qos=0
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

    def _load_crawler_name(self):
        """Load crawler name from /etc/xpresscan/network.conf."""
        try:
            with open(NETWORK_CONF_PATH, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('CRAWLER_NAME='):
                        return line.split('=', 1)[1].strip()
        except Exception:
            pass
        return socket.gethostname()

    def _load_revision(self):
        """Load software revision from VERSION file."""
        try:
            version_file = os.path.expanduser(
                '~/ros2_ws/src/pipe_crawler_control/VERSION')
            with open(version_file, 'r') as f:
                return f.read().strip()
        except Exception:
            return 'unknown'

    def _load_imu_offsets(self):
        """Load IMU calibration offsets from UI settings file."""
        defaults = {'roll': 0.0, 'pitch': 0.0}
        try:
            active_file = os.path.expanduser('~/active_system')
            with open(active_file, 'r') as f:
                crawler_id = f.read().strip()
            settings_file = f'/etc/xpresscan/ui_settings_{crawler_id}.json'
            with open(settings_file, 'r') as f:
                settings = json.load(f)
            defaults['roll'] = float(settings.get('imuRollOffset', 0.0))
            defaults['pitch'] = float(settings.get('imuPitchOffset', 0.0))
            self.get_logger().info(
                f'IMU offsets loaded: roll={defaults["roll"]}, '
                f'pitch={defaults["pitch"]}')
        except Exception as e:
            self.get_logger().warn(
                f'Could not load IMU offsets, using 0: {e}')
        return defaults

    def _load_motor_params(self):
        """Load motor parameters from UI settings file."""
        defaults = {
            'encoderRes': 1000,
            'wheelDia': 1.77,       # inches
            'gearboxRatio': 20,
            'gearRatio': 30,
            'peakTorqueNm': 2.0,
        }
        try:
            active_file = os.path.expanduser('~/active_system')
            with open(active_file, 'r') as f:
                crawler_id = f.read().strip()
            settings_file = f'/etc/xpresscan/ui_settings_{crawler_id}.json'
            with open(settings_file, 'r') as f:
                settings = json.load(f)
            if 'motorParams' in settings:
                defaults.update(settings['motorParams'])
                self.get_logger().info(
                    f'Motor params loaded: encoderRes={defaults["encoderRes"]}, '
                    f'wheelDia={defaults["wheelDia"]}, '
                    f'gearboxRatio={defaults["gearboxRatio"]}, '
                    f'gearRatio={defaults["gearRatio"]}')
        except Exception as e:
            self.get_logger().warn(
                f'Could not load motor params, using defaults: {e}')
        return defaults

    def _calc_steps_per_metre(self):
        """Calculate steps per metre from motor parameters."""
        p = self._motor_params
        total_ratio = p['gearboxRatio'] * p['gearRatio']
        wheel_circ_in = math.pi * p['wheelDia']
        travel_per_rev_in = wheel_circ_in / total_ratio
        steps_per_inch = p['encoderRes'] / travel_per_rev_in
        return steps_per_inch / 0.0254  # convert steps/inch to steps/metre

    def _steps_to_ms(self, steps_per_sec):
        """Convert steps/sec to m/s."""
        if not steps_per_sec or self._steps_per_metre == 0:
            return 0.0
        return round(abs(steps_per_sec) / self._steps_per_metre, 4)

    def _torque_to_nm(self, torque_percent):
        """Convert torque percentage to Nm."""
        if torque_percent is None or torque_percent == -9999:
            return None
        return round(abs(torque_percent) / 100.0 * self._peak_torque_nm, 2)

    def _get_uptime(self):
        try:
            with open('/proc/uptime', 'r') as f:
                return float(f.read().split()[0])
        except Exception:
            return 0.0

    def _get_ram_usage(self):
        """Return RAM usage dict from /proc/meminfo."""
        try:
            with open('/proc/meminfo', 'r') as f:
                info = {}
                for line in f:
                    parts = line.split()
                    if parts[0] in ('MemTotal:', 'MemAvailable:'):
                        info[parts[0]] = int(parts[1])
                total_kb = info.get('MemTotal:', 0)
                avail_kb = info.get('MemAvailable:', 0)
                used_kb = total_kb - avail_kb
                return {
                    'percent': round(100.0 * used_kb / total_kb, 1) if total_kb else 0,
                    'used_gb': round(used_kb / 1048576.0, 2),
                    'total_gb': round(total_kb / 1048576.0, 2),
                }
        except Exception:
            return {'percent': 0, 'used_gb': 0, 'total_gb': 0}

    def _get_disk_usage(self):
        """Return disk usage dict from os.statvfs."""
        try:
            st = os.statvfs('/')
            total = st.f_blocks * st.f_frsize
            free = st.f_bfree * st.f_frsize
            used = total - free
            return {
                'percent': round(100.0 * used / total, 1) if total else 0,
                'used_gb': round(used / 1073741824.0, 2),
                'total_gb': round(total / 1073741824.0, 2),
            }
        except Exception:
            return {'percent': 0, 'used_gb': 0, 'total_gb': 0}

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
