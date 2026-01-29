"""
Pipe Crawler Launch File

Reads ~/active_system to determine which configuration to use,
then launches the appropriate driver and control nodes.
"""

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def load_system_config():
    """Load the active system configuration."""
    active_system_file = os.path.expanduser('~/active_system')

    # Default to magnetic_phased_array if no active system set
    if not os.path.exists(active_system_file):
        print(f'Warning: {active_system_file} not found. Run setup.sh to configure.')
        print('Defaulting to magnetic_phased_array configuration.')
        system_id = 'magnetic_phased_array'
    else:
        with open(active_system_file, 'r') as f:
            system_id = f.read().strip()

    # Find config file
    possible_paths = [
        os.path.join(
            os.path.expanduser('~/ros2_ws/install/pipe_crawler_control/share/pipe_crawler_control/config'),
            f'{system_id}.yaml'
        ),
        os.path.join(
            os.path.expanduser('~/ros2_ws/src/pipe_crawler_control/config'),
            f'{system_id}.yaml'
        ),
    ]

    config_file = None
    for path in possible_paths:
        if os.path.exists(path):
            config_file = path
            break

    if not config_file:
        raise FileNotFoundError(f'Config file not found for system: {system_id}')

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    return config


def generate_launch_description():
    """Generate the launch description based on active system config."""

    # Load configuration
    config = load_system_config()

    system_name = config['system']['name']
    system_id = config['system']['id']
    driver_type = config['driver']['type']
    driver_config = config['driver']

    nodes = []

    # Log which system we're launching
    nodes.append(
        LogInfo(msg=f'Launching Pipe Crawler: {system_name} ({system_id})')
    )
    nodes.append(
        LogInfo(msg=f'Driver type: {driver_type}')
    )

    # Launch appropriate driver based on type
    if driver_type == 'roboclaw':
        nodes.append(
            Node(
                package='roboclaw_driver2',
                executable='roboclaw_node',
                name='roboclaw',
                parameters=[{
                    'dev_names': driver_config.get('device', '/dev/ttyACM0'),
                    'baud_rate': driver_config.get('baud_rate', 115200),
                    'address': driver_config.get('address', 128),
                    'loop_hz': driver_config.get('loop_hz', 10),
                    'deadman_secs': driver_config.get('deadman_secs', 3),
                    'speed_cmd_topic': 'speed_command',
                    'stats_topic': 'stats',
                    'test_mode': False
                }]
            )
        )

    elif driver_type == 'clearlink':
        # ClearLink EtherNet/IP motor controller
        nodes.append(
            LogInfo(msg=f"ClearLink driver connecting to {driver_config.get('ip_address', '192.168.1.100')}")
        )
        nodes.append(
            Node(
                package='clearlink_driver',
                executable='clearlink_node',
                name='clearlink',
                parameters=[{
                    'ip_address': driver_config.get('ip_address', '192.168.1.100'),
                    'port': driver_config.get('port', 44818),
                    'num_axes': driver_config.get('num_axes', 4),
                    'loop_hz': driver_config.get('loop_hz', 10),
                    'deadman_secs': driver_config.get('deadman_secs', 3),
                }]
            )
        )

    # Motor controller (unified controller that reads config)
    nodes.append(
        Node(
            package='pipe_crawler_control',
            executable='motor_controller',
            name='motor_controller'
        )
    )

    # CPU temperature publisher
    nodes.append(
        Node(
            package='pipe_crawler_control',
            executable='cpu_temp_publisher',
            name='cpu_temp_publisher'
        )
    )

    # Update manager
    nodes.append(
        Node(
            package='pipe_crawler_control',
            executable='update_manager',
            name='update_manager'
        )
    )

    # WT901 IMU sensor
    nodes.append(
        Node(
            package='pipe_crawler_control',
            executable='wt901_imu',
            name='wt901_imu',
            parameters=[{
                'port': '/dev/ttyAMA0',
                'baud_rate': 9600,
                'frame_id': 'imu_link'
            }]
        )
    )

    # Motor data logger (for support report graphs)
    nodes.append(
        Node(
            package='pipe_crawler_control',
            executable='motor_data_logger',
            name='motor_data_logger'
        )
    )

    return LaunchDescription(nodes)
