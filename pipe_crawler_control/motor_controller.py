#!/usr/bin/env python3
"""
Unified Motor Controller for Pipe Crawler Systems

Reads configuration from ~/active_system and loads the corresponding
YAML config file to determine driver type, motor count, and parameters.
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

# Import driver-specific messages
from roboclaw_interfaces.msg import SpeedCommand

# Try to import ClearLink messages (may not be built yet)
try:
    from clearlink_interfaces.msg import MotorCommand as ClearLinkCommand
    CLEARLINK_AVAILABLE = True
except ImportError:
    CLEARLINK_AVAILABLE = False


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Load configuration
        self.config = self.load_config()
        if not self.config:
            self.get_logger().error('Failed to load configuration. Shutting down.')
            raise RuntimeError('Configuration load failed')

        self.system_name = self.config['system']['name']
        self.system_id = self.config['system']['id']
        self.driver_type = self.config['driver']['type']
        self.motors = self.config['motors']

        self.get_logger().info(f'Starting Motor Controller for: {self.system_name}')
        self.get_logger().info(f'Driver type: {self.driver_type}')
        self.get_logger().info(f'Motor count: {len(self.motors)}')

        # Speed/ramp state (percentage 0-100)
        self.speed_percent = 50
        self.ramp_percent = 50

        # Set up publisher based on driver type
        self.setup_driver()

        # Subscribers for commands
        self.cmd_sub = self.create_subscription(
            String, '/robot_command',
            self.command_callback, 10
        )

        self.speed_sub = self.create_subscription(
            String, '/robot_speed',
            self.speed_callback, 10
        )

        self.ramp_sub = self.create_subscription(
            String, '/robot_ramp',
            self.ramp_callback, 10
        )

        self.log_startup_info()

    def load_config(self):
        """Load configuration based on ~/active_system file."""
        active_system_file = os.path.expanduser('~/active_system')

        # Check if active_system file exists
        if not os.path.exists(active_system_file):
            self.get_logger().error(f'Active system file not found: {active_system_file}')
            self.get_logger().error('Run setup.sh to select a system configuration.')
            return None

        # Read the active system ID
        with open(active_system_file, 'r') as f:
            system_id = f.read().strip()

        if not system_id:
            self.get_logger().error('Active system file is empty.')
            return None

        self.get_logger().info(f'Active system: {system_id}')

        # Find the config file
        # Try multiple locations for the config file
        possible_paths = [
            # Installed location (after colcon build)
            os.path.join(
                os.path.expanduser('~/ros2_ws/install/pipe_crawler_control/share/pipe_crawler_control/config'),
                f'{system_id}.yaml'
            ),
            # Source location (during development)
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
            self.get_logger().error(f'Config file not found for system: {system_id}')
            self.get_logger().error(f'Searched: {possible_paths}')
            return None

        self.get_logger().info(f'Loading config: {config_file}')

        # Load and parse YAML
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        return config

    def setup_driver(self):
        """Set up publisher based on driver type."""
        if self.driver_type == 'roboclaw':
            self.speed_pub = self.create_publisher(SpeedCommand, '/speed_command', 10)
            self.get_logger().info('Using RoboClaw driver')

        elif self.driver_type == 'clearlink':
            if CLEARLINK_AVAILABLE:
                self.cmd_pub = self.create_publisher(ClearLinkCommand, '/clearlink/command', 10)
                self.get_logger().info('Using ClearLink driver')
            else:
                # Fallback to stub if clearlink_interfaces not built yet
                self.cmd_pub = self.create_publisher(String, '/clearlink_command', 10)
                self.get_logger().warn('ClearLink interfaces not available - using stub')

        else:
            self.get_logger().error(f'Unknown driver type: {self.driver_type}')
            raise RuntimeError(f'Unknown driver type: {self.driver_type}')

    def log_startup_info(self):
        """Log startup information."""
        self.get_logger().info('Motor Controller ready')
        self.get_logger().info('Commands on /robot_command: forward, backward, stop')
        self.get_logger().info('Speed (0-100) on /robot_speed')
        self.get_logger().info('Ramp (0-100) on /robot_ramp')
        for i, motor in enumerate(self.motors):
            self.get_logger().info(
                f'  Motor {i+1}: {motor["name"]} '
                f'(max={motor["max_speed"]}, invert={motor["invert"]})'
            )

    def command_callback(self, msg):
        """Handle movement commands."""
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'Received command: {cmd}')

        if cmd == 'forward':
            self.move(direction=1)
        elif cmd == 'backward':
            self.move(direction=-1)
        elif cmd == 'stop':
            self.stop()
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def speed_callback(self, msg):
        """Handle speed adjustment."""
        try:
            speed = int(msg.data)
            if 0 <= speed <= 100:
                old_speed = self.speed_percent
                self.speed_percent = speed
                self.get_logger().info(f'Speed changed from {old_speed}% to {speed}%')
            else:
                self.get_logger().warn(f'Speed must be 0-100, got {speed}')
        except ValueError:
            self.get_logger().warn(f'Invalid speed value: {msg.data}')

    def ramp_callback(self, msg):
        """Handle ramp/acceleration adjustment."""
        try:
            ramp = int(msg.data)
            if 0 <= ramp <= 100:
                old_ramp = self.ramp_percent
                self.ramp_percent = ramp
                self.get_logger().info(f'Ramp changed from {old_ramp}% to {ramp}%')
            else:
                self.get_logger().warn(f'Ramp must be 0-100, got {ramp}')
        except ValueError:
            self.get_logger().warn(f'Invalid ramp value: {msg.data}')

    def calculate_motor_speed(self, motor_config, direction):
        """Calculate actual speed for a motor based on config and direction."""
        base_speed = motor_config['default_speed']
        actual_speed = int(base_speed * self.speed_percent / 100)

        # Apply direction
        actual_speed *= direction

        # Apply inversion if configured
        if motor_config.get('invert', False):
            actual_speed *= -1

        return actual_speed

    def calculate_accel(self, motor_config):
        """Calculate acceleration based on ramp percentage."""
        max_accel = motor_config.get('max_accel', 10000)
        # 0% ramp = 500 (slow), 100% ramp = max_accel (fast)
        min_accel = 500
        return int(min_accel + (self.ramp_percent / 100) * (max_accel - min_accel))

    def move(self, direction):
        """Move all motors in the specified direction."""
        if self.driver_type == 'roboclaw':
            self.move_roboclaw(direction)
        elif self.driver_type == 'clearlink':
            self.move_clearlink(direction)

    def move_roboclaw(self, direction):
        """Send movement command to RoboClaw driver."""
        msg = SpeedCommand()

        # Get motor speeds (support 1 or 2 motors)
        if len(self.motors) >= 1:
            msg.m1_qpps = self.calculate_motor_speed(self.motors[0], direction)
            msg.accel = self.calculate_accel(self.motors[0])

        if len(self.motors) >= 2:
            msg.m2_qpps = self.calculate_motor_speed(self.motors[1], direction)
        else:
            # Single motor system - set M2 to 0
            msg.m2_qpps = 0

        msg.max_secs = 120

        self.speed_pub.publish(msg)

        direction_str = 'forward' if direction > 0 else 'backward'
        self.get_logger().info(
            f'Moving {direction_str} at {self.speed_percent}% '
            f'(M1={msg.m1_qpps}, M2={msg.m2_qpps}, accel={msg.accel})'
        )

    def move_clearlink(self, direction):
        """Send movement command to ClearLink driver."""
        if CLEARLINK_AVAILABLE:
            msg = ClearLinkCommand()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Get motor speeds and map to axes
            for i, motor in enumerate(self.motors):
                speed = self.calculate_motor_speed(motor, direction)
                accel = self.calculate_accel(motor)
                axis = motor.get('axis', i + 1)  # Default to sequential axis

                # Enable and set velocity for each axis
                if axis == 1:
                    msg.axis1_enable = True
                    msg.axis1_steps_per_sec = speed
                elif axis == 2:
                    msg.axis2_enable = True
                    msg.axis2_steps_per_sec = speed
                elif axis == 3:
                    msg.axis3_enable = True
                    msg.axis3_steps_per_sec = speed
                elif axis == 4:
                    msg.axis4_enable = True
                    msg.axis4_steps_per_sec = speed

                msg.acceleration = accel

            msg.max_secs = 120
            self.cmd_pub.publish(msg)

            direction_str = 'forward' if direction > 0 else 'backward'
            self.get_logger().info(
                f'Moving {direction_str} at {self.speed_percent}% via ClearLink'
            )
        else:
            # Fallback to stub
            import json
            command = {
                'action': 'move',
                'direction': direction,
                'motors': []
            }
            for motor in self.motors:
                speed = self.calculate_motor_speed(motor, direction)
                accel = self.calculate_accel(motor)
                command['motors'].append({
                    'name': motor['name'],
                    'speed': speed,
                    'accel': accel
                })
            msg = String()
            msg.data = json.dumps(command)
            self.cmd_pub.publish(msg)
            direction_str = 'forward' if direction > 0 else 'backward'
            self.get_logger().info(f'ClearLink stub: Moving {direction_str} at {self.speed_percent}%')

    def stop(self):
        """Stop all motors."""
        if self.driver_type == 'roboclaw':
            self.stop_roboclaw()
        elif self.driver_type == 'clearlink':
            self.stop_clearlink()

    def stop_roboclaw(self):
        """Stop motors via RoboClaw driver."""
        msg = SpeedCommand()
        msg.m1_qpps = 0
        msg.m2_qpps = 0
        msg.accel = 5000
        msg.max_secs = 1
        self.speed_pub.publish(msg)
        self.get_logger().info('Stopping')

    def stop_clearlink(self):
        """Stop motors via ClearLink driver."""
        if CLEARLINK_AVAILABLE:
            msg = ClearLinkCommand()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Set all velocities to 0 to stop
            for motor in self.motors:
                axis = motor.get('axis', 1)
                if axis == 1:
                    msg.axis1_enable = True
                    msg.axis1_steps_per_sec = 0
                elif axis == 2:
                    msg.axis2_enable = True
                    msg.axis2_steps_per_sec = 0
                elif axis == 3:
                    msg.axis3_enable = True
                    msg.axis3_steps_per_sec = 0
                elif axis == 4:
                    msg.axis4_enable = True
                    msg.axis4_steps_per_sec = 0

            msg.acceleration = 50000  # Fast deceleration for stop
            msg.max_secs = 1
            self.cmd_pub.publish(msg)
            self.get_logger().info('Stopping via ClearLink')
        else:
            # Fallback to stub
            import json
            command = {
                'action': 'stop',
                'motors': [motor['name'] for motor in self.motors]
            }
            msg = String()
            msg.data = json.dumps(command)
            self.cmd_pub.publish(msg)
            self.get_logger().info('ClearLink stub: Stopping')


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = MotorController()
        rclpy.spin(controller)
    except RuntimeError as e:
        print(f'Motor Controller failed to start: {e}')
        return 1
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    exit(main())
