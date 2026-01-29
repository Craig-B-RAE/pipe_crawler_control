#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class UpdateManager(Node):
    def __init__(self):
        super().__init__('update_manager')
        self.subscription = self.create_subscription(
            String,
            '/system_command',
            self.command_callback,
            10
        )
        self.status_pub = self.create_publisher(String, '/update_status', 10)
        self.get_logger().info('Update Manager started')

    def command_callback(self, msg):
        if msg.data == 'check_update':
            self.check_for_update()
        elif msg.data == 'run_update':
            self.run_update()
        elif msg.data == 'forget_wifi':
            self.forget_wifi()
        elif msg.data == 'reboot':
            self.reboot_system()

    def check_for_update(self):
        try:
            os.chdir('/home/craig/ros2_ws/src/pipe_crawler_control')
            subprocess.run(['git', 'fetch'], capture_output=True)
            result = subprocess.run(['git', 'status', '-uno'], capture_output=True, text=True)
            
            if 'behind' in result.stdout:
                self.publish_status('update_available')
            else:
                self.publish_status('up_to_date')
        except Exception as e:
            self.publish_status(f'error: {str(e)}')

    def run_update(self):
        self.publish_status('updating')
        try:
            result = subprocess.run(
                ['/home/craig/update_crawler.sh'],
                capture_output=True,
                text=True,
                timeout=120
            )
            if result.returncode == 0:
                self.publish_status('update_complete')
            else:
                self.publish_status(f'update_failed: {result.stderr}')
        except Exception as e:
            self.publish_status(f'error: {str(e)}')

    def forget_wifi(self):
        """Forget current WiFi connection and enable hotspot."""
        self.get_logger().info('Forgetting WiFi and enabling hotspot...')
        try:
            # Get current WiFi connection name
            result = subprocess.run(
                ['nmcli', '-t', '-f', 'GENERAL.CONNECTION', 'device', 'show', 'wlan0'],
                capture_output=True, text=True, timeout=5
            )
            connection_name = None
            for line in result.stdout.split('\n'):
                if 'GENERAL.CONNECTION:' in line:
                    connection_name = line.split(':', 1)[1].strip()
                    break

            if connection_name and connection_name != '--':
                # Delete the connection (forget it)
                subprocess.run(
                    ['sudo', 'nmcli', 'connection', 'delete', connection_name],
                    capture_output=True, timeout=10
                )
                self.get_logger().info(f'Forgot WiFi connection: {connection_name}')

            # Enable hotspot
            subprocess.run(
                ['sudo', 'nmcli', 'connection', 'up', 'Hotspot'],
                capture_output=True, timeout=10
            )
            self.get_logger().info('Hotspot enabled')
            self.publish_status('wifi_forgotten')
        except Exception as e:
            self.get_logger().error(f'Error forgetting WiFi: {e}')
            self.publish_status(f'error: {str(e)}')

    def reboot_system(self):
        """Reboot the system."""
        self.get_logger().info('Rebooting system...')
        self.publish_status('rebooting')
        try:
            subprocess.Popen(['sudo', 'reboot'], start_new_session=True)
        except Exception as e:
            self.get_logger().error(f'Error rebooting: {e}')
            self.publish_status(f'error: {str(e)}')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = UpdateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
