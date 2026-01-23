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
