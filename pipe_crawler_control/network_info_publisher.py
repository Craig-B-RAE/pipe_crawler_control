#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import json

class NetworkInfoPublisher(Node):
    def __init__(self):
        super().__init__('network_info_publisher')
        self.publisher = self.create_publisher(String, '/network_info', 10)
        self.timer = self.create_timer(5.0, self.publish_network_info)
        self.get_logger().info('Network Info Publisher started')

    def get_ip(self, interface):
        try:
            result = subprocess.run(
                ['ip', '-4', 'addr', 'show', interface],
                capture_output=True, text=True, timeout=5
            )
            for line in result.stdout.split('\n'):
                if 'inet ' in line:
                    return line.strip().split()[1].split('/')[0]
        except:
            pass
        return None

    def get_wifi_ssid(self):
        try:
            result = subprocess.run(
                ['nmcli', '-t', '-f', 'GENERAL.CONNECTION', 'device', 'show', 'wlan0'],
                capture_output=True, text=True, timeout=5
            )
            for line in result.stdout.split('\n'):
                if 'GENERAL.CONNECTION:' in line:
                    ssid = line.split(':', 1)[1].strip()
                    if ssid and ssid != '--':
                        return ssid
        except:
            pass
        return None

    def publish_network_info(self):
        info = {
            'wifi': self.get_ip('wlan0') or '--',
            'eth': self.get_ip('eth0') or '--',
            'wifi_ssid': self.get_wifi_ssid() or '--'
        }
        msg = String()
        msg.data = json.dumps(info)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NetworkInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
