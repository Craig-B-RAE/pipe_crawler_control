#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import subprocess
import json

class CpuTempPublisher(Node):
    def __init__(self):
        super().__init__('cpu_temp_publisher')
        self.temp_publisher = self.create_publisher(Float32, '/cpu_temp', 10)
        self.network_publisher = self.create_publisher(String, '/network_info', 10)
        self.timer = self.create_timer(2.0, self.publish_temp)
        self.network_timer = self.create_timer(5.0, self.publish_network)
        self.get_logger().info('CPU Temp Publisher started')

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

    def publish_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read().strip()) / 1000.0
            msg = Float32()
            msg.data = temp
            self.temp_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to read CPU temp: {e}')

    def publish_network(self):
        info = {
            'wifi': self.get_ip('wlan0') or '--',
            'eth': self.get_ip('eth0') or '--'
        }
        msg = String()
        msg.data = json.dumps(info)
        self.network_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CpuTempPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
