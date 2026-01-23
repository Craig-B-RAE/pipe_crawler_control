#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CpuTempPublisher(Node):
    def __init__(self):
        super().__init__('cpu_temp_publisher')
        self.publisher = self.create_publisher(Float32, '/cpu_temp', 10)
        self.timer = self.create_timer(2.0, self.publish_temp)
        self.get_logger().info('CPU Temp Publisher started')

    def publish_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read().strip()) / 1000.0
            msg = Float32()
            msg.data = temp
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to read CPU temp: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CpuTempPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
