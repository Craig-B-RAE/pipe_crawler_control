#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roboclaw_interfaces.msg import SpeedCommand
from std_msgs.msg import String

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        
        # Publisher to send commands to RoboClaw
        self.speed_pub = self.create_publisher(SpeedCommand, '/speed_command', 10)
        
        # Subscriber to receive high-level commands
        self.cmd_sub = self.create_subscription(
        String, '/robot_command',
        self.command_callback, 10
	)

         # NEW: Subscriber for speed adjustment (0-100%)
        self.speed_sub = self.create_subscription(
            String,
            '/robot_speed',
            self.speed_callback,
            10
        )
    
        self.default_speed = 300  # Default motor speed (QPPS)
        self.speed_percent = 50  # Speed percentage (0-100), matches web interface default
        self.default_accel = 200  # Default acceleration
    
        self.get_logger().info('Simple Controller started')
        self.get_logger().info('Send commands to /robot_command:')
        self.get_logger().info('  "forward" - move forward')
        self.get_logger().info('  "backward" - move backward')  
        self.get_logger().info('  "stop" - stop motors')
        self.get_logger().info('Send speed (0-100) to /robot_speed')
    
    def command_callback(self, msg):
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'Received command: {cmd}')
        if cmd == 'forward':
            self.move_forward()
        elif cmd == 'backward':
            self.move_backward()
        elif cmd == 'stop':
            self.stop()
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def speed_callback(self, msg):
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

    def move_forward(self):
        msg = SpeedCommand()
        actual_speed = int(self.default_speed * self.speed_percent / 100)
        # Scale acceleration with speed (min 50 to avoid issues at very low speeds)
        actual_accel = max(50, int(self.default_accel * self.speed_percent / 100))
        msg.m1_qpps = actual_speed
        msg.m2_qpps = actual_speed
        msg.accel = actual_accel
        msg.max_secs = 10
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Moving forward at {self.speed_percent}% ({actual_speed} QPPS, accel={actual_accel})')
    
    def move_backward(self):
        msg = SpeedCommand()
        actual_speed = int(self.default_speed * self.speed_percent / 100)
        # Scale acceleration with speed (min 50 to avoid issues at very low speeds)
        actual_accel = max(50, int(self.default_accel * self.speed_percent / 100))
        msg.m1_qpps = -actual_speed
        msg.m2_qpps = -actual_speed
        msg.accel = actual_accel
        msg.max_secs = 10
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Moving backward at {self.speed_percent}% ({actual_speed} QPPS, accel={actual_accel})')

    def stop(self):
        msg = SpeedCommand()
        msg.m1_qpps = 0
        msg.m2_qpps = 0
        msg.accel = 5000
        msg.max_secs = 1
        self.speed_pub.publish(msg)
        self.get_logger().info('Stopping')

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
