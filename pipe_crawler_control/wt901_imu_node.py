#!/usr/bin/env python3
"""
WT901 IMU ROS2 Node

Reads data from WT901 IMU sensor over serial and publishes to ROS2 topics.
Protocol: 9600 baud, packets start with 0x55, followed by type byte:
  0x51: Acceleration
  0x52: Angular velocity (gyroscope)
  0x53: Angle (roll, pitch, yaw)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import serial
import struct
import math


class WT901ImuNode(Node):
    def __init__(self):
        super().__init__('wt901_imu')

        # Parameters
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.angle_pub = self.create_publisher(Vector3Stamped, 'imu/angle', 10)
        self.roll_pub = self.create_publisher(Float32, 'imu/roll', 10)
        self.pitch_pub = self.create_publisher(Float32, 'imu/pitch', 10)
        self.yaw_pub = self.create_publisher(Float32, 'imu/yaw', 10)

        # Data storage
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.angle = [0.0, 0.0, 0.0]  # roll, pitch, yaw in degrees

        # Serial connection
        self.serial = None
        self.connect_serial()

        # Timer for reading serial data
        self.create_timer(0.01, self.read_serial)  # 100Hz read rate

        self.get_logger().info(f'WT901 IMU node started on {self.port} at {self.baud_rate} baud')

    def connect_serial(self):
        """Connect to serial port."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial = None

    def read_serial(self):
        """Read and parse serial data from WT901."""
        if self.serial is None or not self.serial.is_open:
            return

        try:
            # Read available data
            while self.serial.in_waiting >= 11:
                # Look for packet header 0x55
                byte = self.serial.read(1)
                if byte != b'\x55':
                    continue

                # Read packet type and data (10 more bytes)
                packet = self.serial.read(10)
                if len(packet) < 10:
                    continue

                packet_type = packet[0]
                data = packet[1:9]
                checksum = packet[9]

                # Verify checksum
                calc_checksum = (0x55 + sum(packet[:9])) & 0xFF
                if calc_checksum != checksum:
                    continue

                # Parse based on packet type
                if packet_type == 0x51:  # Acceleration
                    self.parse_acceleration(data)
                elif packet_type == 0x52:  # Angular velocity
                    self.parse_gyro(data)
                elif packet_type == 0x53:  # Angle
                    self.parse_angle(data)
                    self.publish_data()  # Publish after receiving angle

        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def parse_acceleration(self, data):
        """Parse acceleration packet (0x51)."""
        # Data format: AxL, AxH, AyL, AyH, AzL, AzH, TL, TH
        ax = struct.unpack('<h', data[0:2])[0] / 32768.0 * 16.0 * 9.81
        ay = struct.unpack('<h', data[2:4])[0] / 32768.0 * 16.0 * 9.81
        az = struct.unpack('<h', data[4:6])[0] / 32768.0 * 16.0 * 9.81
        self.accel = [ax, ay, az]

    def parse_gyro(self, data):
        """Parse angular velocity packet (0x52)."""
        # Data format: wxL, wxH, wyL, wyH, wzL, wzH, TL, TH
        # Range: +/- 2000 deg/s, convert to rad/s
        wx = struct.unpack('<h', data[0:2])[0] / 32768.0 * 2000.0 * math.pi / 180.0
        wy = struct.unpack('<h', data[2:4])[0] / 32768.0 * 2000.0 * math.pi / 180.0
        wz = struct.unpack('<h', data[4:6])[0] / 32768.0 * 2000.0 * math.pi / 180.0
        self.gyro = [wx, wy, wz]

    def parse_angle(self, data):
        """Parse angle packet (0x53)."""
        # Data format: RollL, RollH, PitchL, PitchH, YawL, YawH, VL, VH
        # Range: +/- 180 degrees
        roll = struct.unpack('<h', data[0:2])[0] / 32768.0 * 180.0
        pitch = struct.unpack('<h', data[2:4])[0] / 32768.0 * 180.0
        yaw = struct.unpack('<h', data[4:6])[0] / 32768.0 * 180.0
        self.angle = [roll, pitch, yaw]

    def publish_data(self):
        """Publish IMU data to ROS2 topics."""
        now = self.get_clock().now().to_msg()

        # Publish full IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = self.accel[0]
        imu_msg.linear_acceleration.y = self.accel[1]
        imu_msg.linear_acceleration.z = self.accel[2]

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = self.gyro[0]
        imu_msg.angular_velocity.y = self.gyro[1]
        imu_msg.angular_velocity.z = self.gyro[2]

        # Orientation quaternion from Euler angles
        roll_rad = math.radians(self.angle[0])
        pitch_rad = math.radians(self.angle[1])
        yaw_rad = math.radians(self.angle[2])

        # Convert to quaternion (ZYX convention)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Covariance (unknown, set to -1 for first element)
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        # Publish angle as Vector3Stamped (degrees)
        angle_msg = Vector3Stamped()
        angle_msg.header.stamp = now
        angle_msg.header.frame_id = self.frame_id
        angle_msg.vector.x = self.angle[0]  # roll
        angle_msg.vector.y = self.angle[1]  # pitch
        angle_msg.vector.z = self.angle[2]  # yaw
        self.angle_pub.publish(angle_msg)

        # Publish individual angles
        self.roll_pub.publish(Float32(data=self.angle[0]))
        self.pitch_pub.publish(Float32(data=self.angle[1]))
        self.yaw_pub.publish(Float32(data=self.angle[2]))

    def destroy_node(self):
        """Clean up on shutdown."""
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WT901ImuNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
