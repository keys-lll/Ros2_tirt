"""One serial owner for STM32 commands and feedback."""
import math
import struct
import time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from stm32.stm32_feedback import STM32Feedback
from stm32.frame_alignment import rotate_xy
from rclpy.executors import ExternalShutdownException


class STM32Bridge(STM32Feedback):
    def __init__(self):
        super().__init__()
        self.serial_port.timeout = 0.0
        self.serial_port.write_timeout = 0.1
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('max_vx', 0.2)
        self.declare_parameter('max_vy', 0.15)
        self.declare_parameter('max_wz', 0.5)
        self.require_fresh_scan = self.declare_parameter('require_fresh_scan', False).value
        self.last_scan = None
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_received, qos_profile_sensor_data)
        self.last_command = time.monotonic()
        self.stopped = True
        command_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.subscription = self.create_subscription(Twist, command_topic, self.command, 10)
        self.watchdog = self.create_timer(0.05, self.check_timeout)
        self.send_velocity(0.0, 0.0, 0.0)

    def send_velocity(self, vx, vy, wz):
        # Inverse of the feedback transform: ROS command -> native STM32 axes.
        vx, vy = rotate_xy(vx, vy, -self.chassis_yaw_offset)
        packet = bytearray([0x7B, 0, 0])
        for value in (vx, vy, wz):
            packet.extend(struct.pack('>h', round(value * 1000)))
        checksum = 0
        for value in packet:
            checksum ^= value
        packet.extend([checksum, 0x7D])
        self.serial_port.write(packet)

    def scan_received(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if msg.ranges and -0.2 <= age <= 0.5:
            self.last_scan = time.monotonic()

    def scan_ready(self):
        return (not self.require_fresh_scan or
                (self.last_scan is not None and time.monotonic() - self.last_scan <= 0.5))

    def command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not self.scan_ready() or not all(math.isfinite(value) for value in values):
            self.send_velocity(0.0, 0.0, 0.0)
            self.stopped = True
            return
        limits = [float(self.get_parameter(key).value) for key in ('max_vx', 'max_vy', 'max_wz')]
        values = [max(-limit, min(limit, value)) for value, limit in zip(values, limits)]
        self.send_velocity(*values)
        self.last_command = time.monotonic()
        self.stopped = not any(values)

    def check_timeout(self):
        if not self.stopped and (not self.scan_ready() or time.monotonic() - self.last_command > self.get_parameter('command_timeout').value):
            self.send_velocity(0.0, 0.0, 0.0)
            self.stopped = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = STM32Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.send_velocity(0.0, 0.0, 0.0)
            finally:
                node.serial_port.close()
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
