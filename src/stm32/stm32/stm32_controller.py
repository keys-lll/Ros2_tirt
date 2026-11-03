#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class STM32Controller(Node):

    def __init__(self):
        super().__init__("stm32_controller") # 初始化ROS2節點
        self.get_logger().info("stm32_controller")

        # 初始化串口連接
        self.serial_port = serial.Serial(
            port='/dev/ttySTM32',    # 串口位置 可能是/dev/ttyUSB0
            baudrate=115200,        # 波特率需與STM32設定一致
            timeout=1               # 讀取超時時間（秒）
        )
        self.get_logger().info(f"Connected to serial port: {self.serial_port.name}")

        # 訂閱ROS2的 /cmd_vel 話題，回調函數為 cmd_vel_callback
        self.subscription = self.create_subscription(
            Twist,                  # 消息類型：geometry_msgs/Twist
            '/cmd_vel',             # 話題名稱
            self.cmd_vel_callback,  # 回調函數
            10                      # 消息隊列長度
        )
        self.get_logger().info("Ready to send commands to STM32...")

    def cmd_vel_callback(self, msg):
        # 將ROS2的 Twist 消息轉換為STM32的自定義協議並發送
        x_speed_mm = int(msg.linear.x * 1000)   # 速度（前後）
        y_speed_mm = int(msg.linear.y * 1000)   # 速度（左右）
        z_speed_rad = int(msg.angular.z * 1000) # 速度（旋轉）

        # 構建11字節的資料結構
        packet = bytearray()
        packet.append(0x7B)                             # 頭（固定值0x7B）
        packet.extend([0x00, 0x00])                     # 預留位（2字節，填0）
        packet.extend(struct.pack('>h', x_speed_mm))    # X 軸速度
        packet.extend(struct.pack('>h', y_speed_mm))    # Y 軸速度
        packet.extend(struct.pack('>h', z_speed_rad))   # Z 軸速度
        packet.append(self.alculate_bcc(packet))        # BCC互斥或(XOR)校驗
        packet.append(0x7D)                             # 尾（固定值0x7D）

        # 發送資料
        self.serial_port.write(packet)
        self.get_logger().info(
            f"Sent command: X={x_speed_mm} mm/s, Y={y_speed_mm} mm/s, Z={z_speed_rad/1000} rad/s"
        )

    def alculate_bcc(self, data):
        # 計算BCC互斥或(XOR)校驗
        bcc = 0
        for byte in data:
            bcc ^= byte
        return bcc

def main(args=None):
    rclpy.init(args=args)
    node = STM32Controller()
    rclpy.spin(node)
    if node.serial_port.is_open():
        node.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()