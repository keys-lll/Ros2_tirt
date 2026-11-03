#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import serial
import struct
import numpy as np
from tf_transformations import quaternion_from_euler
from stm32.frame_alignment import rotate_xy

class STM32Feedback(Node):

    def __init__(self):
        super().__init__("stm32_feedback")
        
        # 參數設置
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_597B018072-if00')
        self.declare_parameter('baudrate', 115200)
        self.imu_yaw_rate_variance = float(
            self.declare_parameter('imu_yaw_rate_variance', 0.02).value)
        if not math.isfinite(self.imu_yaw_rate_variance) or self.imu_yaw_rate_variance <= 0.0:
            raise ValueError('imu_yaw_rate_variance must be finite and positive')
        # R(offset) converts native STM32 planar axes into ROS base/IMU axes.
        self.chassis_yaw_offset = float(self.declare_parameter('chassis_yaw_offset', 0.0).value)
        if not math.isfinite(self.chassis_yaw_offset):
            raise ValueError('chassis_yaw_offset must be finite')
        
        # 初始化串口連接
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1  # 更短的超時時間
            )
            self.get_logger().info(f"Connected to {self.serial_port.name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise

        # 創建ROS2發布者
        self.odom_pub = self.create_publisher(Odometry, '/stm32/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/stm32/imu', 10)
        self.battery_pub = self.create_publisher(Float32, '/stm32/battery', 10)

        # 里程計參數
        self.odom_position = np.zeros(3)  # [x, y, yaw]
        self.last_time = self.get_clock().now()
        
        # 數據緩衝區
        self.buffer = bytearray()
        
        # 低通濾波器參數 (用於角度估算)
        self.filter_alpha = 0.2
        self.filtered_yaw = 0.0

        # 定時讀取串口數據（100Hz）
        self.timer = self.create_timer(0.01, self.read_serial_data)
 

    def read_serial_data(self):
        try:
            # 讀取所有可用數據
            data = self.serial_port.read(self.serial_port.in_waiting or 1)
            if data:
                self.buffer.extend(data)
                
                # 處理緩衝區中的所有完整幀
                while len(self.buffer) >= 24:
                    # 查找幀頭
                    start_idx = self.buffer.find(b'\x7B')
                    if start_idx == -1:
                        self.buffer.clear()
                        break
                        
                    # 移除幀頭前的無效數據
                    if start_idx > 0:
                        del self.buffer[:start_idx]
                        continue
                        
                    # 檢查是否有足夠數據
                    if len(self.buffer) < 24:
                        break
                        
                    # 檢查幀尾
                    if self.buffer[23] != 0x7D:
                        # 幀尾不匹配，移除幀頭繼續查找
                        del self.buffer[0]
                        continue
                        
                    # 提取完整幀
                    frame = self.buffer[:24]
                    del self.buffer[:24]
                    
                    self.parse_frame(frame)
        
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")


    def parse_frame(self, data):
        if len(data) != 24 or data[0] != 0x7B or data[23] != 0x7D:
            return
        checksum = 0
        for value in data[:22]:
            checksum ^= value
        if checksum != data[22]:
            return
        # 解析數據
        # 速度(mm/s)(rad/s)
        x_speed = struct.unpack('>h', data[2:4])[0]
        y_speed = struct.unpack('>h', data[4:6])[0]
        z_speed = struct.unpack('>h', data[6:8])[0] / 1000.0

        # 加速度(m/s²)
        accel_x = struct.unpack('>h', data[8:10])[0] / 1672.0
        accel_y = struct.unpack('>h', data[10:12])[0] / 1672.0
        accel_z = struct.unpack('>h', data[12:14])[0] / 1672.0

        # 角速度(rad/s)
        gyro_x = struct.unpack('>h', data[14:16])[0] / 3753.0
        gyro_y = struct.unpack('>h', data[16:18])[0] / 3753.0
        gyro_z = struct.unpack('>h', data[18:20])[0] / 3753.0

        # 電池電壓(V)
        voltage = struct.unpack('>h', data[20:22])[0] / 1000.0

        # 發布數據
        self.publish_data(x_speed, y_speed, z_speed, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, voltage)


    def publish_data(self, x_speed, y_speed, z_speed, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, voltage):
        try:
            # Transform feedback before integrating pose or publishing base-frame data.
            x_speed, y_speed = rotate_xy(x_speed, y_speed, self.chassis_yaw_offset)
            accel_x, accel_y = rotate_xy(accel_x, accel_y, self.chassis_yaw_offset)
            gyro_x, gyro_y = rotate_xy(gyro_x, gyro_y, self.chassis_yaw_offset)
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # 發布IMU數據
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = 'imu_link'  # 使用標準IMU座標系

            # 填充線加速度
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            # 填充角速度
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            # Initial variance estimate; tune from stationary sensor recordings.
            imu_msg.angular_velocity_covariance[8] = self.imu_yaw_rate_variance

            # 方向信息不可用
            imu_msg.orientation_covariance[0] = -1

            self.imu_pub.publish(imu_msg)

            # 發布電池電壓
            voltage_msg = Float32()
            voltage_msg.data = voltage
            self.battery_pub.publish(voltage_msg)
            
            # 計算位置變化
            vx = x_speed / 1000.0  # mm/s -> m/s
            vy = y_speed / 1000.0
            vth = z_speed

            # 使用當前朝向計算位移
            delta_x = vx * np.cos(self.odom_position[2]) * dt - vy * np.sin(self.odom_position[2]) * dt
            delta_y = vx * np.sin(self.odom_position[2]) * dt + vy * np.cos(self.odom_position[2]) * dt
            delta_th = vth * dt

            # 更新位置
            self.odom_position[0] += delta_x
            self.odom_position[1] += delta_y
            self.odom_position[2] += delta_th
            
            # 規範化角度到[-π, π]
            self.odom_position[2] = np.arctan2(
                np.sin(self.odom_position[2]), 
                np.cos(self.odom_position[2])
            )

            # 創建並發布里程計消息
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            # 設置位置
            odom_msg.pose.pose.position.x = self.odom_position[0]
            odom_msg.pose.pose.position.y = self.odom_position[1]
            odom_msg.pose.pose.position.z = 0.0

            # 設置朝向
            q = quaternion_from_euler(0, 0, self.odom_position[2])
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # 設置速度
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = vth



            # Initial velocity variance estimates, to be calibrated on the real base.
            odom_msg.twist.covariance[0] = 0.02
            odom_msg.twist.covariance[7] = 0.04
            odom_msg.twist.covariance[35] = 0.02
            self.odom_pub.publish(odom_msg)

            # 日誌
            self.get_logger().info(
                f"Position: {self.odom_position[0]:.2f}, {self.odom_position[1]:.2f}, {np.degrees(self.odom_position[2]):.1f}° | "
                f"Voltage: {voltage:.2f}V",
                throttle_duration_sec=1.0
            )
            
        except Exception as e:
            self.get_logger().error(f"Publishing error: {e}")
    

def main(args=None):
    rclpy.init(args=args)
    try:
        node = STM32Feedback()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'serial_port') and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()