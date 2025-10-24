#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import re
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header


class SerialLidarNode(Node):
    def __init__(self):
        super().__init__('serial_lidar_publisher')

        # === 参数 ===
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('publish_rate', 10.0)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # === 打开串口 ===
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f'✅ 成功打开串口 {port}，波特率 {baudrate}')
        except Exception as e:
            self.get_logger().error(f'❌ 打开串口失败: {e}')
            exit(1)

        # === 发布 LaserScan ===
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.scan_data = [float('inf')] * 360
        self.pattern = re.compile(r'Angle:\s*([\d\.]+)\s*deg\s*Dist:\s*([\d\.]+)\s*mm')

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_scan)
        self.last_log_time = time.time()

        # === 订阅 cmd_vel ===
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.last_cmd = 'S'  # 初始为停止

    # ============ 发布雷达数据 =============
    def publish_scan(self):
        try:
            lines = self.ser.readlines()
            for line in lines:
                text = line.decode('utf-8', errors='ignore')
                match = self.pattern.search(text)
                if match:
                    angle_deg = float(match.group(1))
                    dist_mm = float(match.group(2))
                    index = int(angle_deg) % 360
                    dist_m = dist_mm / 1000.0
                    if dist_m <= 0.05 or dist_m > 20.0:
                        dist_m = float('inf')
                    self.scan_data[index] = dist_m

            scan_msg = LaserScan()
            scan_msg.header = Header()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 2 * math.pi
            scan_msg.angle_increment = math.radians(1.0)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0 / self.publish_rate
            scan_msg.range_min = 0.05
            scan_msg.range_max = 20.0
            scan_msg.ranges = self.scan_data

            self.scan_pub.publish(scan_msg)

            # 限频打印
            now = time.time()
            if now - self.last_log_time > 2.0:
                self.get_logger().info('📡 已发布一帧 LaserScan')
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f'串口读取错误: {e}')

    # ============ 处理 cmd_vel =============
    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        command = 'S'

        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            command = 'S'  # 停止
        elif linear_x > 0.01:
            command = 'B'  # 前进
        elif linear_x < -0.01:
            command = 'F'  # 后退
        elif angular_z > 0.01:
            command = 'L'  # 左转
        elif angular_z < -0.01:
            command = 'R'  # 右转

        # 如果命令变化才下发
        if command != self.last_cmd:
            try:
                self.ser.write(command.encode('utf-8'))
                self.last_cmd = command
                self.get_logger().info(f'🚗 下发运动指令: {command}')
            except Exception as e:
                self.get_logger().error(f'串口发送错误: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点已退出')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
