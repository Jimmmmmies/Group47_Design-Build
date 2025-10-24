#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)  # 10Hz (0.1秒周期)
        self.get_logger().info("Publishing static transform from base_link to laser_link at 10Hz")

    def publish_transform(self):
        # 创建并填充TransformStamped消息
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser_link'
        
        # 设置平移 (x, y, z)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        # 设置旋转 (四元数，无旋转)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # w=1表示无旋转
        
        # 发布静态变换
        self.broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()