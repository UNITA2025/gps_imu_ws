#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class GPSTimestampRepublisher(Node):
    def __init__(self):
        super().__init__('imu_repub')

        # 원래 GPS 토픽 구독 (드라이버가 퍼블리시하는 토픽)
        self.sub = self.create_subscription(Imu, '/imu/data', self.callback, 10)  # queue_size

        # 보정 후 퍼블리시할 토픽
        self.pub = self.create_publisher(Imu, '/imure/data', 10)

        self.get_logger().info("IMU timestamp republisher started.")

    def callback(self, msg):
        # 현재 시간으로 타임스탬프 업데이트
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imure_link'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = GPSTimestampRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
