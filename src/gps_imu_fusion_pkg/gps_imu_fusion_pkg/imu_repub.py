#!/usr/bin/env python3

#==================================================#
# 기능: IMU 메시지의 타임스탬프를 현재 시간으로 업데이트 후 frame id를 교체하여 재발행
# - 원래 IMU 센서가 발행하는 /imu/data 토픽을 구독하여 imure/data라는 새로운 토픽으로 재발행한다.
# - header.stamp를 현재 ROS 시스템 시간으로 덮어쓰고, 메시지가 속한 좌표계를 imure_link로 변경한다.
#
# 송신 토픽 (publish):
#   - /imure/data (msg: sensor_msgs/msg/Imu)
# 수신 토픽 (subscribe):
#   - /ekf_global, /ekf_local
#
# TODO : 
# 최종 수정일: 2025.08.18
# 편집자: 
#==================================================#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class GPSTimestampRepublisher(Node):
    def __init__(self):
        super().__init__('imu_repub')

        # 원래 IMU 토픽 구독 (드라이버가 퍼블리시하는 토픽)
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
