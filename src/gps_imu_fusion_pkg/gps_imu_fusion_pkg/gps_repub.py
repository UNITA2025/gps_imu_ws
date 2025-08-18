#!/usr/bin/env python3

#==================================================#
# 기능: GPS 메시지의 타임스탬프를 현재 시간으로 업데이트하여 재발행
# - 원래 GPS 드라이버가 발행하는 /fix 토픽을 구독하여 header.stamp 필드를 현재 노드 시스템 시간으로 바꿔준다.
# - 타임스탬프가 갱신된 메시지를 /gps/fix 토픽으로 재발행한다.
#
# 송신 토픽 (publish):
#   - /gps/fix (msg: sensor_msgs/msg/NavSatFix)
# 수신 토픽 (subscribe):
#   - /navsat_transform
#
# TODO : 
# 최종 수정일: 2025.08.18
# 편집자: 
#==================================================#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSTimestampRepublisher(Node):
    def __init__(self):
        super().__init__('gps_repub')

        # 원래 GPS 토픽 구독 (드라이버가 퍼블리시하는 토픽)
        self.sub = self.create_subscription(NavSatFix, '/fix', self.callback, 10)  # queue_size

        # 보정 후 퍼블리시할 토픽
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.get_logger().info("GPS timestamp republisher started.")

    def callback(self, msg):
        # 현재 시간으로 타임스탬프 업데이트
        msg.header.stamp = self.get_clock().now().to_msg()
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
