#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import math

class GPSToPoint(Node):
    def __init__(self):
        super().__init__('gps_to_point')

        # GPS 구독
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )

        # Point 발행
        self.publisher = self.create_publisher(PointStamped, '/gps_point', 10)

        # 첫 번째 GPS 포인트를 원점으로 설정
        self.origin_lat = None
        self.origin_lon = None

    def gps_callback(self, msg):
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f'Origin set: {self.origin_lat}, {self.origin_lon}')

        # GPS를 미터 단위로 변환 (간단한 근사)
        lat_diff = msg.latitude - self.origin_lat
        lon_diff = msg.longitude - self.origin_lon

        # 대략적인 미터 변환 (정확하지는 않지만 시각화용으로 충분)
        x = lon_diff * 111320.0 * math.cos(math.radians(msg.latitude))
        y = lat_diff * 110540.0

        # Point 메시지 생성
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "map"
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = msg.altitude

        self.publisher.publish(point_msg)

def main():
    rclpy.init()
    node = GPSToPoint()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
