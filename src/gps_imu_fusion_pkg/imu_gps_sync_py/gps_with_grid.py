#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import math
import numpy as np

class GPSWithGrid(Node):
    def __init__(self):
        super().__init__('gps_with_grid')

        # GPS 구독
        self.gps_sub = self.create_subscription(
            NavSatFix, '/fix', self.gps_callback, 10)

        # Grid Map 발행
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # GPS Point 발행
        self.point_pub = self.create_publisher(PointStamped, '/gps_point', 10)

        # 첫 GPS를 원점으로 설정
        self.origin_lat = None
        self.origin_lon = None

        # Grid 설정
        self.grid_size = 1000  # 1000x1000 grid
        self.resolution = 1.0  # 1미터/픽셀

        # Grid 발행 타이머
        self.timer = self.create_timer(1.0, self.publish_grid)

    def gps_callback(self, msg):
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f'Origin: {self.origin_lat}, {self.origin_lon}')

        # GPS를 미터로 변환
        lat_diff = msg.latitude - self.origin_lat
        lon_diff = msg.longitude - self.origin_lon

        x = lon_diff * 111320.0 * math.cos(math.radians(msg.latitude))
        y = lat_diff * 110540.0

        # GPS Point 발행
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "map"
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        self.point_pub.publish(point_msg)

    def publish_grid(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"

        # Grid 정보 설정
        grid.info.resolution = self.resolution
        grid.info.width = self.grid_size
        grid.info.height = self.grid_size
        grid.info.origin.position.x = -self.grid_size * self.resolution / 2
        grid.info.origin.position.y = -self.grid_size * self.resolution / 2
        grid.info.origin.orientation.w = 1.0

        # 빈 grid 데이터 (회색 배경)
        grid.data = [-1] * (self.grid_size * self.grid_size)

        self.grid_pub.publish(grid)

def main():
    rclpy.init()
    node = GPSWithGrid()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
