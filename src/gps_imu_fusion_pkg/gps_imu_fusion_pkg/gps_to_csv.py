#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
from datetime import datetime

class GPSToCSVNode(Node):
    def __init__(self):
        super().__init__('gps_to_csv_node')

        # 파라미터 설정
        self.declare_parameter('csv_file_path', 'gps_fix_data.csv')
        self.declare_parameter('append_mode', True)

        # 파라미터 가져오기
        self.csv_file_path = self.get_parameter('csv_file_path').get_parameter_value().string_value
        self.append_mode = self.get_parameter('append_mode').get_parameter_value().bool_value

        # CSV 파일 초기화
        self.init_csv_file()

        # /fix 토픽 구독
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.fix_callback,
            10
        )

        self.get_logger().info(f'GPS Fix to CSV converter started. Saving to: {self.csv_file_path}')
        self.get_logger().info(f'Append mode: {self.append_mode}')

    def init_csv_file(self):
        """CSV 파일 초기화 및 헤더 작성"""
        file_exists = os.path.exists(self.csv_file_path)

        # append 모드가 아니거나 파일이 없으면 새로 생성
        if not self.append_mode or not file_exists:
            with open(self.csv_file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # CSV 헤더 작성
                writer.writerow([
                    'ros_time_sec', # ROS 시간 초
                    'ros_time_nanosec', # ROS 시간 나노초
                    'frame_id', # 프레임 ID: gps
                    'status', # 상태: 0:GNSS, 2: GSNN + RTK, -1: error
                    'service',
                    'latitude',
                    'longitude',
                    'altitude'
                ])
            self.get_logger().info('CSV file initialized with headers')

    def fix_callback(self, msg):
        """NavSatFix 메시지를 받아서 CSV에 저장"""
        try:
            # CSV 데이터 준비
            row_data = [
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id,
                msg.status.status,
                msg.status.service,
                msg.latitude,
                msg.longitude,
                msg.altitude
            ]

            # CSV 파일에 데이터 추가
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row_data)

            self.get_logger().info(f'GPS data saved: Status={msg.status.status} Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, Alt={msg.altitude:.3f}')

        except Exception as e:
            self.get_logger().error(f'Error saving GPS data to CSV: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    node = GPSToCSVNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GPS to CSV converter stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
