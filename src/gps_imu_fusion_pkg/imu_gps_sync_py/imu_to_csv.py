#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime

class IMUToCSVNode(Node):
    def __init__(self):
        super().__init__('imu_to_csv_node')

        # 파라미터 설정
        self.declare_parameter('csv_file_path', 'imu_data.csv')
        self.declare_parameter('append_mode', True)

        # 파라미터 가져오기
        self.csv_file_path = self.get_parameter('csv_file_path').get_parameter_value().string_value
        self.append_mode = self.get_parameter('append_mode').get_parameter_value().bool_value

        # CSV 파일 초기화
        self.init_csv_file()

        # /imu/data 토픽 구독
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.get_logger().info(f'IMU to CSV converter started. Saving to: {self.csv_file_path}')
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
                    'ros_time_sec',
                    'ros_time_nanosec',
                    'frame_id',
                    'orientation_x',
                    'orientation_y',
                    'orientation_z',
                    'orientation_w',
                    'angular_velocity_x',
                    'angular_velocity_y',
                    'angular_velocity_z',
                    'linear_acceleration_x',
                    'linear_acceleration_y',
                    'linear_acceleration_z'
                ])
            self.get_logger().info('CSV file initialized with headers')

    def imu_callback(self, msg):
        """Imu 메시지를 받아서 CSV에 저장"""
        try:
            # CSV 데이터 준비
            row_data = [
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]

            # CSV 파일에 데이터 추가
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row_data)

            self.get_logger().info(f'IMU data saved: Accel=({msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}), '
                                  f'Gyro=({msg.angular_velocity.x:.3f}, {msg.angular_velocity.y:.3f}, {msg.angular_velocity.z:.3f})')

        except Exception as e:
            self.get_logger().error(f'Error saving IMU data to CSV: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    node = IMUToCSVNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU to CSV converter stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
