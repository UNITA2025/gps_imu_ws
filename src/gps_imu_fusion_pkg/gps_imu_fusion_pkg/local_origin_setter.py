#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from scipy.spatial.transform import Rotation

class LocalOriginSetter(Node):
    def __init__(self):
        super().__init__('local_origin_setter')
        
        # 초기 원점 설정용 변수
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        
        # Heading 보정용 변수
        self.first_position = None
        self.second_position = None
        self.heading_offset = 0.0
        self.positions_captured = 0
        self.heading_calibrated = False
        
        # Global odometry subscriber
        self.global_sub = self.create_subscription(
            Odometry,
            '/odometry/global2',
            self.global_odom_callback,
            10
        )
        
        # Local odometry publisher (원점 기준 + heading 보정)
        self.local_pub = self.create_publisher(
            Odometry,
            '/odometry/local_enu',
            10
        )
        
        self.get_logger().info('Local Origin Setter with Heading Correction started!')
        self.get_logger().info('Waiting for first GPS fix...')
        
    def global_odom_callback(self, msg):
        # 디버깅: 수신된 데이터 확인
        # self.get_logger().info(f'Received global odom: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
        
        # 첫 번째 GPS 데이터를 원점으로 설정
        if not self.origin_set:
            self.origin_x = msg.pose.pose.position.x
            self.origin_y = msg.pose.pose.position.y
            self.origin_z = msg.pose.pose.position.z
            self.origin_set = True
            
            self.get_logger().info(f'🎯 Origin set to: ({self.origin_x:.2f}, {self.origin_y:.2f}, {self.origin_z:.2f})')
        
        # 원점 기준으로 상대 좌표 계산
        local_x = msg.pose.pose.position.x - self.origin_x
        local_y = msg.pose.pose.position.y - self.origin_y
        local_z = msg.pose.pose.position.z - self.origin_z
        
        # Heading 보정이 아직 안 됐으면 캘리브레이션 수행
        if not self.heading_calibrated:
            self.calibrate_heading(local_x, local_y, msg)
        
        # Local ENU odometry 메시지 생성
        local_odom = Odometry()
        local_odom.header = msg.header
        local_odom.header.frame_id = 'map'
        local_odom.child_frame_id = msg.child_frame_id
        
        # 위치 설정
        local_odom.pose.pose.position.x = local_x
        local_odom.pose.pose.position.y = local_y
        local_odom.pose.pose.position.z = local_z
        
        # Heading 보정 적용
        if self.heading_calibrated:
            corrected_orientation = self.apply_heading_correction(msg.pose.pose.orientation)
            local_odom.pose.pose.orientation = corrected_orientation
        else:
            local_odom.pose.pose.orientation = msg.pose.pose.orientation
        
        # 속도 정보 복사
        local_odom.twist = msg.twist
        local_odom.pose.covariance = msg.pose.covariance
        
        # 발행
        self.local_pub.publish(local_odom)
        
        # 디버깅: 변환된 좌표 확인 (가끔씩만)
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # 1초에 한 번 정도
            status = "✅ Calibrated" if self.heading_calibrated else "🔄 Calibrating"
            self.get_logger().info(f'📍 Local coords: x={local_x:.2f}, y={local_y:.2f} | {status}')
        
    def calibrate_heading(self, local_x, local_y, msg):
        """처음 두 위치를 사용하여 heading 오프셋 계산"""
        current_pos = (local_x, local_y)
        
        if self.first_position is None:
            self.first_position = current_pos
            self.get_logger().info(f'📍 First position captured: ({local_x:.2f}, {local_y:.2f})')
            
        elif self.second_position is None:
            # 충분히 움직였을 때만 두 번째 위치로 인정
            distance = math.sqrt(
                (current_pos[0] - self.first_position[0])**2 + 
                (current_pos[1] - self.first_position[1])**2
            )
            
            if distance > 3.0:  # 3m 이상 움직였을 때
                self.second_position = current_pos
                self.calculate_heading_offset(msg)
                self.heading_calibrated = True
                
                self.get_logger().info(f'📍 Second position captured: ({local_x:.2f}, {local_y:.2f})')
                self.get_logger().info(f'🧭 Heading offset: {math.degrees(self.heading_offset):.1f}°')
                self.get_logger().info('✅ Heading calibration completed!')
                
    def calculate_heading_offset(self, msg):
        """실제 이동 방향과 현재 heading의 차이 계산"""
        # 실제 이동 방향 계산
        dx = self.second_position[0] - self.first_position[0]
        dy = self.second_position[1] - self.first_position[1]
        actual_heading = math.atan2(dy, dx)
        
        # 현재 IMU heading 추출
        orientation = msg.pose.pose.orientation
        r = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        current_heading = r.as_euler('xyz')[2]  # yaw
        
        # 오프셋 계산
        self.heading_offset = actual_heading - current_heading
        
        # -π ~ π 범위로 정규화
        while self.heading_offset > math.pi:
            self.heading_offset -= 2 * math.pi
        while self.heading_offset < -math.pi:
            self.heading_offset += 2 * math.pi
            
        self.get_logger().info(f'🔍 Actual direction: {math.degrees(actual_heading):.1f}°')
        self.get_logger().info(f'🔍 IMU heading: {math.degrees(current_heading):.1f}°')
        
    def apply_heading_correction(self, orientation):
        """Heading 보정 적용"""
        # 현재 orientation을 euler로 변환
        r = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        euler = r.as_euler('xyz')
        
        # Yaw에 오프셋 적용
        corrected_yaw = euler[2] + self.heading_offset
        
        # 새로운 quaternion 생성
        corrected_r = Rotation.from_euler('xyz', [euler[0], euler[1], corrected_yaw])
        corrected_quat = corrected_r.as_quat()
        
        # geometry_msgs/Quaternion으로 변환
        from geometry_msgs.msg import Quaternion
        result = Quaternion()
        result.x = corrected_quat[0]
        result.y = corrected_quat[1] 
        result.z = corrected_quat[2]
        result.w = corrected_quat[3]
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = LocalOriginSetter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()