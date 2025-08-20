#!/usr/bin/env python3

#========================================================================================
# 기능 : /odometry/global을 받아 최초 좌표를 원점으로 삼아 /odometry/local_enu를 발행하고,
# “최근 이동 궤적이 직선일 때” 이동 방향을 기준으로 heading_offset을 추정·누적 적용해 Yaw를 자동 보정해주는 코드
# 동작 
# /odometry/global을 구독하여 첫 위치를 원점(0,0,0)으로 설정한 뒤,
# 로컬 ENU 좌표계로 변환된 /odometry/local_enu를 발행.
# 최근 이동 궤적이 직선으로 감지되면 실제 진행 방향을 기준으로 IMU yaw를 보정하여
# 주행 시 발생하는 heading 드리프트를 완화.
# TODO : 
# 최종 수정일: 2025.08.18
# 편집자 : 송준상, 이다빈
#========================================================================================

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
        
        # Dynamic heading 보정용 변수
        self.heading_offset = 0.0
        self.last_calibration_pos = None
        self.current_trajectory = []  # 최근 위치들 저장
        self.max_trajectory_length = 10  # 최근 10개 포지션만 유지
        self.calibration_count = 0  # 캘리브레이션 횟수 추적
        
        # 직진 감지 파라미터 (더 관대하게 설정)
        self.min_straight_distance = 0.3  # 최소 0.3m 직진 (더 낮춤)
        self.max_direction_deviation = 15.0  # 최대 15도 방향 변화 허용 (더 관대함)
        self.min_calibration_distance = 0.3  # 캘리브레이션 간 최소 거리 (더 낮춤)
        
        # Global odometry subscriber
        self.global_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.global_odom_callback,
            10
        )
        
        # Local odometry publisher (원점 기준 + heading 보정)
        self.local_pub = self.create_publisher(
            Odometry,
            '/odometry/local_enu',
            10
        )
        
        self.get_logger().info('Dynamic Local Origin Setter started!')
        self.get_logger().info('Will recalibrate heading during straight movements...')
        
    def global_odom_callback(self, msg):
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
        
        # 현재 위치를 trajectory에 추가
        current_pos = (local_x, local_y)
        self.current_trajectory.append(current_pos)
        
        # trajectory 길이 제한
        if len(self.current_trajectory) > self.max_trajectory_length:
            self.current_trajectory.pop(0)
        
        # 직진 구간 감지 및 heading 재보정
        self.check_and_recalibrate_heading(msg)
        
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
        corrected_orientation = self.apply_heading_correction(msg.pose.pose.orientation)
        local_odom.pose.pose.orientation = corrected_orientation
        
        # 속도 정보 복사
        local_odom.twist = msg.twist
        local_odom.pose.covariance = msg.pose.covariance
        
        # 발행
        self.local_pub.publish(local_odom)
        
        # 디버깅: 변환된 좌표 확인 (가끔씩만)
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # 1초에 한 번 정도
            trajectory_len = len(self.current_trajectory)
            is_straight = self.is_straight_trajectory() if trajectory_len >= 3 else False
            
            if trajectory_len >= 3:
                start_pos = self.current_trajectory[0]
                end_pos = self.current_trajectory[-1]
                distance = math.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
                self.get_logger().info(f'📍 Local coords: x={local_x:.2f}, y={local_y:.2f} | Heading offset: {math.degrees(self.heading_offset):.1f}° | Traj: {trajectory_len} pts, {distance:.1f}m, Straight: {is_straight}')
            else:
                self.get_logger().info(f'📍 Local coords: x={local_x:.2f}, y={local_y:.2f} | Heading offset: {math.degrees(self.heading_offset):.1f}° | Traj: {trajectory_len} pts')
        
    def check_and_recalibrate_heading(self, msg):
        """직진 구간을 감지하고 heading을 재보정"""
        if len(self.current_trajectory) < 3:
            return
            
        # 최근 trajectory가 직진인지 확인
        is_straight = self.is_straight_trajectory()
        
        if is_straight:
            start_pos = self.current_trajectory[0]
            end_pos = self.current_trajectory[-1]
            
            # 충분한 거리 이동했는지 확인
            distance = math.sqrt(
                (end_pos[0] - start_pos[0])**2 + 
                (end_pos[1] - start_pos[1])**2
            )
            
            self.get_logger().info(f'🔍 Straight detected! Distance: {distance:.2f}m (need: {self.min_straight_distance:.1f}m)')
            
            if distance >= self.min_straight_distance:
                # 마지막 캘리브레이션 위치와 충분히 떨어져 있는지 확인
                dist_from_last = self.distance_from_last_calibration(end_pos)
                self.get_logger().info(f'🔍 Distance from last calibration: {dist_from_last:.2f}m')
                
                if (self.last_calibration_pos is None or dist_from_last > self.min_calibration_distance):
                    
                    # Heading 재보정 수행
                    self.recalibrate_heading(start_pos, end_pos, msg)
                    self.last_calibration_pos = end_pos
                else:
                    self.get_logger().info(f'⏸️  Too close to last calibration point, skipping...')
            else:
                self.get_logger().info(f'⏸️  Distance too short for calibration')
                    
    def is_straight_trajectory(self):
        """현재 trajectory가 직진인지 판단"""
        if len(self.current_trajectory) < 3:
            return False
            
        # 시작점과 끝점을 잇는 직선의 방향 계산
        start_pos = self.current_trajectory[0]
        end_pos = self.current_trajectory[-1]
        
        expected_direction = math.atan2(
            end_pos[1] - start_pos[1],
            end_pos[0] - start_pos[0]
        )
        
        # 중간 지점들이 이 직선에서 얼마나 벗어나는지 확인
        max_deviation = 0.0
        deviations = []
        
        for i in range(1, len(self.current_trajectory) - 1):
            pos = self.current_trajectory[i]
            
            # 현재 지점까지의 방향 계산
            current_direction = math.atan2(
                pos[1] - start_pos[1],
                pos[0] - start_pos[0]
            )
            
            # 방향 차이 계산
            direction_diff = abs(self.normalize_angle(current_direction - expected_direction))
            deviations.append(math.degrees(direction_diff))
            max_deviation = max(max_deviation, direction_diff)
        
        # 최대 허용 편차보다 작으면 직진으로 판단
        is_straight = max_deviation < math.radians(self.max_direction_deviation)
        
        # 디버깅 정보 (가끔씩만)
        if len(self.current_trajectory) >= 5 and self.get_clock().now().nanoseconds % 2000000000 < 100000000:
            self.get_logger().info(f'🔍 Trajectory analysis: max_dev={math.degrees(max_deviation):.1f}°, deviations={deviations}, straight={is_straight}')
        
        return is_straight
        
    def distance_from_last_calibration(self, current_pos):
        """마지막 캘리브레이션 위치로부터의 거리"""
        if self.last_calibration_pos is None:
            return float('inf')
            
        return math.sqrt(
            (current_pos[0] - self.last_calibration_pos[0])**2 + 
            (current_pos[1] - self.last_calibration_pos[1])**2
        )
        
    def recalibrate_heading(self, start_pos, end_pos, msg):
        """실제 이동 방향을 기반으로 heading 재보정"""
        # 실제 이동 방향 계산
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        actual_heading = math.atan2(dy, dx)
        
        # 현재 IMU heading 추출
        orientation = msg.pose.pose.orientation
        r = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        current_heading = r.as_euler('xyz')[2]  # yaw
        
        # 새로운 오프셋 계산
        new_offset = actual_heading - current_heading
        new_offset = self.normalize_angle(new_offset)
        
        # 급격한 변화는 제한 (노이즈 방지)
        max_offset_change = math.radians(150.0)  # 최대 150도 변화만 허용
        
        if abs(self.normalize_angle(new_offset - self.heading_offset)) < max_offset_change:
            # 기존 오프셋과 새 오프셋의 가중평균 (부드러운 보정)
            alpha = 0.3  # 새로운 값의 가중치
            self.heading_offset = self.normalize_angle(
                (1 - alpha) * self.heading_offset + alpha * new_offset
            )
            
            self.get_logger().info(f'🧭 Heading recalibrated! New offset: {math.degrees(self.heading_offset):.1f}°')
            self.get_logger().info(f'📏 Straight distance: {math.sqrt(dx*dx + dy*dy):.1f}m')
        else:
            self.get_logger().warn(f'⚠️  Large heading change detected ({math.degrees(new_offset - self.heading_offset):.1f}°), ignoring...')
        
    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
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