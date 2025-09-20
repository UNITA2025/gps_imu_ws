#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#========================================================================================
# 기능: /odometry/local_enu의 Odometry를 구독해 Pose과 Twist(선형/각속도)를
#       각각 pose_data.txt, twist_data.txt 저장하는 코드.
#       (헤딩을 고려한 정확한 전진/후진 방향 판단)
# 동작
# 노드 시작 시 빈 파일에 구독 시작.
# 콜백마다 Pose(x, y, z, qx, qy, qz, qw), Twist(vx, vy, vz, wx, wy, wz) 와
# 방향 정보를 로그로 출력, 파일에 저장.
# 최종 수정일: 2025.09.20
# 편집자 : 송준상, 이다빈
#========================================================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os
import math

class OdometryLogger(Node):
    def __init__(self, pose_file="pose_data.txt", twist_file="twist_data.txt"):
        super().__init__('odometry_logger')
        
        self.pose_file = pose_file
        self.twist_file = twist_file
        
        # 이전 위치와 헤딩 저장 (위치 기반 검증용)
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        
        # 임계값 설정
        self.velocity_threshold = 0.05  # m/s - 정지 판단 임계값
        self.distance_threshold = 0.02  # m - 위치 변화 임계값
        
        # 파일 초기화 (기존 내용 삭제)
        # 헤더를 추가하여 데이터의 의미를 명확하게 함
        with open(self.pose_file, 'w') as f:
            f.write("x y z qx qy qz qw yaw_deg direction\n")
        with open(self.twist_file, 'w') as f:
            f.write("vx vy vz wx wy wz direction\n")
        
        # /odometry/local_enu 토픽 구독
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/local_enu',
            self.odometry_callback,
            10  # QoS history depth
        )
        
        self.get_logger().info(f"Odometry logger started. Saving to {pose_file} and {twist_file}")
    
    def quaternion_to_yaw(self, x, y, z, w):
        """쿼터니언을 yaw 각도(라디안)로 변환"""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def determine_direction(self, linear_x, linear_y, pose_x, pose_y, yaw):
        """헤딩을 고려한 정확한 전진/후진 판단"""
        
        # 방법 1: 로봇 좌표계에서의 속도 기반 판단
        # 이 로봇에서는 linear_x < 0이 전진, linear_x > 0이 후진
        forward_velocity = linear_x
        
        if abs(forward_velocity) < self.velocity_threshold:
            velocity_direction = "stop"
        else:
            # 로봇 좌표계가 반대로 설정된 경우
            velocity_direction = "forward" if forward_velocity < 0 else "reverse"
        
        # 방법 2: 실제 위치 변화와 헤딩을 비교한 검증
        position_direction = "unknown"
        
        if self.prev_x is not None and self.prev_y is not None:
            # 실제 이동한 거리와 방향
            dx = pose_x - self.prev_x
            dy = pose_y - self.prev_y
            distance_moved = math.sqrt(dx**2 + dy**2)
            
            if distance_moved > self.distance_threshold:
                # 실제 이동한 방향 (월드 좌표계)
                movement_angle = math.atan2(dy, dx)
                
                # 로봇이 바라보는 방향과 실제 이동 방향의 차이
                angle_diff = movement_angle - yaw
                
                # 각도를 -π ~ π 범위로 정규화
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                
                # 각도 차이로 전진/후진 판단
                # -π/2 ~ π/2 범위면 전진, 그 외는 후진
                if abs(angle_diff) <= math.pi/2:
                    position_direction = "forward"
                else:
                    position_direction = "reverse"
            else:
                position_direction = "stop"
        
        # 최종 판단: 속도 기반을 우선하되, 위치 기반으로 검증
        if velocity_direction == "stop":
            final_direction = "stop"
        elif position_direction == "unknown":
            final_direction = velocity_direction
        elif velocity_direction == position_direction:
            final_direction = velocity_direction
        else:
            # 불일치시 속도 기반을 우선 (더 즉각적이고 정확)
            final_direction = velocity_direction
            self.get_logger().debug(
                f"Direction mismatch: velocity={velocity_direction}, position={position_direction}, using velocity"
            )
        
        # 현재 위치와 헤딩 저장
        self.prev_x = pose_x
        self.prev_y = pose_y
        self.prev_yaw = yaw
        
        return final_direction
    
    def odometry_callback(self, msg):
        try:
            # Pose 데이터 추출
            pose_x = msg.pose.pose.position.x
            pose_y = msg.pose.pose.position.y
            pose_z = msg.pose.pose.position.z
            orient_x = msg.pose.pose.orientation.x
            orient_y = msg.pose.pose.orientation.y
            orient_z = msg.pose.pose.orientation.z
            orient_w = msg.pose.pose.orientation.w
            
            # 헤딩(yaw) 계산
            yaw = self.quaternion_to_yaw(orient_x, orient_y, orient_z, orient_w)
            yaw_degrees = math.degrees(yaw)
            
            # Twist 데이터 추출
            linear_x = msg.twist.twist.linear.x
            linear_y = msg.twist.twist.linear.y
            linear_z = msg.twist.twist.linear.z
            angular_x = msg.twist.twist.angular.x
            angular_y = msg.twist.twist.angular.y
            angular_z = msg.twist.twist.angular.z
            
            # 헤딩을 고려한 정확한 전진/후진 방향 판단
            direction = self.determine_direction(linear_x, linear_y, pose_x, pose_y, yaw)
            
            # Pose 데이터를 파일에 저장 (yaw 각도 포함)
            pose_line = f"{pose_x:.6f} {pose_y:.6f} {pose_z:.6f} {orient_x:.6f} {orient_y:.6f} {orient_z:.6f} {orient_w:.6f} {yaw_degrees:.2f} {direction}\n"
            with open(self.pose_file, 'a') as f:
                f.write(pose_line)
            
            # Twist 데이터를 파일에 저장 (방향 정보 추가)
            twist_line = f"{linear_x:.6f} {linear_y:.6f} {linear_z:.6f} {angular_x:.6f} {angular_y:.6f} {angular_z:.6f} {direction}\n"
            with open(self.twist_file, 'a') as f:
                f.write(twist_line)
            
            self.get_logger().info(
                f"Data saved - Direction: {direction}, Yaw: {yaw_degrees:.1f}°, "
                f"linear_x: {linear_x:.3f}, Pose: ({pose_x:.3f}, {pose_y:.3f})"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error in odometry callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    logger = None
    try:
        logger = OdometryLogger("pose_data.txt", "twist_data.txt")
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("Odometry logger stopped by user.")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        if logger is not None:
            logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()