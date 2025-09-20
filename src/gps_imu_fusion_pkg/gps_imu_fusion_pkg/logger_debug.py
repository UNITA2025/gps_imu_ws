#!/usr/bin/env python3
# 헤딩을 고려한 전진/후진 판단 디버깅용 코드

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class DebugHeadingLogger(Node):
    def __init__(self):
        super().__init__('debug_heading_logger')
        
        self.prev_x = None
        self.prev_y = None
        self.count = 0
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/local_enu',
            self.debug_callback,
            10
        )
        
        print("=== 헤딩 기반 전진/후진 판단 디버그 ===")
        print("로봇을 전진/후진 시켜보세요...")
        print("로봇이 어느 방향을 바라보든 정확히 판단해야 합니다!")
    
    def quaternion_to_yaw(self, x, y, z, w):
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    def debug_callback(self, msg):
        self.count += 1
        
        # 3번마다 한 번씩만 출력
        if self.count % 3 != 0:
            return
            
        # 데이터 추출
        linear_x = msg.twist.twist.linear.x  # 로봇 기준 전진 속도
        linear_y = msg.twist.twist.linear.y  # 로봇 기준 좌우 속도
        
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        
        # 헤딩 계산
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        yaw_degrees = math.degrees(yaw)
        
        print(f"\n--- 측정 #{self.count//3} ---")
        print(f"로봇 헤딩: {yaw_degrees:.1f}°")
        print(f"로봇 기준 속도 - 전진: {linear_x:.4f}, 좌우: {linear_y:.4f}")
        print(f"현재 위치: ({pose_x:.3f}, {pose_y:.3f})")
        
        # 로봇 기준 전진/후진 판단 (이 로봇은 좌표계가 반대)
        if abs(linear_x) < 0.02:
            robot_direction = "STOP"
        else:
            # 이 로봇에서는 linear_x < 0이 전진!
            robot_direction = "FORWARD" if linear_x < 0 else "REVERSE"
        
        print(f"🤖 로봇 기준 방향: {robot_direction}")
        
        # 월드 좌표계 속도로 변환
        world_vx = linear_x * math.cos(yaw) - linear_y * math.sin(yaw)
        world_vy = linear_x * math.sin(yaw) + linear_y * math.cos(yaw)
        
        print(f"🌍 월드 기준 속도: x={world_vx:.4f}, y={world_vy:.4f}")
        
        # 실제 위치 변화 기반 검증
        if self.prev_x is not None:
            dx = pose_x - self.prev_x
            dy = pose_y - self.prev_y
            distance = math.sqrt(dx**2 + dy**2)
            
            print(f"📍 위치 변화: dx={dx:.4f}, dy={dy:.4f}, 거리={distance:.4f}")
            
            if distance > 0.01:
                # 실제 이동 방향과 로봇 헤딩 비교
                movement_angle = math.atan2(dy, dx)
                movement_degrees = math.degrees(movement_angle)
                
                angle_diff = movement_angle - yaw
                angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                angle_diff_degrees = math.degrees(angle_diff)
                
                print(f"🧭 실제 이동 방향: {movement_degrees:.1f}°")
                print(f"📐 헤딩과의 차이: {angle_diff_degrees:.1f}°")
                
                # 각도 차이로 전진/후진 판단
                if abs(angle_diff_degrees) <= 90:
                    position_direction = "FORWARD"
                else:
                    position_direction = "REVERSE"
                
                print(f"📍 위치 기반 방향: {position_direction}")
                
                # 일치 여부 확인
                if robot_direction == position_direction:
                    print("✅ 로봇 기준과 위치 기준 판단 일치!")
                elif robot_direction == "STOP":
                    print("⏸️  로봇이 정지 상태")
                else:
                    print("❌ 판단 불일치 - 확인 필요!")
            else:
                print("⏸️  움직임이 너무 작음")
        
        self.prev_x = pose_x
        self.prev_y = pose_y
        
        print("=" * 40)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        logger = DebugHeadingLogger()
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("\n디버그 종료")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()