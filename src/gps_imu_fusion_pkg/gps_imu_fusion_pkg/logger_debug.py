#!/usr/bin/env python3
# 전진/후진 판단 문제 디버깅용 코드

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class DebugDirectionLogger(Node):
    def __init__(self):
        super().__init__('debug_direction_logger')
        
        self.prev_x = None
        self.prev_y = None
        self.count = 0
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/local_enu',
            self.debug_callback,
            10
        )
        
        print("=== 전진/후진 판단 디버그 시작 ===")
        print("로봇을 전진/후진 시켜보세요...")
    
    def debug_callback(self, msg):
        self.count += 1
        
        # 5번마다 한 번씩만 출력 (너무 많은 출력 방지)
        if self.count % 5 != 0:
            return
            
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        
        print(f"\n--- 측정 #{self.count//5} ---")
        print(f"linear_x: {linear_x:.4f}")
        print(f"linear_y: {linear_y:.4f}")
        print(f"Position: ({pose_x:.3f}, {pose_y:.3f})")
        
        # 속도 기반 판단
        if abs(linear_x) < 0.02:
            vel_direction = "STOP"
        else:
            vel_direction = "FORWARD" if linear_x > 0 else "REVERSE"
        
        print(f"속도 기반 방향: {vel_direction}")
        
        # 위치 변화 기반 판단
        if self.prev_x is not None:
            dx = pose_x - self.prev_x
            dy = pose_y - self.prev_y
            distance = math.sqrt(dx**2 + dy**2)
            
            print(f"위치 변화: dx={dx:.4f}, dy={dy:.4f}, 거리={distance:.4f}")
            
            if distance > 0.01:
                if abs(dx) > abs(dy):  # X축 이동이 주된 이동
                    pos_direction = "FORWARD" if dx > 0 else "REVERSE"
                else:
                    pos_direction = "LATERAL (좌우이동)"
            else:
                pos_direction = "STOP"
                
            print(f"위치 기반 방향: {pos_direction}")
            
            # 일치 여부 확인
            if vel_direction == pos_direction:
                print("✅ 속도와 위치 기반 판단 일치!")
            else:
                print("❌ 속도와 위치 기반 판단 불일치!")
        
        self.prev_x = pose_x
        self.prev_y = pose_y
        
        print("-" * 30)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        logger = DebugDirectionLogger()
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("\n디버그 종료")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()