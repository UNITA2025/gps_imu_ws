#!/usr/bin/env python3
# yaw_bias_cal_no_twist.py
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

def yaw_from_q(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def angle_diff(a, b):
    d = (a - b + math.pi) % (2*math.pi) - math.pi
    return d

class YawBiasCalNoTwist(Node):
    def __init__(self):
        super().__init__('yaw_bias_cal_no_twist')
        self.sub = self.create_subscription(Odometry, '/odometry/global', self.cb, 50)
        self.start = None
        self.d_min = 5.0       # 최소 누적거리(m)
        self.curv_max = math.radians(3.0)  # 평균 헤딩변화 임계(직진성 체크, 선택)
        self.last_pose = None
        self.dist_sum = 0.0
        self.yaw_change_sum = 0.0
        self.done = False

    def cb(self, msg: Odometry):
        if self.done:
            return
        p = msg.pose.pose
        x, y = p.position.x, p.position.y
        yaw = yaw_from_q(p.orientation)

        if self.start is None:
            self.start = (x, y, yaw)
            self.last_pose = (x, y, yaw)
            return

        # 누적거리 & 직진성 평가
        lx, ly, lyaw = self.last_pose
        dx, dy = x - lx, y - ly
        step = math.hypot(dx, dy)
        self.dist_sum += step
        self.yaw_change_sum += abs(angle_diff(yaw, lyaw))
        self.last_pose = (x, y, yaw)

        if self.dist_sum < self.d_min:
            return
        # 평균 헤딩변화가 너무 크면(=곡선 주행) 초기화하고 다시 측정
        avg_curv = self.yaw_change_sum / max(self.dist_sum, 1e-6)
        if avg_curv > self.curv_max:
            self.get_logger().warn(f'곡선 주행 감지(avg_curv={math.degrees(avg_curv):.2f}°/m). 다시 측정...')
            self.start = (x, y, yaw)
            self.last_pose = (x, y, yaw)
            self.dist_sum = 0.0
            self.yaw_change_sum = 0.0
            return

        # 진행방향(코스) φ와 odom yaw 비교 → Δψ
        x0, y0, yaw0 = self.start
        phi = math.atan2(y - x0 if False else (y - y0), x - x0)  # (= atan2(dy, dx))
        dpsi = angle_diff(phi, yaw)  # [-pi, pi]

        self.get_logger().info(f'Δψ ≈ {dpsi:.6f} rad ({math.degrees(dpsi):.2f}°)')
        self.get_logger().info('→ base_link→imure_link TF에 yaw=Δψ 추가(권장) '
                               '또는 navsat_transform.yaw_offset=Δψ 설정하세요.')
        self.done = True
        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(YawBiasCalNoTwist())

if __name__ == '__main__':
    main()
