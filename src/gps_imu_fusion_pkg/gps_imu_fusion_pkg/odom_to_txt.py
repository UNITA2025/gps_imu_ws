
#========================================================================================
# 기능: /odometry/local_enu2의 Odometry를 구독해 Pose과 Twist(선형/각속도)를
#       각각 pose_data.txt, twist_data.txt 저장하는 코드.
# 동작
# 노드 시작 시 빈 파일에 구독 시작.
# 콜백마다 Pose(x, y, z, qx, qy, qz, qw), Twist(vx, vy, vz, wx, wy, wz) 를 로그로 출력, 파일에 저장
# TODO : 작업 완료
# 최종 수정일: 2025.08.18
# 편집자 : 송준상, 이다빈
#========================================================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os

class OdometryLogger(Node):
    def __init__(self, pose_file="pose_data.txt", twist_file="twist_data.txt"):
        super().__init__('odometry_logger')
        
        self.pose_file = pose_file
        self.twist_file = twist_file
        
        # 파일 초기화 (기존 내용 삭제)
        open(self.pose_file, 'w').close()
        open(self.twist_file, 'w').close()
        
        # /odometry/global 토픽 구독
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/local_enu2',
            self.odometry_callback,
            10  # QoS history depth
        )
        
        self.get_logger().info(f"Odometry logger started. Saving to {pose_file} and {twist_file}")
    
    def odometry_callback(self, msg):
        # Pose 데이터 추출
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        pose_z = msg.pose.pose.position.z
        orient_x = msg.pose.pose.orientation.x
        orient_y = msg.pose.pose.orientation.y
        orient_z = msg.pose.pose.orientation.z
        orient_w = msg.pose.pose.orientation.w
        
        # Twist 데이터 추출
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        linear_z = msg.twist.twist.linear.z
        angular_x = msg.twist.twist.angular.x
        angular_y = msg.twist.twist.angular.y
        angular_z = msg.twist.twist.angular.z
        
        # Pose 데이터를 파일에 저장
        pose_line = f"{pose_x} {pose_y} {pose_z} {orient_x} {orient_y} {orient_z} {orient_w}\n"
        
        with open(self.pose_file, 'a') as f:
            f.write(pose_line)
        
        # Twist 데이터를 파일에 저장
        twist_line = f"{linear_x} {linear_y} {linear_z} {angular_x} {angular_y} {angular_z}\n"
        
        with open(self.twist_file, 'a') as f:
            f.write(twist_line)
        
        self.get_logger().info(
            f"Data saved - Pose: ({pose_x:.3f}, {pose_y:.3f}, {pose_z:.3f}), "
            f"Twist: ({linear_x:.3f}, {linear_y:.3f}, {linear_z:.3f})"
        )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 저장할 파일명 지정 (선택사항)
        logger = OdometryLogger("pose_data.txt", "twist_data.txt")
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("Odometry logger stopped by user.")
    finally:
        if 'logger' in locals():
            logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()