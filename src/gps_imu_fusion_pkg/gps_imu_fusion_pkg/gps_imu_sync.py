#!/usr/bin/env python3

#==================================================#
# 기능: GPS와 IMU 데이터를 0.1초 이내의 시간 와로 동기화 시켜서 fusion에 사용한다.
# - message_filters.Subscriber를 사용해서 GPS와 IMU 메시지를 동시에 구독하여 시간차가 0.1초 이내면 같은 시각으로 취급한다.
# - GPS와 IMU 메시지가 동기화 되면 호출하여 각각의 메시지에서 타임스탬프(header.stamp)를 초 단위로 변환하여 출력한다.
#
# 송신 토픽 (publish): 
#   - 
# 수신 토픽 (subscribe): 
#   - /gps/fix(msg: sensor_msgs/NavSatFix), /imure/data(msg: sensor_msgs/Imu)
#
# TODO : 
# 최종 수정일: 2025.08.18
# 편집자: 
#==================================================#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer

class GPSIMUSyncNode(Node):
    def __init__(self):
        super().__init__('gps_imu_sync_node')

        # 두 센서 토픽 구독
        self.gps_sub = Subscriber(self, NavSatFix, '/gps/fix')
        self.imu_sub = Subscriber(self, Imu, '/imure/data')

        # 동기화 객체 생성
        self.ats = ApproximateTimeSynchronizer(
            [self.gps_sub, self.imu_sub],  # 센서들
            queue_size=30,                 # 버퍼 크기
            slop=0.1                       # 시간 차 허용 범위 (0.1초 이내면 OK)
        )
        self.ats.registerCallback(self.sync_callback)

        self.get_logger().info("GPS-IMU 동기화 노드 실행 중...")

    def sync_callback(self, gps_msg, imu_msg):
        # ROS 2에서 타임스탬프를 초 단위로 변환
        gps_time = gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec * 1e-9
        imu_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9

        self.get_logger().info(
            f"SYNCED GPS time: {gps_time:.3f} | IMU time: {imu_time:.3f}"
        )
        # 여기에 EKF 입력 처리 or publish 가능

def main(args=None):
    rclpy.init(args=args)

    node = GPSIMUSyncNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
