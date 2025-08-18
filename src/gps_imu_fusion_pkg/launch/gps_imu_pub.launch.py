#!/usr/bin/env python3

#=====================================================#
# 기능:
# - UM7, IMU 시리얼 데이터를 ros2 topic으로 발행하고 싱크를 맞춰주는 launch
# - NODE: 5개
#   - umx_driver um7_driver
#   - gps_imu_fusion_pkg imu_repub
#   - ublox_gps ublox_gps_node_base-launch.py
#   - gps_imu_fusion_pkg gps_repub
#   - gps_imu_fusion_pkg gps_imu_sync
#
# TODO : 작업 완료
#
# 최종 수정일: 2025.08.18
# 편집자: 송준상, 이다빈, 신민규
#=====================================================#

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # UM7 드라이버 노드
    um7_driver_node = Node(
        package='umx_driver',
        executable='um7_driver',
        name='imu',
        output='screen',
        parameters=[{'port': '/dev/ttyUSB0'}, {'tf_ned_to_enu': False}, {'orientation_in_robot_frame': False}]  # arguments → parameters로 수정
    )
    # IMU 재발행 노드
    imu_repub_node = Node(
        package='gps_imu_fusion_pkg',
        executable='imu_repub',
        name='imu_repub',
        output='screen'
    )
    # 다른 launch 파일 포함
    # ublox GPS 노드
    ublox_gps_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ublox_gps'),
            '/launch/ublox_gps_node_base-launch.py'
        ]),
    )
    # GPS 재발행 노드
    gps_repub_node = Node(
        package='gps_imu_fusion_pkg',
        executable='gps_repub',
        name='gps_repub'
    )
    # GPS와 IMU 싱크 맞는지 확인하는 노드
    gps_imu_sync = Node(
        package='gps_imu_fusion_pkg',
        executable='gps_imu_sync',
        name='gps_imu_sync'
    )
    return LaunchDescription([
        um7_driver_node,
        imu_repub_node,
        ublox_gps_node,
        gps_repub_node,
        gps_imu_sync
    ])
