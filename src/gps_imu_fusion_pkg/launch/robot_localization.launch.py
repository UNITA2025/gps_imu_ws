#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 설정
    pkg_path = get_package_share_directory('gps_imu_fusion_pkg')
    # 설정 파일 경로
    ekf_local_config_file = os.path.join(pkg_path, 'config', 'ekf_local.yaml')
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf_global.yaml')
    navsat_config_file = os.path.join(pkg_path, 'config', 'navsat_transform.yaml')

    return LaunchDescription([
        # # GPS 타임스탬프 재퍼블리셔
        # Node(
        #     package='gps_imu_fusion_pkg',
        #     executable='gps_repub',
        #     name='gps_repub_node',
        #     output='screen'
        # ),

        #         # 선택사항: GPS-IMU 동기화 모니터링
        # Node(
        #     package='gps_imu_fusion_pkg',
        #     executable='gps_imu_sync',
        #     name='gps_imu_sync_node',
        #     output='screen'
        # ),
        # Local EKF (IMU only for local odometry)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_local_config_file],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered'),
            ]
        ),
          # GPS coordinate transformation
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config_file],
            remappings=[
                ('/imu/data', '/imu/data'),
                ('/gps/fix', '/gps/fix'),
                ('/odometry/filtered', '/odometry/filtered'),
                ('/odometry/gps', '/odometry/gps')
            ]
        ),

        # Global EKF (IMU + GPS fusion)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_config_file],
            remappings=[
                ('/odometry/filtered', '/odometry/global'),
            ]
        )
    ])
