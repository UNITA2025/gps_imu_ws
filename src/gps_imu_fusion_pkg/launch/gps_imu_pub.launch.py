#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('gps_imu_fusion_pkg')
    
    ekf_local_config_file = os.path.join(pkg_path, 'config', 'ekf_local.yaml')
    ekf_config_file = os.path.join(pkg_path, 'config', 'ekf_global.yaml')
    navsat_config_file = os.path.join(pkg_path, 'config', 'navsat_transform.yaml')
    
    return LaunchDescription([
        Node(
            package='umx_driver',
            executable='um7_driver',
            name='imu',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}]  # arguments → parameters로 수정
        ),
        Node(
            package='gps_imu_fusion_pkg',
            executable='imu_repub',
            name='imu_repub',
            output='screen'
        ),
        # 다른 launch 파일 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ublox_gps'),
                '/launch/ublox_gps_node_base-launch.py'
            ]),
        ),
        Node(
            package='gps_imu_fusion_pkg',
            executable='gps_repub',
            name='gps_repub'
        ),
        Node(
            package='gps_imu_fusion_pkg',
            executable='gps_imu_sync',
            name='gps_imu_sync'
        ),
    ])