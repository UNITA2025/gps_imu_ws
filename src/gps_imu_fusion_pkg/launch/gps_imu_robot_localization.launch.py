#!/usr/bin/env python3

#=====================================================#
# 기능:
# - GPS와 IMU 데이터를 로봇의 로컬 좌표계로 변환하고 EKF를 통해 필터링하여 odometry를 생성하는 launch
# - NODE: 7개
#   - tf2_ros static_transform_publisher (map to odom)
#   - tf2_ros static_transform_publisher (base_link to gps)
#   - tf2_ros static_transform_publisher (base_link to imure_link)
#   - robot_localization ekf_node (ekf_local)
#   - robot_localization navsat_transform_node (navsat_transform)
#   - robot_localization ekf_node (ekf_global)
#   - gps_imu_fusion_pkg local_origin_setter
#   - gps_imu_fusion_pkg osm_map_publisher

# TODO : 작업 완료
#
# 최종 수정일: 2025.08.18
# 편집자: 송준상, 이다빈, 신민규
#=====================================================#


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'gps_imu_fusion_pkg'
    config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    ekf_local_params_file = os.path.join(config_dir, 'ekf_local.yaml')
    ekf_global_params_file = os.path.join(config_dir, 'ekf_global.yaml')
    navsat_transform_params_file = os.path.join(config_dir, 'navsat_transform.yaml')
    map_kcity = os.path.join(config_dir, 'map_kcity.osm')
    map_inu = os.path.join(config_dir, 'map_inu.osm')
    map_inu2 = os.path.join(config_dir, 'map_inu2.osm')

    # OSM 맵 좌표계를 odom 좌표계로 변환하는 노드
    tf2_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    # IMU 좌표계 변환
    tf2_base_to_imure = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_transform_publisher',
        arguments=['0', '0', '0.8', '0', '0', '0', 'base_link', 'imure_link'],
    )

    # GPS 좌표계 변환
    tf2_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_transform_publisher',
        arguments=['0', '0', '1', '0', '0', '0', 'base_link', 'gps'],
    )

    #local_ekf only IMU
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        # YAML 파일 경로를 parameters에 바로 넣어줍니다.
        parameters=[ekf_local_params_file]
    )

    #navsat for gps
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[navsat_transform_params_file],
    )

    #global ekf IMU + GPS
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',  # 글로벌 EKF (IMU + GPS)
        parameters=[ekf_global_params_file],
        remappings=[('odometry/filtered', '/odometry/global')]
    )

    #글로벌 좌표를 로컬좌표(0,0)으로 변경하고 헤딩을 실시간 보정해주는 노드
    local_origin_setter_node = Node(
        package='gps_imu_fusion_pkg',
        executable='local_origin_setter',
        name='local_origin_setter',
    )

    osm_map_publisher_node = Node(
        package='gps_imu_fusion_pkg',
        executable='osm_map_publisher',
        name='osm_map_publisher',
        parameters=[{
            'osm_file': map_inu2, # OSM 파일 주소
            'resolution': 0.2,  # 맵 해상도 (m/pixel)
            'map_width': 500.0, # 맵 너비 (m)
            'map_height': 1200.0, # 맵 높이 (m)
            'rotation_angle': 174.0, # 맵 회전 각도 (degrees)
            'gps_topic': '/gps/fix', # GPS 토픽
            'map_frame': 'map', # 맵 프레임
            'publish_rate': 1.0 # 맵 발행 주기 (Hz)
        }]
    )

    return LaunchDescription([
        tf2_map_to_odom,
        tf2_base_to_gps,
        tf2_base_to_imure,
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
        local_origin_setter_node,
        osm_map_publisher_node,
    ])
