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
        arguments=['0', '0', '-0.2', '0', '0', '0', 'base_link', 'imure_link'],
    )

    # GPS 좌표계 변환
    tf2_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps'],
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

    local_origin_settterr_node = Node(
        package='gps_imu_fusion_pkg',
        executable='local_origin_settterr',
        name='local_origin_settterr',
    )

    osm_map_publisher_node = Node(
        package='gps_imu_fusion_pkg',
        executable='osm_map_publisher',
        name='osm_map_publisher',
        parameters=[{
            'osm_file': '/home/songsong/gps_imu_ws/map_imu.osm',
            'resolution': 0.2,
            'map_width': 500.0,
            'map_height': 1000.0,
            'rotation_angle': 174.0,
            'gps_topic': '/gps/fix',
            'map_frame': 'map',
            'publish_rate': 1.0
        }]
    )
    return LaunchDescription([
        tf2_map_to_odom,
        tf2_base_to_gps,
        tf2_base_to_imure,
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
        local_origin_settterr_node,
        osm_map_publisher_node,
    ])
