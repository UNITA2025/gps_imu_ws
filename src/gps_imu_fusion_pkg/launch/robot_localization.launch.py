from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # IMU 좌표계 변환
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_transform_publisher',
            arguments=['0', '0', '0', '0', '3.14159', '0', 'base_link', 'imure_link'],
        ),

        # GPS 좌표계 변환
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps'],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',  # 로컬 EKF (IMU만)
            parameters=[{
                'frequency': 30.0,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',

                # IMU만 사용
                'imu0': '/imure/data',
                'imu0_config': [False, False, False,
                            True,  True,  True,
                            False, False, False,
                            True,  True,  True,
                            True,  True,  True]
            }],
            remappings=[('odometry/filtered', '/odometry/local')]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[{
                'frequency': 30.0,
                'delay': 3.0,
                'magnetic_declination_radians': 0.0,
                'yaw_offset': 0.0,
                'zero_altitude': True,
                'broadcast_utm_transform': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': True,  # True로 변경
                'wait_for_datum': False
            }],
            remappings=[
                ('/gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/local')  # 로컬 EKF 사용
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',  # 글로벌 EKF (IMU + GPS)
            parameters=[{
                'frequency': 30.0,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'map',  # 글로벌은 map 프레임

                # IMU + GPS 둘 다 사용
                'imu0': '/imure/data',
                'imu0_config': [False, False, False,
                            True,  True,  True,
                            False, False, False,
                            True,  True,  True,
                            True,  True,  True],

                'odom0': '/odometry/gps',
                'odom0_config': [True,  True,  True,
                                False, False, False,
                                False, False, False,
                                False, False, False,
                                False, False, False]
            }],
            remappings=[('odometry/filtered', '/odometry/global')]
        )
    ])
