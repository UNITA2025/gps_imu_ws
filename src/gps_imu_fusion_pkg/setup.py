from setuptools import find_packages, setup

package_name = 'gps_imu_fusion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gps_imu_robot_localization.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gps_display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gps_imu_pub.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf_global.yaml']),
        ('share/' + package_name + '/config', ['config/navsat_transform.yaml']),
        ('share/' + package_name + '/config', ['config/ekf_local.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='songsong',
    maintainer_email='songsong@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'gps_repub = {package_name}.gps_repub:main',
            f'gps_imu_sync = {package_name}.gps_imu_sync:main',
            f'gps_path_pub = {package_name}.gps_path_pub:main',
            f'gps_to_point = {package_name}.gps_to_point:main',
            f'gps_to_web = {package_name}.gps_to_web:main',
            f'gps_with_grid = {package_name}.gps_with_grid:main',
            f'gps_to_csv = {package_name}.gps_to_csv:main',
            f'imu_to_csv = {package_name}.imu_to_csv:main',
            f'imu_repub = {package_name}.imu_repub:main',
            f'local_origin_setter = {package_name}.local_origin_setter:main',
            f'local_origin_settterr = {package_name}.local_origin_settterr:main',
            f'odom_to_txt = {package_name}.odom_to_txt:main',
            f'utm_map_publisher = {package_name}.utm_map_publisher:main',
            f'osm_map_publisher = {package_name}.osm_map_publisher:main',
        ],
    },
)
