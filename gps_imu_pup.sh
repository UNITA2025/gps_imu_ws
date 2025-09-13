sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/ttyACM0
source /opt/ros/humble/setup.bash
source ~/ntrip_ws/install/setup.bash
ros2 launch gps_imu_fusion_pkg gps_imu_pub.launch.py
