sudo chmod a+rw /dev/ttyUSB0
ros2 run umx_driver um7_driver --ros-args -p port:=/dev/ttyUSB0
