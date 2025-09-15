#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
import threading
import time
import math
import socket
from threading import Thread, Event
import pynmea2

# GPS와 IMU 처리 클래스 import
import struct


class GPSConnector:
  def __init__(self, network_type): 
    self.gpsClient = None
    self.networkType = network_type
    self.connChk = False        
    self.recvChk = False
    self.event = Event()

    self.pos_x = 126.773287
    self.pos_y = 37.229319
  
  def __del__(self):
    print('gps_del')

  def connect(self, host, port, topic):
    if self.networkType == 'UDP':
      try:
        self.gpsClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gpsClient.setblocking(False)
        self.gpsClient.settimeout(1)
        self.gpsClient.bind((host,port))       

        self.gpsRecvThread = Thread(target = self.position, args=())
        self.gpsRecvThread.setDaemon(True)
        self.gpsRecvThread.start()

      except Exception as e:
        print(f'gps_connect :{e}')
        
    else:
        from morai_msgs.msg import GPSMessage
        import rospy   
        self.gpsClient = rospy.Subscriber(topic ,GPSMessage, self.gpsCB)
        try:
          rospy.wait_for_message(topic,GPSMessage,timeout=1)
        except rospy.exceptions.ROSException:
          pass

    self.connChk = True

  def gpsCB(self,data):    
    self.pos_x = data.longitude
    self.pos_y = data.latitude
    self.recvChk = True

  def disconnect(self):
    if self.networkType == 'UDP':
      if self.connChk:
        self.connChk = False
        if self.gpsRecvThread.is_alive():
          self.event.set()
          self.gpsRecvThread.join()
        self.gpsClient.close()
      
    else:
      self.gpsClient.unregister()
    

  def position(self):
    while True:
      try:
        if self.event.is_set():
          break
        datas, host = self.gpsClient.recvfrom(8196)
        asciiDatas = datas.decode('ascii')
        gpdatas = asciiDatas.split('\r\n')
        
        gpgga = pynmea2.parse(gpdatas[0])

        lats = gpgga.latitude
        longs = gpgga.longitude
        self.pos_y = lats
        self.pos_x = longs #longitude is negaitve

        self.recvChk = True

      except socket.timeout:
        if self.recvChk:
          continue
        else:
          self.recvChk = False
          break
      except Exception as e:
        print(f'gps_poistion : {e}')


  def getPose(self):
    return (self.pos_x, self.pos_y)

class IMUConnector:
  def __init__(self, network_type):
    self.imuClient = None    
    self.networkType = network_type
    self.connChk = False
    self.recvChk = False
    self.event = Event()

    self.imu_data = IMUINFO()
  
  def __del__(self):
    print('imu_del')

  def connect(self, host, port, topic):
    if self.networkType == 'UDP':
      try:
        self.imuClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.imuClient.setblocking(False)
        self.imuClient.settimeout(1)
        self.imuClient.bind((host,port))        

        self.imuRecvThread = Thread(target = self.imu, args=())
        self.imuRecvThread.daemon = True 
        self.imuRecvThread.start()

      except Exception as e:
        print(f'imu_connect : {e}')

    else:
        import rospy
        from sensor_msgs.msg import Imu
        self.imuClient = rospy.Subscriber(topic,Imu,self.imuCB)
        try:
          rospy.wait_for_message(topic,Imu,timeout=1)
        except rospy.exceptions.ROSException:
          pass

    self.connChk = True
          

  def imuCB(self,data):
    self.imu_data.orientation_x = data.orientation.x
    self.imu_data.orientation_y = data.orientation.y
    self.imu_data.orientation_z = data.orientation.z
    self.imu_data.orientation_w = data.orientation.w
    self.imu_data.angular_velocity_x = data.angular_velocity.x
    self.imu_data.angular_velocity_y = data.angular_velocity.y
    self.imu_data.angular_velocity_z = data.angular_velocity.z
    self.imu_data.linear_acceleration_x = data.linear_acceleration.x
    self.imu_data.linear_acceleration_y = data.linear_acceleration.y
    self.imu_data.linear_acceleration_z = data.linear_acceleration.z

    self.recvChk = True
  
  def disconnect(self):
    if self.networkType == 'UDP':
      if self.connChk:
        self.connChk = False
        if self.imuRecvThread.is_alive():
          self.event.set()
          self.imuRecvThread.join()
        self.imuClient.close()
    else:
      self.imuClient.unregister()
      self.imuClient = None

  def imu(self):    
    while True:
      try:
        if self.event.is_set():
          break
        raw_data, sender = self.imuClient.recvfrom(65535)
        header = raw_data[0:9].decode()
        if header == '#IMUData$':
            
            data_length = struct.unpack('i', raw_data[9:13])
            data = struct.unpack('10d', raw_data[33:113])
            self.imu_data.orientation_x = data[1]
            self.imu_data.orientation_y = data[2]
            self.imu_data.orientation_z = data[3]
            self.imu_data.orientation_w = data[0]
            self.imu_data.angular_velocity_x = data[4]
            self.imu_data.angular_velocity_y = data[5]
            self.imu_data.angular_velocity_z = data[6]
            self.imu_data.linear_acceleration_x = data[7]
            self.imu_data.linear_acceleration_y = data[8]
            self.imu_data.linear_acceleration_z = data[9]
            
            self.recvChk = True

      except socket.timeout:
        if self.recvChk:
          continue
        else:
          self.recvChk = False
          break
      except Exception as e:
        print(f'imu_data :{e}')
      
  def getIMU(self):
    return self.imu_data

class IMUINFO :
  def __init__(self):
    self.orientation_x = None
    self.orientation_y = None
    self.orientation_z = None
    self.orientation_w = None
    self.angular_velocity_x = None
    self.angular_velocity_y = None
    self.angular_velocity_z = None
    self.linear_acceleration_x = None
    self.linear_acceleration_y = None
    self.linear_acceleration_z = None 



  def getIMU(self):
    return self.imu_data
  



class UDPToROS2Publisher(Node):
    def __init__(self):
        super().__init__('udp_to_ros2_publisher')

        # ROS2 Publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/morai/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # GPS와 IMU Connectors
        self.gps_connector = GPSConnector('UDP')
        self.imu_connector = IMUConnector('UDP')

        # 연결 설정
        self.gps_host = '127.0.0.1'
        self.gps_port = 9098
        self.imu_host = '127.0.0.1'
        self.imu_port = 9096

        # 발행 주기 설정 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_data)

        # 초기화 및 연결
        self.initialize_connections()

        self.get_logger().info("UDP to ROS2 Publisher started")
        self.get_logger().info(f"GPS: {self.gps_host}:{self.gps_port} -> /fix")
        self.get_logger().info(f"IMU: {self.imu_host}:{self.imu_port} -> /imu/data")

    def initialize_connections(self):
        """UDP 연결 초기화"""
        try:
            # GPS 연결
            self.gps_connector.connect(self.gps_host, self.gps_port, None)
            self.get_logger().info("GPS UDP connection established")
        except Exception as e:
            self.get_logger().error(f"GPS connection failed: {e}")

        try:
            # IMU 연결
            self.imu_connector.connect(self.imu_host, self.imu_port, None)
            self.get_logger().info("IMU UDP connection established")
        except Exception as e:
            self.get_logger().error(f"IMU connection failed: {e}")

    def publish_data(self):
        """GPS와 IMU 데이터를 ROS2 토픽으로 발행"""
        current_time = self.get_clock().now().to_msg()

        # GPS 데이터 발행
        self.publish_gps_data(current_time)

        # IMU 데이터 발행
        self.publish_imu_data(current_time)

    def publish_gps_data(self, timestamp):
        """GPS 데이터를 NavSatFix 메시지로 발행"""
        if not self.gps_connector.connChk:
            return

        try:
            pos_x, pos_y = self.gps_connector.getPose()

            # NavSatFix 메시지 생성
            gps_msg = NavSatFix()

            # Header 설정
            gps_msg.header = Header()
            gps_msg.header.stamp = timestamp
            gps_msg.header.frame_id = 'gps'

            # GPS 데이터 설정
            gps_msg.latitude = pos_y
            gps_msg.longitude = pos_x
            gps_msg.altitude = 0.0  # 고도 정보가 없으므로 0으로 설정

            # 상태 설정
            if self.gps_connector.recvChk:
                gps_msg.status.status = 0  # STATUS_FIX
                gps_msg.status.service = 1  # SERVICE_GPS
            else:
                gps_msg.status.status = -1  # STATUS_NO_FIX
                gps_msg.status.service = 0

            # 공분산 행렬 (단위 행렬로 초기화)
            gps_msg.position_covariance = [1.0, 0.0, 0.0,
                                          0.0, 1.0, 0.0,
                                          0.0, 0.0, 1.0]
            gps_msg.position_covariance_type = 1  # COVARIANCE_TYPE_APPROXIMATED

            # 메시지 발행
            self.gps_pub.publish(gps_msg)

        except Exception as e:
            self.get_logger().error(f"GPS publish error: {e}")

    def publish_imu_data(self, timestamp):
        """IMU 데이터를 Imu 메시지로 발행"""
        if not self.imu_connector.connChk:
            return

        try:
            imu_data = self.imu_connector.getIMU()

            # IMU 메시지 생성
            imu_msg = Imu()

            # Header 설정
            imu_msg.header = Header()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = 'imu_link'

            # Orientation (quaternion) 설정
            if all(val is not None for val in [imu_data.orientation_x, imu_data.orientation_y,
                                              imu_data.orientation_z, imu_data.orientation_w]):
                imu_msg.orientation.x = imu_data.orientation_x
                imu_msg.orientation.y = imu_data.orientation_y
                imu_msg.orientation.z = imu_data.orientation_z
                imu_msg.orientation.w = imu_data.orientation_w
            else:
                # 기본값 설정 (단위 quaternion)
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation.w = 1.0

            # Angular velocity 설정
            if all(val is not None for val in [imu_data.angular_velocity_x, imu_data.angular_velocity_y,
                                              imu_data.angular_velocity_z]):
                imu_msg.angular_velocity.x = imu_data.angular_velocity_x
                imu_msg.angular_velocity.y = imu_data.angular_velocity_y
                imu_msg.angular_velocity.z = imu_data.angular_velocity_z
            else:
                imu_msg.angular_velocity.x = 0.0
                imu_msg.angular_velocity.y = 0.0
                imu_msg.angular_velocity.z = 0.0

            # Linear acceleration 설정
            if all(val is not None for val in [imu_data.linear_acceleration_x, imu_data.linear_acceleration_y,
                                              imu_data.linear_acceleration_z]):
                imu_msg.linear_acceleration.x = imu_data.linear_acceleration_x
                imu_msg.linear_acceleration.y = imu_data.linear_acceleration_y
                imu_msg.linear_acceleration.z = imu_data.linear_acceleration_z
            else:
                # 기본값 설정 (중력 가속도)
                imu_msg.linear_acceleration.x = 0.0
                imu_msg.linear_acceleration.y = 0.0
                imu_msg.linear_acceleration.z = 9.81

            # 공분산 행렬 설정 (단위 행렬로 초기화)
            # Orientation covariance
            imu_msg.orientation_covariance = [0.1, 0.0, 0.0,
                                             0.0, 0.1, 0.0,
                                             0.0, 0.0, 0.1]

            # Angular velocity covariance
            imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                                  0.0, 0.01, 0.0,
                                                  0.0, 0.0, 0.01]

            # Linear acceleration covariance
            imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                                     0.0, 0.01, 0.0,
                                                     0.0, 0.0, 0.01]

            # 메시지 발행
            self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"IMU publish error: {e}")

    def destroy_node(self):
        """노드 종료 시 UDP 연결 해제"""
        try:
            self.gps_connector.disconnect()
            self.get_logger().info("GPS connection closed")
        except Exception as e:
            self.get_logger().error(f"GPS disconnect error: {e}")

        try:
            self.imu_connector.disconnect()
            self.get_logger().info("IMU connection closed")
        except Exception as e:
            self.get_logger().error(f"IMU disconnect error: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = UDPToROS2Publisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
