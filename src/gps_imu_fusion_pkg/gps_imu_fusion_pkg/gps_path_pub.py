#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class GPSPathPublisher(Node):
    def __init__(self):
        super().__init__('gps_path_publisher')

        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/gps',
            self.odometry_callback,
            10
        )

        self.path_publisher = self.create_publisher(Path, '/gps_path', 10)
        self.path = Path()
        self.path.header.frame_id = "map"

    def odometry_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = msg.pose.pose

        self.path.poses.append(pose_stamped)
        self.path.header.stamp = self.get_clock().now().to_msg()

        # 최대 1000개 포인트만 유지
        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)

        self.path_publisher.publish(self.path)

def main():
    rclpy.init()
    node = GPSPathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
