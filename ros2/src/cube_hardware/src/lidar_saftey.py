import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64, String

from cube_utils.msg import LidarSaftey


class ScanSubscriberNode(Node):
    def __init__(self):
        super().__init__('scan_subscriber_node')

        # Create a subscriber for the scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Replace 'scan' with the actual name of your scan topic
            self.scan_callback,
            10  # QoS profile depth
        )

        self.lidar_saftey_debug_pub = self.create_publisher(LaserScan, '/es_status/software/zone/debug', 10)
        self.debug = LaserScan()

        self.lidar_saftey_pub = self.create_publisher(LidarSaftey, '/es_status/software/zone', 10)

        self.forward_dist = 0.393
        self.side_dist = 0.435
        self.back_dist = 0.477


        self.index = [0, 0, 0, 0]
        self.angle_inc = 0.012466637417674065

        self.calculate_index()
        self.lidar_saftey_msg = LidarSaftey()

        self.get_logger().info('Subscribed to scan topic')

    def calculate_index(self):
        self.index[0] = np.arctan(self.side_dist/self.back_dist)
        self.index[1] = np.pi - np.arctan(self.side_dist/self.forward_dist)
        self.index[2] = np.pi + np.arctan(self.side_dist/self.forward_dist)
        self.index[3] = 2 * np.pi - np.arctan(self.side_dist/self.back_dist)
        print(self.index)



    def scan_callback(self, msg):

        self.lidar_saftey_msg = LidarSaftey()

        #debug
        self.debug = msg

        for i in range(len(msg.ranges)):
            angle = i * self.angle_inc

            if angle < self.index[0] or angle > self.index[3]:
                # Region 1
                distance1 = self.back_dist / np.cos(angle)
                if msg.ranges[i] < abs(distance1) and msg.ranges[i] != 0.0:
                    self.get_logger().info("Not safe Back")
                    self.lidar_saftey_msg.back = 1
                    self.lidar_saftey_msg.object = 1
            if angle > self.index[0] and angle < self.index[1]:
                # Region 2
                distance2 = self.side_dist / np.sin(angle)
                if msg.ranges[i] < abs(distance2) and msg.ranges[i] != 0.0:
                    self.get_logger().info("Not safe Right")
                    self.lidar_saftey_msg.right = 1
                    self.lidar_saftey_msg.object = 1
            if angle > self.index[1] and angle < self.index[2]:
                # Region 3
                distance3 = self.forward_dist / np.cos(angle)
                if msg.ranges[i] < abs(distance3) and msg.ranges[i] != 0.0:
                    self.get_logger().info("Not safe Front")
                    self.lidar_saftey_msg.front = 1
                    self.lidar_saftey_msg.object = 1
            if angle > self.index[2] and angle < self.index[3]:
                # Region 4
                distance4 = self.side_dist / np.sin(angle)
                if msg.ranges[i] < abs(distance4) and msg.ranges[i] != 0.0:
                    self.get_logger().info("Not safe left")       
                    self.lidar_saftey_msg.left = 1
                    self.lidar_saftey_msg.object = 1

        for i in range(len(msg.ranges)):
            angle = i * self.angle_inc

            if angle < self.index[0] or angle > self.index[3]:
                # Region 1
                # print(angle)
                distance1 = self.back_dist / np.cos(angle)
                self.debug.ranges[i] = abs(distance1)
            if angle > self.index[0] and angle < self.index[1]:
                # Region 2
                distance2 = self.side_dist / np.sin(angle)
                self.debug.ranges[i] = abs(distance2)
            if angle > self.index[1] and angle < self.index[2]:
                # Region 3
                distance3 = self.forward_dist / np.cos(angle)
                self.debug.ranges[i] = abs(distance3)
            if angle > self.index[2] and angle < self.index[3]:
                # Region 4
                distance4 = self.side_dist / np.sin(angle)
                self.debug.ranges[i] = abs(distance4)

        self.lidar_saftey_pub.publish(self.lidar_saftey_msg)
        self.lidar_saftey_debug_pub.publish(self.debug)
        self.lidar_saftey_msg = LidarSaftey()

def main(args=None):
    rclpy.init(args=args)
    scan_subscriber_node = ScanSubscriberNode()
    rclpy.spin(scan_subscriber_node)
    scan_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
