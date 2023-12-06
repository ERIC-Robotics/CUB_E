import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanSubscriberNode(Node):
    def __init__(self):
        super().__init__('lidar_min')

        # Create a subscriber for the scan topic
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/ydlidar/scan',  # Replace 'scan' with the actual name of your scan topic
            self.scan_callback,
            10  # QoS profile depth
        )
        self.scan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.new_lidar = LaserScan()

        self.get_logger().info('Subscribed to scan topic')


    def scan_callback(self, msg):
        ind = []
        self.new_lidar = msg
        # print(len(msg.ranges))
        # for i in range(len(msg.ranges)):
        #     # angle = i * self.angle_inc
        #     if(msg.ranges[i] < 0.25 and msg.ranges[i] != 0.0):
        #         val = []
        #         val.append(i)
        #         val.append(msg.ranges[i])
        #         ind.append(val)
        #         self.new_lidar.ranges[i] = 0.0
        # print(ind)
        self.new_lidar.ranges[70] = 0.0
        self.new_lidar.ranges[64] = 0.0
        self.new_lidar.ranges[63] = 0.0
        self.new_lidar.ranges[71] = 0.0
        self.new_lidar.ranges[437] = 0.0
        self.new_lidar.ranges[438] = 0.0
        self.new_lidar.ranges[439] = 0.0
        # for i in range(len(self.new_lidar.ranges)):
        #     # angle = i * self.angle_inc
        #     if(self.new_lidar.ranges[i] < 0.25 and self.new_lidar.ranges[i] != 0.0):
        #         print(i, self.new_lidar.ranges[i])
        self.scan_publisher.publish(self.new_lidar)



def main(args=None):
    rclpy.init(args=args)
    scan_subscriber_node = ScanSubscriberNode()
    rclpy.spin(scan_subscriber_node)
    scan_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
