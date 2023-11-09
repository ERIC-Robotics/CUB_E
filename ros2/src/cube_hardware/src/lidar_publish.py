#!/usr/bin/python3

"""
lidar_publish.py: Publish the lidar data on /scan topic

It subscribes on /ydlidar/scan topic and updates its header timestamp
with current time stamp and publishes it on /scan topic.

Authors: Jatin Patil
Version: 1.0
Last Updated: 2023-Nov-09
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

# Author details and metadata
__author__ = "Your Name"
__version__ = "1.0"
__status__ = "Development"
__date__ = "2023-11-09"

class ScanAndPublishNode(Node):
    """
    Node that subscribes to laser scan messages and republishes them.
    
    This node listens to laser scan data from a specified topic, possibly
    performs some processing, and then republishes the scan data to another topic.
    """

    def __init__(self):
        """
        Constructor to initialize the ScanAndPublishNode.
        """
        # Initialize the Node with the name 'scan_and_publish'
        super().__init__('scan_and_publish')
        
        # Subscribe to the LaserScan topic from the LiDAR sensor
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/ydlidar/scan',
            self.scan_callback,
            10  # Quality of Service (QoS) profile depth
        )
        
        # Set up a publisher for the processed LaserScan data
        self.newscan_publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10  # QoS profile depth
        )
        
        # Placeholder for the new scan data to publish
        self.newscan = LaserScan()

    def scan_callback(self, scan_msg):
        """
        Callback function for the LaserScan subscriber.
        
        Processes incoming scan data and publishes it to a new topic.
        
        Parameters:
            scan_msg (LaserScan): The incoming laser scan message.
        """
        # Simply republish the received scan data with an updated timestamp
        self.newscan = scan_msg
        # Debug print to see the scan message's timestamp in seconds
        print(scan_msg.header.stamp.sec)
        # Update the timestamp to the current time
        self.newscan.header.stamp = self.get_clock().now().to_msg()
        # Publish the new scan data
        self.newscan_publisher.publish(self.newscan)

def main(args=None):
    """
    Main function to initialize the ROS2 node and spin it.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    # Instantiate the node
    scan_and_publish_node = ScanAndPublishNode()

    try:
        # Spin the node so the callback function is called.
        rclpy.spin(scan_and_publish_node)
    except KeyboardInterrupt:
        # Handle shutdown on Ctrl+C
        pass
    finally:
        # Clean up before shutting down
        scan_and_publish_node.destroy_node()
        rclpy.shutdown()

# This is the standard boilerplate that calls the main() function.
if __name__ == '__main__':
    main()
