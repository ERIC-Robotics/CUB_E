#!/usr/bin/env python3

"""
odom_to_base_link_tf_broadcaster.py: Broadcasts the transformation from 'odom' frame to 'base_link' frame.

This node subscribes to '/odometry/filtered' to receive odometry information, and then
broadcasts a corresponding transformation to tf2. This allows other nodes to transform
coordinates from the 'odom' frame to the 'base_link' frame and vice versa.

Authors: Jatin Patil
Version: 1.0
Last Updated: 2023-Nov-09
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

class OdomBaseLinkBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_base_link_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            10
        )

    def odom_callback(self, msg):
        # Create a TransformStamped message
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = self.get_clock().now().to_msg()
        odom_to_base_link.header.frame_id = msg.header.frame_id    
        odom_to_base_link.child_frame_id = msg.child_frame_id

        # Set the translation
        odom_to_base_link.transform.translation.x = msg.pose.pose.position.x
        odom_to_base_link.transform.translation.y = msg.pose.pose.position.y
        odom_to_base_link.transform.translation.z = msg.pose.pose.position.z

        # Set the rotation
        odom_to_base_link.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transformation
        self.tf_broadcaster.sendTransform(odom_to_base_link)

def main(args=None):
    try:
        rclpy.init(args=args)
        odom_base_link_broadcaster = OdomBaseLinkBroadcaster()
        rclpy.spin(odom_base_link_broadcaster)
    except KeyboardInterrupt:
        # Handle graceful shutdown
        pass
    except Exception as e:
        # Handle other exceptions such as connectivity loss or ROS errors
        pass
    finally:
        # Clean shutdown of the ROS client library
        rclpy.shutdown()

if __name__ == '__main__':
    main()
