#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class OdomBaseLinkBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_base_link_broadcaster')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        # Create a TransformStamped message
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = self.get_clock().now().to_msg()
        odom_to_base_link.header.frame_id = 'odom'
        odom_to_base_link.child_frame_id = 'base_link'

        # Set the translation
        odom_to_base_link.transform.translation.x = msg.pose.pose.position.x
        odom_to_base_link.transform.translation.y = msg.pose.pose.position.y
        odom_to_base_link.transform.translation.z = msg.pose.pose.position.z

        # Set the rotation
        odom_to_base_link.transform.rotation = msg.pose.pose.orientation

        # Publish the transform
        self.tf_broadcaster.sendTransform(odom_to_base_link)

def main(args=None):
    try:
        rclpy.init(args=args)
        odom_base_link_broadcaster = OdomBaseLinkBroadcaster()
        rclpy.spin(odom_base_link_broadcaster)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
