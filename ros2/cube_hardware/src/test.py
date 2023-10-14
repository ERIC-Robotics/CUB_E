#!/usr/bin/python3
import rclpy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    print("Received Odometry Data:")
    print("Position (x, y, z):", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    print("Orientation (x, y, z, w):", msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

def main():
    rclpy.init()
    node = rclpy.create_node('odom_subscriber')

    # Create a subscriber to the 'odom/filtered' topic with the message type Odometry
    subscriber = node.create_subscription(Odometry, 'odometry/filtered', odom_callback, 10)

    # Spin the node so the callback function can be called
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup when the node is shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
