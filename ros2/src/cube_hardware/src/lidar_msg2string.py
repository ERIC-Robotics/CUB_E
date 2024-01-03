#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from cube_utils.msg import LidarSaftey


class ZoneStatusNode(Node):
    def __init__(self):
        super().__init__("zone_status_node")
        self.lidar_msg_sub = self.create_subscription(
            LidarSaftey, "/es_status/software/zone", self.listener_callback, 10
        )
        self.array_msg_pub = self.create_publisher(
            Int64MultiArray, "/es_status/software/zone/led", 10
        )
        self.array_msg_pub_timer = self.create_timer(0.5, self.array_msg_pub_callback)
        self.led_status = Int64MultiArray()
        self.led_status.data = [0, 0, 0, 0, 0]

    def listener_callback(self, msg):
        self.led_status.data[0] = msg.object
        self.led_status.data[1] = msg.front
        self.led_status.data[2] = msg.left
        self.led_status.data[3] = msg.back
        self.led_status.data[4] = msg.right


    def array_msg_pub_callback(self):
        self.array_msg_pub.publish(self.led_status)


def main(args=None):
    rclpy.init(args=args)
    zone_status_node = ZoneStatusNode()
    rclpy.spin(zone_status_node)
    zone_status_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
