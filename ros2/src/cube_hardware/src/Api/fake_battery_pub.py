#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import random
import time

class BatterySOCPublisher(Node):

    def __init__(self):
        super().__init__('battery_soc_publisher')
        self.publisher_ = self.create_publisher(Int64, '/battery_soc', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Int64()
        msg.data = random.randint(11, 13)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    battery_soc_publisher = BatterySOCPublisher()

    try:
        rclpy.spin(battery_soc_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        battery_soc_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
