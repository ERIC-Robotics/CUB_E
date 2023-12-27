#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64

voltage = 0.0

class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            '/battery_soc',
            self.battery_callback,
            10)

    def battery_callback(self, msg):
        global voltage
        voltage = msg.data

    def get_voltage(self):
        return voltage

def get_latest_voltage():
    return voltage

battery_subscriber = None

def main(args=None):
    rclpy.init(args=args)
    global battery_subscriber
    battery_subscriber = BatterySubscriber()
    rclpy.spin(battery_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
