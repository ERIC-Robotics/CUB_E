#!/usr/bin/python3

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToMotorCommand:
    def __init__(self):
        self.node = rclpy.create_node('cmdvel_motorCommand')
        
        self.wheel_radius = 0.05  # in meters
        self.wheel_separation = 0.33  # in meters

        self.left_pub = self.node.create_publisher(Float64, '/leftmotor/command', 10)
        self.right_pub = self.node.create_publisher(Float64, '/rightmotor/command', 10)
        
        self.cmd_vel_sub = self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, data):
        leftrpm = Float64()
        rightrpm = Float64()

        left_velocity = data.linear.x - (self.wheel_separation / 2) * data.angular.z
        right_velocity = data.linear.x + (self.wheel_separation / 2) * data.angular.z

        leftrpm.data = (left_velocity * 60) / (2 * math.pi * self.wheel_radius)
        rightrpm.data = (right_velocity * 60) / (2 * math.pi * self.wheel_radius)

        self.left_pub.publish(leftrpm)
        self.right_pub.publish(rightrpm)

    def run(self):
        rclpy.spin(self.node)
        self.node.destroy_node()

def main(args=None):
    try:
        rclpy.init()
        cmd_vel_to_motor_command = CmdVelToMotorCommand()
        cmd_vel_to_motor_command.run()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()


