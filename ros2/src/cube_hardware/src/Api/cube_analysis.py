#!/usr/bin/python3


import rclpy
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from rclpy.node import Node
import math
import random


class CubeAnalytics(Node):
    def __init__(self):
        super().__init__("api_analytics_node")

        self.left_position = 0
        self.right_position = 0
        self.last_left_position = 0
        self.last_right_position = 0
        self.wheel_distance = 0.45
        self.wheel_radius = 0.08
        self.count_per_revolution = 9048

        self.nav_feedback = 0
        self.es_hard = 0
        self.es_soft = 0
        self.total_distance = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.battery_voltage = 0.0

        self.left_sub = self.create_subscription(
            Int64, "/leftmotor/feedback", self.left_position_callback, 10
        )
        self.right_sub = self.create_subscription(
            Int64, "/rightmotor/feedback", self.right_position_callback, 10
        )
        self.nav_feedback = self.create_subscription(
            Int64, "/nav_feedback_", self.nav_feedback_callback, 10
        )
        self.es_hard_sub = self.create_subscription(
            Int64, "/es_status/hardware", self.es_hard_callback, 10
        )
        self.es_soft_sub = self.create_subscription(
            Int64, "/es_status/software", self.es_soft_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        self.subscription = self.create_subscription(
            Int64, "/battery_soc", self.battery_callback, 10
        )

    def battery_callback(self, msg):
        self.battery_voltgae = msg.data

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def es_hard_callback(self, msg):
        self.es_hard = msg.data

    def es_soft_callback(self, msg):
        self.es_soft = msg.data

    def nav_feedback_callback(self, msg):
        self.nav_feedback = msg.data

    def left_position_callback(self, msg):
        self.left_position = (-1) * msg.data

    def right_position_callback(self, msg):
        self.right_position = msg.data

    def update_analytics(self):
        while True:
            delta_left = self.left_position - self.last_left_position
            delta_right = self.right_position - self.last_right_position

            self.last_left_position = self.left_position
            self.last_right_position = self.right_position

            distance_left = (
                2 * math.pi * self.wheel_radius * delta_left
            ) / self.count_per_revolution

            distance_right = (
                2 * math.pi * self.wheel_radius * delta_right
            ) / self.count_per_revolution

            delta_distance = (distance_left + distance_right) / 2

            self.total_distance += delta_distance

            rclpy.spin_once(self)

    def get_total_distance(self):
        return random.random() * 100
        # return self.total_distance

    def get_linear_velocity(self):
        return random.random()
        # return self.linear_velocity

    def get_angular_velocity(self):
        return random.random()
        # return self.angular_velocity
    
    def get_battery_voltage(self):
        return random.randint(11,13)
        # return self.battery_voltage
    
    def get_es_hardware(self):
        return random.randint(0,1)
        # return self.es_hard
    
    def get_es_software(self):
        return random.randint(0,1)
        # return self.es_soft

    def get_nav_feedback(self):
        self.nav_feedback = random.randint(1, 2)
        if self.nav_feedback == 2:
            return "Autonomous"
        else:
            return "Manual"

cube_analytics = None

def main(args=None):
    try:
        global cube_analytics
        rclpy.init(args=args)
        cube_analytics = CubeAnalytics()
        cube_analytics.update_analytics()
        rclpy.spin(cube_analytics)
    except KeyboardInterrupt:
        pass
    finally:
        cube_analytics.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
