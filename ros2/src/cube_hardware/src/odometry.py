#!/usr/bin/python3

import rclpy
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from rclpy.node import Node
import math

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # self.node = rclpy.create_node('odometry_node')
        
        self.left_position = 0
        self.right_position = 0
        self.last_left_position = 0
        self.last_right_position = 0
        
        self.wheel_distance = 0.45  # m
        self.wheel_radius = 0.08
        self.count_per_revolution = 9048
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.yaw = 0.0

        # self.rate = self.node.create_rate(10) 
        self.rate = 10.0
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer_ = self.create_timer((1/self.rate), self.odom_callback)
        self.left_sub = self.create_subscription(Int64, '/leftmotor/feedback', self.left_position_callback, 10)
        self.right_sub = self.create_subscription(Int64, '/rightmotor/feedback', self.right_position_callback, 10)
        self.filtered_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)

        self.filtered_odom_msg = Odometry()

    def odom_callback(self):
        self.calculate_odometry()

    def filtered_odom_callback(self, msg):
        self.filtered_odom_msg = msg

    def left_position_callback(self, msg):
        self.left_position = (-1) * msg.data
    
    def right_position_callback(self, msg):
        self.right_position = msg.data
    
    def calculate_odometry(self):
        # Update rate (10 Hz)
        
        # while rclpy.ok():

        delta_left = self.left_position - self.last_left_position
        delta_right = self.right_position - self.last_right_position
            
        self.last_left_position = self.left_position
        self.last_right_position = self.right_position
            
        distance_left = (2 * math.pi * self.wheel_radius * delta_left) / self.count_per_revolution
        distance_right = (2 * math.pi * self.wheel_radius * delta_right) / self.count_per_revolution
            
        delta_distance = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / self.wheel_distance
            
        self.filtered_odom_msg.pose.pose.position.x += delta_distance * math.cos(self.yaw)
        self.filtered_odom_msg.pose.pose.position.y += delta_distance * math.sin(self.yaw)

        self.yaw = 2 * math.asin(self.filtered_odom_msg.pose.pose.orientation.z)
        self.yaw += delta_theta
            
        # print(self.left_position, self.right_position)
        self.publish_odometry()
            
        # self.rate.sleep()

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
    
        # Set position
        odom.pose.pose.position = Point()
        odom.pose.pose.position.x = self.filtered_odom_msg.pose.pose.position.x
        odom.pose.pose.position.y = self.filtered_odom_msg.pose.pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion() 
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        
        # Set velocity (assuming it's zero in this example)
        odom.twist.twist = Twist()
        # Publish the odometry message
        self.odom_pub.publish(odom)


def main(args=None):
    try:
        rclpy.init(args=args)
        odometry_calculator = OdometryCalculator()
        # odometry_calculator.calculate_odometry()
        # odometry_calculator.run()
        rclpy.spin(odometry_calculator)
    except KeyboardInterrupt:
        pass
    

if __name__ == '__main__':
    main()
