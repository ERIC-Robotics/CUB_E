#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
import math

class OdometryCalculator:
    def __init__(self):
        rospy.init_node('odometry_node', anonymous=True)
        
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
        
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        rospy.Subscriber('/leftmotor/feedback', Int64, self.left_position_callback)
        rospy.Subscriber('/rightmotor/feedback', Int64, self.right_position_callback)
        
    def left_position_callback(self, msg):
        self.left_position = (-1) * msg.data
    
    def right_position_callback(self, msg):
        self.right_position = msg.data
    
    def calculate_odometry(self):
        rate = rospy.Rate(10)  # Update rate (10 Hz)
        
        while not rospy.is_shutdown():
            delta_left = self.left_position - self.last_left_position
            delta_right = self.right_position - self.last_right_position
            
            self.last_left_position = self.left_position
            self.last_right_position = self.right_position
            
            distance_left = (2 * math.pi * self.wheel_radius * delta_left) / self.count_per_revolution
            distance_right = (2 * math.pi * self.wheel_radius * delta_right) / self.count_per_revolution
            
            delta_distance = (distance_left + distance_right) / 2
            delta_theta = (distance_right - distance_left) / self.wheel_distance
            
            self.x += delta_distance * math.cos(self.theta)
            self.y += delta_distance * math.sin(self.theta)
            self.theta += delta_theta
            
            self.publish_odometry()
            
            rate.sleep()
    
    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(0, 0, math.sin(self.theta/2), math.cos(self.theta/2))
        
        # Set velocity (assuming it's zero in this example)
        odom.twist.twist = Twist()
        
        # Publish the odometry message
        self.odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        odometry_calculator = OdometryCalculator()
        odometry_calculator.calculate_odometry()
    except rospy.ROSInterruptException:
        pass