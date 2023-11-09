#!/usr/bin/python3
"""
odometry.py: Calculate and publish odometry information.

This ROS2 node calculates odometry based on the differential drive kinematic model
utilizing feedback from left and right wheel encoders. It subscribes to the wheel encoder
topics to receive the number of ticks and computes the robot's pose (position and orientation).
The calculated pose is then published as an Odometry message on the '/odom' topic, and it also
listens to '/odometry/filtered' for filtered odometry data, incorporating it into the pose
estimation. This data is essential for navigation and localization within a ROS2 framework.

Authors: Jatin Patil
Version: 1.0
Last Updated: 2023-Nov-09
"""

import rclpy
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from rclpy.node import Node
import math

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Initialize the positions and wheel parameters
        self.left_position = 0
        self.right_position = 0
        self.last_left_position = 0
        self.last_right_position = 0
        self.wheel_distance = 0.45  # Distance between wheels, in meters
        self.wheel_radius = 0.08  # Radius of the wheels, in meters
        self.count_per_revolution = 9048  # Encoder counts per wheel revolution
        
        # Initialize the pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.yaw = 0.0
        
        # Set the update rate
        self.rate = 10.0  # Update rate in Hz
        
        # Setup ROS2 publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer_ = self.create_timer((1/self.rate), self.odom_callback)
        self.left_sub = self.create_subscription(Int64, '/leftmotor/feedback', self.left_position_callback, 10)
        self.right_sub = self.create_subscription(Int64, '/rightmotor/feedback', self.right_position_callback, 10)
        self.filtered_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.filtered_odom_callback, 10)

        self.filtered_odom_msg = Odometry()

    def odom_callback(self):
        # Callback for the timer event to calculate and publish odometry
        self.calculate_odometry()

    def filtered_odom_callback(self, msg):
        # Callback for receiving filtered odometry data
        self.filtered_odom_msg = msg

    def left_position_callback(self, msg):
        # Callback for receiving left wheel encoder data
        self.left_position = (-1) * msg.data
    
    def right_position_callback(self, msg):
        # Callback for receiving right wheel encoder data
        self.right_position = msg.data
    
    def calculate_odometry(self):
        # Compute the odometry based on wheel encoder data
        
        delta_left = self.left_position - self.last_left_position
        delta_right = self.right_position - self.last_right_position
            
        self.last_left_position = self.left_position
        self.last_right_position = self.right_position
            
        # Calculate the distance each wheel has traveled
        distance_left = (2 * math.pi * self.wheel_radius * delta_left) / self.count_per_revolution
        distance_right = (2 * math.pi * self.wheel_radius * delta_right) / self.count_per_revolution
            
        # Calculate the overall distance and change in orientation
        delta_distance = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / self.wheel_distance
            
        # Update the pose estimation with the filtered data
        self.filtered_odom_msg.pose.pose.position.x += delta_distance * math.cos(self.yaw)
        self.filtered_odom_msg.pose.pose.position.y += delta_distance * math.sin(self.yaw)

        self.yaw = 2 * math.asin(self.filtered_odom_msg.pose.pose.orientation.z)
        self.yaw += delta_theta
            
        self.publish_odometry()

    def publish_odometry(self):
        # Publish the odometry message with the current pose and velocity
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
    
        # Set the current pose
        odom.pose.pose.position = Point(x=self.filtered_odom_msg.pose.pose.position.x,
                                        y=self.filtered_odom_msg.pose.pose.position.y,
                                        z=0.0)
        odom.pose.pose.orientation = Quaternion(x=0.0,
                                                y=0.0,
                                                z=math.sin(self.yaw/2),
                                                w=math.cos(self.yaw/2))
        
        # Set velocity (assuming it's zero in this example)
        odom.twist.twist = Twist()
        
        # Publish the odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    try:
        # Initialize ROS2
        rclpy.init(args=args)
        odometry_calculator = OdometryCalculator()
        # Spin the node to keep it alive and responsive to topics
        rclpy.spin(odometry_calculator)
    except KeyboardInterrupt:
        # Handle clean shutdown on Ctrl+C
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        odometry_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

