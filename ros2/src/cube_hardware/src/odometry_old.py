#!/usr/bin/python3

import rclpy
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from rclpy.node import Node
import math

"""
odometry_old.py: Calculate and publish odometry information.

This ROS2 node calculates odometry based on the differential drive kinematic model
utilizing feedback from left and right wheel encoders. It subscribes to the wheel encoder
topics to receive the number of ticks and computes the robot's pose (position and orientation).
The calculated pose is then published as an Odometry message on the '/odom' topic, providing
necessary data for navigation and localization within a ROS2 framework.

Authors: Jatin Patil
Version: 1.0
Last Updated: 2023-Nov-09
"""

class OdometryCalculator(Node):
    """
    A ROS2 node that calculates odometry based on wheel encoder readings.
    """
    
    def __init__(self):
        """
        Initialize the OdometryCalculator node and create necessary publishers and subscribers.
        """
        super().__init__('odometry_node')
        
        # Initial position and velocity
        self.left_position = 0
        self.right_position = 0
        self.last_left_position = 0
        self.last_right_position = 0
        
        # Robot specific constants
        self.wheel_distance = 0.45  # Distance between wheels, in meters
        self.wheel_radius = 0.08  # Radius of the wheel, in meters
        self.count_per_revolution = 9048  # Encoder counts per revolution
        
        # Initial pose (x, y, and orientation theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Odometry publish rate
        self.rate = 10.0  # 10Hz
        
        # Setup publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer_ = self.create_timer((1/self.rate), self.odom_callback)
        self.left_sub = self.create_subscription(Int64, '/leftmotor/feedback', self.left_position_callback, 10)
        self.right_sub = self.create_subscription(Int64, '/rightmotor/feedback', self.right_position_callback, 10)

    def odom_callback(self):
        """
        Callback function that triggers the odometry calculation.
        """
        self.calculate_odometry()

    def left_position_callback(self, msg):
        """
        Callback function for the left motor encoder position.
        Parameters:
            msg (Int64): The incoming message containing the left encoder position.
        """
        # Left wheel encoder gives a negative value
        self.left_position = (-1) * msg.data
    
    def right_position_callback(self, msg):
        """
        Callback function for the right motor encoder position.
        Parameters:
            msg (Int64): The incoming message containing the right encoder position.
        """
        self.right_position = msg.data
    
    def calculate_odometry(self):
        """
        Calculate the robot's odometry based on encoder readings.
        """
        # Compute the difference in encoder counts since the last measurement
        delta_left = self.left_position - self.last_left_position
        delta_right = self.right_position - self.last_right_position
        
        # Update last known positions
        self.last_left_position = self.left_position
        self.last_right_position = self.right_position
        
        # Calculate the distance traveled by each wheel
        distance_left = (2 * math.pi * self.wheel_radius * delta_left) / self.count_per_revolution
        distance_right = (2 * math.pi * self.wheel_radius * delta_right) / self.count_per_revolution
        
        # Calculate the average distance and change in orientation
        delta_distance = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / self.wheel_distance
        
        # Update the robot's pose
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        self.theta += delta_theta
        
        # Publish the updated odometry
        self.publish_odometry()

    def publish_odometry(self):
        """
        Publish the calculated odometry to a ROS topic.
        """
        # Create and populate the Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        
        # Set the position in the odometry message
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        # Set the orientation in the odometry message
        # The orientation is a quaternion created from the theta angle
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(self.theta/2), w=math.cos(self.theta/2))
        
        # Set the velocity in the odometry message
        # In this case, it's assumed to be zero as we are not calculating velocity
        odom.twist.twist = Twist()
        
        # Publish the odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    """
    Entry point for the odometry node.
    """
    try:
        rclpy.init(args=args)
        odometry_calculator = OdometryCalculator()
        rclpy.spin(odometry_calculator)
    except KeyboardInterrupt:
        # Handle the Ctrl-C keyboard interrupt
        print("Odometry node is shutting down.")
    finally:
        # Cleanup and shutdown the node properly
        odometry_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
