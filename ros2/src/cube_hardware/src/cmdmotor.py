#!/usr/bin/python3

"""
cmdmotor.py: Publish the individual wheel velocities

It subscribes on cmd_vel topic and uses inverse kinematics
too generate individual wheel velocities and publish them 
on /leftmotor/command and /rightmotor/command topics

Authors: Jatin Patil
Version: 1.0
Last Updated: 2023-Nov-09
"""

import rclpy  # ROS2 Python client library
from geometry_msgs.msg import Twist  # ROS2 message type for velocity
from std_msgs.msg import Float64, String, Int64 # ROS2 standard message type for double precision floating point numbers
import math  # Import math module

from cube_utils.msg import LidarSaftey


class CmdVelToMotorCommand:
    """
    This class converts a Twist command (linear and angular velocity)
    to individual motor commands for a differential drive robot.
    """

    def __init__(self):
        """
        Constructor for the CmdVelToMotorCommand class.
        """
        # Initialize the ROS2 node
        self.node = rclpy.create_node('cmdvel_motorCommand')
        
        # Define wheel parameters
        self.wheel_radius = 0.08  # Radius of the wheels in meters
        self.wheel_separation = 0.45  # Distance between the wheels in meters
        self.lidar_saftey = LidarSaftey()

        # Create publishers for left and right motor commands
        self.left_pub = self.node.create_publisher(Float64, '/leftmotor/command', 10)
        self.right_pub = self.node.create_publisher(Float64, '/rightmotor/command', 10)
        self.soft_es_pub = self.node.create_publisher(Int64, '/es_status/software', 10)
        self.soft_es_msg = Int64()
        
        # Create a subscription to the cmd_vel topic
        self.cmd_vel_sub = self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.scan_subscriber = self.node.create_subscription(
            LidarSaftey,
            '/es_status/software/zone',  # Replace 'scan' with the actual name of your scan topic
            self.es_callback,
            10  # QoS profile depth
        )

    def es_callback(self, msg):
        self.lidar_saftey = msg
        if self.lidar_saftey.object:
            self.soft_es_msg.data = 1
        else:
            self.soft_es_msg.data = 0
        self.soft_es_pub.publish(self.soft_es_msg)
        # print(self.es_soft_status)

    def cmd_vel_callback(self, data):
        """
        Callback function that is called when a new cmd_vel message is received.
        """
        # Instantiate messages for publishing RPM
        leftrpm = Float64()
        rightrpm = Float64()

        if self.lidar_saftey.object:
            # if(self.es_soft_status == 'front'):
            #     if data.linear.x > 0.0:
            #         data.linear.x = 0.0
            # elif self.es_soft_status == 'left':
            #     if data.angular.z > 0.0:
            #         data.angular.z = 0.0
            # elif self.es_soft_status == 'back':
            #     if data.linear.x < 0.0:
            #         data.linear.x = 0.0
            # elif self.es_soft_status == 'right':
            #     if data.angular.z < 0.0:
            #         data.angular.z = 0.0
            if self.lidar_saftey.front:
                if data.linear.x > 0.0:
                    data.linear.x = 0.0
            if self.lidar_saftey.left:
                if data.angular.z > 0.0:
                    data.angular.z = 0.0
            if self.lidar_saftey.back:
                if data.linear.x < 0.0:
                    data.linear.x = 0.0
            if self.lidar_saftey.right:
                if data.angular.z < 0.0:
                    data.angular.z = 0.0

        # print("linear: " + str(data.linear.x) + " angular: " + str(data.angular.z))
            self.node.get_logger().info("linear: " + str(data.linear.x) + " angular: " + str(data.angular.z))

        # Calculate wheel velocities from cmd_vel Twist message
        left_velocity = data.linear.x - (self.wheel_separation / 2) * data.angular.z
        right_velocity = data.linear.x + (self.wheel_separation / 2) * data.angular.z

        # Convert velocities to RPM
        leftrpm.data = (left_velocity * 60) / (2 * math.pi * self.wheel_radius)
        rightrpm.data = (right_velocity * 60) / (2 * math.pi * self.wheel_radius)

        # Publish RPM to each motor
        self.left_pub.publish(leftrpm)
        self.right_pub.publish(rightrpm)

    def run(self):
        """
        Runs the ROS2 node until it is shut down.
        """
        # Spin the node to continuously call callbacks
        rclpy.spin(self.node)
        
        # Clean shutdown of the node
        self.node.destroy_node()

def main(args=None):
    """
    Main function for initializing and running the ROS2 node.
    """
    try:
        # Initialize ROS2
        rclpy.init()
        
        # Instantiate the CmdVelToMotorCommand class
        cmd_vel_to_motor_command = CmdVelToMotorCommand()
        
        # Run the CmdVelToMotorCommand node
        cmd_vel_to_motor_command.run()
    except KeyboardInterrupt:
        # Handle shutdown on Ctrl+C
        pass

# Entry point of the script
if __name__ == '__main__':
    main()
