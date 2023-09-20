#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

wheelradius = 0.08 # in meters
wheelSeperation = 0.45 # in meters

def cmd_vel_callback(data):
    leftrpm = Float64()
    rightrpm = Float64()

    leftvelocity = data.linear.x + (wheelSeperation/2) * data.angular.z
    rightvelocity = data.linear.x - (wheelSeperation/2) * data.angular.z

    leftrpm.data = (leftvelocity * 60) / (2 * math.pi * wheelradius)
    rightrpm.data = (rightvelocity * 60) / (2 * math.pi * wheelradius)

    left_pub.publish(leftrpm)
    right_pub.publish(rightrpm)

if __name__ == '__main__':
    rospy.init_node('cmdvel_motorCommand', anonymous=True)
    
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    
    left_pub = rospy.Publisher('/leftmotor/command', Float64, queue_size=10)
    right_pub = rospy.Publisher('/rightmotor/command', Float64, queue_size=10)

    rospy.spin()  