#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


rgb_image = np.empty((480, 640))
depth_image = np.empty((480, 640))
def rgb_callback(data):
    # Convert the RGB image message to OpenCV format
    global rgb_image
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Display the RGB image
    # cv2.imshow("RGB Image", rgb_image)
    # cv2.waitKey(1)

def depth_callback(data):
    # Convert the depth image message to OpenCV format (if needed)
    global depth_image
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    # Display the depth image (you may need to apply a colormap)
    # cv2.imshow("Depth Image", depth_image)
    # cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_viewer_node', anonymous=True)

    # Subscribe to the RGB and depth image topics
    rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
    rate = rospy.Rate(100)
    # Start the ROS node
    # rospy.spin()

    while True:
        rate.sleep()
        # print(rgb_image)
        cv2.imshow("RGB Image", rgb_image)
        cv2.imshow("Depth Image", depth_image)
        # print("hello")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break