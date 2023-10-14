#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

        # Display the image using OpenCV
        cv2.imshow("Image from /abcd", cv_image)
        cv2.waitKey(1)  # Keep the window open until a key is pressed

    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))


def image_viewer_node():
    rospy.init_node('image_viewer_node', anonymous=True)

    # Subscribe to the /abcd topic
    rospy.Subscriber('/abcd', Image, image_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        # Initialize the OpenCV window
        cv2.namedWindow("Image from /abcd", cv2.WINDOW_NORMAL)

        # Create the image viewer node
        image_viewer_node()

    except rospy.ROSInterruptException:
        pass

