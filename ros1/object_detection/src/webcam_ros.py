#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    # Initialize the ROS node
    rospy.init_node('usb_camera_publisher')

    # Initialize the video capture from the USB camera
    camera = cv2.VideoCapture(2)  # 0 represents the default camera (you may need to change this if you have multiple cameras)

    # Create a publisher for the /video_stream topic
    image_publisher = rospy.Publisher('/video_stream', Image, queue_size=10)

    # Create a CvBridge object to convert between OpenCV images and ROS messages
    bridge = CvBridge()

    rate = rospy.Rate(30)  # Set the publishing rate (30 Hz in this example)

    while not rospy.is_shutdown():
        ret, frame = camera.read()  # Capture a frame from the camera
        if not ret:
            rospy.logwarn("Failed to capture frame from the camera!")
            continue

        # Convert the OpenCV image to a ROS Image message
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publish the ROS Image message to the /video_stream topic
        image_publisher.publish(ros_image)

        rate.sleep()

    # Release the camera when the node is terminated
    camera.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
