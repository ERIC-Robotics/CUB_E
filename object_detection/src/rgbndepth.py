#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from ultralytics import YOLO
# model = YOLO('yolov8x-seg.pt')


# rgb_image = np.empty((480, 640))
# depth_image = np.empty((480, 640))
def rgb_callback(data):
    try:
        cv_image_rgb = bridge.imgmsg_to_cv2(data)
        # rgb_image = cv_image_rgb
        # cv2.circle(cv_image_rgb,(320, 240), 2, (0, 0, 225), 2)
        print("hio")
        cv2.imshow("RGB Image", cv_image_rgb)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error converting ROS Image to OpenCV format: %s", str(e))


def depth_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data)
        # depth_image = cv_image
        # cv2.circle(cv_image,(320, 240), 2, (0, 255, 0), 2)
        print("bye")
        cv2.imshow("Depth Image", cv_image)
        # cv2.waitKey(1)
    except Exception as e:
        rospy.logerr("Error converting ROS Image to OpenCV format: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    bridge = CvBridge()

    rospy.Subscriber("/camera0/infra/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera0/depth/image_rect_raw", Image, depth_callback)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        try:
            # rate.sleep()
            # cv2.waitKey(1)
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    cv2.destroyAllWindows()
