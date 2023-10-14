#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from segmentation import YOLOSegmentation
from ultralytics import YOLO
model = YOLO('yolov8m-seg.pt')

ys = YOLOSegmentation('yolov8x-seg.pt')
rgb_image = np.empty((480, 640))
depth_image = np.empty((480, 640))
def rgb_callback(data):
    # Convert the RGB image message to OpenCV format
    global rgb_image
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(data,  desired_encoding="passthrough")

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

def resize_os30a(image):
    shape = (640, 460)
    image = cv2.resize(image, shape)
    return image

def getdepth_info(image, depth_img, poly_points):

    mask = np.zeros_like(depth_image, dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(poly_points)], (255, 255, 255))
    masked_depth_values = cv2.bitwise_and(depth_img, depth_img, mask=mask)
    average_depth = np.mean(masked_depth_values[np.where(masked_depth_values > 0)])
    average_depth = round(average_depth, 1)
    return image, average_depth


def segment(image):
    bboxes, classes, segmentations, scores = ys.detect(image)
    
    for bbox, class_id, seg, score in zip(bboxes, classes, segmentations, scores):
    # print("bbox:", bbox, "class id:", class_id, "seg:", seg, "score:", score)
        (x, y, x2, y2) = bbox
        
        # cv2.rectangle(image, (x, y), (x2, y2), (255, 0, 0), 2)

        cv2.polylines(image, [seg], True, (255, 0, 255), 2)
        # print(seg)
        image, depth = getdepth_info(image, depth_image , seg)
        cv2.putText(image, (model.names[class_id] +"  D: " + str(depth)), (x, y-2), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 1)

    return image

if __name__ == '__main__':
    rospy.init_node('image_viewer_node', anonymous=True)

    # Subscribe to the RGB and depth image topics
    rospy.Subscriber("/camera0/infra/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera0/depth/image_rect_raw", Image, depth_callback)
    rate = rospy.Rate(100)
    # Start the ROS node
    # rospy.spin()

    while True:
        rate.sleep()
        # print(rgb_image)
        shape = (640, 460)
        rgb_image = cv2.resize(rgb_image, shape)
        rgb_image_2 = rgb_image
        # rgb_image = resize_os30a(rgb_image)
        if rgb_image.dtype != 'uint8':
            rgb_image = (rgb_image * 255).astype('uint8')
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        bgr_image = segment(bgr_image)
        # print(depth_image.shape)
        cv2.imshow("Depth Image", depth_image)
        cv2.imshow("RGB Image", bgr_image)
        
        # print("hello")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break