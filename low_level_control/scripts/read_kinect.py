#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cv_bridge = CvBridge()


def show_depth_image(image_msg: Image):
    depth_image = cv_bridge.imgmsg_to_cv2(image_msg)
    image2display = depth_image - depth_image.min()
    image2display = (
        image2display * (255.0/image2display.max())).astype(np.uint8)
    image2display = 255 - image2display
    image2display = cv2.applyColorMap(image2display, cv2.COLORMAP_HOT)
    cv2.imshow('Depth Sensor', image2display)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('read_kinect_depth')
    rospy.Subscriber('/camera/depth/image_raw', Image, show_depth_image)
    rospy.spin()
