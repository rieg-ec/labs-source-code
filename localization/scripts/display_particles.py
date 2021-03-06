#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import PoseArray

from utils import meters_to_pixel


def show_image(pose_array: PoseArray) -> None:
    img = np.zeros((270, 270, 3), np.uint8)

    odom = pose_array.poses.pop(-1)

    for pose in pose_array.poses:
        # x & y switched on purpose
        x, y = int(pose.position.x), int(pose.position.y)
        cv2.circle(img, (x, y), 5, (255, 0, 0), -1)

    odomx, odomy = meters_to_pixel(odom.position.x, odom.position.y)

    cv2.circle(img, (odomx, odomy), 5, (0, 255, 0), -1)

    cv2.imshow("image", img)

    if cv2.waitKey(25) & 0xFF == ord('q'):
        return


if __name__ == '__main__':
    rospy.init_node('show', anonymous=True)
    rospy.Subscriber('/localization', PoseArray, show_image)
    rospy.spin()
