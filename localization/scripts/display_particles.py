#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray


def prepare_image(pose_array: PoseArray) -> None:
    img_to_show = np.ones((270, 270))*255

    for pose in pose_array.poses:
        img_to_show[int(pose.position.x), int(pose.position.y)] = 0
        img_to_show[int(pose.position.x)-1, int(pose.position.y)+1] = 0
        img_to_show[int(pose.position.x)+1, int(pose.position.y)-1] = 0
        img_to_show[int(pose.position.x)-1, int(pose.position.y)] = 0
        img_to_show[int(pose.position.x)+1, int(pose.position.y)] = 0

    img_to_show = img_to_show

    show(img_to_show)


def show(img_to_show) -> None:
    print("Showing image...")
    cv2.imshow("image", img_to_show)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        return

if __name__ == '__main__':
    rospy.init_node('show', anonymous=True)

    rospy.Subscriber('/localization', PoseArray, prepare_image)

    print('Escuchando para printear...')

    rospy.spin()

# def prepare_image(pose_array: PoseArray) -> None:
#     img_to_show = np.ones((270, 270), )*255
#     backtorgb = cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)

#     color = (255, 0, 0)
#     for pose in pose_array.poses:
#         img_to_show[int(pose.position.x), int(pose.position.y)] = (255, 0, 0)
#         img_to_show[int(pose.position.x)-1, int(pose.position.y)+1] = (255, 0, 0)
#         img_to_show[int(pose.position.x)+1, int(pose.position.y)-1] = (255, 0, 0)

#     img_to_show = img_to_show

#     show(img_to_show)