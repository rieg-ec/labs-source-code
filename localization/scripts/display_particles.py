#!/usr/bin/env python3

import rospy
import numpy as np
import cv2

from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from particle_filter import ParticleFilter, Particle


def prepare_image(pose_array: list) -> None:
    img_to_show = np.zeros( (270, 270) )

    for pose in pose_array:
        img_to_show[ int(pose.posx), int(pose.posy) ] = 255

    img_to_show = img_to_show

    show(img_to_show)

def show( img_to_show ):
    cv2.imshow("image", img_to_show)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        return

if __name__ == '__main__':
    rospy.init_node('show', anonymous=True)

    print('Escuchando para printear...')

    Particleshow = ParticleShow()

    rospy.spin()