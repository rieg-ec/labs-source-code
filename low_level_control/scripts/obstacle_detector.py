#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from sound_play.libsoundplay import SoundClient
import numpy as np


class ObstacleDetector:

    sound_messages = {
        'obstacle_center': "obstacle center",
        'obstacle_left': "obstacle left",
        'obstacle_right': "obstacle right",
    }

    def __init__(self) -> None:
        self.camera_suscriber = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self._image_process)
        self.occupancy_state_publisher = rospy.Publisher(
            '/occupancy_state', String, queue_size=1)

        self.accurate_occupancy_state_publisher = rospy.Publisher(
            '/wall_distance', Float64, queue_size=1)

        self.bridge = CvBridge()
        self._obstacle_pos: str = "free"
        self.soundhandle = SoundClient()

    @property
    def obstacle_pos(self) -> str:
        return self._obstacle_pos

    @obstacle_pos.setter
    def obstacle_pos(self, value: str) -> None:
        if value != "free" and value != self._obstacle_pos:
            self.soundhandle.say(self.sound_messages[value],
                                 voice='voice_kal_diphone')

        self._obstacle_pos = value

    def _image_process(self, image: Image) -> None:
        # Image dimension is 480 (fila), 640 (columnas)
        # With self.bridge we convert the depth image to a matrix.

        self.current_image = np.asarray(
            self.bridge.imgmsg_to_cv2(image, '32FC1'))
        self.current_image_left = self.current_image[:, :210]
        self.current_image_center = self.current_image[:, 210:420]
        self.current_image_right = self.current_image[:, 420:640]

        ## CONTROL PASILLO ##
        self.wall_distance_publish()

        ## REPORTE OBSTACULOS ##
        self.obstacle_changes()

    def obstacle_changes(self):
        distance = 1.2
        if np.mean(np.mean(self.current_image_center)) < distance:
            # we take the mean of the mean
            # because the mean of a matrix is a vector
            # and the mean of a vector is a number.
            self.obstacle_pos = "obstacle_center"

        elif np.mean(np.mean(self.current_image_left)) < distance:
            self.obstacle_pos = "obstacle_left"

        elif np.mean(np.mean(self.current_image_right)) < distance:
            self.obstacle_pos = "obstacle_right"

        elif (np.mean(np.mean(self.current_image_left)) >= distance and
              np.mean(np.mean(self.current_image_center)) >= distance and
              np.mean(np.mean(self.current_image_center)) >= distance
              ):
            self.obstacle_pos = "free"

    def wall_distance_publish(self):
        if self.obstacle_pos == "free":
            return
        if np.isnan(np.mean(np.mean(self.current_image_left))):
            self.accurate_occupancy_state_publisher.publish(
                Float64(3.0)
            )

        elif np.isnan(np.mean(np.mean(self.current_image_right))):
            self.accurate_occupancy_state_publisher.publish(
                Float64(-3.0)
            )

        else:
            diferencia = Float64(np.mean(
                np.mean(self.current_image_right)) - np.mean(np.mean(self.current_image_left)))
            self.accurate_occupancy_state_publisher.publish(
                diferencia
            )

    def publish_occupancy(self) -> None:
        while not rospy.is_shutdown():
            self.occupancy_state_publisher.publish(self.obstacle_pos)
            rospy.Rate(5).sleep()


if __name__ == '__main__':
    rospy.init_node('obstacle_detector', anonymous=True)

    obstacle_detector = ObstacleDetector()
    obstacle_detector.publish_occupancy()
    rospy.spin()
