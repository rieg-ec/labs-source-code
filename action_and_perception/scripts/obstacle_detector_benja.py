#!/usr/bin/env python3
import rospy
#from action_and_perception.msg import OccupancyArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ObstacleDetector:
    def __init__(self):
        self.camera_suscriber = rospy.Subscriber( '/camera/depth/image_raw', Image, self.image_process )
        self.occupancy_state_publisher = rospy.Publisher(
            '/occupancy_state', String, queue_size=1)
        self.obstacle_pos = ''
        self.bridge = CvBridge()

        ## publish function
        self.publish_occupancy()

    def image_process(self, image):
        ## Image dimension is 480 (fila), 640 (columnas)
        ## With self.bridge we convert the depth image to a matrix.
        self.current_image = np.asarray(self.bridge.imgmsg_to_cv2(image,'32FC1'))
        self.current_image_left = self.current_image[:,:210]
        self.current_image_center = self.current_image[:,211:420]
        self.current_image_right = self.current_image[:,421:640]

        if np.mean(np.mean(self.current_image_center)) < 5.0:
            # we take the mean of the mean
            # because the mean of a matrix is a vector
            # and the mean of a vector is a number.
            self.obstacle_pos = 'obstacle_center'

        elif np.mean(np.mean(self.current_image_left)) < 5.0:
            self.obstacle_pos = 'obstacle_left'

        elif np.mean(np.mean(self.current_image_right)) < 5.0:
            self.obstacle_pos = 'obstacle_right'

        elif (
            np.mean(np.mean(self.current_image_left)) >= 5.0 and
            np.mean(np.mean(self.current_image_center)) >= 5.0 and
            np.mean(np.mean(self.current_image_center)) >= 5.0):
            self.obstacle_pos = 'free'

        # print(self.obstacle_pos)
        # while not rospy.is_shutdown():

    def publish_occupancy(self) -> None:    
        while not rospy.is_shutdown():
            print(self.obstacle_pos)
            self.occupancy_state_publisher.publish(
                self.obstacle_pos)
            rospy.Rate(5).sleep()



if __name__ == '__main__':
    rospy.init_node('obstacle_detector', anonymous=True)
    obstacle_detector_ = ObstacleDetector()
    rospy.spin()
