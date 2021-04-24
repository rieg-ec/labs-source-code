#!/usr/bin/env python3

import rospy
#from action_and_perception.msg import OccupancyArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy

class ObstacleDetector:
    def __init__(self):
        self.camera_suscriber = rospy.Subscriber( '/camera/depth/image_raw', Image, self.image_process )

        self.occupancy_state_publisher = rospy.Publisher(
            '/occupancy_state', String, queue_size=10)

        self.bridge = CvBridge()

        self.publish_occupancy()

    def image_process(self, image):
        ## image dimension is 480 (fila), 640 (columnas)
        self.current_image = numpy.asarray(self.bridge.imgmsg_to_cv2(image,'32FC1'))
        self.current_image_left = self.current_image[:,:210]
        self.current_image_center = self.current_image[:,211:420]
        self.current_image_right = self.current_image[:,421:640]
        
        # print(numpy.mean(numpy.mean(self.current_image_left)))

        if numpy.mean(numpy.mean(self.current_image_left)) < 5.0:
            # double mean because the mean of a matrix is a vector
            # with the mean of its columns.
            print('left')
        if numpy.mean(numpy.mean(self.current_image_center)) < 5.0:
            # double mean because the mean of a matrix is a vector
            # with the mean of its columns.
            print('center')
        if numpy.mean(numpy.mean(self.current_image_right)) < 5.0:
            # double mean because the mean of a matrix is a vector
            # with the mean of its columns.
            print('right')
        
        # print(self.current_image.shape)
        elif (
            numpy.mean(numpy.mean(self.current_image_left)) >= 5.0 and
            numpy.mean(numpy.mean(self.current_image_center)) >= 5.0 and
            numpy.mean(numpy.mean(self.current_image_center)) >= 5.0
        ):
            print('free')


        



    def publish_occupancy(self) -> None:    
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.occupancy_state_publisher.publish(['free'])
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('obstacle_detector', anonymous=True)
    dead_reck_nav = ObstacleDetector()
    rospy.spin()
