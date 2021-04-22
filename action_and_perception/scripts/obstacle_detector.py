#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


class ObstacleDetector:
    def __init__(self):
        self.occupancy_state_publisher = rospy.Publisher(
            '/occupancy_state', String, queue_size=10)

        self.publish_occupancy()

    def publish_occupancy(self) -> None:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.occupancy_state_publisher.publish("free")
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('obstacle_detector', anonymous=True)
    dead_reck_nav = ObstacleDetector()
    rospy.spin()
