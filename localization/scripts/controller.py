#!/usr/bin/env python3
import math
import random
from typing import Tuple, List

import rospy
import numpy as np
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from utils import ang_dif, orientation_to_yaw, add_90_degrees, pixel_to_meters, meters_to_pixel


class NavController:

    def __init__(self) -> None:
        rospy.Subscriber('/odom', Odometry, self.odom_listener)

        # rospy.Subscriber('/map', OccupancyGrid,
        #                  lambda map: setattr(self, 'map_grid', map.data))

        rospy.Subscriber('/scan', LaserScan, self.lidar_cb)

        rospy.Subscriber('/localization', PoseArray, self.localization_cb)

        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.odom = Pose()

        self.velocity: Twist = Twist()
        self.velocity.linear.x = 0.2
        self.ang_speed: float = 1.2

        self.laser_scan_intensities: List = []
        self.localization = False

        self.obstacles = []

    def localization_cb(self, particles: PoseArray) -> None:
        sum_x = sum([particle.position.x for particle in particles.poses])
        sum_y = sum([particle.position.y for particle in particles.poses])

        mc_x = sum_x / len(particles.poses)
        mc_y = sum_y / len(particles.poses)

        dist_x = 0
        dist_y = 0

        for particle in particles.poses:
            dist_x += abs(particle.position.x - mc_x)
            dist_y += abs(particle.position.y - mc_y)

    def lidar_cb(self, depth_array: LaserScan) -> None:
        depth_limit = 0.55

        self.obstacles = []

        if np.mean(depth_array.ranges[81:100]) < depth_limit:
            self.obstacles.append('center')

        if np.mean(depth_array.ranges[65:81]) < depth_limit:
            self.obstacles.append('left')

    def odom_listener(self, odom: Odometry) -> None:
        self.odom = odom.pose.pose

    def move(self) -> None:
        while not self.localization:
            if 'left' not in self.obstacles:
                self.velocity.angular.z = -self.ang_speed

            elif 'center' in self.obstacles:
                self.velocity.angular.z = self.ang_speed

            self.velocity_publisher.publish(self.velocity)

            rospy.Rate(20).sleep()


if __name__ == '__main__':
    rospy.init_node('navigation_controller', anonymous=True)
    navigation_controller = NavController()
    navigation_controller.move()
    rospy.spin()
