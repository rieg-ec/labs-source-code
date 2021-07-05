#!/usr/bin/env python3
import random
from os import path

import cv2
import rospy
import rospkg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from utils import orientation_to_yaw
from localization import monte_carlo_localization, Particle


rospack = rospkg.RosPack()

MAP_PATH = filepath = path.join(rospack.get_path(
    'localization'), 'maps', 'map.pgm')

map_array = cv2.imread(MAP_PATH, cv2.IMREAD_GRAYSCALE)


class ParticleFilter:

    def __init__(self, n_particles: int) -> None:
        self.create_particles(n_particles)

        rospy.Subscriber('/scan', LaserScan,
                         lambda v: setattr(self, 'scan', v.intensities))
        rospy.Subscriber('/odom', Odometry, self.localization_cb)

    def localization_cb(self, odom: Odometry) -> None:
        if not hasattr(self, 'last_pose'):
            self.last_pose = odom.pose.pose
            return

        if not hasattr(self, 'scan') or not hasattr(self, 'particles'):
            return

        dx = self.last_pose.position.x - odom.pose.pose.position.x
        dy = self.last_pose.position.y - odom.pose.pose.position.y
        dtheta = (
            orientation_to_yaw(odom.pose.pose.orientation) -
            orientation_to_yaw(self.last_pose.orientation)
        )

        self.particles = monte_carlo_localization(
            self.particles, [dx, dy, dtheta], self.scan, map_array)

    def create_particles(self, number_of_particles: int) -> None:
        self.particles = []
        initial_theta = 0
        for _ in range(number_of_particles):
            x = random.randint(0, 269)
            y = random.randint(0, 269)
            while map_array[x][y] < 205 or map_array[x][y] == 255:
                x = random.randint(0, 269)
                y = random.randint(0, 269)

            self.particles.append(Particle(x, y, initial_theta))


if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    particle_filter = ParticleFilter(1000)
    rospy.spin()
