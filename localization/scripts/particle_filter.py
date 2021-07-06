#!/usr/bin/env python3
import random
import numpy as np

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Point, Quaternion

from utils import orientation_to_yaw, yaw_to_orientation
from localization import monte_carlo_localization, Particle


class ParticleFilter:

    def __init__(self, n_particles: int) -> None:
        self.n_particles = n_particles

        self.localization_publisher = rospy.Publisher(
            '/localization', PoseArray)

        rospy.Subscriber(
            '/map', OccupancyGrid,
            lambda m: setattr(self, 'map_grid',
                              np.resize(m.data, (270, 270)).T)
        )

        rospy.Subscriber('/scan', LaserScan,
                         lambda v: setattr(self, 'scan', v.intensities))
        rospy.Subscriber('/odom', Odometry, self.localization_cb)

    def localization_cb(self, odom: Odometry) -> None:
        if not hasattr(self, 'last_pose'):
            self.last_pose = odom.pose.pose
            return

        if (
            not hasattr(self, 'scan')
            or not hasattr(self, 'map_grid')
        ):
            return

        if not hasattr(self, 'particles'):
            self.create_particles(self.n_particles)

        dx = self.last_pose.position.x - odom.pose.pose.position.x
        dy = self.last_pose.position.y - odom.pose.pose.position.y
        dtheta = (
            orientation_to_yaw(odom.pose.pose.orientation) -
            orientation_to_yaw(self.last_pose.orientation)
        )

        self.particles = monte_carlo_localization(
            self.particles, [dx, dy, dtheta], self.scan, self.map_grid)

        pose_array = PoseArray()
        pose_array.poses = self.particles
        self.localization_publisher.publish(pose_array)
        
        self.last_pose = odom.pose.pose

    def create_particles(self, number_of_particles: int) -> None:
        self.particles = []
        initial_theta = 0
        for _ in range(number_of_particles):
            x = random.randint(0, 269)
            y = random.randint(0, 269)
            while self.map_grid[x][y] != 100:
                x = random.randint(0, 269)
                y = random.randint(0, 269)

            particle = Particle(
                Point(x, y, 0), Quaternion(*yaw_to_orientation(initial_theta)))

            self.particles.append(particle)


if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    particle_filter = ParticleFilter(1000)
    rospy.spin()
