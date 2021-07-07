#!/usr/bin/env python3
import random
import numpy as np

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Point, Quaternion

from utils import orientation_to_yaw, yaw_to_orientation, meters_to_pixel
from localization import monte_carlo_localization, Particle


class ParticleFilter:

    def __init__(self, n_particles: int) -> None:
        self.n_particles = n_particles

        self.localization_publisher = rospy.Publisher(
            '/localization', PoseArray, queue_size=1)

        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)

        rospy.Subscriber('/scan', LaserScan,
                         lambda v: setattr(self, 'scan', v.intensities))
        rospy.Subscriber('/odom', Odometry,
                         lambda v: setattr(self, 'odom', v.pose.pose))

    def map_cb(self, grid: OccupancyGrid):
        self.map_grid = np.resize(grid.data, (270, 270)).T

    def localization(self) -> None:
        while not rospy.is_shutdown():
            if (
                not hasattr(self, 'scan')
                or not hasattr(self, 'map_grid')
                or not hasattr(self, 'odom')
            ):
                rospy.Rate(5).sleep()
                continue

            if not hasattr(self, 'last_pose'):
                self.last_pose = self.odom

            if not hasattr(self, 'particles'):
                self.create_particles(self.n_particles)

            dx = self.last_pose.position.x - self.odom.position.x
            dy = self.last_pose.position.y - self.odom.position.y

            dx, dy = meters_to_pixel(dx, dy, 0.01)

            dtheta = (
                orientation_to_yaw(self.odom.orientation) -
                orientation_to_yaw(self.last_pose.orientation)
            )

            self.particles = monte_carlo_localization(
                self.particles, [dx, dy, dtheta], self.scan, self.map_grid)

            pose_array = PoseArray()
            pose_array.poses = self.particles
            self.localization_publisher.publish(pose_array)

            self.last_pose = self.odom
            rospy.Rate(0.5).sleep()

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
    particle_filter = ParticleFilter(50)
    particle_filter.localization()
    rospy.spin()
