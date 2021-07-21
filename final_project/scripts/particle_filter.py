#!/usr/bin/env python3
import random
import numpy as np
import math

import rospy
from scipy import spatial
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose

from utils import orientation_to_yaw, yaw_to_orientation, meters_to_pixel
from localization import monte_carlo_localization, Particle


class ParticleFilter:

    def __init__(self, n_particles: int) -> None:
        self.n_particles = n_particles

        self.localization_publisher = rospy.Publisher(
            '/localization', PoseArray, queue_size=1)

        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)

        rospy.Subscriber('/scan', LaserScan,
                         lambda v: setattr(self, 'scan', v.ranges))

        rospy.Subscriber('/odom', Odometry, self.odom_cb)

    def odom_cb(self, odom: Odometry) -> None:
        self.odom = odom.pose.pose

    def map_cb(self, grid: OccupancyGrid):
        self.map_grid = np.resize(grid.data, (270, 270)).T

        obstacle_coords = []
        for x in range(270):
            for y in range(270):
                if self.map_grid[x][y] == 0:
                    obstacle_coords.append((x, y))

        self.ckdtree = spatial.cKDTree(obstacle_coords)

    def localization(self) -> None:
        while not rospy.is_shutdown():
            rospy.Rate(20).sleep()
            if (
                not hasattr(self, 'scan')
                or not hasattr(self, 'ckdtree')
                or not hasattr(self, 'odom')
            ):
                continue

            if not hasattr(self, 'last_pose'):
                self.last_pose = self.odom

            if not hasattr(self, 'particles'):
                self.create_particles(self.n_particles)

            dx = self.odom.position.x - self.last_pose.position.x
            dy = - (self.odom.position.y - self.last_pose.position.y)

            dx, dy = meters_to_pixel(dx, dy)

            dtheta = (
                orientation_to_yaw(self.odom.orientation) -
                orientation_to_yaw(self.last_pose.orientation)
            )

            # save last pose before calling loc algorithm
            self.last_pose = self.odom

            self.particles = monte_carlo_localization(
                self.particles, [dx, dy, dtheta], self.scan, self.ckdtree)

            pose_array = PoseArray()
            pose_array.poses = self.particles

            self.localization_publisher.publish(pose_array)
            self.particles.pop()

    def create_particles(self, number_of_particles: int) -> None:
        self.particles = []
        for _ in range(number_of_particles):
            x = random.randint(0, 269)
            y = random.randint(0, 269)
            while self.map_grid[x][y] == 100 or self.map_grid[x][y] == 19:
                x = random.randint(0, 269)
                y = random.randint(0, 269)

            # for theta in (0, math.pi/2, -math.pi/2, math.pi):
            theta = 0
            particle = Particle(
                Point(x, y, 0), Quaternion(*yaw_to_orientation(theta)))

            self.particles.append(particle)


if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    particle_filter = ParticleFilter(2000)
    particle_filter.localization()
    rospy.spin()
