#!/usr/bin/env python3
# from localization.scripts.utils import euclidean_distance_2d
# from localization.scripts.utils import pixel_to_meters
import random
import numpy as np
import math

from numpy.lib.function_base import piecewise

import rospy
from scipy import spatial
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose

from utils import orientation_to_yaw, yaw_to_orientation, meters_to_pixel, pixel_to_meters
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

        rospy.Subscriber('/goal_poses', PoseArray, self.postion_cb)

    def postion_cb(self, pose_array: PoseArray) -> None:

        x_pista = pose_array.poses[0].position.x
        y_pista = pose_array.poses[0].position.y
        theta = pose_array.poses[0].orientation.z

        x, y = meters_to_pixel(x_pista, 4.9 - y_pista) # la resta es para reflejar el punto.

        print(x, y)

        self.create_particles(self.n_particles, x_pista=x, y_pista=y, theta_pista=theta)

        self.localization()
 
    def odom_cb(self, odom: Odometry) -> None:
        self.odom = odom.pose.pose

    def map_cb(self, grid: OccupancyGrid):
        self.map_grid = np.resize(grid.data, (100, 196)).T
        print('map loaded')

        obstacle_coords = []
        for x in range(196):
            for y in range(100):
                if self.map_grid[x][y] == 0:
                    obstacle_coords.append(pixel_to_meters(x, y))

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

            # if not hasattr(self, 'particles'):
            #     self.create_particles(self.n_particles)

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
            # self.particles.pop()

    def create_particles(self, number_of_particles: int, x_pista=None, y_pista=None, theta_pista = None) -> None:
        # x, y = meters_to_pixel(1.0, 4.9 - 0.46)
        # x, y = meters_to_pixel(2.4, 4.9 - 0.5)
        pos_pista = (x_pista, y_pista, 0)

        RADIO_PISTA = 100  # El radio de las particulas alrededor de la pista.

        while not hasattr(self, 'map_grid'):
            rospy.Rate(20).sleep()

        print(f'Creando Particulas con {pos_pista}, {theta_pista}')

        self.particles = []
        for _ in range(number_of_particles):
            x = random.randint(0, 195)
            y = random.randint(0, 99)
            while self.map_grid[x][y] == 100 or self.map_grid[x][y] == 19 or \
                    abs(x - pos_pista[0]) + abs(y - pos_pista[1]) > RADIO_PISTA:
                x = random.randint(0, 195)
                y = random.randint(0, 99)

            self.particles.append(Particle(
                Point(x, y, 0), Quaternion(*yaw_to_orientation(theta_pista))))


if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    particle_filter = ParticleFilter(100)
    # particle_filter.localization()
    # pose_array = PoseArray()
    # pose_array.poses.append(Pose(Point(1.0, 0.5, 0), Quaternion(0, 0, math.pi, 0)))
    # particle_filter.postion_cb(pose_array)
    rospy.spin()
