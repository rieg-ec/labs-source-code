#!/usr/bin/env python3
import random
from typing import Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from scipy import spatial
from sensor_msgs.msg import LaserScan

from localization import Particle, monte_carlo_localization
from utils import (MAP_DIM, MAP_DIM_METERS, meters_to_pixel,
                   orientation_to_yaw, pixel_to_meters, yaw_to_orientation)


class ParticleFilter:

    def __init__(self, n_particles: int) -> None:
        self.n_particles = n_particles

        self.particles_publisher = rospy.Publisher(
            '/particles', PoseArray, queue_size=1)

        self.localization_publisher = rospy.Publisher(
            '/localization', Pose, queue_size=1)

        rospy.Subscriber('/map', OccupancyGrid, self.map_cb)

        rospy.Subscriber('/scan', LaserScan,
                         lambda v: setattr(self, 'scan', v.ranges))

        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        rospy.Subscriber('/locate', Pose, self.locate_cb)

    def locate_cb(self, pose: Pose) -> None:
        while (
            not hasattr(self, 'scan') or
            not hasattr(self, 'ckdtree') or
            not hasattr(self, 'odom') or
            not hasattr(self, 'map_grid')
        ):
            rospy.Rate(10).sleep()

        x = pose.position.x
        y = pose.position.y
        theta = orientation_to_yaw(pose.orientation)

        x, y = meters_to_pixel(x, MAP_DIM_METERS[1] - y)

        self.create_particles(x, y, theta)

        self.localization()

    def odom_cb(self, odom: Odometry) -> None:
        self.odom = odom.pose.pose

    def map_cb(self, grid: OccupancyGrid):
        self.map_grid = np.resize(grid.data, MAP_DIM[::-1]).T

        obstacle_coords = []
        for x in range(MAP_DIM[0]):
            for y in range(MAP_DIM[1]):
                if self.map_grid[x][y] == 0:
                    obstacle_coords.append(pixel_to_meters(x, y))

        self.ckdtree = spatial.cKDTree(obstacle_coords)

    def localization(self) -> None:
        localized = False
        DISTANCE_THRESHOLD = 4

        while not rospy.is_shutdown() and not localized:

            if not hasattr(self, 'last_pose'):
                self.last_pose = self.odom

            dx = self.odom.position.x - self.last_pose.position.x
            dy = - (self.odom.position.y - self.last_pose.position.y)

            dx, dy = meters_to_pixel(dx, dy)

            dtheta = (
                orientation_to_yaw(self.odom.orientation) -
                orientation_to_yaw(self.last_pose.orientation)
            )

            # save last pose before calling localization algo
            self.last_pose = self.odom

            self.particles = monte_carlo_localization(
                self.particles, [dx, dy, dtheta], self.scan, self.ckdtree)

            pose_array = PoseArray()
            pose_array.poses = self.particles

            particles_distance = self.particles_distance(pose_array)

            print(particles_distance)

            if particles_distance < DISTANCE_THRESHOLD:
                mc_x, mc_y = self.mass_center(pose_array)

                print(pixel_to_meters(mc_x, MAP_DIM[1] - mc_y), 'AAAAAA')

                # TODO: mass_center but for theta
                pose = Pose(
                    Point(*pixel_to_meters(mc_x, MAP_DIM[1] - mc_y), 0),
                    Quaternion(*yaw_to_orientation(0))
                )

                while self.localization_publisher.get_num_connections() < 1:
                    rospy.Rate(10).sleep()
                self.localization_publisher.publish(pose)
                localized = True

            while self.particles_publisher.get_num_connections() < 1:
                rospy.Rate(10).sleep()
            self.particles_publisher.publish(pose_array)

            rospy.Rate(20).sleep()

    def mass_center(self, particles: PoseArray) -> Tuple[float, float]:
        sum_x = sum([particle.position.x for particle in particles.poses])
        sum_y = sum([particle.position.y for particle in particles.poses])

        mc_x = sum_x / len(particles.poses)
        mc_y = sum_y / len(particles.poses)

        return mc_x, mc_y

    def particles_distance(self, particles: PoseArray) -> float:
        mc_x, mc_y = self.mass_center(particles)

        dist_x = 0
        dist_y = 0

        for particle in particles.poses:
            dist_x += abs(particle.position.x - mc_x)
            dist_y += abs(particle.position.y - mc_y)

        return (dist_x / len(particles.poses)) + (dist_y / len(particles.poses))

    def create_particles(self, x_start: int, y_start: int, theta_start: float) -> None:
        print(x_start, y_start, 'startttttttttttt')
        ERROR = 4

        if x_start - ERROR < 0:
            x_start = ERROR

        if y_start - ERROR < 0:
            y_start = ERROR

        if x_start + ERROR > MAP_DIM[0] - 1:
            x_start = MAP_DIM[0] - ERROR - 1

        if y_start + ERROR > MAP_DIM[1] - 1:
            y_start = MAP_DIM[1] - ERROR - 1

        self.particles = []

        for _ in range(self.n_particles):
            x = random.randint(x_start - ERROR, x_start + ERROR)
            y = random.randint(y_start - ERROR, y_start + ERROR)

            while (
                self.map_grid[x][y] == 100 or
                self.map_grid[x][y] == 19
            ):

                x = random.randint(x_start - ERROR, x_start + ERROR)
                y = random.randint(y_start - ERROR, y_start + ERROR)

            self.particles.append(Particle(
                Point(x, y, 0), Quaternion(*yaw_to_orientation(theta_start))))


if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    particle_filter = ParticleFilter(300)
    rospy.spin()
