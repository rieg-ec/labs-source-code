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

from utils import ang_dif, orientation_to_yaw, add_90_degrees


class NavController:

    def __init__(self) -> None:
        rospy.Subscriber('/odom', Odometry, self.odom_listener)

        # rospy.Subscriber('/map', OccupancyGrid,
        #                  lambda map: setattr(self, 'map_grid', map.data))

        rospy.Subscriber('/scan', LaserScan, self.lidar_cb)

        rospy.Subscriber('/control_effort', Float64,
                         lambda v: setattr(self, 'ang_speed', v.data * -1))

        rospy.Subscriber('/localization', PoseArray, self.localization_cb)

        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.ang_setpoint_publisher = rospy.Publisher(
            '/setpoint', Float64, queue_size=1)

        self.ang_state_publisher = rospy.Publisher(
            '/state', Float64, queue_size=1)

        self.odom_position = Pose()

        self.odom_positions: List[Tuple[float, float]] = []

        self.lin_velocity: Twist = Twist()
        self.lin_velocity.linear.x = 0
        self.ang_speed: float = 0

        self.laser_scan_intensities: List = []
        self.localization = False

        self.obstacles = []

    def localization_cb(self, particles: PoseArray) -> None:
        sum_x = sum([particle.position.x for particle in particles])
        sum_y = sum([particle.position.y for particle in particles])

        mc_x = sum_x / len(particles)
        mc_y = sum_y / len(particles)

        dist_x = 0
        dist_y = 0

        for particle in particles:
            dist_x += abs(particle.position.x - mc_x)
            dist_y += abs(particle.position.y - mc_y)

        print(dist_x + dist_y)

    def lidar_cb(self, depth_array: LaserScan) -> None:
        depth_limit = 0.45
        lin_speed = 0.1

        self.obstacles = []

        if np.mean(depth_array.ranges[81:100]) < depth_limit:
            self.obstacles.append('center')

        if 'center' in self.obstacles:
            self.lin_velocity.linear.x = 0
        else:
            self.lin_velocity.linear.x = lin_speed

    def odom_listener(self, odom: Odometry) -> None:
        self.odom_position = odom.pose.pose

    def move(self) -> None:
        while not self.localization:
            if 'center' not in self.obstacles:
                if random.random() < 0.99:
                    self.velocity_publisher.publish(self.lin_velocity)
                else:
                    if random.random() < 0.5:
                        self.rotate(1)
                    else:
                        self.rotate(-1)

            else:
                if random.random() > 0.5:
                    self.rotate(1)
                else:
                    self.rotate(-1)

            rospy.Rate(10).sleep()

    def rotate(self, direction: int) -> None:
        angle_ = orientation_to_yaw(self.odom_position.orientation)
        angle = min(
            abs(angle_ - 0),
            abs(angle_ - math.pi / 2),
            abs(angle_ - math.pi),
            abs(angle_ - math.pi * 3/2),
        )

        goal_ang = add_90_degrees(angle, direction)

        def get_state() -> float:
            current_ang = orientation_to_yaw(self.odom_position.orientation)
            return ang_dif(current_ang, goal_ang)

        while self.ang_setpoint_publisher.get_num_connections() < 1:
            rospy.Rate(10).sleep()

        self.ang_setpoint_publisher.publish(0)

        ERROR_TOLERANCE = 0.05

        while ERROR_TOLERANCE < abs(get_state()):
            velocity = Twist()
            velocity.angular.z = self.ang_speed
            self.ang_state_publisher.publish(
                get_state()
            )

            velocity = Twist()
            velocity.angular.z = self.ang_speed

            self.velocity_publisher.publish(velocity)

            rospy.Rate(10).sleep()


if __name__ == '__main__':
    rospy.init_node('navigation_controller', anonymous=True)
    navigation_controller = NavController()
    navigation_controller.move()
    rospy.spin()