#!/usr/bin/env python3
import math
import random

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64

from utils import (angular_error, euclidean_distance_2d, orientation_to_yaw,
                   shortest_line_angle)


class NavController:

    def __init__(self) -> None:
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        rospy.Subscriber('/nav_plan', Path, self.path_cb)

        rospy.Subscriber('/control_effort', Float64,
                         lambda v: setattr(self, 'angular_speed', v.data * -1))

        rospy.Subscriber('/localization_pose', Pose, self.localization_cb)

        self.ang_state_publisher = rospy.Publisher(
            '/state', Float64, queue_size=1)

        self.ang_setpoint_publisher = rospy.Publisher(
            '/setpoint', Float64, queue_size=1)

        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.localization = False
        self.obstacles = []

        self.angular_speed = 0

        self.closest_point_index = 0

    @property
    def carrot(self) -> Point:
        GAP = 3

        if self.closest_point_index + GAP > len(self.path_plan.poses) - 1:
            return self.path_plan.poses[-1].pose.position

        return self.path_plan.poses[self.closest_point_index + GAP].pose.position

    def localization_cb(self, pose: Pose):
        self.pose = pose

    def path_cb(self, path: Path) -> None:
        while not hasattr(self, 'odom') or not hasattr(self, 'pose'):
            rospy.Rate(10).sleep()

        self.path_plan = path
        self.follow_the_carrot()

    def odom_cb(self, odom: Odometry) -> None:
        if hasattr(self, 'odom'):
            dx = odom.pose.pose.position.x - self.odom.position.x
            dy = odom.pose.pose.position.y - self.odom.position.y
        else:
            dx, dy = 0, 0

        self.odom = odom.pose.pose

        if hasattr(self, 'pose'):
            self.pose.position.x += dx
            self.pose.position.y += dy
            # TODO: add dtheta instead of rewriting oritentation
            self.pose.orientation = self.odom.orientation

    def update_closes_point_index(self) -> None:
        for idx, pos in enumerate(self.path_plan.poses):
            if (
                euclidean_distance_2d(self.pose.position, pos.pose.position) <
                euclidean_distance_2d(
                    self.pose.position, self.path_plan.poses[self.closest_point_index].pose.position)
            ):
                self.closest_point_index = idx

    def follow_the_carrot(self) -> None:
        self.ang_setpoint_publisher.publish(0)

        velocity = Twist()
        velocity.linear.x = 0.15

        while (
            not rospy.is_shutdown() and
            euclidean_distance_2d(
                self.path_plan.poses[-1].pose.position, self.pose.position
            ) > 0.05
        ):

            print(f'({self.carrot.x}, {self.carrot.y})', euclidean_distance_2d(
                self.pose.position, self.path_plan.poses[self.closest_point_index].pose.position))

            self.update_closes_point_index()

            current_angle = orientation_to_yaw(self.odom.orientation)
            target_angle = shortest_line_angle(self.pose.position, self.carrot)

            self.ang_state_publisher.publish(
                angular_error(current_angle, target_angle)
            )

            velocity.angular.z = self.angular_speed

            self.velocity_publisher.publish(velocity)
            rospy.Rate(10).sleep()


if __name__ == '__main__':
    rospy.init_node('navigation_controller', anonymous=True)
    navigation_controller = NavController()
    rospy.spin()
