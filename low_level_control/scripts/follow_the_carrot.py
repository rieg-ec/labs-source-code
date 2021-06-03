#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, Path
from typing import List
from utils import (
    euclidean_distance_2d, orientation_to_yaw, ang_error,
    shortest_line_angle, plot_path
)


class FollowTheCarrot:

    def __init__(self) -> None:
        rospy.Subscriber('/odom', Odometry, self._odom_listener_cb)
        rospy.Subscriber('/nav_plan', Path, self._path_cb)

        rospy.Subscriber('/control_effort', Float64,
                         lambda v: setattr(self, 'ang_speed', v.data * -1))

        self.ang_setpoint_publisher = rospy.Publisher(
            '/setpoint', Float64, queue_size=1)

        self.ang_state_publisher = rospy.Publisher(
            '/state', Float64, queue_size=1)

        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.odom_position: Pose = Pose()
        self.odom_positions: List[Pose] = []

        self.lin_speed: float = 0.1
        self.ang_speed: float = 0

        self.closest_point_index = 0  # used to update the carrot
        self.path_positions: List[Point] = []

    @property
    def carrot(self) -> Point:
        gap = 10
        if self.closest_point_index < len(self.path_positions) - gap - 1:
            return self.path_positions[self.closest_point_index + gap]

        if (
            euclidean_distance_2d(
                self.odom_position.position,
                self.path_positions[self.closest_point_index]
            ) < 0.05
        ):
            self.lin_speed = 0

        return self.path_positions[self.closest_point_index]

    def _odom_listener_cb(self, odom: Odometry) -> None:
        self.odom_positions.append(odom.pose.pose)
        self.odom_position = odom.pose.pose

    def update_closest_point(self) -> None:
        """ returns index of closest point to current position in path_positions """

        if self.path_positions:
            for i, pos in enumerate(self.path_positions):
                if (
                    euclidean_distance_2d(self.odom_position.position, pos) <
                    euclidean_distance_2d(self.odom_position.position,
                                          self.path_positions[self.closest_point_index])
                ):
                    self.closest_point_index = i

    def _path_cb(self, nav_path: Path) -> None:
        self.path_positions.extend([i.pose.position for i in nav_path.poses])

    def spin(self) -> None:
        while not self.path_positions:
            rospy.Rate(10).sleep()

        self.ang_setpoint_publisher.publish(0)

        while not rospy.is_shutdown() and self.lin_speed:
            self.update_closest_point()

            target_ang = shortest_line_angle(
                self.odom_position.position, self.carrot
            )

            current_ang = orientation_to_yaw(self.odom_position.orientation)

            self.ang_state_publisher.publish(
                ang_error(current_ang, target_ang))

            velocity = Twist()
            velocity.angular.z = self.ang_speed
            velocity.linear.x = self.lin_speed

            self.velocity_publisher.publish(velocity)

            rospy.Rate(10).sleep()

        plot_path([(i.position.x, i.position.y) for i in self.odom_positions],
                  [(i.x, i.y) for i in self.path_positions])


if __name__ == '__main__':
    rospy.init_node('follow_the_carrot', anonymous=True)
    follow_the_carrot = FollowTheCarrot()
    follow_the_carrot.spin()

    rospy.spin()
