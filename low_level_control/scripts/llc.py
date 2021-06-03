#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray
from nav_msgs.msg import Odometry
from typing import Tuple, List
from utils import ang_error, plot_path, orientation_to_yaw, box_mean_error


class NavPID:

    def __init__(self) -> None:
        rospy.Subscriber('/goal_list', PoseArray, self._move_action_cb)
        rospy.Subscriber('/odom', Odometry, self._odom_listener)
        rospy.Subscriber('/occupancy_state', String,
                         lambda state: setattr(self, 'occupancy_state', state.data))

        # PID

        rospy.Subscriber('/pid_linear/control_effort', Float64,
                         lambda v: setattr(self, 'lin_speed', v.data * -1))

        rospy.Subscriber('/pid_angular/control_effort', Float64,
                         lambda v: setattr(self, 'ang_speed', v.data * -1))

        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.ang_setpoint_publisher = rospy.Publisher(
            '/pid_angular/setpoint', Float64, queue_size=1)

        self.ang_state_publisher = rospy.Publisher(
            '/pid_angular/state', Float64, queue_size=1)

        self.linear_setpoint_publisher = rospy.Publisher(
            '/pid_linear/setpoint', Float64, queue_size=1)

        self.linear_state_publisher = rospy.Publisher(
            '/pid_linear/state', Float64, queue_size=1)
        # PID

        self.odom_position = Pose()

        self.odom_positions: List[Tuple[float, float]] = []

        self.lin_speed: float = 0
        self.ang_speed: float = 0

        self.occupancy_state: str = "free"

        self.ERROR_TOLERANCE = 0.05

    def _controlled_forward(self, goal_pos_lin: float, axis: str) -> None:
        def get_state():
            if axis == 'y':
                return self.odom_position.position.y
            else:
                return self.odom_position.position.x

        self.linear_setpoint_publisher.publish(0)

        while self.ERROR_TOLERANCE < abs(get_state() - goal_pos_lin):
            self.linear_state_publisher.publish(
                abs(get_state() - goal_pos_lin)
            )

            velocity = Twist()
            velocity.linear.x = self.lin_speed

            if self.occupancy_state == 'free':
                self.velocity_publisher.publish(velocity)

            rospy.Rate(10).sleep()

    def _controlled_turning(self, goal_ang: float) -> None:
        def get_state() -> float:
            current_ang = orientation_to_yaw(self.odom_position.orientation)
            return ang_error(current_ang, goal_ang)

        self.ang_setpoint_publisher.publish(0)

        while self.ERROR_TOLERANCE < abs(get_state()):
            velocity = Twist()
            velocity.angular.z = self.ang_speed
            self.ang_state_publisher.publish(
                get_state()
            )

            velocity = Twist()
            velocity.angular.z = self.ang_speed * get_state()[0]

            self.velocity_publisher.publish(velocity)

            rospy.Rate(10).sleep()

    def _move_robot_to_destination_ctrl(self, goal_pose: Pose) -> None:
        dist_y = goal_pose.position.y - self.odom_position.position.y
        dist_x = goal_pose.position.x - self.odom_position.position.x

        if self.ERROR_TOLERANCE < abs(dist_y):
            if dist_y > 0:
                self._controlled_turning(math.pi/2)
            else:
                self._controlled_turning(-math.pi/2)

            self._controlled_forward(goal_pose.position.y, axis='y')

        if self.ERROR_TOLERANCE < abs(dist_x):
            if dist_x > 0:
                self._controlled_turning(0)
            else:
                self._controlled_turning(math.pi)

            self._controlled_forward(goal_pose.position.x, axis='x')

        self._controlled_turning(orientation_to_yaw(goal_pose.orientation))

    def _move_action_cb(self, pose_array: PoseArray) -> None:
        for goal_pose in pose_array.poses:
            self._move_robot_to_destination_ctrl(goal_pose)
            self.log_poses()

        plot_path(self.odom_positions,
                  [(i.position.x, i.position.y) for i in pose_array.poses],
                  box_mean_error(self.odom_positions))

    def _odom_listener(self, odom: Odometry) -> None:
        self.odom_position = odom.pose.pose

    def log_poses(self):
        self.odom_positions.append((
            self.odom_position.position.x,
            self.odom_position.position.y
        ))


if __name__ == '__main__':
    rospy.init_node('pid_navigation', anonymous=True)
    dead_reck_nav = NavPID()
    rospy.spin()
