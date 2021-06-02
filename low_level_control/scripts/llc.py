#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray, Point
from nav_msgs.msg import Odometry
from typing import Tuple, List
from utils import calculate_ang, plot_results, get_yaw


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

        self.pose_estimation = Pose()

        self.target_positions: List[Tuple[float, float]] = []
        self.odom_positions: List[Tuple[float, float]] = []

        self.lin_speed: float = 0
        self.ang_speed: float = 0

        self.occupancy_state: str = "free"
        self.in_destination = False

        self.ERROR_TOLERANCE = 0.05

    def _controlled_forward(self, goal_pos_lin: float, axis: str) -> None:
        def get_state():
            if axis == 'y':
                return self.pose_estimation.position.y
            else:
                return self.pose_estimation.position.x

        self.linear_setpoint_publisher.publish(0)

        while self.ERROR_TOLERANCE < abs(get_state() - goal_pos_lin):
            velocity = Twist()
            velocity.linear.x = self.lin_speed

            self.linear_state_publisher.publish(
                abs(get_state() - goal_pos_lin)
            )

            if self.occupancy_state == 'free':
                self.velocity_publisher.publish(velocity)

            rospy.Rate(10).sleep()

    def _controlled_turning(self, goal_ang: float) -> None:
        def get_state():
            current_ang = get_yaw(self.pose_estimation.orientation)
            return calculate_ang(current_ang, goal_ang)

        self.ang_setpoint_publisher.publish(0)

        while self.ERROR_TOLERANCE < abs(get_state()[1]):
            velocity = Twist()
            velocity.angular.z = self.ang_speed * get_state()[0]

            self.ang_state_publisher.publish(
                get_state()[1]
            )

            self.velocity_publisher.publish(velocity)

            rospy.Rate(10).sleep()

    def _move_robot_to_destination_ctrl(self, goal_pose: Pose) -> None:
        dist_y = goal_pose.position.y - self.pose_estimation.position.y
        dist_x = goal_pose.position.x - self.pose_estimation.position.x

        if self.ERROR_TOLERANCE < abs(dist_y):
            if dist_y > 0:
                self._controlled_turning(math.pi/2)
            else:
                self._controlled_turning(math.pi * 3/2)

            self._controlled_forward(goal_pose.position.y, axis='y')

        if self.ERROR_TOLERANCE < abs(dist_x):
            if dist_x > 0:
                self._controlled_turning(0)
            else:
                self._controlled_turning(math.pi)

            self._controlled_forward(goal_pose.position.x, axis='x')

        self._controlled_turning(get_yaw(goal_pose.orientation))

    def _move_action_cb(self, pose_array: PoseArray) -> None:
        while not self.pose_estimation:
            rospy.Rate(10).sleep()

        self.in_destination = False

        for goal_pose in pose_array.poses:
            self.target_positions.append((
                goal_pose.position.x, goal_pose.position.y
            ))
            self._move_robot_to_destination_ctrl(goal_pose)
            self.log_poses()

        self.in_destination = True

        plot_results(self.odom_positions, self.target_positions)
        self.odom_positions = []
        self.target_positions = []

    def _odom_listener(self, odom: Odometry) -> None:
        if not hasattr(self, 'start_pose'):
            self.start_pose = odom.pose.pose.position

        x = odom.pose.pose.position.x - \
            self.start_pose.x
        y = odom.pose.pose.position.y - \
            self.start_pose.y
        z = 0

        self.pose_estimation.position = Point(x, y, z)
        self.pose_estimation.orientation = odom.pose.pose.orientation

    def log_poses(self):
        if hasattr(self, 'pose_estimation'):
            self.odom_positions.append((
                self.pose_estimation.position.x,
                self.pose_estimation.position.y
            ))


if __name__ == '__main__':
    rospy.init_node('pid_navigation', anonymous=True)
    dead_reck_nav = NavPID()
    rospy.spin()
