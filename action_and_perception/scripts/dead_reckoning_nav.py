#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseArray, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from typing import Tuple, List
from utils import calculate_ang, plot_results


class DeadReckonNav:

    def __init__(self) -> None:
        rospy.Subscriber('/goal_list', PoseArray, self._move_action_cb)
        rospy.Subscriber('/odom', Odometry, self._odom_listener)
        rospy.Subscriber('/real_pose', Pose, self._real_pose_listener)
        rospy.Subscriber('/occupancy_state', String,
                         lambda state: setattr(self, 'occupancy_state', state))
        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.pose_estimation = Pose()

        self.real_positions: List[Tuple[float, float]] = []
        self.odom_positions: List[Tuple[float, float]] = []

        self.lin_speed: float = 0.4
        self.ang_speed: float = 1

        self.occupancy_state = "free"

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

    def _real_pose_listener(self, pose: Pose) -> None:
        self.real_pose = pose

    def _move_action_cb(self, pose_array: PoseArray) -> None:
        rate = rospy.Rate(10)
        while not self.pose_estimation:
            rate.sleep()

        for goal_pose in pose_array.poses:
            self.in_destination = False
            self.goal_pose = goal_pose
            self._move_robot_to_destination(goal_pose)

        rospy.loginfo(f"odom: {self.odom_positions}")
        rospy.loginfo(f"real: {self.real_positions}")

        plot_results(self.odom_positions, self.real_positions)

    def _move_robot_to_destination(self, goal_pose: Pose) -> None:
        for (lin_speed, ang_speed, t) in self._calc_velocity(goal_pose):
            self._apply_velocity(lin_speed, ang_speed, t)

    def _apply_velocity(self, lin_speed: float, ang_speed: float, t: rospy.Time) -> None:
        velocity = Twist()
        velocity.linear.x = lin_speed
        velocity.angular.z = ang_speed

        ellapsed_t = rospy.Time(secs=0)

        rate = 100

        while ellapsed_t < t:
            if self.occupancy_state == "free":
                self.velocity_publisher.publish(velocity)
                ellapsed_t.secs += 1/rate
            rospy.Rate(rate).sleep()

        if lin_speed:  # log after having moved with lin_speed
            self.log_pose()

    def _calc_velocity(self, goal_pose: Pose) -> Tuple[Tuple[float, float, rospy.Time], ...]:
        """
            1. rotate until matching vertical direction -> move in vertical
            2. rotate until matching horizotnal direction -> move in horizontal
            3. rotate until matching final angle
        """

        if self.pose_estimation.position.y < goal_pose.position.y:
            goal_ang_y = math.pi/2
        else:
            goal_ang_y = math.pi * 3/2

        y_ang_time, y_ang_dir = calculate_ang(
            euler_from_quaternion((
                self.pose_estimation.orientation.x,
                self.pose_estimation.orientation.y,
                self.pose_estimation.orientation.z,
                self.pose_estimation.orientation.w
            ))[2], goal_ang_y, self.ang_speed)

        y_lin_time = abs(self.pose_estimation.position.y -
                         goal_pose.position.y) / self.lin_speed

        if self.pose_estimation.position.x < goal_pose.position.x:
            goal_ang_x = 0
        else:
            goal_ang_x = math.pi

        x_ang_time, x_ang_dir = calculate_ang(
            goal_ang_y, goal_ang_x, self.ang_speed)

        x_lin_time = abs(self.pose_estimation.position.x -
                         goal_pose.position.x) / self.lin_speed

        goal_ang = euler_from_quaternion((
            goal_pose.orientation.x,
            goal_pose.orientation.y,
            goal_pose.orientation.z,
            goal_pose.orientation.w
        ))[2]

        final_ang_time, finalg_ang_dir = calculate_ang(
            goal_ang_x, goal_ang, self.ang_speed)

        ang_y = (0, self.ang_speed * y_ang_dir, rospy.Time(y_ang_time))
        lin_y = (self.lin_speed, 0, rospy.Time(y_lin_time))
        ang_x = (0, self.ang_speed * x_ang_dir, rospy.Time(x_ang_time))
        lin_x = (self.lin_speed, 0, rospy.Time(x_lin_time))
        final_ang = (0, self.ang_speed * finalg_ang_dir,
                     rospy.Time(final_ang_time))

        return (ang_y, lin_y, ang_x, lin_x, final_ang)

    def log_pose(self):

        self.odom_positions.append((
            self.pose_estimation.position.x,
            self.pose_estimation.position.y
        ))
        self.real_positions.append((
            self.real_pose.position.x,
            self.real_pose.position.y
        ))


if __name__ == '__main__':
    rospy.init_node('dead_reckoning_nav', anonymous=True)
    dead_reck_nav = DeadReckonNav()
    rospy.spin()
