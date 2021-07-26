#!/usr/bin/env python3
from os import path

import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point, Pose, Twist, PoseArray, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64

from utils import (angular_error, euclidean_distance_2d, orientation_to_yaw,
                   shortest_line_angle, yaw_to_orientation)


class NavController:

    def __init__(self) -> None:
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        rospy.Subscriber('/nav_plan', Path, self.path_cb)

        rospy.Subscriber('/control_effort', Float64,
                         lambda v: setattr(self, 'angular_speed', v.data * -1))

        rospy.Subscriber('/localization', Pose, self.localization_cb)

        self.ang_state_publisher = rospy.Publisher(
            '/state', Float64, queue_size=1)

        self.ang_setpoint_publisher = rospy.Publisher(
            '/setpoint', Float64, queue_size=1)

        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.path_planning_publisher = rospy.Publisher(
            '/bfs', PoseArray, queue_size=1
        )

        self.locate_publisher = rospy.Publisher(
            '/locate', Pose, queue_size=1
        )

        self.path_finished = False

        self.angular_speed = 0

        self.closest_point_index = 0

        rospack = rospkg.RosPack()

        filename = 'poses.yaml'
        filepath = path.join(rospack.get_path(
            'final_project'), 'scripts', filename)

        with open(filepath) as f:
            doc = yaml.load(f)

        self.pose_array = PoseArray()

        for i in range(1, 4):
            x, y, yaw = doc[f'pose_{i}']
            point = Point(x, y, 0)
            quaternion = Quaternion(*yaw_to_orientation(yaw))
            self.pose_array.poses.append(Pose(point, quaternion))

        start_pose = self.pose_array.poses.pop(0)
        mid_pose = self.pose_array.poses.pop(0)
        self.last_pose = self.pose_array.poses[-1]
        self.publish_pose(start_pose, mid_pose)

    @property
    def carrot(self) -> Point:
        GAP = 3

        if self.closest_point_index + GAP > len(self.path_plan.poses) - 1:
            return self.path_plan.poses[-1].pose.position

        return self.path_plan.poses[self.closest_point_index + GAP].pose.position

    def localization_cb(self, pose: Pose):
        print('llego el loc cb mano')

        self.pose = pose
        self.localized = True

    def path_cb(self, path: Path) -> None:
        print('llego el path cb mano')
        while (
                not hasattr(self, 'odom') or
                not hasattr(self, 'pose') or
                not self.localized
        ):
            rospy.Rate(10).sleep()

        self.path_plan = path
        self.follow_the_carrot()

        if self.pose_array.poses:
            pose_end = self.pose_array.poses.pop(0)
        else:
            pose_end = None

        self.publish_pose(pose_end=pose_end)

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
            # TODO: add d_theta instead of rewriting oritentation
            self.pose.orientation = self.odom.orientation

    def update_closest_point_index(self) -> None:
        if self.closest_point_index > len(self.path_plan.poses) - 1:
            self.closest_point_index = 0

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
            ) > 0.1
        ):

            self.update_closest_point_index()

            current_angle = orientation_to_yaw(self.pose.orientation)
            target_angle = shortest_line_angle(self.pose.position, self.carrot)

            self.ang_state_publisher.publish(
                angular_error(current_angle, target_angle)
            )

            velocity.angular.z = self.angular_speed

            self.velocity_publisher.publish(velocity)
            rospy.Rate(10).sleep()

        self.localized = False

    def publish_pose(self, pose_start: Pose = None, pose_end: Pose = None) -> None:
        while (
                self.path_planning_publisher.get_num_connections() < 1 or
                self.locate_publisher.get_num_connections() < 1
        ):
            rospy.Rate(10).sleep()

        if not pose_start:
            pose_start = self.pose

        if not pose_end:
            self.locate_publisher.publish(pose_start)
            while not self.localized:
                rospy.Rate(10).sleep()
            print(
                f'({self.pose.position.x}, {self.pose.position.y})',
                f'({pose_start.position.x}, {pose_start.position.y})',
                f'{euclidean_distance_2d(self.pose.position, self.last_pose.position)}'
            )
            return

        path_poses = PoseArray()
        path_poses.poses.extend([pose_start, pose_end])
        self.path_planning_publisher.publish(path_poses)

        self.locate_publisher.publish(pose_start)


if __name__ == '__main__':
    rospy.init_node('navigation_controller', anonymous=True)
    navigation_controller = NavController()
    rospy.spin()
