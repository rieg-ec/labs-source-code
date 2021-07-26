#!/usr/bin/env python3
from os import path

import rospkg
import rospy
import yaml
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion

from utils import yaw_to_orientation

rospack = rospkg.RosPack()

filename = 'poses.yaml'

filepath = path.join(rospack.get_path(
    'final_project'), 'scripts', filename)

with open(filepath) as f:
    doc = yaml.load(f)

    pose_array = PoseArray()

    for i in range(1, 3):
        x, y, yaw = doc[f'pose_{i}']
        point = Point(x, y, 0)
        quaternion = Quaternion(*yaw_to_orientation(yaw))
        pose_array.poses.append(Pose(point, quaternion))


rospy.init_node('goal_poses_reader', anonymous=True)
goal_poses_publisher = rospy.Publisher(
    '/goal_poses', PoseArray, queue_size=1)


def publish_poses(pose_array: PoseArray):
    while goal_poses_publisher.get_num_connections() < 2:
        rospy.Rate(5).sleep()

    goal_poses_publisher.publish(pose_array)


rospy.Subscriber('/new_goal_pose', PoseArray, publish_poses)


if __name__ == '__main__':
    try:
        publish_poses(pose_array)
    except rospy.ROSInterruptException:
        pass
