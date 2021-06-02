#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray
from tf.transformations import quaternion_from_euler
from os import path
import rospkg

rospack = rospkg.RosPack()

pose_msg = PoseArray()

filepath = path.join(rospack.get_path(
    'low_level_control'), 'scripts', 'positions.csv')

with open(filepath) as f:
    positions_list = []
    for line in f.read().split('\n'):
        if line:
            pos_list = [float(i) for i in line.strip().split(',')]
            odom_quat = quaternion_from_euler(0, 0, pos_list[2])
            pose = Pose(Point(*pos_list[:2], 0), Quaternion(*odom_quat))
            pose_msg.poses.append(pose)

rospy.init_node('file_reader', anonymous=True)
positions_publisher = rospy.Publisher(
    '/goal_list', PoseArray, queue_size=10)


def broadcast_pose_array():
    while positions_publisher.get_num_connections() < 1:
        rospy.Rate(1).sleep()
    positions_publisher.publish(pose_msg)


if __name__ == '__main__':
    try:
        broadcast_pose_array()
    except rospy.ROSInterruptException:
        pass
