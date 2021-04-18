#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray
from tf.transformations import quaternion_from_euler

rospy.init_node('file_reader', anonymous=True)
positions_publisher = rospy.Publisher(
    '/goal_list', PoseArray, queue_size=10)

rate = rospy.Rate(10)

pose_msg = PoseArray()

filepath = 'positions.csv'

with open(filepath) as f:
    positions_list = []
    for line in f.read().split('\n'):
        if line:
            pos_list = [float(i) for i in line.strip().split(',')]
            odom_quat = quaternion_from_euler(0, 0, pos_list[2])
            pose = Pose(Point(*pos_list[:2], 0), Quaternion(*odom_quat))
            pose_msg.poses.append(pose)


def broadcast_pose_array():
    rate = rospy.Rate(0.5)
    while positions_publisher.get_num_connections() < 1:
        rate.sleep()

    rospy.loginfo(f"sending pose_array {pose_msg}")
    positions_publisher.publish(pose_msg)


if __name__ == '__main__':
    try:
        broadcast_pose_array()
    except rospy.ROSInterruptException:
        pass
