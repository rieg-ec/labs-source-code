#!/usr/bin/env python3

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from os import path

rospack = rospkg.RosPack()

path_filename = 'path_line.txt'

filepath = path.join(rospack.get_path(
    'low_level_control'), 'scripts', 'paths', path_filename)

with open(filepath) as f:
    path = Path()
    for line in f.read().split('\n'):
        if line:
            x, y = line.strip().split(',')
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(float(x), float(y), 0)
            path.poses.append(pose_stamped)

rospy.init_node('path_reader', anonymous=True)
nav_path_published = rospy.Publisher(
    '/nav_plan', Path, queue_size=1)


def broadcast_path():
    while nav_path_published.get_num_connections() < 1:
        rospy.Rate(1).sleep()
    nav_path_published.publish(path)


if __name__ == '__main__':
    try:
        broadcast_path()
    except rospy.ROSInterruptException:
        pass
