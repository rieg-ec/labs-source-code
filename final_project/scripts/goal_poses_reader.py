#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion

rospy.init_node('goal_poses_reader', anonymous=True)
goal_poses_publisher = rospy.Publisher(
    '/goal_poses', PoseArray, queue_size=1)

goal_poses = PoseArray()

goal_poses.poses.append(
    Pose(Point(0.5, 0.5, 0), Quaternion())
)

goal_poses.poses.append(
    Pose(Point(2.77, 1.9, 0), Quaternion())
)


def publish_poses():
    while not goal_poses_publisher.get_num_connections():
        rospy.Rate(5).sleep()

    goal_poses_publisher.publish(goal_poses)


if __name__ == '__main__':
    try:
        publish_poses()
    except rospy.ROSInterruptException:
        pass
