#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, PoseArray, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class DeadReckonNav:
    def __init__(self):
        self.real_pose = Pose(Point(0, 0, 0),
                              quaternion_from_euler(0, 0, 0))
        self.odom_pose = Pose(Point(0, 0, 0),
                              quaternion_from_euler(0, 0, 0))

        rospy.Subscriber("/goal_list", PoseArray, self._move_action_cb)
        rospy.Subscriber("/odom", Odometry,
                         lambda pos: setattr(self, 'odom_pose', pos))
        rospy.Subscriber("/real_pose", Pose,
                         lambda pos: setattr(self, 'real_pose', pos))
        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.in_destination = False

    def listen(self):
        rospy.spin()

    def _move_action_cb(self, pose_array):
        rospy.loginfo(f"received pose_array {pose_array}")
        for goal_pose in pose_array.poses:
            self._move_robot_to_destination(goal_pose)

    def _move_robot_to_destination(self, goal_pose):
        velocities = calc_velocity(goal_pose)
        for lin_vel, ang_vel, t in velocities:
            self._apply_velocity(lin_vel, ang_vel, t)

    def _apply_velocity(self, lin_vel, ang_vel, t):
        velocity = Twist()
        velocity.linear.x = lin_vel
        velocity.angular.z = ang_vel
        t0 = rospy.Time.now().to_sec()

        rate = rospy.Rate(0.5)

        while (rospy.Time.now().to_sec() - t0 < t
               and not rospy.is_shutdown()
               and not self.in_destination
               ):
            self.velocity_publisher.publish(velocity)
            rate.sleep()

    def _calc_velocity(self, pose):
        pos = (pose.position.x, pose.position.y, 0)
        ang = euler_from_quaternion((
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ))

        rospy.loginfo(f"x: {pos[0]}, y: {pos[1]}, yaw: {ang[2]}")
        return ((1, 0, 1), (0, 1, 1), (1, 0, 1), (2, 0, 2))  # x, y, yaw


if __name__ == '__main__':
    rospy.init_node("dead_reckoning_nav", anonymous=True)
    dead_reck_nav = DeadReckonNav()
    dead_reck_nav.listen()
