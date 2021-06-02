#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from typing import Tuple, List
from utils import calculate_ang, plot_results


class HallNavigation:

    def __init__(self) -> None:
        rospy.Subscriber('/wall_distance', Float64,
                         self.control_proporcional)
        # rospy.Subscriber('/occupancy_state', String,
        #                  self.occupancy_changes)
        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        self.real_positions: List[Tuple[float, float]] = []
        self.odom_positions: List[Tuple[float, float]] = []

        # Constante 
        self.kp = 0.7

        self.lin_speed: float = 0.0
        self.ang_speed: float = 0.0

        self.velocity = Twist()
        self.velocity.linear.x = self.lin_speed
        self.velocity.angular.z = self.ang_speed

        self.referencia: float = 0.0

        self.occupancy_state: str = "free"

    def occupancy_changes(self, occupancy):
        if occupancy == 'obstacle_center':
            self.lin_speed: float = 0.5
            self.velocity.linear.x = self.lin_speed
        else:
            self.lin_speed: float = 1.0
            self.velocity.linear.x = self.lin_speed

    def control_proporcional(self, diferencia_obstaculos):
        error = (self.referencia - diferencia_obstaculos.data)
        self.ang_speed = self.kp * error
        self.velocity.angular.z = self.ang_speed

        self.velocity_publisher.publish(self.velocity)
    

if __name__ == '__main__':
    rospy.init_node('hall_navigation', anonymous=True)
    dead_reck_nav = HallNavigation()
    rospy.spin()