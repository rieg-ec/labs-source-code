#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist


class HallNavigation:

    def __init__(self) -> None:
        rospy.Subscriber('/wall_distance', Float64,
                         self.control_proporcional)
        self.velocity_publisher = rospy.Publisher(
            '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=1)

        # Constante
        self.kp = 0.7

        self.lin_speed: float = 0.0
        self.ang_speed: float = 0.0

        self.velocity = Twist()
        self.velocity.linear.x = self.lin_speed
        self.velocity.angular.z = self.ang_speed

        self.referencia: float = 0.0

    def control_proporcional(self, diferencia_obstaculos):
        error = (self.referencia - diferencia_obstaculos.data)
        self.ang_speed = self.kp * error
        self.lin_speed = 1.0
        self.velocity.angular.z = self.ang_speed
        self.velocity.linear.x = self.lin_speed

        self.velocity_publisher.publish(self.velocity)


if __name__ == '__main__':
    rospy.init_node('hall_navigation', anonymous=True)
    dead_reck_nav = HallNavigation()
    rospy.spin()

