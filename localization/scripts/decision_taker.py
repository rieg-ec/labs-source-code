#!/usr/bin/env python3

import rospy
import math
import numpy as np
import random as rm
import cv2

from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from particle_filter import ParticleFilter, Particle
# from iot_humans_track.msg import int_list_2d, int_list_1d
from display_particles import prepare_image


class DecisionTaker:

    def __init__(self, n_particles) -> None:
        self.map_array = cv2.imread("../maps/map.pgm", cv2.IMREAD_GRAYSCALE)
        # Particles
        self.particle_filter = ParticleFilter()
        self.create_particles(n_particles)

        rospy.Subscriber('/scan', LaserScan, self.lidar_listener) # LIDAR

        self.particle_position_publisher = rospy.Publisher('/particles', PoseArray, queue_size=1)

    def lidar_listener(self, scan):
        for action in ('adelante', 'derecha', 'izquierda'):
            self.particles = self.particle_filter.localization(
                self.particles, action, scan, self.map_array)
            
            self.particle_publish()

    def particle_publish(self):
        prepare_image(self.particles)
        # poses = PoseArray()
        # for p in self.particles:
        #     point = Point( *(p.posx, p.posy, 0) )
        #     quaternion = quaternion_from_euler(0, 0, p.teta)
        #     poses.poses.append( Pose(point, quaternion) )

        #     # msg_list = int_list_2d()
        #     # msg_list.data.append(int_list_1d(data=[p.posx, p.posy]))

        # self.particle_position_publisher.publish(poses)

    def create_particles(self, number_of_particles):
        self.particles = []
        teta_inicial = 0
        while len(self.particles) < number_of_particles:

            x = rm.randint(0, 269)
            y = rm.randint(0, 269)
            if int(self.map_array[x][y]) < 255:
                continue
            else:
                self.particles.append( Particle(x, y, teta_inicial) )
        return
        
if __name__ == '__main__':
    rospy.init_node('decisiontaker', anonymous=True)

    decisiontaker = DecisionTaker(1000)

    rospy.spin()
