#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray
from sensor_msgs.msg import LaserScan
import cv2
from particle_filter import ParticleFilter, Particle
import numpy as np
import random as rm

class DecisionTaker:

    def __init__(self) -> None:
        self.map_array = cv2.imread("../maps/map.pgm", cv2.IMREAD_GRAYSCALE)
        # Particles
        self.particle_filter = ParticleFilter()
        self.create_particles(10)

        rospy.Subscriber('/scan', LaserScan, self.lidar_listener) # LIDAR

    def lidar_listener(self, scan):
        self.particles = self.particle_filter.localization(
            self.particles, None, scan, self.map_array)

    def create_particles(self, number_of_particles):
        self.particles = []
        teta_inicial = 0
        while len(self.particles) < number_of_particles:

            x = rm.randint(0, 269)
            y = rm.randint(0, 269)
            if self.map_array[x][y] < 205 or self.map_array[x][y] == 255:
                continue
            else: 
                self.particles.append( Particle(x, y, teta_inicial) )
        return
        
if __name__ == '__main__':
    rospy.init_node('decisiontaker', anonymous=True)

    decisiontaker = DecisionTaker()

    rospy.spin()
