#!/usr/bin/env python3

import rospy
import math
import numpy as np
import pandas as pd
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray
#!/usr/bin/env python3

from nav_msgs.msg import Odometry

import random as rm


class ParticleFilter:

    def __init__(self):
        pass

    def sample_motion(self, action, particle):
        particle.mover(1, 1, 0)
        return particle

    def localization(self, Particles, action, measures, maparray):
        NewParticlesToFilter = []
        NewParticles = []
        for particle in Particles:
            x = self.sample_motion(action, particle)
            w = self.measurement_model(measures, x, maparray)
            w_to_list = int((w)*10)
            for i in range(w_to_list):
                NewParticlesToFilter.append(x)

        for i in range(len(Particles)):
            index = rm.randint(0, len(NewParticlesToFilter) - 1)
            NewParticles.append(NewParticlesToFilter[index])

        return NewParticles

    def measurement_model(self, measures, particle, maparray):
        return rm.random()

class Particle():

    def __init__(self, eje_x, eje_y, teta):

        self.posx = eje_x
        self.posy = eje_y
        self.teta = teta

    def mover( self, change_in_x, change_in_y, change_in_teta ):
        self.posx += change_in_x
        self.posy += change_in_y
        self.teta += change_in_teta
        return