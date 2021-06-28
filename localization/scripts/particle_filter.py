#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray
#!/usr/bin/env python3

from nav_msgs.msg import Odometry

import random as rm


class ParticleFilter:

    def __init__(self):
        pass

    def sample_motion(self, action, particle):
        
        if action == 'adelante':
            avance_x = np.cos(particle.teta)*1
            avance_y = np.sin(particle.teta)*1
            particle.mover( avance_x, avance_y, 0)
        
        elif action == 'derecha':
            teta_change = - np.pi*(90/180)
            particle.mover( 0, 0, teta_change )

        elif action == 'izquiera':
            teta_change = np.pi*(90/180)
            particle.mover( 0, 0, teta_change )

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
        
        # if self.posx < 269 and self.posx >= 0:
        #     self.posx += change_in_x
        # if self.posy < 269 and self.posy >= 0:
        #     self.posy += change_in_y
        
        # self.teta += change_in_teta

        return