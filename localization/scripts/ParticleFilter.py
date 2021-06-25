#!/usr/bin/env python3

import rospy
import math
import numpy as np
import pandas as pd
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray
from nav_msgs.msg import Odometry



class ParticleFilter:

    def __init__(self) -> None:
        rospy.Subscriber('scan', PoseArray, self.likelihood_field)

        self.map = None
        self.x_t = None
        self.threshold

    def sample_motion(self, action, particle):
        X_t = []
        X_hat_t = []

    def localization(self, Particles, action, measures, maparray):
        NewParticlesToFilter = []
        NewParticles = []
        df = pd.Dataframe
        for particle in Particles:
            x = self.sample_motion(action, particle)
            w = self.measurement_model(measurement, x, maparray)
            df.append( pd.DataFrame({"particle":x,
                        "weight":w}) )

        for i in range(len(Particles)):
            if particle[1] > self.threshold:
                NewParticles.append(particle[0])
            sampledf = df.sample(weights = df.weight)

        return NewParticles


    def draw_particle(self):
        # Publicar peso y cosas para dibujarlo en el mapa.
        pass
