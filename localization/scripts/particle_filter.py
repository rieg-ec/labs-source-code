#!/usr/bin/env python3
<<<<<<< HEAD

import rospy
import math
import numpy as np
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose, PoseArray
#!/usr/bin/env python3

from nav_msgs.msg import Odometry

import random as rm
=======
import random
import numpy as np

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Point, Quaternion

from utils import orientation_to_yaw, yaw_to_orientation
from localization import monte_carlo_localization, Particle
>>>>>>> 94d776b823b3580478dbf02164efe74758d0b376


class ParticleFilter:

<<<<<<< HEAD
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
=======
    def __init__(self, n_particles: int) -> None:
        self.n_particles = n_particles

        self.localization_publisher = rospy.Publisher(
            '/localization', PoseArray)

        rospy.Subscriber(
            '/map', OccupancyGrid,
            lambda m: setattr(self, 'map_grid',
                              np.reshape(m.data, (270, 270)).T)
        )

        rospy.Subscriber('/scan', LaserScan,
                         lambda v: setattr(self, 'scan', v.intensities))
        rospy.Subscriber('/odom', Odometry, self.localization_cb)

    def localization_cb(self, odom: Odometry) -> None:
        if not hasattr(self, 'last_pose'):
            self.last_pose = odom.pose.pose
            return

        if (
            not hasattr(self, 'scan')
            or not hasattr(self, 'map_grid')
        ):
            return

        if not hasattr(self, 'particles'):
            self.create_particles(self.n_particles)

        dx = self.last_pose.position.x - odom.pose.pose.position.x
        dy = self.last_pose.position.y - odom.pose.pose.position.y
        dtheta = (
            orientation_to_yaw(odom.pose.pose.orientation) -
            orientation_to_yaw(self.last_pose.orientation)
        )

        self.particles = monte_carlo_localization(
            self.particles, [dx, dy, dtheta], self.scan, self.map_grid)

        self.last_pose = odom.pose.pose

    def create_particles(self, number_of_particles: int) -> None:
        self.particles = []
        initial_theta = 0
        for _ in range(number_of_particles):
            x = random.randint(0, 269)
            y = random.randint(0, 269)
            while self.map_grid[x][y] != 100:
                x = random.randint(0, 269)
                y = random.randint(0, 269)

            particle = Particle(
                Point(x, y, 0), Quaternion(*yaw_to_orientation(initial_theta)))

            self.particles.append(particle)


if __name__ == '__main__':
    rospy.init_node('particle_filter', anonymous=True)
    particle_filter = ParticleFilter(1000)
    rospy.spin()
>>>>>>> 94d776b823b3580478dbf02164efe74758d0b376
