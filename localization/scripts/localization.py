#!/usr/bin/env python3
import rospy
import math
import random
import numpy as np
from scipy.stats import norm
from scipy import spatial

from geometry_msgs.msg import Pose, Quaternion

from typing import List

from utils import yaw_to_orientation, orientation_to_yaw, shifted_sigmoid


class Particle(Pose):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    @property
    def yaw(self):
        return orientation_to_yaw(self.orientation)

    @yaw.setter
    def yaw(self, value):
        yaw = orientation_to_yaw(self.orientation)
        yaw += value
        self.orientation = Quaternion(*yaw_to_orientation(value))

    def move(self, dx: float, dy: float, dtheta: float) -> None:
        next_x = self.position.x + dx
        next_y = self.position.y - dy
        if next_x < 269 and next_x >= 0:
            self.position.x = next_x
        if next_y < 269 and next_y >= 0:
            self.position.y = next_y

        self.yaw += dtheta


def sample_motion_model(action: list, particle: Particle) -> None:
    particle.move(*action)


def likelihood_fields(measurement: list, particle: Particle, obstacle_coords: list) -> float:
    angle_increment = 0.01745329238474369
    angle = -1.5707963705062866
    q = 1
    z_max = 4.0
    weight = 25

    sigma = 1.0
    stats = norm(0, sigma)

    tree = spatial.cKDTree(obstacle_coords)

    for z_k in measurement:
        angle += angle_increment
        if z_k != z_max:
            z_k = z_k /0.05
            # x_k es la posicion x del objeto que estamos mirando segun la posicion de la particula
            # que estamos mirando. lo mismo para y_k
            obj_from_particle_x = particle.position.x + \
                z_k * np.cos(particle.yaw + angle)
            obj_from_particle_y = particle.position.y - \
                z_k * np.sin(particle.yaw + angle)

            dist, _ = tree.query(
                [obj_from_particle_x, obj_from_particle_y])

            q *= stats.pdf(dist) * weight

    return q


def monte_carlo_localization(
    particles: List[Particle],
    action: list,
    measurement: list,
    map_array: np.array,
) -> List[Particle]:

    obstacle_coords = []
    for x in range(270):
        for y in range(270):
            if map_array[x][y] == 0:
                obstacle_coords.append((x, y))

    weights = []
    largest_float = 0

    for particle in particles:
        sample_motion_model(action, particle)
        w = likelihood_fields(measurement, particle, obstacle_coords)
        weights.append(w)
        if len(str(w)[:2]) > largest_float:
            largest_float = len(str(w)[:2])

    weights = shifted_sigmoid(np.array(weights))
    weights = [float(i)/max(weights) for i in weights]

    scale_factor = 10 ** largest_float
    weights = [int(w * scale_factor) for w in weights]

    new_particles_weighted = []
    for idx, weight in enumerate(weights):
        for _ in range(weight):
            new_particles_weighted.append(particles[idx])

    new_particles = []
    for _ in range(len(particles)):
        index = random.randint(0, len(new_particles_weighted)-1)
        new_particles.append(new_particles_weighted[index])

    if not new_particles:
        return particles

    print(f'Particle pos: {new_particles[0].position}')
    return new_particles