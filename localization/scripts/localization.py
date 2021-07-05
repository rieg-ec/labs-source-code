#!/usr/bin/env python3
import rospy
import math
import random
import numpy as np

from geometry_msgs.msg import Pose, Point, Quaternion

from typing import List

from utils import yaw_to_orientation, orientation_to_yaw


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
        if self.position.x < 269 and self.position.x >= 0:
            self.position.x += dx
        if self.position.y < 269 and self.position.y >= 0:
            self.position.y += dy

        self.yaw += dtheta


def sample_motion_model(action: list, particle: Particle) -> None:
    particle.move(*action)


def likelihood_fields(measurement: list, particle: Particle, map_array: np.array) -> float:
    return random.random() * 3


def monte_carlo_localization(
    particles: List[Particle],
    action: list,
    measurement: list,
    map_array: np.array,
) -> List[Particle]:

    weights = []
    largest_float = 0

    for particle in particles:
        sample_motion_model(action, particle)
        w = likelihood_fields(measurement, particle, map_array)
        weights.append(w)
        if len(str(w)[:2]) > largest_float:
            largest_float = len(str(w)[:2])

    scale_factor = 10 ** largest_float
    weights = [int(w * scale_factor) for w in weights]

    new_particles_weighted = []
    for idx, weight in enumerate(weights):
        for _ in range(weight):
            new_particles_weighted.append(particles[idx])

    new_particles = []
    for _ in range(len(particles)):
        index = random.randint(0, len(new_particles_weighted) - 1)
        new_particles.append(new_particles_weighted[index])

    return new_particles
