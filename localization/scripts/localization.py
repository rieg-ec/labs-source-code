#!/usr/bin/env python3
import rospy
import math
import random
from typing import List


class Particle:
    def __init__(self, x: float, y: float, theta: float) -> None:
        self.x = x
        self.y = y
        self.theta = theta

    def move(self, dx: float, dy: float, dtheta: float) -> None:
        if self.x < 269 and self.x >= 0:
            self.x += dx
        if self.y < 269 and self.y >= 0:
            self.y += dy

        self.theta += dtheta


def sample_motion_model(action: list, particle: Particle) -> None:
    particle.move(*action)


def likelihood_fields(measurement: list, particle: Particle, map_array: list) -> float:
    return random.random() * 3


def monte_carlo_localization(
    particles: List[Particle],
    action: list,
    measurement: list,
    map_array: list,
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
