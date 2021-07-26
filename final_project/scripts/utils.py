import math
from typing import List, Tuple

import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

RESOLUTION = 0.025
MAP_DIM = (196, 100)
MAP_DIM_METERS = (4.9, 2.5)


def pixel_to_meters(px: int, py: int) -> Tuple[float, float]:
    x = px * RESOLUTION
    y = py * RESOLUTION
    return (x, y)


def meters_to_pixel(mx: float, my: float) -> Tuple[int, int]:
    x = int(mx / RESOLUTION)
    y = int(my / RESOLUTION)
    return (x, y)


def shifted_sigmoid(x: np.array):
    shift = 3
    return 1 / (1 + np.exp(-x + shift))


def euclidean_distance_2d(a: Point, b: Point) -> float:
    return math.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)


def angular_error(current: float, target: float) -> float:
    if target > current:
        if target - current < math.pi:
            return target - current
        else:
            return - (2*math.pi - target + current)
    else:
        if current - target < math.pi:
            return target - current
        else:
            return - (2*math.pi + target - current)


def shortest_line_angle(current: Point, target: Point) -> float:
    """
    returns angle of straight line from current to target_pos
    """
    distance_x = target.x - current.x
    distance_y = target.y - current.y

    if not distance_x:
        if distance_y > 0:
            theta = math.pi/2
        else:
            theta = -math.pi/2

        return theta

    elif not distance_y:
        if distance_x < 0:
            theta = math.pi
        else:
            theta = 0

        return theta

    alpha = math.atan(abs(distance_y/distance_x))

    if distance_x > 0:
        theta = alpha
        if distance_y < 0:
            theta *= -1
    elif distance_x < 0:
        theta = math.pi - alpha
        if distance_y < 0:
            theta *= -1

    return theta


def orientation_to_yaw(orientation: Pose.orientation) -> float:
    yaw = euler_from_quaternion((
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ))[2]

    if yaw > math.pi:
        return - (2*math.pi - yaw)
    else:
        return yaw


def yaw_to_orientation(yaw: float) -> Pose.orientation:
    if yaw < 0:
        yaw = 2*math.pi + yaw

    quaternion = quaternion_from_euler(0, 0, yaw)
    return quaternion


def ang_dif(current: float, target: float) -> float:
    if target > current:
        if target - current < math.pi:
            return target - current
        else:
            return - (2*math.pi - target + current)
    else:
        if current - target < math.pi:
            return target - current
        else:
            return - (2*math.pi + target - current)


def add_90_degrees(angle: float, dir: int) -> float:
    assert dir == 1 or dir == -1
    assert angle < 2*math.pi
    if dir == 1:
        if angle >= math.pi * 3/2 and angle < 2 * math.pi:
            dist_to_zero = 2*math.pi - angle
            zero_to_target = math.pi/2 - dist_to_zero
            return zero_to_target
        else:
            return angle + math.pi/2
    else:
        if angle >= math.pi/2:
            return angle - math.pi/2
        else:
            return 2*math.pi - (math.pi/2 - angle)


if __name__ == "__main__":
    assert angular_error(math.pi*3/4, math.pi/4) == -math.pi/2
    assert angular_error(math.pi, math.pi/2) == -math.pi/2
    assert angular_error(-math.pi/4, math.pi/4) == math.pi/2
    assert angular_error(math.pi/2, 0) == -math.pi/2
    assert angular_error(-math.pi*3/4, 0) == math.pi*3/4

    assert ang_dif(math.pi*3/4, math.pi/4) == -math.pi/2
    assert ang_dif(math.pi, math.pi/2) == -math.pi/2
    assert ang_dif(-math.pi/4, math.pi/4) == math.pi/2
    assert ang_dif(math.pi/2, 0) == -math.pi/2
    assert ang_dif(-math.pi*3/4, 0) == math.pi*3/4

    assert add_90_degrees(math.pi/2, -1) == 0
    assert add_90_degrees(0, 1) == math.pi/2
    assert add_90_degrees(0, -1) == math.pi * 3/2
    assert add_90_degrees(math.pi, -1) == math.pi/2
    assert add_90_degrees(math.pi, 1) == math.pi * 3/2
    assert add_90_degrees(math.pi * 3/2, 1) == 0
    assert add_90_degrees(math.pi/2, 1) == math.pi

    assert ang_dif(0, math.pi) == -math.pi
    assert ang_dif(math.pi, 0) == -math.pi
    assert ang_dif(math.pi/2, -math.pi/2) == -math.pi

    RESOLUTION = 0.01

    assert pixel_to_meters(100, 100) == (1, 1)
    assert meters_to_pixel(1, 1) == (100, 100)

    def test(yaw, x_y_current, x_y_target, sla_output, caf_output):
        pose = Pose()
        pose.position.x, pose.position.y = x_y_current
        pose.orientation = yaw_to_orientation(yaw)

        target_pos = Point(*x_y_target, 0)

        sla = shortest_line_angle(pose.position, target_pos)
        caf = angular_error(yaw, sla)

        try:
            assert sla == sla_output
            assert caf == caf_output
        except AssertionError:
            print('test failed:')
            print('input: ', yaw, x_y_current,
                  x_y_target, sla_output, caf_output)
            print('output: ', sla, caf)

    yaw = math.pi
    x_y_current = [1, 1]
    x_y_target = [2, 2]
    test(yaw, x_y_current, x_y_target, math.pi/4, -(math.pi*3/4))

    yaw = math.pi/2
    test(yaw, x_y_current, x_y_target, math.pi/4, -(math.pi/4))

    yaw = math.pi/4
    x_y_current = [0, 0]
    x_y_target = [1, -1]
    test(yaw, x_y_current, x_y_target, -math.pi/4, -math.pi/2)

    x_y_current = [2, -2]
    test(yaw, x_y_current, x_y_target, math.pi*3/4, math.pi/2)

    print('tests passed')
