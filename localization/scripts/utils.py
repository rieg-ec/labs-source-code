import math
import numpy as np
from typing import Tuple, List
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def euclidean_distance_2d(a: Point, b: Point) -> float:
    return math.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)


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
    print('tests passed')
