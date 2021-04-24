from matplotlib import pyplot as plt
import math
import numpy as np
from typing import Tuple, List


def plot_results(odom: List[Tuple[float, float]], real: List[Tuple[float, float]]):
    """
    x: x-axis values for odom and real_pose
    y: y-axis values for odom and real_pose
    """
    _, [ax1, ax2] = plt.subplots(1, 2, figsize=(10, 5))

    odom = np.array(odom)
    real = np.array(real)

    mean_error = np.around(np.sum(np.linalg.norm(odom-real))/odom.shape[0], 2)

    ax1.scatter(odom[:, 0], odom[:, 1], c='b', marker='x')
    ax1.scatter(real[:, 0], real[:, 1], c='r', marker='s')
    ax1.legend(["odom", "real"], loc='lower right')
    ax1.text(0.2, 0.9, f"mean error: {mean_error}", horizontalalignment='center',
             verticalalignment='center', transform=ax1.transAxes)

    ax1.set_xlim([-1, 4])
    ax1.set_ylim([-1, 3])

    real -= 1
    mean_error = np.around(np.sum(np.linalg.norm(odom-real))/odom.shape[0], 2)

    ax2.scatter(odom[:, 0], odom[:, 1], c='b', marker='x')
    ax2.scatter(real[:, 0], real[:, 1], c='r', marker='s')
    ax2.legend(["odom", "real"], loc='lower right')
    ax2.text(0.2, 0.9, f"mean error: {mean_error}", horizontalalignment='center',
             verticalalignment='center', transform=ax2.transAxes)

    ax2.set_xlim([-1, 3])
    ax2.set_ylim([-2, 2])

    plt.show()


def calculate_ang(current_ang: float, goal_ang: float, speed: float) -> Tuple[float, int]:
    """
    returns time at <speed> at which the target will be met,
    plus the direction in which speed should be applied (-1 or 1).
    """
    if goal_ang < 0:
        goal_ang += + 2*math.pi
    if current_ang < 0:
        current_ang += + 2*math.pi

    if goal_ang == current_ang:
        return (0, 0)

    if goal_ang < math.pi:
        if current_ang < goal_ang or current_ang > goal_ang + math.pi:
            dir = 1
        else:
            dir = -1
    elif current_ang < goal_ang and current_ang > goal_ang - math.pi:
        dir = 1
    else:
        dir = -1

    if ((goal_ang > current_ang and dir == 1) or
            (goal_ang < current_ang and dir == -1)
            ):
        time = abs((goal_ang - current_ang) / speed)

    elif goal_ang > current_ang and dir == -1:
        time = ((math.pi * 2 - goal_ang) + current_ang) / speed

    elif goal_ang < current_ang and dir == 1:
        time = ((math.pi * 2 - current_ang) + goal_ang) / speed

    return (time, dir)


if __name__ == "__main__":
    speed = math.pi/2
    assert calculate_ang(0, math.pi/2, speed) == (1, 1)
    assert calculate_ang(math.pi/2, 0, speed) == (1, -1)
    assert calculate_ang(0, math.pi * 3/2, speed) == (1, -1)
    assert calculate_ang(0, math.pi/4, speed) == (0.5, 1)
    assert calculate_ang(math.pi * 3/2, math.pi, speed) == (1, -1)
    assert calculate_ang(math.pi + math.pi/5, math.pi/2, speed)[1] == -1
    plot_results([(0, 0), (1, 1)], [(1, 1), (2, 2)])
