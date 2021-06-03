from matplotlib import pyplot as plt
import math
import numpy as np
from typing import Tuple, List
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion


def plot_results(odom: List[Tuple[float, float]], real: List[Tuple[float, float]]):
    """
    x: x-axis values for odom and real_pose
    y: y-axis values for odom and real_pose
    """
    _, ax1 = plt.subplots(1, 1, figsize=(10, 5))

    odom = np.array(odom)
    real = np.array(real)

    # square bottom left axis is at indexes 0, 4, 8 ...
    bl_corners_odom = np.array([odom[i]
                               for i in range(len(odom)) if i % 4 == 0 and i != 0])
    bl_corners_real = np.array([real[i]
                               for i in range(len(real)) if i % 4 == 0 and i != 0])

    mean_error = np.around(np.sum(np.linalg.norm(
        bl_corners_odom-bl_corners_real))/bl_corners_real.shape[0], 2)

    ax1.scatter(odom[:, 0], odom[:, 1], c='b', marker='x')
    ax1.scatter(real[:, 0], real[:, 1], c='r', marker='s')
    ax1.legend(["odom", "real"], loc='lower right')
    ax1.text(0.2, 0.9, f"Mean error: {mean_error}\nSize: {len(bl_corners_real)}",
             horizontalalignment='center',
             verticalalignment='center',
             transform=ax1.transAxes)

    ax1.set_xlim([-1, 2])
    ax1.set_ylim([-1, 2])

    plt.show()


def calculate_ang(current_ang: float, goal_ang: float) -> Tuple[float, float]:
    """
    return rotation direction and degrees for efficient rotation
    """

    if goal_ang == current_ang:
        return (0, 0)

    elif current_ang < goal_ang:
        if goal_ang - current_ang < math.pi:
            return (1, goal_ang - current_ang)
        else:
            return (-1, current_ang + (2*math.pi - goal_ang))

    else:
        if current_ang - goal_ang < math.pi:
            return (-1, current_ang - goal_ang)
        else:
            return (1, goal_ang + (2*math.pi - current_ang))


def get_yaw(orientation: Pose.orientation):
    return euler_from_quaternion((
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    ))[2]


if __name__ == "__main__":
    assert calculate_ang(math.pi + 0.01, 0) == (1,
                                                2*math.pi - math.pi - 0.01)
    assert calculate_ang(math.pi/2, 0) == (-1, math.pi/2)
    assert calculate_ang(math.pi - 0.05, 0) == (-1, math.pi - 0.05)
    assert calculate_ang(3 * math.pi/2, math.pi/3)[0] == 1
    assert calculate_ang(0, math.pi/2) == (1, math.pi/2)

    # plot_results([(0, 0), (1, 1)], [(1, 1), (2, 2)])
