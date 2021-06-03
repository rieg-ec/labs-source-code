from matplotlib import pyplot as plt
import math
import numpy as np
from typing import Tuple, List
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def plot_path(odom: List[Tuple[float, float]], real: List[Tuple[float, float]]):
    """
    x: x-axis values for odom and real_pose
    y: y-axis values for odom and real_pose
    """
    _, ax1 = plt.subplots(1, 1, figsize=(10, 5))

    odom = np.array(odom)
    # real = np.array(real)

    # square bottom left axis is at indexes 0, 4, 8 ...
    # bl_corners_odom = np.array([odom[i]
    #                            for i in range(len(odom)) if i % 4 == 0 and i != 0])
    # bl_corners_real = np.array([real[i]
    # for i in range(len(real)) if i % 4 == 0 and i != 0])

    # mean_error = np.around(np.sum(np.linalg.norm(
    #     bl_corners_odom-bl_corners_real))/bl_corners_real.shape[0], 2)

    ax1.scatter(odom[:, 0], odom[:, 1], c='b', marker='x')
    # ax1.scatter(real[: , 0], real[: , 1], c = 'r', marker = 's')
    ax1.legend(["odom", "real"], loc='lower right')
    # ax1.text(0.2, 0.9, f"Mean error: {mean_error}\nSize: {len(bl_corners_real)}",
    # horizontalalignment = 'center',
    # verticalalignment = 'center',
    # transform = ax1.transAxes)

    ax1.set_xlim([-1, 5])
    ax1.set_ylim([-1, 5])

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


def euclidean_distance_2d(a: Point, b: Point) -> float:
    return math.sqrt((b.x - a.x)**2 + (b.y - a.y)**2)


def get_yaw_fix(orientation: Pose.orientation) -> float:
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


def calculate_ang_fix(current: float, target: float) -> float:
    return target - current


def shortest_line_angle(current: Point, target: Point) -> float:
    """
    returns angle of straight line from pose to target_pos
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


if __name__ == "__main__":
    assert calculate_ang(math.pi + 0.01, 0) == (1,
                                                2*math.pi - math.pi - 0.01)
    assert calculate_ang(math.pi/2, 0) == (-1, math.pi/2)
    assert calculate_ang(math.pi - 0.05, 0) == (-1, math.pi - 0.05)
    assert calculate_ang(3 * math.pi/2, math.pi/3)[0] == 1
    assert calculate_ang(0, math.pi/2) == (1, math.pi/2)

    def get_orientation_from_yaw(yaw: float) -> Quaternion:
        return quaternion_from_euler(0, 0, yaw)

    def test(yaw, x_y_current, x_y_target, sla_output, caf_output):
        pose = Pose()
        pose.position.x, pose.position.y = x_y_current
        pose.orientation = get_orientation_from_yaw(yaw)

        target_pos = Point(*x_y_target, 0)

        sla = shortest_line_angle(pose.position, target_pos)
        caf = calculate_ang_fix(yaw, sla)

        try:
            assert sla == sla_output
            assert caf == caf_output
        except AssertionError:
            print('test failed:')
            print('input: ', yaw, x_y_current,
                  x_y_target, sla_output, caf_output)
            print('output: ', sla, caf)
            exit(0)

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
