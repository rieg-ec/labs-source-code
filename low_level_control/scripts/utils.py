from matplotlib import pyplot as plt
import math
import numpy as np
from typing import Tuple, List
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def box_mean_error(
    odom: List[Tuple[float, float]],
):
    bl_corners_odom = np.array([odom[i]
                               for i in range(len(odom)) if i % 4 == 0 and i != 0])

    real = np.array([(0, 0) for _ in range(len(bl_corners_odom))])

    mean_error = np.around(np.sum(np.linalg.norm(
        bl_corners_odom-real))/real.shape[0], 2)

    return mean_error


def plot_path(
        odom: List[Tuple[float, float]],
        real: List[Tuple[float, float]],
        error: float = 0
):
    """
    x: x-axis values for odom and real_pose
    y: y-axis values for odom and real_pose
    """
    _, ax1 = plt.subplots(1, 1, figsize=(10, 5))

    odom = np.array(odom)
    real = np.array(real)

    ax1.scatter(real[:, 0], real[:, 1], c='r', marker='s')
    ax1.scatter(odom[:, 0], odom[:, 1], c='b', marker='x')
    ax1.legend(["odom", "real"], loc='lower right')
    ax1.text(0.2, 0.9, f"Mean error: {error}",
             horizontalalignment='center',
             verticalalignment='center',
             transform=ax1.transAxes)

    ax1.set_xlim([-1, 5])
    ax1.set_ylim([-1, 5])

    plt.show()


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


def ang_error(current: float, target: float) -> float:
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
    assert ang_error(math.pi*3/4, math.pi/4) == -math.pi/2
    assert ang_error(math.pi, math.pi/2) == -math.pi/2
    assert ang_error(-math.pi/4, math.pi/4) == math.pi/2
    assert ang_error(math.pi/2, 0) == -math.pi/2
    assert ang_error(-math.pi*3/4, 0) == math.pi*3/4

    def get_orientation_from_yaw(yaw: float) -> Quaternion:
        return quaternion_from_euler(0, 0, yaw)

    def test(yaw, x_y_current, x_y_target, sla_output, caf_output):
        pose = Pose()
        pose.position.x, pose.position.y = x_y_current
        pose.orientation = get_orientation_from_yaw(yaw)

        target_pos = Point(*x_y_target, 0)

        sla = shortest_line_angle(pose.position, target_pos)
        caf = ang_error(yaw, sla)

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
