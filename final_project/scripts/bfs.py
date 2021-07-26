#!/usr/bin/env python3

from os import path
from typing import List, Tuple

import cv2
import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import Path

from utils import meters_to_pixel, pixel_to_meters

ROBOTRADIUS = 10


def c_space_map(map: np.array) -> np.array:
    c_space_color = 205

    def limits(x, y, map_shape):
        return 0 < x < map_shape[0] and 0 < y < map_shape[1]

    new_map = map
    shape = map.shape
    for x in range(shape[0]):
        for y in range(shape[1]):
            if map[x][y] == 0:
                for x_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x + x_radio, y, shape) and map[x + x_radio][y] != 0:
                        new_map[x + x_radio][y] = c_space_color
                for y_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x, y + y_radio, shape) and map[x][y + y_radio] != 0:
                        new_map[x][y + y_radio] = c_space_color
                for x_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x - x_radio, y, shape) and map[x - x_radio][y] != 0:
                        new_map[x - x_radio][y] = c_space_color
                for y_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x, y - y_radio, shape) and map[x][y - y_radio] != 0:
                        new_map[x][y - y_radio] = c_space_color

                for y_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x - y_radio, y - y_radio, shape) and map[x - y_radio][y - y_radio] != 0:
                        new_map[x - y_radio][y - y_radio] = c_space_color
                    if limits(x + y_radio, y + y_radio, shape) and map[x + y_radio][y + y_radio] != 0:
                        new_map[x + y_radio][y + y_radio] = c_space_color
                    if limits(x - y_radio, y + y_radio, shape) and map[x - y_radio][y + y_radio] != 0:
                        new_map[x - y_radio][y + y_radio] = c_space_color
                    if limits(x + y_radio, y - y_radio, shape) and map[x + y_radio][y - y_radio] != 0:
                        new_map[x + y_radio][y - y_radio] = c_space_color

    return new_map.T


class State:
    def __init__(self, node_id: Tuple[int, int], pixmap: np.array, cell_size=(7, 7)) -> None:
        self.node_id = node_id      # Current cell coordinate (x, y)
        self.pixmap = pixmap
        self.cell_size = cell_size
        self.prev_action = None
        self.parent = None          # Previous state

    def expand(self):
        successors = list()

        def limits(x: int, y: int) -> bool:
            shape = self.pixmap.shape
            return 0 < x < shape[0] - 7 and 0 < y < shape[1] - 7

        def create_state(
            node_id: Tuple[int, int],
            pixmap: np.array,
            parent: State,
            action: str
        ) -> State:
            state = State(node_id, pixmap)
            state.parent = parent
            state.prev_action = action
            return state

        def detect_wall(origin: tuple, destin: tuple) -> bool:
            if limits(destin[0]+1, destin[1]+1):
                if destin[0] <= origin[0]:
                    aux_map_x = self.pixmap[destin[0]:origin[0]+1, origin[1]:destin[1]+1]
                elif origin[0] <= destin[0]:
                    aux_map_x = self.pixmap[origin[0]:destin[0]+1, origin[1]:destin[1]+1]

                if 205 in aux_map_x or 0 in aux_map_x:
                    return True

                if destin[1] < origin[1]:
                    aux_map_y = self.pixmap[destin[0]:origin[0]+1, destin[1]:origin[1]+1]
                elif origin[1] <= destin[1]:
                    aux_map_y = self.pixmap[origin[0]:destin[0]+1, origin[1]:destin[1]+1]

                if 205 in aux_map_y or 0 in aux_map_y:
                    return True

                return False

            return True

        x_state = self.node_id[0]
        y_state = self.node_id[1]

        west = self.node_id
        east = self.node_id
        north = self.node_id
        south = self.node_id

        if limits(x_state - self.cell_size[0], y_state):
            west = (x_state - self.cell_size[0], y_state)
        if limits(x_state + self.cell_size[0], y_state):
            east = (x_state + self.cell_size[0], y_state)
        if limits(x_state, y_state + self.cell_size[1]):
            south = (x_state, y_state + self.cell_size[1])
        if limits(x_state, y_state - self.cell_size[1]):
            north = (x_state, y_state - self.cell_size[1])

        if not detect_wall(self.node_id, north):
            successors.append(create_state(
                north, self.pixmap, self, "go_north"))

        if not detect_wall(self.node_id, south):
            successors.append(create_state(
                south, self.pixmap, self, "go_south"))

        if not detect_wall(self.node_id, west):
            successors.append(create_state(west, self.pixmap, self, "go_west"))

        if not detect_wall(self.node_id, east):
            successors.append(create_state(east, self.pixmap, self, "go_east"))

        return successors

    def __eq__(self, other):
        if isinstance(other, State):
            return self.node_id == other.node_id
        else:
            return False

    def __hash__(self):
        return hash(self.node_id)

    def __str__(self):
        return str(self.node_id)


def bf_search(s0: State, sg: State) -> State:
    open_queue = list()
    closed_queue = list()
    open_queue.append(s0)
    while len(open_queue) > 0:
        s = open_queue.pop(0)
        closed_queue.append(s)
        dist = (abs(s.node_id[0] - sg.node_id[0]) +
                abs(s.node_id[1] - sg.node_id[1]))
        if s == sg or dist < 8:
            break
        successors = s.expand()
        successors = list(set(successors) -
                          set(open_queue) - set(closed_queue))
        open_queue += successors
    return s


def get_sequence(sg: State) -> List[State]:
    aseq = list()
    s = sg
    while s.parent is not None:  # Is this state the initial state
        aseq.append((s.parent.node_id, s.prev_action, s.node_id))
        s = s.parent
    return aseq[::-1]  # Invert sequence order from sg->...->s0 to s0->...->sg


class SearchBFS:
    def __init__(self, pixmap: np.array) -> None:
        self.c_space_map = c_space_map(pixmap)

        rospy.Subscriber('/bfs', PoseArray, self.goal_poses_cb)

        self.nav_path_publisher = rospy.Publisher(
            '/nav_plan', Path, queue_size=1)

    def goal_poses_cb(self, pose_array: PoseArray) -> None:
        _, y_max_meters = pixel_to_meters(*self.c_space_map.shape)
        y_max_pixels = self.c_space_map.shape[1]

        cell_s = meters_to_pixel(
            pose_array.poses[0].position.x, y_max_meters - pose_array.poses[0].position.y)

        cell_g = meters_to_pixel(
            pose_array.poses[1].position.x, y_max_meters - pose_array.poses[1].position.y)

        s0 = State(cell_s, self.c_space_map)  # Initial state in graph
        sg = State(cell_g, self.c_space_map)  # Goal state in graph

        path = Path()

        sg = bf_search(s0, sg)

        for _, _, cell in get_sequence(sg):
            cell = pixel_to_meters(cell[0], y_max_pixels - cell[1])
            pose_stamped = PoseStamped()
            pose_stamped.pose.position = Point(
                float(cell[0]), float(cell[1]), 0)
            path.poses.append(pose_stamped)

        while self.nav_path_publisher.get_num_connections() < 1:
            rospy.Rate(10).sleep()
        self.nav_path_publisher.publish(path)


if __name__ == '__main__':
    rospack = rospkg.RosPack()

    filepath = path.join(rospack.get_path(
        'final_project'), 'maps', 'project_map_4x.pgm')

    map_img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)

    rospy.init_node('nav_path', anonymous=True)
    bfs_node = SearchBFS(map_img)
    rospy.spin()
