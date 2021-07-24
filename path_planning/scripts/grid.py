import numpy as np
import cv2
import sys

def c_space_map(map: np.array, robot_radius = 10) -> np.array:
    ROBOTRADIUS = robot_radius

    color_c_space = 205

    def limits(x, y, map_shape):
        return 0 < x < map_shape[0] and 0 < y < map_shape[1]

    new_map = map
    shape = map.shape
    for x in range(shape[0]):
        for y in range(shape[1]):
            if map[x][y] == 0:
                for x_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x + x_radio, y, shape) and map[x + x_radio][y] != 0:
                        new_map[x + x_radio][y] = color_c_space
                for y_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x, y + y_radio, shape) and map[x][ y + y_radio] != 0:
                        new_map[x][y + y_radio] = color_c_space
                for x_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x - x_radio, y, shape) and map[x - x_radio][y] != 0:
                        new_map[x - x_radio][y] = color_c_space
                for y_radio in range(1, ROBOTRADIUS + 1):
                    if limits(x, y - y_radio, shape) and map[x][y - y_radio] != 0:
                        new_map[x][y - y_radio] = color_c_space

    return new_map

class State(object):

    def __init__(self, node_id, pixmap, cell_size=(7, 7)) -> None:
        self.node_id = node_id      # Current cell coordinate (x, y)
        # Map image. For example, in ROS format (-1: unknown, 0: free, 100: busy)
        self.pixmap = pixmap
        self.cell_size = cell_size  # Cell size in pixels ( without walls )
        # Action to get this state. Initial state must have a None value.
        self.prev_action = None 
        self.parent = None          # Previous state

    # IMPLEMENT ME!
    def expand(self):
        successors = list()

        def limits(x: int, y: int) -> bool:
            shape = self.pixmap.shape
            return 0 < x < shape[0] - 7 and 0 < y < shape[1] - 7
        
        def create_state(node_id: tuple, pixmap: np.array, parent: State, action: str) -> State:
            state = State(node_id, pixmap)
            state.parent = parent
            state.prev_action = action
            return state

        def detect_wall(origin: tuple, destin: tuple) -> bool:
            if limits(destin[0]+1, destin[1]+1):
                if destin[0] <= origin[0]:
                    aux_map = self.pixmap[destin[0]:origin[0]+1, origin[1]:destin[1]+1]
                elif origin[0] <= destin[0]:
                    aux_map = self.pixmap[origin[0]:destin[0]+1, origin[1]:destin[1]+1]
                if destin[1] < origin[1]:
                    aux_map = self.pixmap[destin[0]:origin[0]+1, destin[1]:origin[1]+1]
                elif origin[1] <= destin[1]:
                    aux_map = self.pixmap[origin[0]:destin[0]+1, origin[1]:destin[1]+1]
                if 205 in aux_map or 0 in aux_map:
                    return True
                else:
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
            successors.append(create_state(north, self.pixmap, self, "go_north"))

        if not detect_wall(self.node_id, south):
            successors.append(create_state(south, self.pixmap, self, "go_south"))
        
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

#
# BFS Algorithm
#

def bf_search(s0, sg):
    open_queue = list()
    closed_queue = list()
    open_queue.append(s0)
    while len(open_queue) > 0:
        s = open_queue.pop(0)
        closed_queue.append(s)
        dist = (abs(s.node_id[0] - sg.node_id[0]) + abs(s.node_id[1] - sg.node_id[1]))
        if s == sg or dist < 5:
            break
        successors = s.expand()
        successors = list(set(successors) -
                          set(open_queue) - set(closed_queue))
        open_queue += successors
        sys.stdout.write("Download progress: %d%%   \r" % (len(closed_queue)/2800) )
        sys.stdout.flush()
    return s

#
# Build sequence from goal state
#

def get_sequence(sg):
    aseq = list()
    s = sg
    while None != s.parent:  # Is this state the initial state
        aseq.append((s.parent.node_id, s.prev_action, s.node_id))
        s = s.parent
    return aseq[::-1]  # Invert sequence order from sg->...->s0 to s0->...->sg


def img2map(pixvalue, occupied_thresh, free_thresh):
    p = (255 - pixvalue) / 255.0
    if p > occupied_thresh:
        return 100
    elif p < free_thresh:
        return 0
    else:
        return -1

if __name__ == '__main__':
    # These three lines can be replaced by the map received from map_server node.
    map_img = cv2.imread('../maps/project_map_4x.pgm', cv2.IMREAD_GRAYSCALE)
    vect_img2map = np.vectorize(img2map)
    ros_map = vect_img2map(map_img, 0.65, 0.196)

    CSpaceMap = np.ndarray.transpose( c_space_map(map_img) )

    cell_s = (20, 80)  # Initial cell
    cell_g = (111, 24)  # Goal cell
    
    s0 = State(cell_s, CSpaceMap)  # Initial state in graph
    sg = State(cell_g, CSpaceMap)  # Goal state in graph
    print('Going from %s to %s\n' % (s0, sg))

    cv2.circle(map_img, cell_s, 5, 100, thickness=-1)
    cv2.circle(map_img, cell_g, 5, 100, thickness=-1)

    sg = bf_search(s0, sg)  # Breadth-First Search algorithm execution
    print('Plan found (cell_a, action, cell_b):')
    for cell_a, action, cell_b in get_sequence(sg):
        print('%s, %s, %s' % (cell_a, action, cell_b))
        cv2.circle(map_img, cell_b, 2, 0, thickness=-1)

    cv2.imwrite("path.pgm", map_img)