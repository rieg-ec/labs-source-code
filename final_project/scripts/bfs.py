class Graph():

    def __init__(self) -> None:
        pass

    def c_space_map(self):
        pass


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

        # Step 1: Convert from the cell's coordinate (x, y) to the cell's central pixel of map image (x_pix, y_pix).
        # Step 2: Determine whether exist a wall between the current cell's central pixel and the adjacent cell's
        #         central pixel, based on the pixel's values between centers.
        # Step 3: If there is not a wall, create the successor state and add it to the 'successors' list.
        #         Consider the following action definition: 'go_north', 'go_south', 'go_east', 'go_west'

        x_state = self.node_id[0] + 3
        y_state = self.node_id[1] + 3

        west = (x_state - self.cell_size[0], y_state)
        east = (x_state + self.cell_size[0], y_state)
        north = (x_state, y_state + self.cell_size[1])
        south = (x_state, y_state - self.cell_size[1])

        asign = True
        for y_step in range(1, north[1] - y_state + 1):
            if self.pixmap[x_state][y_state + y_step] == 100 or \
                    self.pixmap[x_state][y_state + y_step] == 19:
                asign = False
                break
        if asign:
            successors.append(State(west, self.pixmap))

        asign = True
        for y_step in range(1, abs(south[1] - x_state),):
            if self.pixmap[x_state][y_state - y_step] == 100 or \
                    self.pixmap[x_state][y_state - y_step] == 19:
                asign = False
                break
        if asign:
            successors.append(State(south, self.pixmap))

        asign = True
        for x_step in range(1, abs(west[0] - self.node_id[1]) + 1):
            if self.pixmap[x_state - x_step][y_state] == 100 or \
                    self.pixmap[x_state - x_step][y_state] == 19:
                asign = False
                break
        if asign:
            successors.append(State(west, self.pixmap))

        asign = True
        for x_step in range(1, east[0] - self.node_id[1] + 1):
            if self.pixmap[x_state + x_step][y_state] == 100 or \
                    self.pixmap[x_state + x_step][y_state] == 19:
                asign = False
                break

        if asign:
            successors.append(State(east, self.pixmap))

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
        # print(f'State: {s}')
        closed_queue.append(s)
        if s == sg:
            break
        # print(f'STATE BEFORE FAILING {s}')
        successors = s.expand()
        successors = list(set(successors) -
                          set(open_queue) - set(closed_queue))
        open_queue += successors
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

    import numpy as np
    import cv2

    # These three lines can be replaced by the map received from map_server node.
    map_img = cv2.imread('map.pgm', cv2.IMREAD_GRAYSCALE)
    vect_img2map = np.vectorize(img2map)
    ros_map = vect_img2map(map_img, 0.65, 0.196)

    cell_s = (1, 1)  # Initial cell
    cell_g = (3, 4)  # Goal cell
    s0 = State(cell_s, ros_map)  # Initial state in graph
    sg = State(cell_g, ros_map)  # Goal state in graph
    print('Going from %s to %s\n' % (s0, sg))

    print('Plan found (cell_a, action, cell_b):')
    sg = bf_search(s0, sg)  # Breadth-First Search algorithm execution
    for cell_a, action, cell_b in get_sequence(sg):
        print('%s, %s, %s' % (cell_a, action, cell_b))
