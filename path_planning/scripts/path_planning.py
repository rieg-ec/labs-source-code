import numpy as np









class PathPlanning():


    def __init__(self, map: np.array) -> None:
        self.visited = set()
        pass

    def bfs(self, start_state, goal_state):
        Queue = []
        Queue.append(start_state)
        while len(Queue) != 0:
            state = Queue.pop()
            if state == goal_state:
                return state
            for succ in state.expand():
                if succ not in self.visited:
                    self.visited.add(succ)
                    Queue.append( succ )