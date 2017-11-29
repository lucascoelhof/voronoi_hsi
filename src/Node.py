
class Node:

    def __init__(self):
        self.pose = [0, 0]
        self.indexes = []  # indexes on graph matrix
        self.cost = float('inf')  # cost for modified dijkstra algorithm
        self.power_dist = float('inf')  # power dist used by tesselation and control computation on priority queue
        self.robot_id = -1  # id of the robot that owns this node, used by tesselation and control computation algorithm
        self.neighbors = []  # neighbor nodes on graph
        self.valid = False  # true if the node has no obstacle, this information comes from occupancy grid
        self.s = None  # type: Node

    def clear(self):
        self.cost = float('inf')
        self.power_dist = float('inf')
        self.robot_id = -1

    def set_pose(self, pose):
        # type: (list) -> None
        self.pose = pose

    def is_neighbor(self, n):
        if self.neighbors is None:
            raise ValueError("Neighbor not initialized for node " + str(self))
        else:
            return n in self.neighbors
