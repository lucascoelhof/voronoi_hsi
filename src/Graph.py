import math
import numpy as np

import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid


from Node import Node


class Graph:
    def __init__(self, service_map_name, occ_grid_topic):
        self.nodes = None
        self.resolution = 0
        self.width = 0
        self.height = 0
        # self.occ_grid_sub = rospy.Subscriber(occ_grid_topic, OccupancyGrid, self.occ_grid_callback)

        service_map = rospy.ServiceProxy(service_map_name, GetMap)
        occ_g = service_map()
        self.occ_grid = np.mat(occ_g.map.data).reshape(occ_g.map.info.height, occ_g.map.info.width)
        self.build_graph(occ_g.map.info.height, occ_g.map.info.width, occ_g.map.info.resolution)

    def occ_grid_callback(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.occ_grid = np.mat(msg.data).reshape(self.height, self.width)

    def get_node(self, pose):
        """
        Gets a node based on the pose provided as an array [x,y]
        :type pose: []
        """
        xc = math.floor(pose[0] / self.resolution)
        yc = math.floor(pose[1] / self.resolution)
        return self.nodes[xc][yc]

    def build_graph(self, height, width, resolution):
        """
        Builds a graph based on the occupancy grid information
        :type height: int
        :type width: int
        :type resolution: float
        """
        self.height = height
        self.width = width
        self.resolution = resolution
        self.nodes = np.empty((self.height, self.width), dtype=object)

        for i in range(0, self.width):
            for j in range(0, self.height):
                node = Node()
                node.indexes = [i, j]
                node.set_pose(self.get_pose(i, j))
                if 0 <= self.occ_grid[i, j] <= 20:
                    node.valid = True
                else:
                    node.valid = False
                self.nodes[i][j] = node

        for i in range(0, self.width):
            for j in range(0, self.height):
                if self.nodes[i, j].valid:
                    min_i = max(0, i - 1)
                    max_i = min(self.width - 1, i + 1) + 1
                    min_j = max(0, j - 1)
                    max_j = min(self.height - 1, j + 1) + 1

                    neighbors = self.nodes[min_i:max_i, min_j:max_j].flatten()
                    for n in neighbors:
                        if n is not self.nodes[i, j] and n.valid:
                            self.nodes[i, j].neighbors.append(n)

    def get_pose(self, i, j):
        """
        Get the pose correspondent to the node at index i,j of the graph matrix
        :type i: int
        :type j: int
        """
        x = i * self.resolution + self.resolution/2
        y = j * self.resolution + self.resolution/2
        return [x, y]

    def clear_graph(self):

        for i in range(0, self.width):
            for j in range(0, self.height):
                self.nodes[i, j].clear()
