import sys
import math
import numpy as np
from Queue import PriorityQueue

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError

from Node import Node
from Graph import Graph
from Robot import Robot
from ControlLaw import ControlLawVoronoi


class Voronoi:

    def __init__(self):
        self.robots = {}

        self.dir_info = {}
        self.topic_info = {}
        self.robot_control_info = {}

        self.density = None
        self.density_sub = None  # type: rospy.Publisher

        self.base_image = None  # type: Image
        self.tesselation_image = None  # type: Image
        self.tesselation_image_pub = None  # type: rospy.Publisher

        self.priority_queue = PriorityQueue()

        self.get_params()
        self.set_robot_publishers()
        self.set_robot_subscribers()
        self.set_output_publishers()

        self.graph = Graph(self.topic_info["occupancy_grid_service"], self.topic_info["occupancy_grid_topic"])

        self.init_density_dist()
        self.init_tesselation_image()

    def init_density_dist(self):
        if self.graph is None:
            raise ValueError("Graph is None or not initialized, can't initiate density distribuition")
        self.clear_density_dist()
        self.density_sub = rospy.Subscriber(self.topic_info["density_topic"], Float64MultiArray, self.density_callback)

    def init_tesselation_image(self):
        if self.graph is None:
            raise ValueError("Graph is None or not initialized, can't initiate density distribuition")
        self.clear_image()

    def clear_image(self):
        self.base_image = np.empty((self.graph.height, self.graph.width, 3), dtype=np.uint8)
        self.base_image[:, :, 0] = self.graph.occ_grid
        self.base_image[:, :, 1] = self.graph.occ_grid
        self.base_image[:, :, 2] = self.graph.occ_grid
        self.tesselation_image = Image()
        self.tess
        np.copy(self.base_image)

    def clear_density_dist(self):
        self.density = np.full((self.graph.height, self.graph.width), 1)

    def density_callback(self, msg):
        try:
            width = msg.layout.dim[0].size
            height = msg.layout.dim[1].size
            self.density = np.mat(msg.data).reshape(height, width)
        except:
            print("Error while getting density info")
            pass

    def set_robot_subscribers(self):
        for robot in self.robots.values():
            pose_topic = self.topic_info["robot_prefix"] + str(robot.id) + "/" + self.topic_info["pose_topic"]
            robot.set_pose_subscriber(pose_topic)

    def set_robot_publishers(self):
        for robot in self.robots.values():
            speed_topic = self.topic_info["robot_prefix"] + str(robot.id) + "/" + self.topic_info["speed_topic"]
            robot.set_speed_publisher(speed_topic)

    @staticmethod
    def power_dist(x, r):
        return pow(x, 2) - pow(r, 2)

    def tesselation_and_control_computation(self):

        del self.priority_queue
        self.priority_queue = PriorityQueue()

        for robot in self.robots.values():
            node = self.graph.get_node(robot.get_pose_array())  # type: Node
            node.cost = 0
            node.power_dist = - pow(robot.weight, 2)
            robot.control.control_law.clear_i()
            self.priority_queue.put((node.power_dist, node, robot.id))
            for q in node.neighbors:
                if q is not node:
                    q.s = q

        h_func = 0

        while not self.priority_queue.empty():
            elem = self.priority_queue.get()
            q = elem[1]  # type: Node
            if q.power_dist == float('inf'):
                break
            if q.robot_id is not -1:
                continue

            q.robot_id = elem[2]
            robot = self.robots[elem[2]]  # type: Robot
            robot_node = self.graph.get_node(robot.get_pose_array())  # type: Node

            h_func = h_func + (pow(q.power_dist, 2) + pow(robot.weight, 2)) * self.density[q.indexes[0], q.indexes[1]] * pow(self.graph.resolution, 2)
            self.mark_node(robot_node, robot.color)

            if q is not robot_node:
                i_cl = self.density[q.indexes[0], q.indexes[1]] * q.cost * np.subtract(q.s.pose, robot_node.pose)
                robot.control.control_law.add_control_law(i_cl)

            for n in q.neighbors:
                _cost = q.cost + np.linalg.norm(np.subtract(q.pose, n.pose))
                _power_dist = self.power_dist(_cost, robot.weight)
                if _power_dist < n.power_dist and n.robot_id == -1:
                    n.cost = _cost
                    n.power_dist = _power_dist
                    if not n.is_neighbor(robot_node):
                        n.s = q.s
                    self.priority_queue.put((n.power_dist, n, robot.id))

        for robot in self.robots.values():  # type: Robot
            print("\n\nRobot " + str(robot.id))
            control_integral = robot.control.control_law.get_control_integral()
            print("Control integral: " + str(control_integral))
            robot_node = self.graph.get_node(robot.get_pose_array())
            best_node = self.get_best_aligned_node(control_integral, robot_node)  # type: Node
            if best_node is None:
                print("Best node is none")
                continue
            else:
                print("Goal: " + str(best_node.pose))
                robot.control.set_goal(best_node.pose)

        #self.publish_tesselation_image()
        self.graph.clear_graph()
        return h_func

    def get_best_aligned_node(self, i_func, robot_node):
        # type: (list, Node) -> Node
        max_dpi = 0
        best_node = None
        for n in robot_node.neighbors:
            dpi = math.fabs(np.dot(i_func, (np.subtract(robot_node.pose, n.pose))))
            if dpi > max_dpi:
                max_dpi = dpi
                best_node = n
        return best_node

    def mark_node(self, node, color):
        coord = node.indexes
        self.tesselation_image[coord[0], coord[1]] = color

    def publish_tesselation_image(self):
        if self.tesselation_image_pub is None:
            raise ValueError("Tesselation Image publisher not initialized")
        self.tesselation_image_pub.publish(self.tesselation_image)

    def set_output_publishers(self):
        try:
            if self.topic_info["tesselation_topic"] is None:
                raise KeyError("tesselation_topic parameter not found")
            else:
                topic = self.topic_info["tesselation_topic"]
                self.tesselation_image_pub = rospy.Publisher(topic, Image, queue_size=1)
        except KeyError as e:
            print(str(e))
        except Exception as e:
            print(str(e))



    def get_params(self):
        self.get_dir_info_param()
        self.get_topic_info_param()
        self.get_robots_param()
        self.get_robot_control_info_param()

    def get_robots_param(self):
        try:
            robots = rospy.get_param("/voronoi/robots")
            if robots is not None and len(robots) > 0:
                for r in robots:
                    robot = Robot(r["id"], r["weight"], r["color"])
                    self.robots[robot.id] = robot
        except KeyError:
            print("Parameter robots not found. Exiting.")
            sys.exit(1)
        except:
            print("A non recognized exception raised while getting robots parameter. Exiting")
            sys.exit(1)

    def get_topic_info_param(self):
        try:
            self.topic_info = rospy.get_param("/voronoi/topic_info")
        except KeyError:
            print("Parameter topic_info not found. Exiting.")
            sys.exit(1)
        except:
            print("A non recognized exception raised while getting topic_info parameter. Exiting")
            sys.exit(1)

    def get_dir_info_param(self):
        try:
            self.dir_info = rospy.get_param("/voronoi/dir_info")
        except KeyError:
            print("Parameter dir_info not found. Exiting.")
            sys.exit(1)
        except:
            print("A non recognized exception raised while getting dir_info parameter. Exiting")
            sys.exit(1)

    def get_robot_control_info_param(self):
        try:
            self.robot_control_info = rospy.get_param("/voronoi/robot_control_info")
            for robot in self.robots.values():

                control_law = ControlLawVoronoi(self.robot_control_info["d"], self.robot_control_info["kv"],
                                                self.robot_control_info["kw"])
                robot.control.set_control_law(control_law)
        except KeyError:
            print("Parameter robot_control_info not found. Exiting.")
            sys.exit(1)
        except Exception as e:
            print("A non recognized exception raised while getting robot_control_info parameter. Exiting. " + str(e))
            sys.exit(1)

    def image_builder(self):
        raise NotImplementedError("image_builder not implemented yet")


