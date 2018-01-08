import sys
import math
import numpy as np
from Queue import PriorityQueue

import rospy
from nav_msgs.srv import GetMap
from sensor_msgs.msg import Image
from voronoi_hsi.msg import Matrix2D, Gaussian

import Util
from Node import Node
from Graph import Graph
from Robot import Robot
from Util import tic, toc
from ControlLaw import ControlLawVoronoi
import matplotlib.pyplot as plt


class Voronoi:
    def __init__(self, loop_time):
        self.robots = {}

        self.loop_time = loop_time

        self.dir_info = {}
        self.topic_info = {}
        self.robot_control_info = {}
        self.adapting_weight_constant = 1

        self.density = None
        self.density_sub = None  # type: rospy.Publisher

        self.base_image = None
        self.tesselation_image = None
        self.tesselation_image_pub = None  # type: rospy.Publisher

        self.priority_queue = PriorityQueue()

        self.get_params()
        self.set_robot_publishers()
        self.set_robot_subscribers()
        self.set_output_publishers()

        self.gaussian = Gaussian()
        self.gaussian.a = 1
        self.gaussian.x_c = 10
        self.gaussian.y_c = 10
        self.gaussian.sigma_x = 999999999999
        self.gaussian.sigma_y = 999999999999


        self.occ_grid = None
        self.grey_img = None
        self.img_width = 0
        self.img_height = 0
        self.robot_color = [50, 50, 50]

        self.graph = Graph(self.topic_info["occupancy_grid_service"], self.topic_info["occupancy_grid_topic"])

        self.init_density_dist()
        self.init_tesselation_image()

    def init_density_dist(self):
        if self.graph is None:
            raise ValueError("Graph is None or not initialized, can't initiate density distribuition")
        self.update_density_dist()
        self.density_sub = rospy.Subscriber(self.topic_info["gaussian_topic"], Gaussian, self.density_callback)

    def init_tesselation_image(self):
        occ_grid_service = rospy.ServiceProxy(self.topic_info["occupancy_grid_service"], GetMap)
        self.occ_grid = occ_grid_service().map
        self.set_image()

    def set_image(self):
        self.img_width = self.occ_grid.info.width
        self.img_height = self.occ_grid.info.height
        self.grey_img = np.mat(self.occ_grid_to_img(self.occ_grid.data)).reshape(self.img_height, self.img_width).transpose()
        self.clear_image()

    @staticmethod
    def occ_grid_to_img(occ_grid):
        # type: (np.array) -> np.array
        image = np.copy(occ_grid)
        for i in image:
            if 0 <= i <= 100:
                i = 255 - i*255
            else:
                i = 170
        return image

    def clear_image(self):
        self.base_image = np.empty((self.img_width, self.img_height, 3), dtype=np.uint8)
        self.base_image[:, :, 0] = self.grey_img
        self.base_image[:, :, 1] = self.grey_img
        self.base_image[:, :, 2] = self.grey_img
        self.tesselation_image = np.copy(self.base_image)

    def update_density_dist(self):
        self.density = np.ones((self.graph.width, self.graph.height))
        for i in range(self.graph.width):
            for j in range(self.graph.height):
                pose = self.graph.nodes[i][j].pose  # type: list
                val = self.gaussian2d(self.gaussian, pose[0], pose[1])
                self.density[i, j] = val
        # x_axis = np.linspace(0, self.graph.width*self.graph.resolution, self.graph.width)
        # y_axis = np.linspace(0, self.graph.height*self.graph.resolution, self.graph.height)

        # plt.figure(1)
        # plt.imshow(self.density, extent=[0, 20, 0, 20])
        # plt.grid(True)
        # plt.interactive(False)
        # plt.show()

    @staticmethod
    def gaussian2d(gaussian, x, y):
        # type: (Gaussian, float, float) -> float
        x_part = math.pow(x - gaussian.x_c, 2) / (2 * math.pow(gaussian.sigma_x, 2))
        y_part = math.pow(y - gaussian.y_c, 2) / (2 * math.pow(gaussian.sigma_y, 2))
        return gaussian.a * math.exp(-(x_part + y_part)) + 1

    def density_callback(self, msg):
        # type: (Gaussian) -> None
        try:
            self.gaussian = msg
            self.update_density_dist()
        except:
            rospy.logerr("Error while getting density info")
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
        # type: (float, float) -> float
        return pow(x, 2) - pow(r, 2)

    def tesselation_and_control_computation(self):
        tic()

        for robot in self.robots.values():  # type: Robot
            node = self.graph.get_node(robot.get_pose_array())  # type: Node
            node.cost = 0  # np.linalg.norm(np.subtract(node.pose, robot.get_pose_array()))
            node.power_dist = node.cost - pow(robot.weight, 2)
            robot.control.control_law.clear_i()
            self.priority_queue.put((node.power_dist, node, robot.id))

            for q in node.neighbors:  # type: Node
                if q is not node and not bool(set(q.obstacle_neighbors) & set(node.obstacle_neighbors)):
                    q.s = q

        h_func = 0
        iterations = 0

        while not self.priority_queue.empty():
            iterations = iterations + 1
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
            self.mark_node(q, robot)

            if q.s is not None:
                i_cl = self.get_density(q) * q.cost * np.subtract(q.s.pose, robot.get_pose_array())
                robot.control.control_law.add_control_law(i_cl)

            for n in q.neighbors:
                _cost = q.cost + np.linalg.norm(np.subtract(q.pose, n.pose))
                _power_dist = self.power_dist(_cost, robot.weight)
                if _power_dist < n.power_dist:
                    n.cost = _cost
                    n.power_dist = _power_dist
                    if not n.is_neighbor(robot_node):
                        n.s = q.s
                    self.priority_queue.put((n.power_dist, n, robot.id))

        for robot in self.robots.values():  # type: Robot
            #print("\n\nRobot " + str(robot.id))
            control_integral = robot.control.control_law.get_control_integral()
            #print("Control integral: " + str(control_integral))
            robot_node = self.graph.get_node(robot.get_pose_array())
            # self.mark_node(robot_node, self.robot_color)
            best_node = self.get_best_aligned_node(control_integral, robot_node)  # type: Node
            if best_node is None:
                print("Best node is none robot_" + str(robot.id))
                continue
            else:
                print("Goal: " + str(best_node.pose))
                robot.control.set_goal(best_node.pose)


        self.publish_tesselation_image()
        self.adapt_weights()
        self.clear()
        rospy.loginfo("Tesselation finished with iter=" + str(iterations) + " and " + str(toc()) + "s")
        return h_func

    def adapt_weights(self):
        w_del_robots = []
        for robot in self.robots.values():  # type: Robot
            Kp = robot.control.get_kp()
            Kdel = robot.get_kdel()
            diff_sum = 0
            for robot_neigh in self.robots.values():  # type: Robot
                kdel_neigh = robot_neigh.get_kdel()
                kp_neigh = robot_neigh.control.get_kp()
                if robot is robot_neigh:
                    continue
                diff_sum += (robot.weight - self.k_func(Kp, Kdel)) - (robot_neigh.weight - self.k_func(kp_neigh, kdel_neigh))
            w_del = - self.adapting_weight_constant / robot.mass * diff_sum * self.loop_time
            w_del_robots.append(w_del)

        for robot, w_dot in zip(self.robots.values(), w_del_robots):
            robot.weight += w_dot
            robot.weight_publisher.publish(robot.weight)


    def k_func(self, kp, kdel):
        return np.linalg.norm(kp + kdel)/np.linalg.norm(kp)

    def get_density(self, node):
        # type: (Node) -> double
        return self.density[node.indexes[0], node.indexes[1]]

    def get_best_aligned_node(self, i_func, robot_node):
        # type: (list, Node) -> Node
        max_dpi = 0
        best_node = None
        for n in robot_node.neighbors:  # type: Node
            if n != n.s:
                continue
            dpi = np.dot(i_func, (np.subtract(n.pose, robot_node.pose)))
            if dpi > max_dpi:
                max_dpi = dpi
                best_node = n
        return best_node

    def clear(self):
        self.graph.clear_graph()
        self.tesselation_image = np.copy(self.base_image)
        for robot in self.robots.values():  # type: Robot
            robot.clear()

    def mark_node(self, node, robot):
        # type: (Node, Robot) -> None
        coord = node.indexes
        self.tesselation_image[coord[0], coord[1]] = robot.color
        robot.mass += self.get_density(node)

    def publish_tesselation_image(self):
        if self.tesselation_image_pub is None:
            raise ValueError("Tesselation Image publisher not initialized")
        image_message = Util.numpy_matrix_to_rosimg(np.rot90(self.tesselation_image), "rgb8")
        self.tesselation_image_pub.publish(image_message)

    def set_output_publishers(self):
        try:
            if self.topic_info["tesselation_topic"] is None:
                raise KeyError("tesselation_topic parameter not found")
            else:
                topic = self.topic_info["tesselation_topic"]
                self.tesselation_image_pub = rospy.Publisher(topic, Image, queue_size=1)
        except KeyError as e:
            rospy.logerr("Error while setting tesselation publishers: " + str(e))
        except Exception as e:
            rospy.logerr("Error while setting tesselation publishers: " + str(e))

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
                    print("robot" + str(robot.id) + " w:" + str(robot.weight))
                    self.robots[robot.id] = robot
        except KeyError:
            rospy.logfatal("Parameter robots not found. Exiting.")
            sys.exit(1)
        except:
            rospy.logfatal("A non recognized exception raised while getting robots parameter. Exiting")
            sys.exit(1)

    def get_topic_info_param(self):
        try:
            self.topic_info = rospy.get_param("/voronoi/topic_info")
        except KeyError:
            rospy.logfatal("Parameter topic_info not found. Exiting.")
            sys.exit(1)
        except:
            rospy.logfatal("A non recognized exception raised while getting topic_info parameter. Exiting")
            sys.exit(1)

    def get_dir_info_param(self):
        try:
            self.dir_info = rospy.get_param("/voronoi/dir_info")
        except KeyError:
            rospy.logfatal("Parameter dir_info not found. Exiting.")
            sys.exit(1)
        except:
            rospy.logfatal("A non recognized exception raised while getting dir_info parameter. Exiting")
            sys.exit(1)

    def get_robot_control_info_param(self):
        try:
            self.robot_control_info = rospy.get_param("/voronoi/robot_control_info")
            self.adapting_weight_constant = self.robot_control_info["kd"]
            for robot in self.robots.values():

                control_law = ControlLawVoronoi(self.robot_control_info["d"], self.robot_control_info["kv"],
                                                self.robot_control_info["kw"])
                robot.control.set_control_law(control_law)
        except KeyError:
            rospy.logfatal("Parameter robot_control_info not found. Exiting.")
            sys.exit(1)
        except Exception as e:
            rospy.logfatal("A non recognized exception raised while getting robot_control_info parameter. Exiting. " + str(e))
            sys.exit(1)

    def image_builder(self):
        raise NotImplementedError("image_builder not implemented yet")


