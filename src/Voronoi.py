import math
import numpy as np
import random
from threading import Semaphore
from Queue import PriorityQueue

import rospy
import time
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from voronoi_hsi.msg import *

import Util
from Node import Node
from Graph import Graph
from Robot import Robot
from Util import tic, toc
from ControlLaw import ControlLawVoronoi
from voronoi_hsi.msg import RobotGainArray, RobotGain


class Voronoi:
    def __init__(self, loop_time):
        self.robots = {}

        self.loop_time = loop_time

        self.topic_info = {}
        self.robot_control_info = {}
        self.adapting_weight_constant = 1

        self.density = None
        self.density_sub = None  # type: rospy.Publisher

        self.base_image = None
        self.tesselation_image = None
        self.tesselation_image_pub = None  # type: rospy.Publisher
        self.semaphore = Semaphore()

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

        self.obstacle_id_start = 10000
        self.obstacle_id = self.obstacle_id_start

        self.graph = Graph(self.topic_info["occupancy_grid_service"], self.topic_info["occupancy_grid_topic"])
        self.occ_grid_seq = 0

        self.occ_grid_subscriber = rospy.Subscriber(self.topic_info["occupancy_grid_topic"], OccupancyGrid, self.occ_grid_callback)
        self.robot_gain_subscriber = rospy.Subscriber(self.topic_info["robot_gains"], RobotGainArray, self.robot_gains_callback)
        self.grey_img = None
        self.img_width = 0
        self.img_height = 0
        self.robot_color = [50, 50, 50]

        self.init_density_dist()
        self.init_tesselation_image()

    def init_density_dist(self):
        if self.graph is None:
            raise ValueError("Graph is None or not initialized, can't initiate density distribuition")
        self.update_density_dist()
        self.density_sub = rospy.Subscriber(self.topic_info["gaussian_topic"], Gaussian, self.density_callback, queue_size=1)

    def init_tesselation_image(self):
        while self.graph.occ_grid is None:
            time.sleep(0.2)
        self.img_width = self.graph.width
        self.img_height = self.graph.height
        self.grey_img = np.mat(self.graph.occ_grid)
        self.set_image()
        self.clear_image()

    def create_obstacle(self, i, j):
        # type: (int, int) -> None
        node = self.graph.get_node_from_index(i, j)
        if node is None:
            rospy.logwarn("Node is none! [{0}, {1}]".format(str(i), str(j)))
        rospy.logwarn("Creating obstacle at " + str(node.pose[0]) + ":" + str(node.pose[1]))
        self.obstacle_id += 1
        id = self.obstacle_id
        color = [150, 150, 150]
        obst = Robot(id, 0.2, color, xd=0.5, yd=0.5)
        obst.set_pose(node.pose)
        obst.control.control_law = ControlLawVoronoi(kv=0.1)
        self.robots[id] = obst

    def occ_grid_callback(self, msg):
        # type: (OccupancyGrid) -> None
        if self.occ_grid_seq == 0:
            self.occ_grid_seq = 1
            return
        self.semaphore.acquire()
        rospy.loginfo("Received new occ_grid image")
        new_occ_grid = self.graph.build_occ_grid(msg)
        is_different = False
        if self.graph.occ_grid is not None:
            for i in range(msg.info.width):
                for j in range(msg.info.height):
                    if new_occ_grid[i, j] != self.graph.occ_grid[i, j]:
                        if new_occ_grid[i, j] > 50:
                            self.create_obstacle(i, j)
                            new_occ_grid[i, j] = 0
                        else:
                            is_different = True

        if is_different:
            self.graph.set_occ_grid(msg)
            self.img_width = msg.info.width
            self.img_height = msg.info.height
            self.grey_img = np.mat(self.graph.occ_grid)
            self.set_image()
        self.semaphore.release()

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

    def set_image(self):
        self.base_image = np.empty((self.img_width, self.img_height, 3), dtype=np.uint8)
        self.base_image[:, :, 0] = self.grey_img
        self.base_image[:, :, 1] = self.grey_img
        self.base_image[:, :, 2] = self.grey_img

    def clear_image(self):
        self.tesselation_image = np.copy(self.base_image)

    def update_density_dist(self):
        self.semaphore.acquire()
        self.density = np.ones((self.graph.width, self.graph.height))
        for i in range(self.graph.width):
            for j in range(self.graph.height):
                n = self.graph.nodes[i, j]
                if n is not None:
                    pose = self.graph.nodes[i, j].pose  # type: list
                    val = self.gaussian2d(self.gaussian, pose[0], pose[1])
                    self.density[i, j] = val
                else:
                    self.density[i, j] = 0
        #np.savetxt("/home/lady/density.txt", self.density, newline="\n")
        rospy.loginfo("Density updated with a: {0}; x: {1}; y: {2}".format(str(self.gaussian.a), str(self.gaussian.x_c), str(self.gaussian.y_c)))
        self.semaphore.release()

    @staticmethod
    def gaussian2d(gaussian, x, y):
        # type: (Gaussian, float, float) -> float
        x_part = math.pow(x - gaussian.x_c, 2) / (2 * math.pow(gaussian.sigma_x, 2))
        y_part = math.pow(y - gaussian.y_c, 2) / (2 * math.pow(gaussian.sigma_y, 2))
        return gaussian.a * math.exp(-(x_part + y_part)) + 0.1

    def density_callback(self, msg):
        # type: (Gaussian) -> None
        try:
            self.gaussian = msg
            self.update_density_dist()
        except Exception as e:
            rospy.logerr("Error while getting density info " + str(e))

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
        return pow(x, 2) - math.fabs(r)/r*pow(r, 2)

    def publish_voronoi(self):
        voro_tess = VoronoiTesselation()
        voro_tess.width = self.graph.width
        voro_tess.height = self.graph.height
        voro_tess.data = np.empty((voro_tess.width*voro_tess.height), dtype=int)
        for i in range(0, self.graph.width):
            for j in range(0, self.graph.height):
                voro_tess.data[i*voro_tess.width + j] = self.graph.nodes[i, j].robot_id
        #self.voronoi_publisher.publish(voro_tess)

    def tesselation_and_control_computation(self, list_robots=None):
        begin = rospy.Time.now()
        self.semaphore.acquire()
        if list_robots is None:
            list_robots = []

        for robot in self.robots.values():  # type: Robot

            pose = robot.get_pose_array()
            node = self.graph.get_node(pose)  # type: Node
            node.cost = 0  # np.linalg.norm(np.subtract(node.pose, robot.get_pose_array()))
            node.power_dist = node.cost - pow(robot.weight, 2)
            robot.control.control_law.clear_i()
            #robot.mass = self.get_density(node)*math.pow(self.graph.resolution, 2)
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

            for n in q.neighbors:  # type: Node
                _cost = q.cost + np.linalg.norm(np.subtract(q.pose, n.pose))
                _power_dist = self.power_dist(_cost, robot.weight)
                if _power_dist < n.power_dist:
                    n.cost = _cost
                    n.power_dist = _power_dist
                    if not n.is_neighbor(robot_node):
                        n.s = q.s
                    self.priority_queue.put((n.power_dist, n, robot.id))
                else:
                    if n.robot_id is not -1:
                        robot.neighbors[n.robot_id] = self.robots[n.robot_id]
                        self.robots[n.robot_id].neighbors[robot.id] = robot

        for robot in self.robots.values():  # type: Robot
            if robot.id in list_robots:
                # print("\n\nRobot " + str(robot.id))
                control_integral = robot.control.control_law.get_control_integral()
                # print("Control integral: " + str(control_integral))
                robot_node = self.graph.get_node(robot.get_pose_array())
                # self.mark_node(robot_node, self.robot_color)
                best_node = self.get_best_aligned_node(control_integral, robot_node)  # type: Node
                if best_node is None:
                    rospy.logerr("Best node is none robot_" + str(robot.id))
                    continue
                else:
                    robot.control.set_goal(best_node.pose)

        self.publish_tesselation_image()
        self.adapt_weights()
        self.clear()
        time_diff = (rospy.Time.now() - begin).to_sec()
        rospy.loginfo("Finished! iter=" + str(iterations) + ",h = " + str(h_func) + ", " + str(time_diff) + "s")
        self.semaphore.release()
        return h_func

    def robot_reached_goal(self, robot):
        if not robot.control.goal:
            return False
        goal = self.graph.get_node(robot.control.goal)
        node = self.graph.get_node(robot.pose)
        return goal == node

    def adapt_weights(self):
        w_del_robots = []
        for robot in self.robots.values():  # type: Robot
            Kp = robot.control.get_kp()
            Kdel = robot.get_kdel()
            diff_sum = 0
            if robot.id >= self.obstacle_id_start:
                rospy.logerr("Obstacle id= " + str(robot.id) + " weight: " + str(robot.weight))

            for robot_neigh in robot.neighbors.values():  # type: Robot
                if robot_neigh.id >= self.obstacle_id_start:
                    continue
                kdel_neigh = robot_neigh.get_kdel()
                kp_neigh = robot_neigh.control.get_kp()
                if robot is robot_neigh:
                    continue
                diff_sum += (robot.weight - self.k_func(Kp, Kdel)) - (robot_neigh.weight - self.k_func(kp_neigh, kdel_neigh))
            try:
                w_del = - self.adapting_weight_constant / robot.mass * diff_sum * self.loop_time
            except ZeroDivisionError:
                w_del = - 1000
                rospy.logerr("Zero division on adjusting weight of robot id=" + str(robot.id))
            w_del_robots.append(w_del)

        for robot, w_dot in zip(self.robots.values(), w_del_robots):  # type: (Robot, float)
            if robot.id >= self.obstacle_id_start:
                rospy.loginfo("wdel= " + str(w_dot))
                robot.weight = robot.weight - math.fabs(w_dot)
            else:
                robot.weight += w_dot
            robot.weight_publisher.publish(robot.weight)
            robot.weight_publisher_str.publish(str(robot.weight))
            node = self.graph.get_node(robot.get_pose_array())
            mass = self.get_density(node)*math.pow(self.graph.resolution, 2)
            if robot.weight < -100 or robot.mass <= mass*2 and robot.id >= self.obstacle_id_start:
                del self.robots[robot.id]
                if robot.id >= self.obstacle_id_start:
                    self.graph.occ_grid[node.indexes[0], node.indexes[1]] = 100
                    self.graph.build_graph()
                    self.init_tesselation_image()
                rospy.logerr("Removed robot {0} w={1}".format(str(robot.id), robot.weight))


    def k_func(self, kp, kdel):
        return np.linalg.norm(kp + kdel)*np.sign(np.linalg.norm(kp))

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
        self.clear_image()
        for robot in self.robots.values():  # type: Robot
            robot.clear()

    def mark_node(self, node, robot):
        # type: (Node, Robot) -> None
        coord = node.indexes
        self.tesselation_image[coord[0], coord[1]] = robot.color
        robot.mass += self.get_density(node)*math.pow(self.graph.resolution, 2)

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
        self.get_topic_info_param()
        self.get_robots_param()
        self.get_robot_control_info_param()

    def get_robots_param(self):
        try:
            robots = rospy.get_param("/voronoi/robots")
            if robots is not None and len(robots) > 0:
                for r in robots:
                    robot = Robot(r["id"], r["weight"], r["color"], xd=r["xd"], yd=r["yd"])
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

    def robot_gains_callback(self, msg):
        # type: (RobotGainArray) -> None

        # we don't want gains changing in the middle of a tesselation.
        self.semaphore.acquire()
        for gain in msg.robot_gain_list:  # type: RobotGain
            if gain.id in self.robots:
                rospy.logwarn("Weight of robot {0} has changed".format(gain.id))
                control_law = self.robots[gain.id].control.control_law  # type: ControlLawVoronoi
                control_law.set_control_parameters(kv=gain.kp, d=control_law.d, kw=control_law.kw)
        self.semaphore.release()
