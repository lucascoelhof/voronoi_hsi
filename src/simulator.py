import math
import threading

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist
from matplotlib import patches
from matplotlib.collections import PatchCollection
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import GetMap
from voronoi_hsi.msg import VoronoiTesselation
from voronoi_hsi.srv import *

import Util
import simulator_util


def almost_equal(n, m, diff=0.005):
    return abs(n-m) <= diff


class RobotSimulator(simulator_util.DraggablePoint):

    def __init__(self, fig_handler, pose, color, id_robot="0"):
        # type: (plt.Figure, Pose, list, str) -> None

        super(RobotSimulator, self).__init__(fig_handler, x=pose.position.x, y=pose.position.y, color=color)
        self.fig_handler = fig_handler
        self.pose = pose
        self.speed = Twist()
        self.color = color
        self.id = id_robot
        self.speed_callback = rospy.Subscriber("robot_" + str(self.id) + "/cmd_vel", Twist, self.robot_vel_callback, queue_size=1)
        self.pose_publisher = rospy.Publisher("robot_" + str(self.id) + "/base_pose_ground_truth", Odometry, queue_size=10)

    def robot_vel_callback(self, msg):
        # type: (Twist) -> None
        self.speed = msg

    def pose_publisher(self):
        self.pose_publisher.publish(self.pose)

    def set_pose(self, pose):
        if isinstance(pose, Pose):
            self.pose = pose
        elif isinstance(pose, list) and len(pose) is 2:
            self.pose = Simulator.conf_to_pose([pose, Util.quaternion_get_yaw(self.pose.orientation)])
        else:
            raise ValueError("Type should be either a list of geometry_msgs/Pose")
        self.set_point_pose(self.pose.position.x, self.pose.position.y)

    def remove(self):
        super(RobotSimulator, self).remove()
        self.speed_callback.unregister()
        self.pose_publisher.unregister()

    def update_pose_diff(self, occ_grid):
        # type: (OccGrid) -> None
        w = Util.quaternion_get_yaw(self.pose.orientation)

        w_dot = self.speed.angular.z * Simulator.physics_time
        x_dot = self.speed.linear.x * math.cos(Util.quaternion_get_yaw(self.pose.orientation))
        y_dot = self.speed.linear.x * math.sin(Util.quaternion_get_yaw(self.pose.orientation))
        new_pose = Pose()

        w = w + w_dot
        new_pose.position.x = self.pose.position.x + x_dot
        new_pose.position.y = self.pose.position.y + y_dot
        new_pose.orientation = Util.get_quaternion_fom_euler([0, 0, w])
        if occ_grid.is_free(new_pose):
            self.pose = new_pose
            if not almost_equal(x_dot, 0) or not almost_equal(y_dot, 0):
                self.set_point_pose(new_pose.position.x, new_pose.position.y)

    def publish_pose(self):
        odom = Odometry()
        odom.pose.pose.position = self.pose.position
        odom.pose.pose.orientation = self.pose.orientation
        self.pose_publisher.publish(odom)


class OccGrid(object):

    should_update = False

    def __init__(self, service_name, figure):
        self.width = 0
        self.height = 0
        self.fig = figure
        self.resolution = 0
        self.occ_grid = None  # type: np.matrix
        self.origin = Pose()
        self.end = Pose()
        self.service_name = service_name
        self.robot_pose_service = rospy.Service("occ_grid_update", SetOccGrid, self.set_occ_grid_service)
        self.should_update = False
        self.axes = None
        self.patches = None

    def get_occ_grid(self):
        occ_grid_service = rospy.ServiceProxy(self.service_name, GetMap)
        occ_grid = occ_grid_service().map
        self.set_occ_grid(occ_grid)

    def set_occ_grid(self, occ_grid):
        # type: (OccupancyGrid) -> None
        print("got occ grid")
        self.occ_grid = occ_grid
        self.width = self.occ_grid.info.width
        self.height = self.occ_grid.info.height
        self.resolution = self.occ_grid.info.resolution
        self.origin = self.occ_grid.info.origin
        self.end.position.x = self.width*self.resolution + self.origin.position.x
        self.end.position.y = self.height * self.resolution + self.origin.position.y
        self.occ_grid = np.mat(self.occ_grid.data).reshape(self.height, self.width).transpose()
        print("Should update")
        self.draw_rectangles()

    def set_occ_grid_service(self, req):
        # type: (SetOccGridRequest) -> None
        self.set_occ_grid(req.map)

    def occ_grid_callback(self, msg):
        self.set_occ_grid(msg)

    def is_free(self, pose, radius=0.2):
        sub_pose = Util.subtract_pose(pose, self.origin)
        x = int(math.floor(sub_pose.position.x/self.resolution))
        y = int(math.floor(sub_pose.position.y/self.resolution))
        if 0 <= x < self.width and 0 <= y < self.height:
            if 0 <= self.occ_grid[x, y] <= 20:
                return True
        return False

    def get_extent(self):
        return [self.origin.position.x, self.end.position.x, self.origin.position.y, self.end.position.y]

    def occ_grid_to_img(self):
        image = np.copy(self.occ_grid)
        for i in range(self.width):
            for j in range(self.height):
                elem = self.occ_grid[i, j]
                if elem == -1:
                    image[i, j] = 173
                else:
                    image[i, j] = int((1 - elem/100.0)*255)
        return np.rot90(image)

    def draw_rectangles(self, fig):
        image = self.occ_grid_to_img()
        extent = self.get_extent()
        plt.imshow(image, extent=extent, zorder=0, interpolation='nearest', cmap="gray")


class Simulator(object):
    physics_time = 0.1

    def __init__(self):
        self.printing_voronoi = False
        self.robots = {}  # type: dict
        self.physics_time = 0.05
        rospy.init_node('simulator')
        self.vis_time = 0.05

        self.robot_pose_service = rospy.Service("set_robot_pose", SetRobotPose, self.robot_service)
        self.occ_grid_topic = ""
        self.tesselation_topic = ""
        self.robot_param = ""
        self.occ_grid_subscriber = None  # type: rospy.Subscriber
        self.tesselation_subscriber = None  # type: rospy.Subscriber

        self.voronoi_collection = None
        self.voronoi_axes = None
        self.voronoi_should_draw = False

        self.obstacle_collection = None
        self.obstacle_axes = None

        self.fig = plt.figure(1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.axis([0, 20, 0, 20])
        self.occ_grid = OccGrid("static_map", self.fig)
        self.occ_grid.get_occ_grid()
        self.fig.canvas.draw()

        self.read_simulator_params()
        self.read_robot_parameters()

    def start(self):
        self.physics_thread()
        self.visual_thread()
        plt.show()

    def read_simulator_params(self):
        try:
            sim_p = rospy.search_param("simulator")
            sim_params = rospy.get_param(sim_p)
            self.occ_grid_topic = sim_params["occupancy_grid_topic"]
            self.occ_grid_subscriber = rospy.Subscriber(self.occ_grid_topic, OccupancyGrid, self.occ_grid.set_occ_grid, queue_size=1)
            self.tesselation_topic = sim_params["tesselation_topic"]
            self.tesselation_subscriber = rospy.Subscriber(self.tesselation_topic, VoronoiTesselation, self.voronoi_callback, queue_size=1)
            self.robot_param = sim_params["robots_param"]
        except KeyError:
            rospy.logfatal("Parameter robots not found. Exiting.")
            sys.exit(1)
        except:
            rospy.logfatal("A non recognized exception raised while getting robots parameter. Exiting")
            sys.exit(1)

    def read_robot_parameters(self):
        try:
            robots = rospy.get_param(self.robot_param)
            if robots is not None and len(robots) > 0:
                for r in robots:  # type: dict
                    self.create_robot(r["id"], self.conf_to_pose(r["pose"]), r["color"])
        except KeyError:
            rospy.logfatal("Parameter robots not found. Exiting.")
            sys.exit(1)
        except Exception as e:
            rospy.logfatal("A non recognized exception raised while getting robots parameter. Exiting\n" + str(e))
            sys.exit(1)

    def voronoi_callback(self, msg):
        if self.printing_voronoi:
            print("Skipping printing voronoi, not done yet.")
            return
        else:
            self.printing_voronoi = True
        Util.tic()
        # type: (VoronoiTesselation) -> None
        height = msg.height
        width = msg.width
        matrix = np.reshape(msg.data, (width, height))

        image = np.empty((width, height, 4))
        for i in range(width):
            for j in range(height):
                elem = matrix[i, j]
                if elem != -1:
                    color = self.robots[elem].color
                    image[i, j] = (np.asarray(color + [0])/255.0)
                    image[i, j, 3] = 1
                else:
                    image[i, j] = np.asarray([0, 0, 0, 0])
        extent = self.occ_grid.get_extent()

        plt.imshow(np.rot90(image), extent=extent, zorder=10, interpolation="nearest")

    @staticmethod
    def conf_to_pose(pose_conf):
        pose = Pose()
        pose.position.x = pose_conf[0]
        pose.position.y = pose_conf[1]
        pose.orientation = Util.get_quaternion_fom_euler([0, 0, pose_conf[2]])
        return pose

    def create_robot(self, id_r, pose, color=None):
        if color is None:
            color = [20, 20, 20]
        if id_r in self.robots:
            robot = self.robots[id_r]
        else:
            if not isinstance(pose, Pose):
                raise ValueError("pose is not geometry_msgs/Pose type.")
            robot = RobotSimulator(self.fig, pose, color, id_r)
            robot.color = [20, 20, 20]

        if len(color) == 3:
            robot.color = color

        self.robots[robot.id] = robot

    def remove_robot(self, id):
        # type: (str) -> None
        if id in self.robots:  # type: RobotSimulator
            robot = self.robots[id]
            robot.remove()
            self.robots.pop(id, None)

    def plot_image(self, image, extent):
        return plt.imshow(image, extent=extent, interpolation="nearest")

    def plot_occ_grid(self):
        occ_grid_img = np.zeros((self.occ_grid.width, self.occ_grid.height, 4), dtype=float)
        occ_grid_img[:,:,3] = self.occ_grid.occ_grid

    def robot_service(self, req):
        # type: (SetRobotPoseRequest) -> object
        try:
            self.create_robot(req.id, req.pose, req.color)
        except ValueError as e:
            rospy.logerr(e.message)
        return None

    def physics_thread(self):
        for robot in self.robots.values():  # type: RobotSimulator
            robot.update_pose_diff(self.occ_grid)
            robot.publish_pose()
        threading.Timer(self.physics_time, self.physics_thread).start()

    def visual_thread(self):
        should_draw = False
        if self.occ_grid.should_update:
            should_draw = True
            OccGrid.should_update = False
        if self.voronoi_should_draw:
            self.voronoi_should_draw = False
            should_draw = True
        # for robot in self.robots.values():  # type: RobotSimulator
        #     if robot.should_draw:
        #         should_draw = True
        #     robot.should_draw = False
        if should_draw:
            self.fig.canvas.draw()


def main():
    sim = Simulator()
    sim.start()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

