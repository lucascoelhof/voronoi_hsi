import sys
import math
import numpy as np
import threading
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import GetMap
from voronoi_hsi.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


import Util
import simulator_util


class RobotSimulator(simulator_util.DraggablePoint):

    def __init__(self, fig_handler, pose, color, id_robot="0"):
        # type: (plt.Figure, Pose, list, str) -> None

        super(RobotSimulator, self).__init__(fig_handler, x=pose.position.x, y=pose.position.y, color=color)
        self.fig_handler = fig_handler
        self.pose = Pose()
        self.speed = Twist()
        self.color = []
        self.id = id_robot
        self.speed_callback = rospy.Subscriber("robot_" + str(self.id) + "/cmd_vel", Twist, queue_size=1)
        self.pose_publisher = rospy.Publisher("robot_" + str(self.id) + "/pose", Odometry, queue_size=10)

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


class OccGrid(object):

    def __init__(self, service_name):
        self.width = 0
        self.height = 0
        self.resolution = 0
        self.occ_grid = None  # type: np.matrix
        self.origin = Pose()
        self.end = Pose()
        self.service_name = service_name
        self.robot_pose_service = rospy.Service("occ_grid_update", SetOccGrid, self.set_occ_grid_service)
        self.should_update = False

    def get_occ_grid(self):
        occ_grid_service = rospy.ServiceProxy(self.service_name, GetMap)
        occ_grid = occ_grid_service().map
        self.set_occ_grid(occ_grid)
        self.should_update = True

    def set_occ_grid(self, occ_grid):
        # type: (OccupancyGrid) -> None
        self.width = self.occ_grid.info.width
        self.height = self.occ_grid.info.height
        self.resolution = self.occ_grid.info.resolution
        self.end.position.x = self.width*self.resolution + self.origin.x
        self.end.position.y = self.height * self.resolution + self.origin.y
        if self.occ_grid.info.origin:
            self.origin = self.occ_grid.info.origin
        self.occ_grid = np.mat(self.convert_occ_grid(occ_grid.data)).reshape(self.height, self.width).transpose()

    @staticmethod
    def convert_occ_grid(occ_grid):
        for elem in np.nditer(occ_grid, op_flags=['readwrite']):
            if 0 <= elem <= 100:
                elem = float(elem/100)
            else:
                elem = 0.7
        return occ_grid

    def set_occ_grid_service(self, req):
        # type: (SetOccGridRequest) -> None
        self.set_occ_grid(req.map)

    def is_free(self, pose):
        sub_pose = Util.subtract_pose(pose, self.origin)
        x = math.floor(sub_pose/self.width)
        y = math.floor(sub_pose/self.height)
        if 0 <= self.occ_grid[x, y] <= 20:
            return True
        return False

    def get_extent(self):
        return [self.origin.position.x, self.end.position.x, self.origin.position.y, self.end.position.y]


class Simulator(object):

    def __init__(self):
        self.robots = {}  # type: dict[RobotSimulator]
        self.physics_time = 0.1
        self.vis_time = 0.2
        self.occ_grid = OccGrid("static_map")
        # self.occ_grid.get_occ_grid()
        self.physics_thread()
        self.visual_thread()
        self.robot_pose_service = rospy.Service("set_robot_pose", SetRobotPose, self.robot_service)
        self.occ_grid_topic = ""
        self.tesselation_topic = ""

        self.occ_grid_subscriber = None  # type: rospy.Subscriber
        self.tesselation_subscriber = None  # type: rospy.Subscriber

        self.fig = plt.figure(1)
        self.bridge = CvBridge()
        self.images_plot = {}

    def start(self):
        plt.show()

    def read_parameters(self):
        try:
            robots_p = rospy.search_param("robots")
            robots = rospy.get_param(robots_p)
            if robots is not None and len(robots) > 0:
                for r in robots:  # type: dict
                    self.create_robot(r["id"], self.conf_to_pose(r["pose"]), r["color"])
        except KeyError:
            rospy.logfatal("Parameter robots not found. Exiting.")
            sys.exit(1)
        except:
            rospy.logfatal("A non recognized exception raised while getting robots parameter. Exiting")
            sys.exit(1)
        try:
            sim_p = rospy.search_param("simulator")
            sim_params = rospy.get_param(sim_p)
            self.occ_grid_topic = sim_params["occupancy_grid_topic"]
            self.occ_grid_subscriber = rospy.Subscriber(self.occ_grid_topic, OccupancyGrid, queue_size=1)
            self.tesselation_topic = sim_params["tesselation_topic"]
            self.tesselation_subscriber = rospy.Subscriber(self.tesselation_topic, )
        except KeyError:
            rospy.logfatal("Parameter robots not found. Exiting.")
            sys.exit(1)
        except:
            rospy.logfatal("A non recognized exception raised while getting robots parameter. Exiting")
            sys.exit(1)

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
        return plt.imshow(image, extent=extent)

    def plot_occ_grid(self):
        occ_grid_img = np.zeros((self.occ_grid.width, self.occ_grid.height, 4), dtype=float)
        occ_grid_img[:,:,3] = self.occ_grid.occ_grid
        pass

    def add_image(self, data, extent, id=""):
        # type: (Image) -> None
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.images_plot[id] = self.plot_image(cv_image, extent)
        except CvBridgeError as e:
            print(e)

    def remove_image(self, img):
        # type: (mpimg.AxesImage) -> None
        img.remove()
        img.figure.canvas.draw()

    def robot_service(self, req):
        # type: (SetRobotPoseRequest) -> object
        try:
            self.create_robot(req.id, req.pose, req.color)
        except ValueError as e:
            rospy.logerr(e.message)
        return None

    def surf_service(self, req):
        pass

    def colorplot_service(self, req):
        # type: (SetColorPlotRequest) -> object
        pass

    def physics_thread(self):
        # threading.Timer(self.physics_time, self.physics_thread).start()
        # for robot in self.robots.values():
        #    robot.pose = self.update_pose_diff(robot.pose, robot.s, self.physics_time)
        pass

    def visual_thread(self):
        # threading.Timer(self.vis_time, self.visual_thread).start()
        pass

    def update_pose_diff(self, pose, vel, time):
        # type: (Pose, Twist, float) -> Pose
        w = Util.quaternion_get_yaw(pose.orientation)

        w_dot = vel.angular.z * time
        x_dot = vel.linear.x * math.cos(Pose.orientation.z)
        y_dot = vel.linear.x * math.sin(Pose.orientation.z)
        new_pose = Pose()

        w = w + w_dot
        new_pose.position.x = pose.position.x + x_dot
        new_pose.position.y = pose.position.y + y_dot
        new_pose.orientation = Util.get_quaternion_fom_euler([0, 0, w])
        if self.occ_grid.is_free(new_pose):
            return new_pose
        else:
            return pose

