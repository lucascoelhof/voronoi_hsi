import math
import numpy as np
import threading

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.srv import GetMap
from voronoi_hsi.srv import *

import Util


class RobotSimulator(object):

    def __init__(self, id="0"):
        self.pose = Pose()
        self.speed = Twist()
        self.color = []
        self.id = id
        self.speed_callback = rospy.Subscriber("robot_".join([self.id, "/cmd_vel"]), Twist, queue_size=1)
        self.pose_publisher = rospy.Publisher("robot_".join([self.id, "/pose"]), Odometry, queue_size=10)

    def robot_vel_callback(self, msg):
        # type: (Twist) -> None
        self.speed = msg

    def pose_publisher(self):
        self.pose_publisher.publish(self.pose)


class OccGrid(object):

    def __init__(self, service_name):
        self.width = 0
        self.height = 0
        self.resolution = 0
        self.occ_grid = None  # type: np.matrix
        self.origin = Pose()
        self.service_name = service_name
        self.robot_pose_service = rospy.Service("occ_grid_update", SetOccGrid, self.set_occ_grid_service)

    def get_occ_grid(self):
        occ_grid_service = rospy.ServiceProxy(self.service_name, GetMap)
        occ_grid = occ_grid_service().map
        self.set_occ_grid(occ_grid)

    def set_occ_grid(self, occ_grid):
        # type: (OccupancyGrid) -> None
        self.width = self.occ_grid.info.width
        self.height = self.occ_grid.info.height
        self.resolution = self.occ_grid.info.resolution
        if self.occ_grid.info.origin:
            self.origin = self.occ_grid.info.origin
        self.occ_grid = np.mat(occ_grid.data).reshape(self.height, self.width).transpose()

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


class Simulator(object):

    def __init__(self):
        self.robots = {}  # type: dict[RobotSimulator]
        self.physics_time = 0.1
        self.vis_time = 0.2
        self.occ_grid = OccGrid("static_map")
        self.occ_grid.get_occ_grid()
        self.physics_thread()
        self.visual_thread()
        self.robot_pose_service = rospy.Service("set_robot_pose", RobotPose, self.robot_service)

    def read_parameters(self):
        try:
            robots = rospy.search_param("robots")
            if robots is not None and len(robots) > 0:
                for r in robots:  # type: dict
                    robot = RobotSimulator()
                    robot.pose = Pose(r["pose"])
                    robot.id = r["id"]
                    robot.color = r["color"]
                    self.robots[robot.id] = robot
        except KeyError:
            rospy.logfatal("Parameter robots not found. Exiting.")
            sys.exit(1)
        except:
            rospy.logfatal("A non recognized exception raised while getting robots parameter. Exiting")
            sys.exit(1)

    def robot_service(self, req):
        # type: (RobotPoseRequest) -> object
        if req.id in self.robots:
            robot = self.robots[req.id]
        else:
            robot = RobotSimulator(req.id)
            robot.color = [0, 0, 0]

        robot.Pose = req.pose
        if len(req.color) == 3:
           robot.color = req.color

        self.robots[robot.id] = robot
        return None

    def surf_service(self, req):
        pass

    def colorplot_service(self, req):
        # type: (ColorPlotRequest) -> object
        pass

    def physics_thread(self):
        threading.Timer(self.physics_time, self.physics_thread).start()
        for robot in self.robots.values():
            robot.pose = self.update_pose_diff(robot.pose, robot.s, self.physics_time)

    def visual_thread(self):
        threading.Timer(self.vis_time, self.visual_thread).start()
        pass

    def update_pose_diff(self, pose, vel, time):
        # type: (Pose, Twist, double) -> Pose
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

