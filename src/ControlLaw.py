import math
import numpy as np

from geometry_msgs.msg import Pose

import Util


class ControlLawDiff(object):

    def __init__(self, d=0.05, kv=1, kw=5):
        # type: (float, float, float) -> ControlLawDiff
        self.d = d
        self.kv = kv
        self.kw = kw
        self.i_control = []

    def set_control_parameters(self, d, kv, kw):
        # type: (float, float, float) -> None
        self.d = d
        self.kv = kv
        self.kw = kw

    def holo_to_diff(self, vx, vy, phi):
        # type: (float, float, float) -> tuple
        v = self.kv*(math.cos(phi)*vx + math.sin(phi)*vy)
        w = self.kw*(-math.sin(phi)*vx/self.d + math.cos(phi)*vy/self.d)
        return v, w

    def get_speed(self, pose_robot, pose_goal):
        # type: (list, list) -> tuple
        error = np.subtract(pose_goal, pose_robot)
        return self.holo_to_diff(error[0], error[1], Util.pose_array_get_yaw(pose_robot))

    def get_speed_from_holo_speed(self, holo_speed, phi):
        # type: (list, float) -> tuple
        v, w = self.holo_to_diff(holo_speed[0], holo_speed[1], phi)
        return v, w


class ControlLawVoronoi(ControlLawDiff):

    def __init__(self, d=0.05, kv=1, kw=5):
        # type: (float, float, float) -> ControlLawVoronoi
        super(ControlLawVoronoi, self).__init__(d, kv, kw)

    def clear_i(self):
        self.i_control = [0, 0]

    def add_control_law(self, v):
        # type: (list) -> None
        self.i_control += v

    def get_control_integral(self):
        return self.i_control

    def get_speed(self, pose_robot, pose_goal):
        # type: (Pose, list) -> tuple
        pose_robot_arr = [pose_robot.position.x, pose_robot.position.y]
        error = np.subtract(pose_goal, pose_robot_arr)
        v, w = self.holo_to_diff(error[0], error[1], Util.quaternion_get_yaw(pose_robot.orientation))
        return v, w


class ControlLawEnergy(ControlLawVoronoi):

    def __init__(self, d=0.05, kv=1, kw=5, td=0.1, e_tot=100, e_k=1):
        # type: (float, float, float, float, float, float) -> ControlLawEnergy
        super(ControlLawEnergy, self).__init__(d, kv, kw)
        self.td = td
        self.e_tot = e_tot
        self.e_t = e_tot
        self.e_k = e_k

    def get_speed(self, pose_robot, pose_goal):
        # type: (list, list) -> tuple
        error = np.subtract(pose_goal, pose_robot)
        v, w = self.holo_to_diff(error[0], error[1], Util.pose_array_get_yaw(pose_robot))
        return self.get_charge()*v, self.get_charge()*w

    def energy_loss(self, v, w, d, td, e_k):
        # type: (float, float, float, float, float) -> None
        e_d = v*td*e_k + w*d*td*e_k
        self.e_t = self.e_t - e_d

    def get_charge_percent(self):
        return self.e_t/self.e_tot

    def get_charge(self):
        return self.e_t
