import math
import numpy as np

import Util


class ControlLawDiff(object):

    def __init__(self, d=0.05, kv=1, kw=5):
        self.d = d
        self.kv = kv
        self.kw = kw
        self.I = []

    def set_control_parameters(self, d, kv, kw):
        self.d = d
        self.kv = kv
        self.kw = kw

    def holo_to_diff(self, vx, vy, phi):
        v = self.kv*(math.cos(phi)*vx + math.sin(phi)*vy)
        w = self.kw*(-math.sin(phi)*vx/self.d + math.cos(phi)*vy/self.d)
        return v, w

    def get_speed(self, pose_robot, pose_goal):
        error = np.subtract(pose_goal, pose_robot)
        return self.holo_to_diff(error[0], error[1], Util.pose_array_get_yaw(pose_robot))

    def get_speed_from_holo_speed(self, holo_speed, phi):
        v, w = self.holo_to_diff(holo_speed[0], holo_speed[1], phi)
        return v, w


class ControlLawVoronoi(ControlLawDiff):

    def __init__(self, d=0.05, kv=1, kw=5):
        super(ControlLawVoronoi, self).__init__(d, kv, kw)

    def clear_i(self):
        self.I = [0, 0]

    def add_control_law(self, v):
        self.I += v

    def get_control_integral(self):
        return self.I

    def get_speed(self, pose_robot, pose_goal):
        pose_robot_arr = [pose_robot.position.x, pose_robot.position.y]
        error = np.subtract(pose_goal, pose_robot_arr)
        v, w = self.holo_to_diff(error[0], error[1], Util.quaternion_get_yaw(pose_robot.orientation))
        return v, w


class ControlLawAlyssa(ControlLawVoronoi):

    def __init__(self, d=0.05, kv=1, kw=5, xd=1, yd=1):
        super(ControlLawAlyssa, self).__init__(d, kv, kw)
        self.xd = xd
        self.yd = yd
        self.dev_mat = [[xd, yd], [yd, xd]]

    def get_speed(self, pose_robot, pose_goal):
        error = np.subtract(pose_goal, pose_robot) * self.dev_mat
        return self.holo_to_diff(error[0], error[1], Util.pose_array_get_yaw(pose_robot))


class ControlLawEnergy(ControlLawVoronoi):

    def __init__(self, d=0.05, kv=1, kw=5, td=0.1, e_tot=100, e_k=1):
        super(ControlLawEnergy, self).__init__(d, kv, kw)
        self.td = td
        self.e_tot = e_tot
        self.e_t = e_tot
        self.e_k = e_k

    def get_speed(self, pose_robot, pose_goal):
        error = np.subtract(pose_goal, pose_robot)
        return self.get_charge() * self.holo_to_diff(error[0], error[1], Util.pose_array_get_yaw(pose_robot))

    def energy_loss(self, v, w, d, td, e_k):
        e_d = v*td*e_k + w*d*td*e_k
        self.e_t = self.e_t - e_d

    def get_charge_percent(self):
        return self.e_t/self.e_tot

    def get_charge(self):
        return self.e_t