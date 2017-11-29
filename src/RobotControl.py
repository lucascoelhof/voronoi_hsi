from threading import Thread

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

import Util
from ControlLaw import ControlLawDiff


class RobotControl(Thread):

    def __init__(self, loop_rate=10):
        Thread.__init__(self)
        self.loop_rate = loop_rate

        self.pose = Pose()
        self.pose_updater = None
        self.speed_pub = None

        self.goal = []
        self.control_law = None

    def update_pose(self):
        self.pose = self.pose_updater()

    def set_pose_updater(self, pose_updater):
        # type: (function) -> RobotControl
        self.pose_updater = pose_updater
        return self

    def set_speed_publisher(self, speed_pub):
        # type: (rospy.Publisher) -> RobotControl
        self.speed_pub = speed_pub
        return self

    def set_control_law(self, control_law):
        # type: (ControlLawDiff) -> RobotControl
        self.control_law = control_law
        return self

    def set_goal(self, goal):
        # type: (list) -> None
        self.goal = goal

    def run(self):
        rate = Util.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if self.goal is not None and len(self.goal) > 0:  # and self.goal check if the list is not empty
                v, w = self.control_law.get_speed(self.pose_updater(), self.goal)
                speed_msg = Twist()
                speed_msg.linear.x = v
                speed_msg.angular.z = w
                self.speed_pub.publish(speed_msg)
                rate.sleep()



