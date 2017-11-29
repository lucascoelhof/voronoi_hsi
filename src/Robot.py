import json

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import Util
from RobotControl import RobotControl


class Robot:
    def __init__(self, id_r, weight, color):
        self.id = id_r
        self.weight = weight
        self.color = color
        self.speed_v = 0
        self.speed_w = 0
        self.speed_pub = None
        self.pose_sub = None
        self.pose = Pose()

        self.control = RobotControl()

    def get_yaw(self):
        (roll, pitch, yaw) = euler_from_quaternion(self.pose.orientation)
        return yaw

    def set_speed_publisher(self, topic):
        self.speed_pub = rospy.Publisher(topic, Twist, queue_size=5)
        self.init_robot_control()

    def init_robot_control(self):
        self.control.set_speed_publisher(self.speed_pub)
        self.control.set_pose_updater(self.get_pose)
        self.control.start()

    def set_pose_subscriber(self, topic):
        # type: (basestring) -> None
        self.pose_sub = rospy.Subscriber(topic, Odometry, self.pose_callback)

    def publish_speed(self, v, w):
        # type: (float, float) -> None
        if self.speed_pub is None:
            raise ValueError("geometry_msgs/Twist publisher for robot " + self.id + " not initialized.")
        else:
            self.speed_v = v
            self.speed_w = w
            speed = Twist()
            speed.linear.x = v
            speed.angular.w = w
            self.speed_pub.publish(speed)

    def pose_callback(self, msg):
        # type: (float) -> None
        self.pose = msg.pose.pose
        Util.publish_tf_transformation(self.pose, "robot_" + str(self.id) + "/odom", "/map")

    def get_pose_array(self):
        return [self.pose.position.x, self.pose.position.y]

    def get_pose(self):
        return self.pose

    def __repr__(self):
        rep = self.__dict__.copy()
        del rep["pose"]
        if rep["speed_pub"] is not None:
            rep["speed_pub"] = "value set"
        if rep["pose_sub"] is not None:
            rep["pose_sub"] = "value set"

        return json.dumps(rep)

    def __str__(self):
        return self.__repr__()

    def set_pose(self, arr):
        # type: (list) -> None
        self.pose.position.x = arr[0]
        self.pose.position.y = arr[1]

