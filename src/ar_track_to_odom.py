#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class ArTrackToOdom(object):

    def __init__(self):
        self.ar_track_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self.tracker_callback, queue_size=1)
        self.odom_publishers = {}  # type: dict[int, rospy.Publisher]
        self.shift = [0, 0, 0]
        self.mult = [1, 1, 1]
        self.robot_prefix = "robot_"
        self.robot_suffix = "/base_pose_ground_truth"

    def tracker_callback(self, msg):
        # type: (AlvarMarkers) -> None
        for marker in msg.markers:  # type: AlvarMarker
            id = marker.id
            odom = Odometry()
            odom.pose.pose = self.odom_operations(marker.pose.pose)
            if not id in self.odom_publishers:
                self.odom_publishers[id] = rospy.Publisher(self.robot_prefix + str(id) + self.robot_suffix, Odometry, queue_size=1)
            self.odom_publishers[id].publish(odom)

    def odom_operations(self, origin):
        # type: (Pose) -> Pose
        pose = Pose()
        pose.position.x = (origin.position.x + self.shift[0]) * self.mult[0]
        pose.position.y = (origin.position.y + self.shift[1]) * self.mult[1]
        pose.position.z = (origin.position.z + self.shift[2]) * self.mult[2]
        pose.orientation = origin.orientation
        return pose


if __name__ == '__main__':
    try:
        rospy.init_node("ar_track_bridge")
        ar = ArTrackToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit(1)

