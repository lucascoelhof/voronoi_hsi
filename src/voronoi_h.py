#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

from Voronoi import Voronoi


class VoronoiRunner(object):

    def __init__(self):
        rospy.init_node('voronoi')
        self.loop_time = 0.5
        self.h_pub = rospy.Publisher("/voronoi/h_func", Float64, queue_size=1)
        self.voronoi = Voronoi(self.loop_time)
        self.iterations = 0

    def main(self):
        rospy.loginfo("Starting Voronoi")
        rospy.sleep(rospy.Duration.from_sec(1))
        list_robots = []
        for robot in self.voronoi.robots.values():
            list_robots.append(robot.id)
        self.voronoi.tesselation_and_control_computation(list_robots)
        rospy.Timer(rospy.Duration.from_sec(self.loop_time), self.loop)
        rospy.spin()

    def loop(self, event):
        list_robots = []
        for robot in self.voronoi.robots.values():
            #if self.voronoi.robot_reached_goal(robot):
            list_robots.append(robot.id)
            rospy.loginfo("Robot " + str(robot.id) + " has reached goal")
        if list_robots:
            h = self.voronoi.tesselation_and_control_computation(list_robots)
            self.h_pub.publish(h)
            self.iterations += 1


if __name__ == '__main__':
    try:
        vs = VoronoiRunner()
        vs.main()
    except rospy.ROSInterruptException:
        exit(1)
