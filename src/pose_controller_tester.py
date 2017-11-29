import time
import random

import rospy

from Voronoi import Voronoi
from Robot import Robot


"""

Takes the robots for a ride on the map. Does not avoid obstacles.

"""


def main():
    rospy.init_node('voronoi_hsi', anonymous=True)
    rate = rospy.Rate(1)

    voronoi = Voronoi()

    time.sleep(2)

    bounds = [0, 20, 0, 20]  # bounds of the goals set to robots

    while not rospy.is_shutdown():
        rate.sleep()
        for robot in voronoi.robots.values():  # type: Robot
            robot_pose = robot.pose
            robot_node = voronoi.graph.get_node(robot_pose)
            goal_pose = robot.control.goal
            goal_node = voronoi.graph.get_node(goal_pose)
            if robot_node is goal_node or goal_node is None:
                x = random.uniform(bounds[0], bounds[1])
                y = random.uniform(bounds[2], bounds[3])
                robot.control.set_goal([x, y])
                rospy.loginfo("Robot " + str(robot.id) + " goal now is:", str(x), str(y))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass