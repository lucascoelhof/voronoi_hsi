import threading
import matplotlib.pyplot as plt

import simulator

from geometry_msgs.msg import Pose, Twist


from simulator_util import DraggablePoint


sim = simulator.Simulator()
pose1 = Pose()
pose1.position.x = 10
pose1.position.y = 10
robot1 = sim.create_robot("0", pose1)
pose2 = Pose()
pose2.position.x = 10
pose2.position.y = 5
robot2 = sim.create_robot("1", pose2)

sim.start()
