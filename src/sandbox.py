import threading
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection
import numpy as np
import time
from pylab import *

import simulator

import Util
from geometry_msgs.msg import Pose, Twist
from simulator_util import DraggablePoint


# def remove_image(img):
#     # type: (mpimg.AxesImage) -> None
#     img.remove()
#     img.figure.canvas.draw()
#
#
# def draw_rectangles(fig, obstacles, origin, resolution, colors):
#     ax = fig.add_subplot(111, aspect='equal')
#     patchs = []
#     for index, elem in np.ndenumerate(obstacles):
#         if elem != 0:
#             pose = tuple(np.array(origin) + np.array(index)*resolution)
#             patchs.append(patches.Rectangle(pose, resolution, resolution, color=(0,0,0)))
#     pc = PatchCollection(patchs)
#     ax.add_collection(pc)
#     return ax
#
#
# def remove_plot(fig, ax):
#     Util.tic()
#     fig.delaxes(ax)
#     ax.figure.canvas.draw()
#     print Util.toc()
#
#
# def create_robot(sim, pose):
#     # type: (simulator.Simulator, list) -> None
#     pose_obj = Pose()
#     pose_obj.position.x = pose[0]
#     pose_obj.position.y = pose[1]
#     sim.create_robot("a", pose_obj)
#
#
# def get_points(occ_grid, origin, resolution):
#     points_x = []
#     points_y = []
#     origin = np.array(origin)
#     for index, elem in np.ndenumerate(occ_grid):
#         if elem != 0:
#             points_x.append(origin + np.array(index[0])*resolution)
#             points_y.append(origin + np.array(index[1])*resolution)
#
#     return points_x, points_y


#image = plt.imread("/home/lady/Pictures/hex1.jpg")
randimg = np.random.rand(100, 100)
img = plt.imshow(image, extent=[4, 16, 5, 25], zorder=0, interpolation='nearest')

sim = simulator.Simulator()
pose1 = Pose()
pose1.position.x = 5
pose1.position.y = 5
sim.create_robot("0", pose1, [0, 255, 0])

pose2 = Pose()
pose2.position.x = 6
pose2.position.y = 8
sim.create_robot("1", pose2, [255, 0, 0])

ones_matrix = np.zeros((40, 40, 4))
rand_matrix = np.around(np.random.rand(40, 40))
ones_matrix[:, :, 3] = rand_matrix

sim.plot_image(ones_matrix, [0, 20, 0, 20])
plt.draw()
plt.show()


# fig = plt.figure(1)
# sim = simulator.Simulator()
# rand_matrix = np.round(np.random.rand(80, 80))
# Util.tic()
# ax = draw_rectangles(fig, rand_matrix, [0, 0], 0.25)
# print Util.toc()
# threading.Timer(5, create_robot, [sim, [5, 6]]).start()
# show()

