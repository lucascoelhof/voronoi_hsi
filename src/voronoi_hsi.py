import time
import numpy as np
import matplotlib.pyplot as plt

import rospy

from Voronoi import Voronoi


h_func = []
time_arr = []
h1, = plt.plot([], [])


def plot_h(h, d):
    np.append(h_func, h)
    np.append(time_arr, d)
    h1.set_xdata(h_func)
    h1.set_ydata(time_arr)
    plt.draw()
    print(h, d)


def main():
    rospy.init_node('voronoi_hsi')
    rate = rospy.Rate(1)
    iterations = 0

    start_time = time.time()

    voronoi = Voronoi()

    time.sleep(2)

    print("Starting Voronoi")

    voronoi.tesselation_and_control_computation()

    while not rospy.is_shutdown():
        h = voronoi.tesselation_and_control_computation()
        d = time.time() - start_time
        iterations += 1
        plot_h(h, d)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass