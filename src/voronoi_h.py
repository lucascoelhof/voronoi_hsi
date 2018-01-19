#! /usr/bin/env python

import time
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float64

import Util
from Voronoi import Voronoi

h_func = []
time_arr = []
h1, = plt.plot([], [])

def main():
    loop_time = 5.0
    rospy.init_node('voronoi_hsi')
    rate = Util.Rate(1/loop_time)
    iterations = 0

    start_time = time.time()

    h_publisher = rospy.Publisher("/voronoi/h_func", Float64, queue_size=1)

    voronoi = Voronoi(loop_time)

    time.sleep(2)

    rospy.loginfo("Starting Voronoi")

    voronoi.tesselation_and_control_computation()

    while not rospy.is_shutdown():
        h = voronoi.tesselation_and_control_computation()
        d = time.time() - start_time
        h_publisher.publish(h)
        iterations += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass