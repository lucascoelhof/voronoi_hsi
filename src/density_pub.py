import math
import numpy as np
from threading import Thread

import rospy
import time
from nav_msgs.srv import GetMap, GetMapResponse

from Util import Rate
from voronoi_hsi.msg import Gaussian


class DensityPublisher(Thread):

    @staticmethod
    def gaussian2d(gaussian, x, y):
        # type: (Gaussian, float, float) -> float
        x_part = math.pow(x - gaussian.x_c, 2)/(2*math.pow(gaussian.sigma_x, 2))
        y_part = math.pow(y - gaussian.y_c, 2)/(2*math.pow(gaussian.sigma_y, 2))
        return gaussian.a*math.exp(-(x_part + y_part)) + 1

    def publish_density(self):
        if self.density and self.gaussian is not None:
            self.density_pub.publish(self.density)
            density_data = np.reshape(self.density.data, (self.height, self.width))
            #np.savetxt("/home/lucas/test/density.txt", density_data, newline="\n")
            print("Saved.")

    def gaussian_callback(self, msg):
        # type: (Gaussian) -> None
        self.gaussian = msg

    def calculate_density(self):
        if self.gaussian is not None:
            for i in range(self.width):
                for j in range(self.height):
                    x = self.resolution * float(1/2.0 + i)
                    y = self.resolution * float(1/2.0 + j)
                    val = self.gaussian2d(self.gaussian, x, y)
                    self.density.data[j*self.width + i] = val

    def __init__(self):
        Thread.__init__(self)

        self.gaussian = None  # type: Gaussian

        self.gaussian = Gaussian()
        self.gaussian.a = 10
        self.gaussian.x_c = 5
        self.gaussian.y_c = 5
        self.gaussian.sigma_x = 1
        self.gaussian.sigma_y = 1

        #self.density = Matrix2D()
        self.loop_rate = Rate(1)

        rospy.init_node('density', anonymous=True)
        self.gaussian_topic = rospy.get_param("/voronoi/topic_info/gaussian_topic")
        self.density_topic = rospy.get_param("/voronoi/topic_info/density_topic")
        self.map_service = rospy.get_param("/voronoi/topic_info/occupancy_grid_service")
        self.resize = rospy.get_param("/voronoi/topic_info/occ_grid_resize")
        #self.gaussian_sub = rospy.Subscriber(self.gaussian_topic, Gaussian, self.gaussian_callback)
        self.density_pub = rospy.Publisher(self.density_topic, Matrix2D, queue_size=1)

        service_map = rospy.ServiceProxy(self.map_service, GetMap)
        occ_g = service_map()  # type: GetMapResponse
        self.height = int(math.ceil(occ_g.map.info.height/float(self.resize)))
        self.width = int(math.ceil(occ_g.map.info.width/float(self.resize)))
        self.resolution = occ_g.map.info.resolution*self.resize
        #self.density.width = self.width
        #self.density.height = self.height
        #self.density.data = np.zeros((self.width*self.height, 1))

        self.start()

    def run(self):
        time.sleep(1)
        while not rospy.is_shutdown():
            self.calculate_density()
            self.publish_density()
            self.loop_rate.sleep()


if __name__ == '__main__':
    try:
        g = DensityPublisher()
    except rospy.ROSInterruptException:
        pass