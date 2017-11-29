import time
import subprocess
import unittest

import rospy

import UtilTests
from src.Voronoi import Voronoi


class VoronoiTests(unittest.TestCase):

    def test_creating_voronoi(self):
        try:
            voronoi = Voronoi()
        except Exception as e:
            self.fail(UtilTests.error_str(self.test_creating_voronoi.__name__, e))

    def test_loading_robots(self):
        try:
            voronoi = Voronoi()
        except Exception as e:
            self.fail(UtilTests.error_str(self.test_loading_robots.__name__, e))

if __name__ == '__main__':
    rospy.init_node('voronoi_hsi_tests', anonymous=True)
    subprocess.Popen("roslaunch voronoi_hsi voronoi_params.launch")
    time.sleep(5)
    unittest.main()
