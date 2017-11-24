from src.Graph import Graph
import unittest
import UtilTests
import rospy
import time
import subprocess
from nav_msgs.srv import GetMap
import rosnode


class GraphTests(unittest.TestCase):

    def test_init(self):
        try:
            graph = Graph('static_map', "/map")
        except Exception as e:
            self.fail(UtilTests.error_str(self.test_init.__name__, e))

    def test_creation_graph(self):
        try:
            graph = Graph('static_map', "/map")

        except Exception as e:
            self.fail(UtilTests.error_str(self.test_creation_graph.__name__, e))

    def test_get_node(self):
        try:
            graph = Graph('static_map', "/map")
            graph.nodes[10, 10].cost = 10
            pose = graph.get_pose(10, 10)
            n = graph.get_node(pose)
            self.assertEquals(n.cost, 10)
        except Exception as e:
            self.fail(UtilTests.error_str(self.test_get_node.__name__, e))

    def test_connectivity_neighbors(self):
        try:
            graph = Graph('static_map', "/map")
            n1 = graph.nodes[20, 20]
            n2 = graph.nodes[21, 20]
            n3 = graph.nodes[18, 18]

            self.assertGreater(len(n1.neighbors), 4)
            self.assertGreater(len(n2.neighbors), 4)
            self.assertGreater(len(n3.neighbors), 4)

            self.assertFalse(n3 in n1.neighbors)
            self.assertFalse(n3 in n2.neighbors)
            self.assertFalse(n1 in n1.neighbors)
            self.assertTrue(n1 in n2.neighbors)
            self.assertTrue(n2 in n1.neighbors)

        except Exception as e:
            self.fail(UtilTests.error_str(self.test_connectivity_neighbors.__name__, e))

    def test_clean(self):
        try:
            graph = Graph('static_map', "/map")
            n1 = graph.nodes[10, 10]
            n1.cost = 10
            graph.clear_graph()
            self.assertNotEqual(n1.cost, 10)
            self.assertEquals(n1.cost, float('inf'))
        except Exception as e:
            self.fail(UtilTests.error_str(self.test_clean.__name__, e))


if __name__ == '__main__':
    rospy.init_node('voronoi_hsi_tests', anonymous=True)
    node_names = rosnode.get_node_names()
    if not any("map_server" in node for node in node_names):
        subprocess.Popen("roslaunch voronoi_hsi map_server.launch")
    time.sleep(5)
    unittest.main()
