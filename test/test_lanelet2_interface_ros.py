#!/usr/bin/env python
PKG = "lanelet2_interface_ros"

import unittest
import rospy

import lanelet2
from lanelet2_interface_ros import Lanelet2InterfaceRos


class TestLanelet2InterfacePython(unittest.TestCase):

    def test_init_and_project(self):

        ll2if = Lanelet2InterfaceRos()
        projector = ll2if.waitForProjectorPtr(10.,10.)

        gps_point = lanelet2.core.GPSPoint(49., 8.)
        xy_point = projector.forward(gps_point)

        self.assertEqual(0., xy_point.x)
        self.assertEqual(0., xy_point.x)

        self.assertEqual("map", ll2if.waitForFrameIdMap(10., 10.))

        llmap = ll2if.waitForNonConstMapPtr(10.,10.)
        self.assertEqual(371, len(llmap.laneletLayer))  # expect 371 elements


if __name__ == '__main__':
    import rostest

    rospy.init_node('lanelet2_interface_ros_python_test')
    rostest.rosrun(PKG, 'lanelet2_interface_ros', TestLanelet2InterfacePython)
