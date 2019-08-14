#!/usr/bin/env python
PKG = "lanelet2_interface_ros"

import unittest
import rospy
import tempfile
import os
import shutil
import lanelet2
import lanelet2_interface_ros as ll2if

def getTempMapPath():
    path = os.path.join(tempfile.mkdtemp(), 'testmap.osm')
    point = lanelet2.core.Point3d(0, 0, 0, 0)
    map = lanelet2.core.createMapFromPoints([point])
    lanelet2.io.write(path, map, ll2if.getProjector())
    return [path, map]

class TestLanelet2InterfacePython(unittest.TestCase):

    def test_init_and_project(self):
        projector = ll2if.getProjector()
        lat = rospy.get_param("/lanelet2_interface_ros/lat_origin")
        self.assertEqual(lat, projector.origin().position.lat)

        mapFrame = rospy.get_param("/lanelet2_interface_ros/map_frame_id")
        self.assertEqual(mapFrame, ll2if.getFrameIdMap())

        path, refMap = getTempMapPath()
        rospy.set_param("/lanelet2_interface_ros/map_file_name", path)
        llmap = ll2if.getLaneletMap()
        self.assertEqual(len(refMap.pointLayer), len(llmap.pointLayer))
        shutil.rmtree(os.path.dirname(path))


if __name__ == '__main__':
    import rostest

    rospy.init_node('lanelet2_interface_ros_python_test')
    rostest.rosrun(PKG, 'lanelet2_interface_ros', TestLanelet2InterfacePython)
