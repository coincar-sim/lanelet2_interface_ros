#pragma once

#include <lanelet2/primitives/Point.h>
#include <lanelet2_msgs_ros/Point.h>

namespace lanelet2_interface_ros {
lanelet::ConstPoint3d toPointData(const lanelet2_msgs_ros::Point&);
lanelet2_msgs_ros::Point toPointMsg(const lanelet::ConstPoint3d&);
}