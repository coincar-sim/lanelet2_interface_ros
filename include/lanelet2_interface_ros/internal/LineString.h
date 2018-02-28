#pragma once

#include <lanelet2_msgs_ros/LineString.h>
#include <lanelet2/primitives/LineString.h>

namespace lanelet2_interface_ros {
lanelet::LineString3d toLineString(const lanelet2_msgs_ros::LineString&);
lanelet2_msgs_ros::LineString toLineStringMsg(const lanelet::ConstLineString3d&);
}