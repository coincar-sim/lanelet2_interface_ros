#pragma once

#include <lanelet2/primitives/Lanelet.h>
#include <lanelet2_msgs_ros/Lanelet.h>

namespace lanelet2_interface_ros {
lanelet::ConstLanelet toLanelet(const lanelet2_msgs_ros::Lanelet&);
lanelet2_msgs_ros::Lanelet toLaneletMsg(const lanelet::ConstLanelet&);
}