#pragma once

#include <lanelet2_msgs_ros/RegulatoryElement.h>
#include <lanelet2/primitives/RegulatoryElement.h>

namespace lanelet2_interface_ros {
lanelet::RegulatoryElementPtr toRegulatoryElement(const lanelet2_msgs_ros::RegulatoryElement&);
lanelet2_msgs_ros::RegulatoryElement toRegulatoryElementMsg(const lanelet::RegulatoryElementConstPtr&);
lanelet::RegulatoryElementPtrs toRegulatoryElements(const std::vector<lanelet2_msgs_ros::RegulatoryElement>&);
std::vector<lanelet2_msgs_ros::RegulatoryElement> toRegulatoryElementsMsg(const lanelet::RegulatoryElementConstPtrs&);
}