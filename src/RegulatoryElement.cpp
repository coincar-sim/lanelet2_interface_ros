#include "internal/RegulatoryElement.h"

namespace lanelet2_interface_ros {
lanelet::RegulatoryElementPtr toRegulatoryElement(const lanelet2_msgs_ros::RegulatoryElement& in) {
    return std::make_shared<lanelet::GenericRegulatoryElement>();
}
lanelet2_msgs_ros::RegulatoryElement toRegulatoryElementMsg(const lanelet::RegulatoryElementConstPtr& in) {
    lanelet2_msgs_ros::RegulatoryElement out;
    return out;
}
lanelet::RegulatoryElementPtrs toRegulatoryElements(const std::vector<lanelet2_msgs_ros::RegulatoryElement>& in) {
    lanelet::RegulatoryElementPtrs out;
    return out;
}
std::vector<lanelet2_msgs_ros::RegulatoryElement> toRegulatoryElementsMsg(const lanelet::RegulatoryElementConstPtrs& in) {
    std::vector<lanelet2_msgs_ros::RegulatoryElement> out;
    return out;
}
}