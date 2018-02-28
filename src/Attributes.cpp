#include "internal/Attributes.h"

namespace lanelet2_interface_ros {

lanelet::AttributeMap toAttributeMap(const lanelet2_msgs_ros::Attributes& in) {
    lanelet::AttributeMap out;
    return out;
}
lanelet2_msgs_ros::Attributes toAttributesMsg(const lanelet::AttributeMap& in) {
    lanelet2_msgs_ros::Attributes out;
    return out;
}
}