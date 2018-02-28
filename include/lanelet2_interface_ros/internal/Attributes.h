#include <lanelet2/Attribute.h>
#include <lanelet2_msgs_ros/Attributes.h>

namespace lanelet2_interface_ros {
lanelet::AttributeMap toAttributeMap(const lanelet2_msgs_ros::Attributes&);
lanelet2_msgs_ros::Attributes toAttributesMsg(const lanelet::AttributeMap&);
}