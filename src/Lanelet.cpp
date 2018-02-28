#include "Lanelet.h"
#include "internal/Attributes.h"
#include "internal/LineString.h"
#include "internal/RegulatoryElement.h"

namespace lanelet2_interface_ros {

lanelet::ConstLanelet toLanelet(const lanelet2_msgs_ros::Lanelet& in) {
    return lanelet::Lanelet{std::make_shared<lanelet::LaneletData>(in.id,
                                                                   toLineString(in.left),
                                                                   toLineString(in.right),
                                                                   toAttributeMap(in.attributes),
                                                                   toRegulatoryElements(in.regulatory_elements))};
}
lanelet2_msgs_ros::Lanelet toLaneletMsg(const lanelet::ConstLanelet& in) {
    lanelet2_msgs_ros::Lanelet out;
    out.regulatory_elements = toRegulatoryElementsMsg(in.regulatoryElements());
    out.attributes = toAttributesMsg(in.attributes());
    out.left = toLineStringMsg(in.leftBound());
    return out;
}
}