#include "internal/LineString.h"

#include "internal/Attributes.h"
#include "internal/Point.h"
#include "../../cmake-build-release/devel/include/lanelet2_msgs_ros/LineString.h"

namespace lanelet2_interface_ros {

lanelet::LineString3d toLineString(const lanelet2_msgs_ros::LineString& in) {
    lanelet::Points3d points;
    points.reserve(in.points.size());
    for (const auto& p : in.points) {
        points.emplace_back(toPointData(p));
    }
    return lanelet::LineString3d{in.id, points, toAttributeMap(in.attributes)};
}
lanelet2_msgs_ros::LineString toLineStringMsg(const lanelet::ConstLineString3d& in) {
    lanelet2_msgs_ros::LineString out;
    out.attributes = toAttributesMsg(in.attributes());
    out.id = in.id();
    out.points.reserve(in.size());
    for (const auto& p : in) {
        out.points.emplace_back(toPointMsg(p));
    }
    return out;
}
}