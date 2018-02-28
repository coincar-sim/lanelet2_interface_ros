#include "internal/Point.h"

#include <eigen_conversions/eigen_msg.h>

#include "internal/Attributes.h"

namespace lanelet2_interface_ros {
lanelet::ConstPoint3d toPointData(const lanelet2_msgs_ros::Point& in) {
    lanelet::BasicPoint3d point;
    tf::pointMsgToEigen(in.point, point);
    return {in.id, point, toAttributeMap(in.attributes)};
}
lanelet2_msgs_ros::Point toPointMsg(const lanelet::ConstPoint3d& in) {
    lanelet2_msgs_ros::Point out;
    tf::pointEigenToMsg(in, out.point);
    out.id = in.id();
    out.attributes = toAttributesMsg(in.attributes());
    return out;
}
}