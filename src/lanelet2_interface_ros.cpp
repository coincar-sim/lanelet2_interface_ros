#include "lanelet2_interface_ros.hpp"

#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

#include "exceptions.hpp"

using namespace lanelet2_interface_ros;

Lanelet2InterfaceRos::Lanelet2InterfaceRos() {
}

std::string Lanelet2InterfaceRos::waitForFrameIdMap(double pollRateHz, double timeOutSecs) {
    if (!params_.frameIdMapFound) {
        waitForParams_(pollRateHz, timeOutSecs);
    }
    return params_.frameIdMap;
}

std::string Lanelet2InterfaceRos::waitForFrameIdMapWithOffset(double pollRateHz, double timeOutSecs) {
    if (!params_.frameIdMapWithOffsetFound) {
        waitForParams_(pollRateHz, timeOutSecs);
    }
    return params_.frameIdMapWithOffset;
}

lanelet::LaneletMapConstPtr Lanelet2InterfaceRos::waitForMapPtr(double pollRateHz, double timeOutSecs) {
    if (!mapPtr_) {
        waitForParams_(pollRateHz, timeOutSecs);
        mapPtr_ = lanelet::load(
            params_.mapFileName,
            lanelet::projection::UtmProjector(lanelet::Origin({params_.latOrigin, params_.lonOrigin}), false));
    }
    return mapPtr_;
}

lanelet::LaneletMapConstPtr Lanelet2InterfaceRos::waitForMapWithOffsetPtr(double pollRateHz, double timeOutSecs) {
    if (!mapWithOffsetPtr_) {
        waitForParams_(pollRateHz, timeOutSecs);
        mapWithOffsetPtr_ = lanelet::load(
            params_.mapFileName,
            lanelet::projection::UtmProjector(lanelet::Origin({params_.latOrigin, params_.lonOrigin}), true));
    }
    return mapWithOffsetPtr_;
}
lanelet::LaneletMapPtr Lanelet2InterfaceRos::waitForNonConstMapPtr(double pollRateHz, double timeOutSecs) {
    if (!nonConstMapPtr_) {
        waitForParams_(pollRateHz, timeOutSecs);
        nonConstMapPtr_ = lanelet::load(
            params_.mapFileName,
            lanelet::projection::UtmProjector(lanelet::Origin({params_.latOrigin, params_.lonOrigin}), false));
    }
    return nonConstMapPtr_;
}

void Lanelet2InterfaceRos::waitForParams_(double pollRateHz, double timeOutSecs) {
    ros::NodeHandle nh;
    ros::Rate rate(pollRateHz);

    size_t counterMax = std::max(1ul, size_t(timeOutSecs * pollRateHz));
    for (size_t i = 0; i < counterMax; ++i) {
        if (!ros::ok()) {
            throw InitializationError("!ros::ok()");
        }
        params_.frameIdMapFound = nh.getParam("/lanelet2_interface_ros/map_frame_id", params_.frameIdMap);
        params_.frameIdMapWithOffsetFound =
            nh.getParam("/lanelet2_interface_ros/map_with_offset_frame_id", params_.frameIdMapWithOffset);
        params_.latOriginFound = nh.getParam("/lanelet2_interface_ros/lat_origin", params_.latOrigin);
        params_.lonOriginFound = nh.getParam("/lanelet2_interface_ros/lon_origin", params_.lonOrigin);
        params_.mapFileNameFound = nh.getParam("/lanelet2_interface_ros/map_file_name", params_.mapFileName);
        if (params_.frameIdMapFound && params_.frameIdMapWithOffsetFound && params_.latOriginFound &&
            params_.lonOriginFound && params_.mapFileNameFound) {
            return;
        } else {
            ROS_INFO_STREAM_THROTTLE(5., "lanelet2_interface_ros: Waiting... ");
            rate.sleep();
        }
    }
    std::string errMsg{"waitForInit failed due to timeout, information up to now: "};
    errMsg = errMsg + "frameIdMap=\"" + params_.frameIdMap + "\", ";
    errMsg = errMsg + "frameIdMapWithOffset=\"" + params_.frameIdMapWithOffset + "\", ";
    errMsg = errMsg + "mapFileName=\"" + params_.mapFileName + "\", ";
    errMsg = errMsg + "latOrigin=\"" + std::to_string(params_.latOrigin) + "\", ";
    errMsg = errMsg + "lonOrigin=\"" + std::to_string(params_.lonOrigin) + "\".";
    throw InitializationError(errMsg);
}
