#include "lanelet2_interface_ros.hpp"

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include "exceptions.hpp"

using namespace lanelet2_interface_ros;

std::string Lanelet2InterfaceRos::waitForFrameIdMap(double pollRateHz, double timeOutSecs) {
    if (!params_.frameIdMapFound) {
        waitForParams(pollRateHz, timeOutSecs);
    }
    return params_.frameIdMap;
}

lanelet::LaneletMapConstPtr Lanelet2InterfaceRos::waitForMapPtr(double pollRateHz, double timeOutSecs) {
    return waitForNonConstMapPtr(pollRateHz, timeOutSecs);
}

lanelet::LaneletMapPtr Lanelet2InterfaceRos::waitForNonConstMapPtr(double pollRateHz, double timeOutSecs) {
    if (!nonConstMapPtr_) {
        waitForParams(pollRateHz, timeOutSecs);
        loadMap();
    }
    return nonConstMapPtr_;
}

std::shared_ptr<lanelet::Projector> Lanelet2InterfaceRos::waitForProjectorPtr(double pollRateHz, double timeOutSecs) {
    if (!utmProjectorPtr_) {
        waitForParams(pollRateHz, timeOutSecs);
        loadMap();
    }
    return utmProjectorPtr_;
}

void Lanelet2InterfaceRos::waitForParams(double pollRateHz, double timeOutSecs) {
    ros::NodeHandle nh;
    ros::Rate rate(pollRateHz);

    size_t counterMax = size_t(std::max(1., timeOutSecs * pollRateHz));
    for (size_t i = 0; i < counterMax; ++i) {
        if (!ros::ok()) {
            throw InitializationError("!ros::ok()");
        }
        params_.frameIdMapFound = nh.getParam("/lanelet2_interface_ros/map_frame_id", params_.frameIdMap);
        params_.latOriginFound = nh.getParam("/lanelet2_interface_ros/lat_origin", params_.latOrigin);
        params_.lonOriginFound = nh.getParam("/lanelet2_interface_ros/lon_origin", params_.lonOrigin);
        params_.mapFileNameFound = nh.getParam("/lanelet2_interface_ros/map_file_name", params_.mapFileName);
        if (params_.frameIdMapFound && params_.latOriginFound && params_.lonOriginFound && params_.mapFileNameFound) {
            return;
        } else {
            ROS_INFO_STREAM_THROTTLE(5., "lanelet2_interface_ros: Waiting... ");
            rate.sleep();
        }
    }
    std::string errMsg{"waitForInit failed due to timeout, information up to now: "};
    errMsg = errMsg + "frameIdMap=\"" + params_.frameIdMap + "\", ";
    errMsg = errMsg + "mapFileName=\"" + params_.mapFileName + "\", ";
    errMsg = errMsg + "latOrigin=\"" + std::to_string(params_.latOrigin) + "\", ";
    errMsg = errMsg + "lonOrigin=\"" + std::to_string(params_.lonOrigin) + "\".";
    throw InitializationError(errMsg);
}

void Lanelet2InterfaceRos::loadMap() {
    utmProjectorPtr_ = std::make_shared<lanelet::projection::UtmProjector>(
        lanelet::Origin({params_.latOrigin, params_.lonOrigin}), true);
    nonConstMapPtr_ = lanelet::load(params_.mapFileName, *utmProjectorPtr_);
}
