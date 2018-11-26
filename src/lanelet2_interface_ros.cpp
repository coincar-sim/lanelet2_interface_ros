#include "lanelet2_interface_ros.hpp"

#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/UTM.h>

#include "exceptions.hpp"

using namespace lanelet2_interface_ros;

Lanelet2InterfaceRos::Lanelet2InterfaceRos() : isInitialized_(false), frameIdOrigin_("") {
}

bool Lanelet2InterfaceRos::isInitialized() const {
    return isInitialized_;
}

void Lanelet2InterfaceRos::waitForInit(double pollRateHz, double timeOutSecs) {
    ros::NodeHandle nh;
    ros::Rate rate(pollRateHz);

    bool frameIdFound{false}, mapFileNameFound{false}, latOriginFound{false}, lonOriginFound{false};
    double latOrigin, lonOrigin;
    std::string mapFileName;

    size_t counter = 0;
    size_t counterMax = std::max(1ul, size_t(timeOutSecs * pollRateHz));
    while (ros::ok() && counter < counterMax) {
        frameIdFound = nh.getParam("/lanelet2_interface_ros/frame_id_origin", frameIdOrigin_);
        latOriginFound = nh.getParam("/lanelet2_interface_ros/lat_origin", latOrigin);
        lonOriginFound = nh.getParam("/lanelet2_interface_ros/lon_origin", lonOrigin);
        mapFileNameFound = nh.getParam("/lanelet2_interface_ros/map_file_name", mapFileName);
        if (frameIdFound && latOriginFound && lonOriginFound && mapFileNameFound) {
            mapPtr_ =
                lanelet::load(mapFileName, lanelet::projection::UtmProjector(lanelet::Origin({latOrigin, lonOrigin})));
            isInitialized_ = true;
            return;
        } else {
            ROS_INFO_STREAM_THROTTLE_NAMED(5., "init_info", "lanelet2_interface_ros: Waiting... ");
            counter++;
            rate.sleep();
        }
    }
    std::string errMsg{"waitForInit failed due to "};
    if (!ros::ok()) {
        errMsg = errMsg + "!ros::ok() ";
    } else if (!(counter < counterMax)) {
        errMsg = errMsg + "timeOutSecs reached ";
    } else {
        errMsg = errMsg + "pedantic ";
    }
    errMsg = errMsg + " but still has not received the following info: ";
    if (!frameIdFound)
        errMsg = errMsg + " frameId ";
    if (!latOriginFound)
        errMsg = errMsg + " latOrigin ";
    if (!lonOriginFound)
        errMsg = errMsg + " lonOrigin ";
    if (!mapFileNameFound)
        errMsg = errMsg + " mapFileName ";
    throw InitializationError(errMsg);
}

std::string Lanelet2InterfaceRos::getFrameIdOrigin() const {
    if (!isInitialized_) {
        throw std::runtime_error("Access before initialization, use waitForInit()!");
    } else {
        return frameIdOrigin_;
    }
}

lanelet::LaneletMapConstPtr Lanelet2InterfaceRos::getMapPtr() const {
    if (!isInitialized_) {
        throw std::runtime_error("Access before initialization, use waitForInit()!");
    } else {
        return mapPtr_;
    }
}
