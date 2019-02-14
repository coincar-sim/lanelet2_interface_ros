/*
 * Copyright (c) 2018
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
