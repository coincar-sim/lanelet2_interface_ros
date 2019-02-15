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

// It is very important that this is the first header in every file.
// This includes some modifications to the boost::python headers that
// make them work with memory-aligned Eigen types.
// For more usage examples, look at
// https://github.com/ethz-asl/programming_guidelines/wiki/Adding-python-bindings-to-your-cpp-catkin-package
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// Now bring in the headers of functions and classes you are wrapping.
#include "exceptions.hpp"
#include "lanelet2_interface_ros.hpp"

#include <ros/ros.h>

namespace rospy_helpers {

template <typename T>
inline bool get_param_like_in_cpp(const std::string& paramName, T& paramValue) {
    auto rospy = boost::python::import("rospy");

    bool has_param = boost::python::extract<bool>(rospy.attr("has_param")(paramName));
    if (!has_param) {
        return false;
    } else {
        paramValue = boost::python::extract<T>(rospy.attr("get_param")(paramName));
        return true;
    }
}
inline bool ros_ok() {
    auto rospy = boost::python::import("rospy");
    bool is_shutdown = boost::python::extract<bool>(rospy.attr("core").attr("is_shutdown")());
    return !is_shutdown;
}

} // namespace rospy_helpers

// The module name here *must* match the name of the python project. You can use the PYTHON_API_MODULE_NAME definition.
BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {
    // This using statement is just for convenience
    using namespace boost::python;
    using namespace lanelet2_interface_ros;

    // instead of initializing roscpp which takes the signals from rospy, we read the params using rospy
    class Lanelet2InterfaceRosPythonWrapper : Lanelet2InterfaceRos {
    public:
        std::string waitForFrameIdMapNoDefault(double pollRateHz, double timeOutSecs) {
            return waitForFrameIdMap(pollRateHz, timeOutSecs);
        }
        lanelet::LaneletMapPtr waitForNonConstMapPtrNoDefault(double pollRateHz, double timeOutSecs) {
            return waitForNonConstMapPtr(pollRateHz, timeOutSecs);
        }
        std::shared_ptr<lanelet::Projector> waitForProjectorPtrNoDefault(double pollRateHz, double timeOutSecs) {
            return waitForProjectorPtr(pollRateHz, timeOutSecs);
        }

    private:
        void waitForParams(double pollRateHz, double timeOutSecs) override {
            // duplicate of cpp code but using rospy instead of roscpp
            auto rospy = import("rospy");
            auto rospy__rate = rospy.attr("Rate")(pollRateHz);

            auto rospy__loginfo_throttle = rospy.attr("loginfo_throttle");

            size_t counterMax = size_t(std::max(1., timeOutSecs * pollRateHz));
            for (size_t i = 0; i < counterMax; ++i) {
                if (!rospy_helpers::ros_ok()) {
                    throw InitializationError("!ros::ok()");
                }
                params_.frameIdMapFound =
                    rospy_helpers::get_param_like_in_cpp("/lanelet2_interface_ros/map_frame_id", params_.frameIdMap);
                params_.latOriginFound =
                    rospy_helpers::get_param_like_in_cpp("/lanelet2_interface_ros/lat_origin", params_.latOrigin);
                params_.lonOriginFound =
                    rospy_helpers::get_param_like_in_cpp("/lanelet2_interface_ros/lon_origin", params_.lonOrigin);
                params_.mapFileNameFound =
                    rospy_helpers::get_param_like_in_cpp("/lanelet2_interface_ros/map_file_name", params_.mapFileName);
                if (params_.frameIdMapFound && params_.latOriginFound && params_.lonOriginFound &&
                    params_.mapFileNameFound) {
                    return;
                } else {
                    rospy__loginfo_throttle(5., "lanelet2_interface_ros: Waiting... ");
                    rospy__rate.attr("sleep")();
                }
            }
            std::string errMsg{"waitForInit failed due to timeout, information up to now: "};
            errMsg = errMsg + "frameIdMap=\"" + params_.frameIdMap + "\", ";
            errMsg = errMsg + "mapFileName=\"" + params_.mapFileName + "\", ";
            errMsg = errMsg + "latOrigin=\"" + std::to_string(params_.latOrigin) + "\", ";
            errMsg = errMsg + "lonOrigin=\"" + std::to_string(params_.lonOrigin) + "\".";
            throw InitializationError(errMsg);
        }
    };

    class_<Lanelet2InterfaceRosPythonWrapper>("Lanelet2InterfaceRos")
        .def("waitForFrameIdMap", &Lanelet2InterfaceRosPythonWrapper::waitForFrameIdMapNoDefault)
        .def("waitForNonConstMapPtr", &Lanelet2InterfaceRosPythonWrapper::waitForNonConstMapPtrNoDefault)
        .def("waitForProjectorPtr", &Lanelet2InterfaceRosPythonWrapper::waitForProjectorPtrNoDefault);
}
