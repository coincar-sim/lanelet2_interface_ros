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

#include <boost/python.hpp>
#include "Exceptions.h"
#include "Interface.h"

namespace {
template <typename T>
inline T getRospyParam(const std::string& key) {
    auto rospy = boost::python::import("rospy");
    return boost::python::extract<T>(rospy.attr("get_param")("/lanelet2_interface_ros/" + key));
}

class PyRosparamReader : public lanelet::interface::ParamReader {
public:
    std::string interfaceType() const override {
        return "";
    }

    std::string getString(const std::string& key) const override {
        return getRospyParam<std::string>(key);
    }
    double getDouble(const std::string& key) const override {
        return getRospyParam<double>(key);
    }
};

lanelet::interface::LaneletRosInterface& ifInstance() {
    return lanelet::interface::LaneletRosInterface::instance(std::make_unique<PyRosparamReader>());
}
} // namespace

// The module name here *must* match the name of the python project. You can use the PYTHON_API_MODULE_NAME definition.
BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {
    // This using statement is just for convenience
    using namespace boost::python;
    using namespace lanelet::interface;

    def("getFrameIdMap", +[] { return ifInstance().getFrameIdMap(); }, "Returns the tf frame id for the lanelet map");
    def("getLaneletMap", +[] { return ifInstance().getLaneletMap(); }, "Returns the current lanelet map");
    def("getProjector", +[] { return ifInstance().getProjector(); }, "Returns the current transformation object");
}
