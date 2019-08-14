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
#include "LocalInterface.h"
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <ros/node_handle.h>
#include "Exceptions.h"

namespace lanelet {
namespace interface {
namespace {
template <typename T>
T getRosParam(const std::string& key) {
    ros::NodeHandle nh;
    T value;
    bool found = nh.getParam("/lanelet2_interface_ros/" + key, value);
    if (!found) {
        throw InitializationError("Lanelet interface failed to find parameter " + key +
                                  " make sure to set it at launch time!");
    }
    return std::move(value);
}

std::shared_ptr<Projector> readProjector(const ParamReader& params) {
    auto latOrigin = params.getDouble(params::LatOrigin);
    auto lonOrigin = params.getDouble(params::LonOrigin);
    return std::make_shared<lanelet::projection::UtmProjector>(lanelet::Origin({latOrigin, lonOrigin}), true);
}

std::string readFrameId(const ParamReader& params) {
    return params.getString(params::MapFrameId);
}
LaneletMapPtr readLaneletMap(const ParamReader& params) {
    auto path = params.getString(params::MapFileName);
    auto projector = readProjector(params);
    return lanelet::load(path, *projector);
}
} // namespace

std::string CppRosparamReader::interfaceType() const {
    return "";
}

std::string CppRosparamReader::getString(const std::string& key) const {
    return getRosParam<std::string>(key);
}

double CppRosparamReader::getDouble(const std::string& key) const {
    return getRosParam<double>(key);
}

LocalLaneletInterface::LocalLaneletInterface(ParamReaderUPtr params)
        : LaneletRosInterface{std::move(params)}, frameId_{[&] {
              return std::async(std::launch::async, [&] { return readFrameId(paramReader()); }).share();
          }},
          map_{[&] {
              // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
              return std::async(std::launch::async, [&] { return readLaneletMap(paramReader()); }).share();
          }},
          constMap_{[&] {
              return std::async(std::launch::async, [&] { return LaneletMapConstPtr(readLaneletMap(paramReader())); })
                  .share();
          }},
          projector_{[&] {
              // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
              return std::async(std::launch::async, [&] { return readProjector(paramReader()); }).share();
          }} {
}

LaneletRosInterface& LaneletRosInterface::instance(ParamReaderUPtr paramReader) {
    // currently we only have one type of interface
    auto type = paramReader->interfaceType();
    if (!type.empty()) {
        throw InvalidInputError("Unsupported interface type " + type);
    }
    static LocalLaneletInterface interface(std::move(paramReader));
    return interface;
}

} // namespace interface
} // namespace lanelet
