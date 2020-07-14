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

#pragma once

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>


namespace lanelet2_interface_ros {
class Lanelet2InterfaceRos { // NOLINT(cppcoreguidelines-special-member-functions)
public:
    Lanelet2InterfaceRos() = default;
    virtual ~Lanelet2InterfaceRos() = default;
    std::string waitForFrameIdMap(double pollRateHz = 10, double timeOutSecs = -1);
    lanelet::LaneletMapConstPtr waitForMapPtr(double pollRateHz = 10, double timeOutSecs = -1);
    lanelet::LaneletMapPtr waitForNonConstMapPtr(double pollRateHz = 10, double timeOutSecs = -1);
    std::shared_ptr<lanelet::Projector> waitForProjectorPtr(double pollRateHz = 10, double timeOutSecs = -1);

protected:
    struct InterfaceParams {
        bool frameIdMapFound{false}, mapFileNameFound{false}, latOriginFound{false}, lonOriginFound{false};
        double latOrigin{0.}, lonOrigin{0.};
        std::string mapFileName{""}, frameIdMap{""};
    };

private:
    void loadMap();
    void waitForParams(double pollRateHz, double timeOutSecs);
    virtual InterfaceParams waitForParamsImpl(double pollRateHz, double timeOutSecs);

    std::string frameIdMap_;
    lanelet::LaneletMapPtr nonConstMapPtr_;
    std::shared_ptr<lanelet::Projector> utmProjectorPtr_;
    InterfaceParams params_;
};
} // namespace lanelet2_interface_ros
