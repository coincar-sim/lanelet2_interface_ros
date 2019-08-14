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
#include <future>
#include <memory>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>
#include "Exceptions.h"

namespace lanelet {
namespace interface {
namespace params {
constexpr char LatOrigin[] = "lat_origin";
constexpr char LonOrigin[] = "lon_origin";
constexpr char MapFrameId[] = "map_frame_id";
constexpr char MapFileName[] = "map_file_name";
} // namespace params

class ParamReader { // NOLINT(cppcoreguidelines-special-member-functions)
public:
    virtual ~ParamReader() = default;
    virtual std::string interfaceType() const = 0;
    virtual std::string getString(const std::string& key) const = 0;
    virtual double getDouble(const std::string& key) const = 0;
};
using ParamReaderUPtr = std::unique_ptr<ParamReader>;

class CppRosparamReader : public ParamReader {
public:
    std::string interfaceType() const override;
    std::string getString(const std::string& key) const override;
    double getDouble(const std::string& key) const override;
};

//! Abstract base clase for an interface to lanelet2 maps/map data
class LaneletRosInterface {
public:
    static LaneletRosInterface& instance(ParamReaderUPtr paramReader = std::make_unique<CppRosparamReader>());

    explicit LaneletRosInterface(ParamReaderUPtr paramReader = std::make_unique<CppRosparamReader>())
            : paramReader_{std::move(paramReader)} {
    }
    LaneletRosInterface(LaneletRosInterface&& rhs) noexcept = delete;
    LaneletRosInterface& operator=(LaneletRosInterface&& rhs) noexcept = delete;
    LaneletRosInterface(const LaneletRosInterface& rhs) = delete;
    LaneletRosInterface& operator=(const LaneletRosInterface& rhs) = delete;
    virtual ~LaneletRosInterface() noexcept = default;

    // Synchrononous versions
    std::string getFrameIdMap() {
        return getFrameIdMapAsync().get();
    }
    LaneletMapConstPtr getLaneletMap() {
        return getLaneletMapAsync().get();
    }
    LaneletMapPtr getMutableLaneletMap() {
        return getMutableLaneletMapAsync().get();
    }
    std::shared_ptr<lanelet::Projector> getProjector() {
        return getProjectorAsync().get();
    }

    // Asynchronous versions. Careful: The lifetime of this future is bound to the lifetime of this object! Make sure it
    // still exists when calling "get()"!
    virtual std::shared_future<std::string> getFrameIdMapAsync() = 0;
    virtual std::shared_future<LaneletMapConstPtr> getLaneletMapAsync() = 0;
    virtual std::shared_future<LaneletMapPtr> getMutableLaneletMapAsync() = 0;
    virtual std::shared_future<std::shared_ptr<lanelet::Projector>> getProjectorAsync() = 0;

protected:
    const ParamReader& paramReader() const {
        return *paramReader_;
    }

private:
    std::unique_ptr<ParamReader> paramReader_;
};
} // namespace interface
} // namespace lanelet
