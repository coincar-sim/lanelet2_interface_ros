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
//! These params might be requested from ParamReader
namespace params {
constexpr char LatOrigin[] = "lat_origin";
constexpr char LonOrigin[] = "lon_origin";
constexpr char MapFrameId[] = "map_frame_id";
constexpr char MapFileName[] = "map_file_name";
} // namespace params

//! @brief Abstract base class to retrieve parameters needed by LaneletRosInterface and its childs.
//!
//! Provides a customization point to provide different sources of parameters
class ParamReader { // NOLINT(cppcoreguidelines-special-member-functions)
public:
    virtual ~ParamReader() = default;
    //! Returns the requested interface class (used by LaneletRosInterface::instance)
    virtual std::string interfaceType() const = 0;

    //! Returns the value of a string parameter
    virtual std::string getString(const std::string& key) const = 0;

    //! Returns the value of a double parameter
    virtual double getDouble(const std::string& key) const = 0;
};
using ParamReaderUPtr = std::unique_ptr<ParamReader>;

//! Implements a parameter reader that reads parameters from ROS parameter server
class CppRosparamReader : public ParamReader {
public:
    std::string interfaceType() const override;
    std::string getString(const std::string& key) const override;
    double getDouble(const std::string& key) const override;
};

//! @brief Abstract base clase providing an interface to lanelet2 maps/map data.
//!
//! All functions are safe to be used from multiple threads.
class LaneletRosInterface {
public:
    //! @brief Returns a global interface object. The actual type returns depends on the interfaceType returned by
    //! paramReader
    //!
    //! @note The parameters are only used when the function is called for the first time. From then on, it is assumed
    //! that they are identical.
    static LaneletRosInterface& instance(ParamReaderUPtr paramReader = std::make_unique<CppRosparamReader>());

    //! Instanciates a LaneletRosInterface object
    explicit LaneletRosInterface(ParamReaderUPtr paramReader = std::make_unique<CppRosparamReader>())
            : paramReader_{std::move(paramReader)} {
    }
    LaneletRosInterface(LaneletRosInterface&& rhs) noexcept = delete;
    LaneletRosInterface& operator=(LaneletRosInterface&& rhs) noexcept = delete;
    LaneletRosInterface(const LaneletRosInterface& rhs) = delete;
    LaneletRosInterface& operator=(const LaneletRosInterface& rhs) = delete;
    virtual ~LaneletRosInterface() noexcept = default;

    //! Synchronous version to obtain the frame id of the map
    //! @throws InitializationError if parameters could not be retrieved
    std::string getFrameIdMap() const {
        return getFrameIdMapAsync().get();
    }

    //! Synchronous version to obtain the laneletMap. The lanelet map is cached, so multiple calls will retrieve the
    //! same map. Be aware that this call might take a while to return since the whole map has to be parsed first.
    //! @throws InitializationError if parameters could not be retrieved
    LaneletMapConstPtr getLaneletMap() const {
        return getLaneletMapAsync().get();
    }

    //! Synchronous version to obtain a mutable laneletMap. The lanelet map is cached, so multiple calls will retrieve
    //! the same map and see all changes to this map. Mutations from multiple threads are not thread safe.
    //! Depending on the implementation, changes might not be visible in the immutable lanelet map.
    //! @throws InitializationError if parameters could not be retrieved
    LaneletMapPtr getMutableLaneletMap() const {
        return getMutableLaneletMapAsync().get();
    }

    //! Synchronous version to retrieve the projector used for transforming the map
    //! @throws InitializationError if parameters could not be retrieved
    std::shared_ptr<lanelet::Projector> getProjector() const {
        return getProjectorAsync().get();
    }

    /// @name Asynchronous versions
    /// @note The lifetime of this future is bound to the lifetime of this object! Make sure it still exists when
    /// calling "get()"!
    ///@{
    virtual std::shared_future<std::string> getFrameIdMapAsync() const = 0;          //!<@see getFrameIdMap
    virtual std::shared_future<LaneletMapConstPtr> getLaneletMapAsync() const = 0;   //!<@see getLaneletMap
    virtual std::shared_future<LaneletMapPtr> getMutableLaneletMapAsync() const = 0; //!<@see getMutableLaneletMap
    virtual std::shared_future<std::shared_ptr<lanelet::Projector>> getProjectorAsync() const = 0; //!<@see getProjector
    ///@}

protected:
    const ParamReader& paramReader() const {
        return *paramReader_;
    }

private:
    std::unique_ptr<ParamReader> paramReader_;
};
} // namespace interface
} // namespace lanelet
