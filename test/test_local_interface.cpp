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

#include <chrono>
#include <string>
#include <gtest/gtest.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include "LocalInterface.h"

constexpr double Lat0 = 45;
constexpr double Lon0 = 8;
constexpr char FrameId[] = "frame";

namespace {
class TempMap {
public:
    TempMap() {
        using namespace lanelet;
        Point3d p{0, 0, 0};
        LineString3d ls1{1, {p}};
        LineString3d ls2{2, {p}};
        Lanelet ll{2, ls1, ls2};
        map_ = utils::createMap({ll});
        lanelet::write(path(), *map_, projection::UtmProjector(Origin{{Lat0, Lon0}}));
    }
    TempMap(TempMap&& rhs) noexcept = delete;
    TempMap& operator=(TempMap&& rhs) noexcept = delete;
    TempMap(const TempMap& rhs) = delete;
    TempMap& operator=(const TempMap& rhs) = delete;
    ~TempMap() {
        std::remove(path_.c_str());
    }

    lanelet::LaneletMap& get() {
        return *map_;
    }
    const std::string& path() const noexcept {
        return path_;
    }

private:
    lanelet::LaneletMapUPtr map_;
    std::string path_{std::tmpnam(nullptr) + std::string(".osm")};
};

class MockParamReader : public lanelet::interface::ParamReader {
public:
    explicit MockParamReader(std::string mapFile) : mapFile_{std::move(mapFile)} {
    }

    std::string interfaceType() const override {
        return "";
    }
    std::string getString(const std::string& key) const override {
        using namespace lanelet::interface;
        if (key == params::MapFileName) {
            return mapFile_;
        }
        if (key == params::MapFrameId) {
            return FrameId;
        }
        throw std::runtime_error("Invalid key: " + key);
    }

    double getDouble(const std::string& key) const override {
        using namespace lanelet::interface;
        if (key == params::LatOrigin) {
            return Lat0;
        }
        if (key == params::LonOrigin) {
            return Lon0;
        }
        throw std::runtime_error("Invalid key: " + key);
    }

private:
    std::string mapFile_;
};
} // namespace


TEST(LocalLaneletInterface, synchronously) {
    TempMap referenceMap;
    auto params = std::make_unique<MockParamReader>(referenceMap.path());
    lanelet::interface::LocalLaneletInterface ll2if(std::move(params));

    EXPECT_EQ(ll2if.getFrameIdMap(), FrameId);
    ASSERT_TRUE(!!ll2if.getProjector());
    EXPECT_EQ(ll2if.getProjector()->origin().position.lat, Lat0);
    EXPECT_EQ(ll2if.getProjector()->origin().position.lon, Lon0);
    auto map = ll2if.getLaneletMap();
    map = ll2if.getLaneletMap();


    ASSERT_TRUE(!!map);
    EXPECT_EQ(referenceMap.get().laneletLayer.size(), map->laneletLayer.size());
}

TEST(LocalLaneletInterface, asynchronously) {
    TempMap referenceMap;
    auto params = std::make_unique<MockParamReader>(referenceMap.path());
    lanelet::interface::LocalLaneletInterface ll2if(std::move(params));

    using Time = std::chrono::steady_clock;
    auto tStart = Time::now();
    auto mapFut = ll2if.getMutableLaneletMapAsync();
    auto tFut = Time::now();
    auto t = std::thread([&] {
        auto& map = ll2if.getMutableLaneletMapAsync().get();
        EXPECT_TRUE(!!map);
    });
    auto& map = mapFut.get();
    auto tEnd = Time::now();
    t.join();
    EXPECT_TRUE(!!map);
    EXPECT_GT(tEnd - tFut, tFut - tStart);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
