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

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <lanelet2_io/Projection.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include "lanelet2_interface_ros.hpp"


TEST(lanelet2_interface_ros, init) {
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;

    lanelet::LaneletMapConstPtr mapPtr = ll2if.waitForMapPtr();
    ASSERT_TRUE(!!mapPtr);
    EXPECT_EQ(371, mapPtr->laneletLayer.size()) << "expected 371 lanelets in map";

    lanelet::LaneletMapPtr nonConstMapPtr = ll2if.waitForNonConstMapPtr();
    ASSERT_TRUE(!!nonConstMapPtr);

    std::shared_ptr<lanelet::Projector> projectorPtr = ll2if.waitForProjectorPtr();
    ASSERT_TRUE(!!projectorPtr);

    lanelet::BasicPoint3d point = projectorPtr->forward(lanelet::GPSPoint{49., 8.});
    ASSERT_EQ(0., point.x());
    ASSERT_EQ(0., point.y());

    ASSERT_EQ(std::string("map"), ll2if.waitForFrameIdMap());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "unittest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
