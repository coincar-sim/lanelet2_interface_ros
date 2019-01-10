#include <gtest/gtest.h>
#include <ros/ros.h>

#include <lanelet2_io/Projection.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include "lanelet2_interface_ros.hpp"


TEST(lanelet2_interface_ros, init) {
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;

    lanelet::LaneletMapConstPtr mapPtr = ll2if.waitForMapPtr();
    ASSERT_TRUE(!!mapPtr);
    EXPECT_EQ(372, mapPtr->laneletLayer.size()) << "expected 372 lanelet in map";

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
