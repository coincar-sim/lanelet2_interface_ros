#include <gtest/gtest.h>
#include <ros/ros.h>

#include "lanelet2_interface_ros.hpp"

TEST(lanelet2_interface_ros, init) {
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;

    lanelet::LaneletMapConstPtr mapPtr = ll2if.waitForMapPtr();
    ASSERT_TRUE(!!mapPtr);

    lanelet::LaneletMapConstPtr mapWithOffsetPtr = ll2if.waitForMapWithOffsetPtr();
    ASSERT_TRUE(!!mapWithOffsetPtr);

    lanelet::LaneletMapPtr nonConstMapPtr = ll2if.waitForNonConstMapPtr();
    ASSERT_TRUE(!!nonConstMapPtr);

    ASSERT_EQ(std::string("map"), ll2if.waitForFrameIdMap());
    ASSERT_EQ(std::string("map_with_offset"), ll2if.waitForFrameIdMapWithOffset());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "unittest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
