#include <gtest/gtest.h>
#include <ros/ros.h>

#include "exceptions.hpp"
#include "lanelet2_interface_ros.hpp"

TEST(lanelet2_interface_ros, throwOnTimeout) {
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;

    ASSERT_THROW(ll2if.waitForMapPtr(10, 5), lanelet2_interface_ros::InitializationError);

    ASSERT_THROW(ll2if.waitForMapWithOffsetPtr(10, 5), lanelet2_interface_ros::InitializationError);

    ASSERT_THROW(ll2if.waitForNonConstMapPtr(10, 5), lanelet2_interface_ros::InitializationError);

    ASSERT_THROW(ll2if.waitForFrameIdMap(10, 5), lanelet2_interface_ros::InitializationError);

    ASSERT_THROW(ll2if.waitForFrameIdMapWithOffset(10, 5), lanelet2_interface_ros::InitializationError);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "unittest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
