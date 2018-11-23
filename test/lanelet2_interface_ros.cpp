#include <gtest/gtest.h>
#include <ros/ros.h>

#include "lanelet2_interface_ros.hpp"

TEST(lanelet2_interface_ros, throwOnTimeout) {
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
    ASSERT_THROW(ll2if.getMapPtr(), std::runtime_error);
    ASSERT_THROW(ll2if.getFrameIdOrigin(), std::runtime_error);
}

TEST(lanelet2_interface_ros, throwOnUninitializedUse) {
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
    ASSERT_THROW(ll2if.waitForInit(), std::runtime_error);
}

TEST(lanelet2_interface_ros, init) {
    lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
    ros::NodeHandle nh;
    nh.setParam("/lanelet2_interface_ros/frame_id_origin", "map");
    nh.setParam("/lanelet2_interface_ros/lat_origin", 49.);
    nh.setParam("/lanelet2_interface_ros/lon_origin", 8.);
    std::string exampleMapPath = std::string(PKG_DIR) + "/../lanelet2/lanelet2_maps/res/mapping_example.osm";
    nh.setParam("/lanelet2_interface_ros/map_file_name", exampleMapPath);

    ll2if.waitForInit();

    // the interface is initialized
    ASSERT_TRUE(ll2if.isInitialized());

    // we can retreive the map
    lanelet::LaneletMapConstPtr mapPtr = ll2if.getMapPtr();
    ASSERT_TRUE(!!mapPtr);

    // and the frame_id
    ASSERT_STREQ(std::string("map").c_str(), ll2if.getFrameIdOrigin().c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "unittest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
