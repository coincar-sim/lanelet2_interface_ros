#pragma once

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>

namespace lanelet2_interface_ros {
class Lanelet2InterfaceRos {
public:
    Lanelet2InterfaceRos();
    std::string waitForFrameIdMap(double pollRateHz = 10, double timeOutSecs = 30);
    lanelet::LaneletMapConstPtr waitForMapPtr(double pollRateHz = 10, double timeOutSecs = 30);
    lanelet::LaneletMapPtr waitForNonConstMapPtr(double pollRateHz = 10, double timeOutSecs = 30);

private:
    void waitForParams_(double pollRateHz, double timeOutSecs);
    struct InterfaceParams {
        bool frameIdMapFound{false}, mapFileNameFound{false}, latOriginFound{false}, lonOriginFound{false};
        double latOrigin{0.}, lonOrigin{0.};
        std::string mapFileName{""}, frameIdMap{""};
    };
    InterfaceParams params_;

    std::string frameIdMap_;
    lanelet::LaneletMapPtr nonConstMapPtr_;
};
} // namespace lanelet2_interface_ros
