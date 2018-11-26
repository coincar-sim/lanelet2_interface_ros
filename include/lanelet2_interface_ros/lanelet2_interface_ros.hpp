#pragma once

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>

namespace lanelet2_interface_ros {
class Lanelet2InterfaceRos {
public:
    Lanelet2InterfaceRos();
    bool isInitialized() const;
    void waitForInit(double pollRateHz = 10, double timeOutSecs = 30);
    std::string getFrameIdOrigin() const;
    lanelet::LaneletMapConstPtr getMapPtr() const;

private:
    bool isInitialized_;
    std::string frameIdOrigin_;
    lanelet::LaneletMapConstPtr mapPtr_;
};
} // namespace lanelet2_interface_ros
