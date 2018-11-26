#pragma once

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Projection.h>


namespace lanelet2_interface_ros {
class Lanelet2InterfaceRos {
public:
    Lanelet2InterfaceRos() = default;
    std::string waitForFrameIdMap(double pollRateHz = 10, double timeOutSecs = 30);
    lanelet::LaneletMapConstPtr waitForMapPtr(double pollRateHz = 10, double timeOutSecs = 30);
    lanelet::LaneletMapPtr waitForNonConstMapPtr(double pollRateHz = 10, double timeOutSecs = 30);
    std::shared_ptr<lanelet::Projector> waitForProjectorPtr(double pollRateHz = 10, double timeOutSecs = 30);

private:
    void waitForParams(double pollRateHz, double timeOutSecs);
    void loadMap();
    struct InterfaceParams {
        bool frameIdMapFound{false}, mapFileNameFound{false}, latOriginFound{false}, lonOriginFound{false};
        double latOrigin{0.}, lonOrigin{0.};
        std::string mapFileName{""}, frameIdMap{""};
    };
    InterfaceParams params_;

    std::string frameIdMap_;
    lanelet::LaneletMapPtr nonConstMapPtr_;
    std::shared_ptr<lanelet::Projector> utmProjectorPtr_;
};
} // namespace lanelet2_interface_ros
