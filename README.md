# LANELET2 INTERFACE ROS

![build status](https://gitlab.mrt.uni-karlsruhe.de/lanelet2/lanelet2_interface_ros/badges/master/build.svg)
![coverage report](https://gitlab.mrt.uni-karlsruhe.de/lanelet2/lanelet2_interface_ros/badges/master/coverage.svg)

ROS Interface for lanelet2:

** Use this to avoid 1 mio different lat/lon2x/y transformations and different maps being loaded **

Just include this in your ROS node like

```cpp
#include "lanelet2_interface_ros/lanelet2_interface_ros.hpp"

//...

lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
ll2if.waitForInit();
lanelet::LaneletMapConstPtr mapPtr = ll2if.getMapPtr();
```
to receive a loaded map.
