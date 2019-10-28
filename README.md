[![Build Status](https://api.travis-ci.org/coincar-sim/lanelet2_interface_ros.svg)](https://travis-ci.org/coincar-sim/lanelet2_interface_ros)

# LANELET2 INTERFACE ROS

Library for providing a loaded lanelet2 map and the used geodetic (latitude/ longitude) to Cartesian (easting x/ northing y) projection along with the frame that the x/y coordinates are referenced to.

Use this to ensure you use the same map and the same lat/lon to x/y projection in all your nodes.

## Usage

### C++
Just include this in your cpp ROS node like

```cpp
#include "lanelet2_interface_ros/lanelet2_interface_ros.hpp"

//...

lanelet::interface::LaneletRosInterface& ll2if = lanelet::interface::LaneletRosInterface::instance();
lanelet::LaneletMapConstPtr mapPtr = ll2if.getLaneletMap(); // retrieves the map. May take a while if this if the first call.
```

If you don't want to slow down initialization of your node with the time it takes to load a map, use the async version:
```cpp
auto& ll2if = lanelet::interface::LaneletRosInterface::instance();
std::shared_future<lanelet::LaneletMapConstPtr> mapFuture = ll2if.getLaneletMapAsync(); // spawns a new thread that loads the map, returns instantly

//... do other stuff or put this e.g. in a message callback
if(mapFuture.valid()) {
    // map has finally arrived, retrieve it
    lanelet::LaneletMapConstPtr mapPtr = mapFuture.get();
}
```

### Python
or in your python ROS node like

```python
import lanelet2_interface_ros as ll2if
# ...

# somewhere after rospy.init_node()
llmap = ll2if.getLaneletMap()
```

and launch/include the launchfile [set_lanelet_map.launch](/launch/set_lanelet_map.launch) to receive a loaded map.

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
