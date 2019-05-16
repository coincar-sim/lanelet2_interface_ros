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

lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
lanelet::LaneletMapConstPtr mapPtr = ll2if.waitForMapPtr(10, 30); // pull with 10Hz for max. 30s
```

### Python
or in your python ROS node like

```python
from lanelet2_interface_ros import Lanelet2InterfaceRos

# ...

# somewhere after rospy.init_node()
ll2if = Lanelet2InterfaceRos()
llmap = ll2if.waitForNonConstMapPtr(10, 30);  # pull with 10Hz for max. 30s

```

and launch/include the launchfile [set_lanelet_map.launch](/launch/set_lanelet_map.launch) to receive a loaded map.

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
