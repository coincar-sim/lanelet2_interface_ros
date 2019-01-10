# LANELET2 INTERFACE ROS

![build status](https://gitlab.mrt.uni-karlsruhe.de/lanelet2/lanelet2_interface_ros/badges/master/build.svg)
![coverage report](https://gitlab.mrt.uni-karlsruhe.de/lanelet2/lanelet2_interface_ros/badges/master/coverage.svg)

ROS Interface for lanelet2:

**Use this to avoid 1 mio different lat/lon to x/y transformations and different maps being loaded**

Just include this in your cpp ROS node like

```cpp
#include "lanelet2_interface_ros/lanelet2_interface_ros.hpp"

//...

lanelet2_interface_ros::Lanelet2InterfaceRos ll2if;
lanelet::LaneletMapConstPtr mapPtr = ll2if.waitForMapPtr(10, 30); // pull with 10Hz for max. 30s
```

or in your python ROS node like

```python
from lanelet2_interface_ros import Lanelet2InterfaceRos

# ...

# somewhere after rospy.init_node()
ll2if = Lanelet2InterfaceRos()
llmap = ll2if.waitForNonConstMapPtr(10, 30);  # pull with 10Hz for max. 30s

```

and launch/include the launchfile [set_lanelet_map.launch](/launch/set_lanelet_map.launch) to receive a loaded map.
