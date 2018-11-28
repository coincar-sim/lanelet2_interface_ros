// It is very important that this is the first header in every file.
// This includes some modifications to the boost::python headers that
// make them work with memory-aligned Eigen types.
// For more usage examples, look at
// https://github.com/ethz-asl/programming_guidelines/wiki/Adding-python-bindings-to-your-cpp-catkin-package
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// Now bring in the headers of functions and classes you are wrapping.
#include "lanelet2_interface_ros.hpp"

#include <ros/ros.h>

// The module name here *must* match the name of the python project. You can use the PYTHON_API_MODULE_NAME definition.
BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {
    // This using statement is just for convenience
    using namespace boost::python;
    using namespace lanelet2_interface_ros;

    // we need an initialized roscpp which is not done by rospy.init_node, see
    // http://wiki.ros.org/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python for details
    // so we do it here
    class Lanelet2InterfaceRosPythonWrapper : Lanelet2InterfaceRos {
    public:
        Lanelet2InterfaceRosPythonWrapper() : Lanelet2InterfaceRos() {
            ros::init(std::map<std::string, std::string>{},
                      "lanelet2_interface_ros_for_python",
                      ros::init_options::AnonymousName);
        }
        std::string waitForFrameIdMap(double pollRateHz, double timeOutSecs) {
            return Lanelet2InterfaceRos::waitForFrameIdMap(pollRateHz, timeOutSecs);
        }
        lanelet::LaneletMapConstPtr waitForMapPtr(double pollRateHz, double timeOutSecs) {
            return Lanelet2InterfaceRos::waitForMapPtr(pollRateHz, timeOutSecs);
        }
        lanelet::LaneletMapPtr waitForNonConstMapPtr(double pollRateHz, double timeOutSecs) {
            return Lanelet2InterfaceRos::waitForNonConstMapPtr(pollRateHz, timeOutSecs);
        }
        std::shared_ptr<lanelet::Projector> waitForProjectorPtr(double pollRateHz, double timeOutSecs) {
            return Lanelet2InterfaceRos::waitForProjectorPtr(pollRateHz, timeOutSecs);
        }
    };

    class_<Lanelet2InterfaceRosPythonWrapper>("Lanelet2InterfaceRos")
        .def("waitForFrameIdMap", &Lanelet2InterfaceRosPythonWrapper::waitForFrameIdMap)
        .def("waitForMapPtr", &Lanelet2InterfaceRosPythonWrapper::waitForMapPtr)
        .def("waitForNonConstMapPtr", &Lanelet2InterfaceRosPythonWrapper::waitForNonConstMapPtr)
        .def("waitForProjectorPtr", &Lanelet2InterfaceRosPythonWrapper::waitForProjectorPtr);
}
