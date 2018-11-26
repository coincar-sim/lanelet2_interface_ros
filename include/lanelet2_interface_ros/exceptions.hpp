#pragma once

#include <stdexcept>

namespace lanelet2_interface_ros {

class InitializationError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

} // lanelet2_interface_ros
