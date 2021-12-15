#pragma once
#include "geometry_msgs/msgs/Position.h"
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct QuadPositionCmd {

  Header header;

  Position position;

  float yaw_angle;
};

} // namespace cpp_msg