#pragma once
#include "geometry_msgs/msgs/Velocity.h"
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct QuadVelocityCmd {

  Header header;

  Velocity velocity;

  float yaw_angle;
};

} // namespace cpp_msg