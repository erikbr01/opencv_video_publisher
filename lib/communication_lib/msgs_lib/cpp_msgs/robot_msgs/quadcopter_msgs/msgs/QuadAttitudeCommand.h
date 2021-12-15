#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct QuadAttitudeCommand {

  Header header;

  float thrust;
  float roll;
  float pitch;
  float yaw;
};

} // namespace cpp_msg
