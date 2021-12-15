#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct QuadMotorCommand {

  Header header;

  float motorspeed[4];
};

} // namespace cpp_msg