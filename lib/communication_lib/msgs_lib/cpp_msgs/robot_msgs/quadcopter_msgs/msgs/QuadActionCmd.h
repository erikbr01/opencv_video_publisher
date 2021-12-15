#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct QuadActionCmd {

  Header header;

  uint8_t cmd;
};

} // namespace cpp_msg