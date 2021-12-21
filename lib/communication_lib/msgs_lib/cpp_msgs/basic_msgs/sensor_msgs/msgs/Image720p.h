#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct Image720p {

  Header header;

  uint8_t frame[1280 * 720 * 3];

  // Array size = rows * columns * channels in image
};

} // namespace cpp_msg
