#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct Image1080p {

  Header header;

  uint8_t frame[1920 * 1080 * 3];

  // Array size = rows * columns * channels in image
};

} // namespace cpp_msg
