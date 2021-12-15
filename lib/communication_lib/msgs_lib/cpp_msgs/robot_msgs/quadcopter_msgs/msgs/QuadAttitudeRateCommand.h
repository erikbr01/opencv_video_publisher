#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct QuadAttitudeRateCommand {

  Header header;

  float thrust;
  float roll_rate;
  float pitch_rate;
  float yaw_rate;
};

} // namespace cpp_msg