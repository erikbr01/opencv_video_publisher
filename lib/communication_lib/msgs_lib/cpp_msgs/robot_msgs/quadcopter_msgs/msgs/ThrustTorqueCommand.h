#pragma once
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct ThrustTorqueCommand {

  Header header;

  float thrust;
  float roll_torque;
  float pitch_torque;
  float yaw_torque;
};

} // namespace cpp_msg