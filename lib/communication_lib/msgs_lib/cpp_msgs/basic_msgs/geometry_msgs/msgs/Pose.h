#pragma once
#include "EulerAngleZYX.h"
#include "Position.h"
#include "Quaternion.h"
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct Pose {

  Header header;

  Position position;

  EulerAngleZYZ orientation_euler;

  Quaternion orientation_quat;
};

} // namespace cpp_msg
