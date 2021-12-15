#pragma once
#include "geometry_msgs/msgs/Pose.h"
#include "std_msgs/msgs/Header.h"

namespace cpp_msg {

struct Mocap {

  Header header;

  Pose pose;

  float latency;
};

} // namespace cpp_msg
