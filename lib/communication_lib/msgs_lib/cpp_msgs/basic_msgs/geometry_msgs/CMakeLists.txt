

add_library(geometry_msgs_lib INTERFACE)

target_sources(geometry_msgs_lib INTERFACE
msgs/EulerAngleZYX.h
msgs/Pose.h
msgs/Position.h
msgs/Quaternion.h
)


target_link_libraries(geometry_msgs_lib INTERFACE
std_msgs_lib
)
