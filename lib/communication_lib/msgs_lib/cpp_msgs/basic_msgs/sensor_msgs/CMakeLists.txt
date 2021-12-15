

add_library(sensor_msgs_lib INTERFACE)

target_sources(sensor_msgs_lib INTERFACE
msgs/Mocap.h
msgs/ImageHD.h
)

target_link_libraries(sensor_msgs_lib INTERFACE
std_msgs_lib
geometry_msgs_lib
)

