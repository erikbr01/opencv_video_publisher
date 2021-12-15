
add_library(robot_msgs_lib INTERFACE)

target_link_libraries(robot_msgs_lib INTERFACE
quadcopter_msgs_lib
)

add_subdirectory(quadcopter_msgs)
