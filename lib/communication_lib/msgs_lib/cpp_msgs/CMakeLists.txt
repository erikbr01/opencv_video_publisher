

add_library(msgs_lib INTERFACE)

target_include_directories(msgs_lib INTERFACE
basic_msgs
robot_msgs
)

add_subdirectory(basic_msgs)
add_subdirectory(robot_msgs)
