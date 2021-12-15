

add_library(actuator_msgs_lib INTERFACE)

target_sources(actuator_msgs_lib INTERFACE
msgs/MotorSpeed.idl
)

target_link_libraries(actuator_msgs_lib INTERFACE
std_msgs_lib
)

