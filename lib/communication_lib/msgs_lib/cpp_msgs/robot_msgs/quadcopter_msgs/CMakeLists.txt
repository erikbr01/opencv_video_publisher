

add_library(quadcopter_msgs_lib INTERFACE)

target_sources(quadcopter_msgs_lib INTERFACE
msgs/PositionCommand.h
msgs/AttitudeCommand.h
msgs/AttitudeRateCommand.h
msgs/MotorCommand.h
msgs/Mocap.h
msgs/ThrustTorqueCommand.h
)


target_link_libraries(quadcopter_msgs_lib INTERFACE
basic_msgs_lib
)
