add_executable(opencv_video_publisher
src/main.cpp
)

target_include_directories(opencv_video_publisher PUBLIC include)

target_link_libraries(opencv_video_publisher PRIVATE
comm_lib
)

# Third party dependencies
target_link_libraries(opencv_video_publisher PRIVATE
fastcdr 
fastrtps
${OpenCV_LIBS}
)
