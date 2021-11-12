add_subdirectory(ros_msg)


add_library(ros_msg 
    src/multi_msg_wraper.cpp
)

target_include_directories(
    ros_msg PUBLIC
    ./include
    ${ROS_INCLUDE_DIRS}
)
target_link_libraries(
    ros_msg
    ${ROS_LIBS}
    ${OpenCV_LIBS}
)
