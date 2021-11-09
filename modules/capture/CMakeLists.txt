

add_library(capture ./capture.cpp)
target_include_directories(capture PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(capture ${OpenCV_LIBS})


add_executable(ros_capture ./ros_capture.cpp)
target_include_directories(
    capture  PUBLIC
    ${ROS_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
    ros_capture 
    ${ROS_LIBS}
    ${OpenCV_LIBS}
    capture
    pyp::yaml
    pyp::fmt
)