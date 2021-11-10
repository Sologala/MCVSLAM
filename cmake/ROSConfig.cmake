
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
)

# set(VAR ${catkin_INCLUDE_DIRS})
set(ROS_INCLUDE_DIRS  "${catkin_INCLUDE_DIRS}")
set(ROS_LIBS  "${catkin_LIBRARIES}")
