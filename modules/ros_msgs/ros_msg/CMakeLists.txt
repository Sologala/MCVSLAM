
project(multi_msgs VERSION 0.0.1)
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs geometry_msgs sensor_msgs)
include_directories(${catkin_INCLUDE_DIRS})

file(GLOB ALL_MSGS "msg/*.msg")
foreach(msg_file ${ALL_MSGS})
    # message("asdfadsf " ${ALL_MSGS})
    STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${ALL_MSGS})
    # message("asdfadsf " ${FILE_NAME})    
    list(APPEND MSGS ${FILE_NAME}.msg)
endforeach()
message("Added msgs : " ${MSGS})


add_message_files(
  FILES
  ${MSGS}
)

#generate_messages必须在catkin_package前面
generate_messages(
 DEPENDENCIES
 sensor_msgs
 std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS  std_msgs  sensor_msgs
)
# build/devel/include/multi_msgs
set(ROS_MSG_GEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/build/devel/include/${PROJECT_NAME})
set(ROS_MSG_GEN_DIST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
file(GLOB header_files ${ROS_MSG_GEN_INCLUDE_DIR}/*.h)
# message(${header_files})
foreach(h ${header_files})
    file(COPY ${h} DESTINATION ${ROS_MSG_GEN_DIST_DIR})
endforeach()




