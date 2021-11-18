# all cpp files
file(GLOB_RECURSE CPP_SRC_LIST ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
set(TEST_LIBS pyp::yaml;pyp::fmt;pyp::timer;MCVSLAM
)
set(TEST_INCLUDE_DIRS ../modules/;${ROS_INCLUDE_DIRS};${OpenCV_INCLUDE_DIRS})
# generate target for every cpp file
foreach(v ${CPP_SRC_LIST})
    # file relative path from src/
    # delete string "src/"
    # message(${v})
    string(REGEX REPLACE "${CMAKE_CURRENT_LIST_DIR}/" "" target_name ${v})
    # message(${target_name})
    # rename '/' and '.' to '_'
    string(REGEX REPLACE ".cpp" "" target_name ${target_name})
    # message(${target_name})
    add_executable(${target_name} ${v})
    target_link_libraries(${target_name} ${TEST_LIBS})
    target_include_directories(${target_name} PUBLIC ${TEST_INCLUDE_DIRS})
endforeach()
