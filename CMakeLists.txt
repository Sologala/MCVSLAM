cmake_minimum_required(VERSION 3.9)
project(MCVSLAM VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 11)

include(cmake/FindOpenCV.cmake)
include (cmake/ROSConfig.cmake)
include (cmake/ThirdPartyConfig.cmake)
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};"$ENV{HOME}/.local")
set(pyp_DIR  "$ENV{HOME}/.local/lib/cmake/pyp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/app)
find_package(pyp REQUIRED)
link_libraries(pyp::fmt pyp::timer pyp::yaml pyp::cmdline)

message(${ROS_INCLUDE_DIRS})
add_subdirectory(ThirdParty)
add_subdirectory(modules)
add_subdirectory(test)