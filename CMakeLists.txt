cmake_minimum_required(VERSION 3.9)
project(MCVSLAM VERSION 0.0.1)
set(PROJECT_VERSION 3.9)
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-fPIC)
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations")

include (cmake/ThirdPartyConfig.cmake)
include(cmake/FindG2O.cmake)
include (cmake/ROSConfig.cmake)
# include(cmake/FindOpenCV.cmake)


# set(pyp_DIR  "$ENV{HOME}/.local/lib/cmake/pyp")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/app)
find_package(pyp REQUIRED)
link_libraries(pyp::fmt pyp::timer pyp::yaml pyp::cmdline)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3 ")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   -O1 ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O1 ")


# include_directories(
# ThirdParty/DBoW2/DBoW2)

find_package(OpenCV REQUIRED)
message(FOUND OpenCV ${OpenCV_VERSION})

add_subdirectory(modules)
file(GLOB src_file "./src/*.c*")

add_library(MCVSLAM ${src_file})
message(${OpenCV_INCLUDE_DIRS})
target_include_directories(MCVSLAM  PUBLIC
    ./include 
    ${OpenCV_INCLUDE_DIRS}
    ${ROS_INCLUDE_DIRS}
)
target_link_libraries( MCVSLAM 
    ${OpenCV_LIBS}
    thread_pool
    # ros_msg
    capture
    ORB
    # SURF
    camera
    DBoW2
    osg_viewer
    ${G2O_LIBS}
)


add_subdirectory(test)

