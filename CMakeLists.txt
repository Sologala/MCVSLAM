cmake_minimum_required(VERSION 3.9)
project(MCVSLAM VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 11)

include (cmake/ROSConfig.cmake)

add_subdirectory(modules)