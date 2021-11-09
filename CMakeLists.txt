cmake_minimum_required(VERSION 3.9)
project(MCVSLAM VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 11)


if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    include(cmake/DarwinOpenCV.cmake)
endif()


include (cmake/ROSConfig.cmake)
include (cmake/ThirdPartyConfig.cmake)
add_subdirectory(modules)