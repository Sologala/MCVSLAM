
find_package(OpenCV 4 REQUIRED)
add_library(capture ./capture.cpp)
target_include_directories(capture PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(capture ${OpenCV_LIBS})
