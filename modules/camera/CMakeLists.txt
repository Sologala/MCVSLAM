
find_package(Eigen3 REQUIRED)
file(GLOB Camera_Src_file "./*.c*")
add_library(
    camera STATIC
    ${Camera_Src_file}
)

target_include_directories(
    camera PUBLIC
    .
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
    camera
    ${OpenCV_LIBS}
    Eigen3::Eigen
)