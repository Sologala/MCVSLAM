
add_library(
    SURF STATIC
    ./SURFExtractor.cpp
)
target_include_directories(
    SURF PRIVATE
    ./
)
target_link_libraries(
    SURF
    ${OpenCV_LIBS}
)
