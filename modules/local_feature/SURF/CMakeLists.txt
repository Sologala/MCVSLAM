
add_library(
    SURF STATIC
    ./SURFExtractor.cpp
)

target_include_directories(
    ORB PUBLIC
    ./
    ../BaseExtractor
)
target_link_libraries(
    SURF
    ${OpenCV_LIBS}
)
