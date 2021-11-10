
add_library(
    ORB STATIC
    ./orb3_extractor/ORBextractor.cc
    ./ORBExtractor.cpp
)
target_include_directories(
    ORB PRIVATE
    ./orb3_extractor
    ./
)
target_link_libraries(
    ORB
    ${OpenCV_LIBS}
)
