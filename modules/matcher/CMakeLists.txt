add_library(matcher STATIC
./Matcher.cpp 
)
target_include_directories(
    matcher PUBLIC
    .
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
    matcher
    ${OpenCV_LIBS}
)
